import global_config as gconfig

import time
import carla
import os
import importlib

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from routing.router import Router
from coderace_stack import CodeRaceStack

from omegaconf import OmegaConf
import numpy as np
import cv2

if gconfig.ENABLE_DEBUG_VIS and gconfig.VIDEO_RECORD_DIR:
    def setup_record_cam(world, ego_vehicle):
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_location = carla.Location(0, 0, 50)
        cam_rotation = carla.Rotation(270, 0, 0)
        cam_transform = carla.Transform(cam_location, cam_rotation)
        cam_bp.set_attribute("image_size_x", str(1280))
        cam_bp.set_attribute("image_size_y", str(720))
        cam_bp.set_attribute("fov", str(110))
        ego_cam = world.spawn_actor(
            cam_bp, cam_transform, attach_to=ego_vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        return ego_cam

    video_name = os.path.join(gconfig.VIDEO_RECORD_DIR, str(time.time()))
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    writer = cv2.VideoWriter('{}.avi'.format(video_name), fourcc, 20, (1280, 720))

    def to_bgra_array(image):
        """Convert a CARLA raw image to a BGRA numpy array."""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        return array

    def save_carla_video(image):
        global writer
        array = to_bgra_array(image)
        # Convert BGRA to RGB.
        array = array[:, :, :3]
        # array = array[:, :, ::-1]
        writer.write(array)


def get_entry_point():
    return "CodeRaceAgent"


class CodeRaceAgent(AutonomousAgent):
    def setup(self, config):
        self.track = Track.MAP
        self.config = OmegaConf.load(config) \
            if isinstance(config, str) else config

        # get perception modules
        objdetection_module = importlib.import_module(
            self.config.object_detection)
        objdetection_obj = getattr(
            objdetection_module, 'get_object_detection')()
        objdetection_obj = getattr(
            objdetection_module, objdetection_obj)(self.config)

        # get localization module
        localization_module = importlib.import_module(self.config.localization)
        localization_obj = getattr(localization_module, 'get_localization')()
        localization_obj = getattr(
            localization_module, localization_obj)(self.config)

        self.perception = [
            objdetection_obj
        ]
        self.localization = localization_obj

        self.router = Router(self.config)
        self.stack = CodeRaceStack(
            self.config,
            self.perception,
            self.localization,
            self.router
        )

        self._initialization = True
        self._map_initialization = True
        self._state = None

        self.scenario_state = True  # False if it's crashed

    def run_step(self, input_data, timestamp, world=None, ego_vehicle=None):
        if self._initialization:
            # Leave cam actor object here so it is terminated along with agent
            if gconfig.ENABLE_DEBUG_VIS and gconfig.VIDEO_RECORD_DIR:
                self.record_cam = setup_record_cam(world, ego_vehicle)
                self.record_cam.listen(save_carla_video)

        map_loaded = False
        if self._map_initialization and "OPENDRIVE_MAP" in input_data:
            print("Loading opendrive map")
            map = carla.Map("map", input_data["OPENDRIVE_MAP"][1]["opendrive"])
            self.router.set_map(map)
            self.router.set_route(self._global_plan_world_coord)
            map_loaded = True

        # perception cheat
        for module in self.perception:
            module.cheat(world, ego_vehicle)

        # localization cheat
        self.localization.cheat(world, ego_vehicle)

        if not self.router.is_initialized():
            return self.stack.stay_still()

        vehicle_control = self.stack.run(input_data, timestamp)

        self._store_data(timestamp)
        if gconfig.ENABLE_DEBUG_VIS:
            self._visualize_on_carla(world)

        self._initialization = False
        if map_loaded:
            self._map_initialization = False

        return vehicle_control

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        """
        Set the plan (route) for the agent
        """
        self._global_plan_world_coord = global_plan_world_coord
        self._global_plan = global_plan_gps

    def _store_data(self, timestamp):
        """
        Topology segments will be store as n x 2 x 6
        The last element is geometry information:
        [x, y, z, roll, pitch, yaw] (in meters and radians)

        Similarly, Waypoints will be store as n x 6
        """
        self._state = {
            "timestamp": timestamp,
            "ego_pose": self.localization.agent.state,
            "ego_dimensions": self.localization.agent.dimensions,
            "ego_dynamics": self.localization.agent.dynamics,
            "global_route": self.router._remaining_route,
            "npcs": self.perception[0].agents
        }

    def destroy(self):
        self.stack.destroy()

    def sensors(self):
        self.camera_x = 1.5
        self.camera_y = 0.0
        self.camera_z = 2.4
        sensors = [
            {
                "type": "sensor.opendrive_map",
                "id": "OPENDRIVE_MAP",
                "reading_frequency": 0.1
            },
            {
                "type": "sensor.speedometer",
                "id": "SPEED"
            }
        ]

        return sensors

    def _visualize_on_carla(self, world):
        ego_pose = self._state["ego_pose"]

        # draw route
        route = self._state["global_route"]
        for i in range(min(30, len(route))):
            wp = route[i]
            world.debug.draw_point(
                carla.Location(
                    wp.transform.location.x,
                    wp.transform.location.y,
                    max(wp.transform.location.z, ego_pose.location.z)
                ),
                size=0.05, color=carla.Color(255, 0, 0), life_time=0.1
            )

        # draw localization
        ego_dimensions = self._state["ego_dimensions"]
        ego_bbox = carla.BoundingBox(
            carla.Location(
                ego_pose.location.x,
                ego_pose.location.y,
                ego_pose.location.z + 0.2
            ), ego_dimensions.extent
        )
        world.debug.draw_box(
            ego_bbox, ego_pose.rotation,
            thickness=0.2, color=carla.Color(0, 255, 0), life_time=0.1
        )

        # draw object detection
        for npc in self._state["npcs"]:
            npc_bbox = carla.BoundingBox(
                carla.Location(
                    npc.state.location.x,
                    npc.state.location.y,
                    npc.state.location.z + 0.2
                ), npc.dimensions.extent
            )
            world.debug.draw_box(
                npc_bbox, npc.state.rotation,
                thickness=0.2, color=carla.Color(255, 0, 0), life_time=0.1
            )
