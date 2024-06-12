from carla import VehicleControl

from .controllers.pid_lat_controller import PidLatController
from .controllers.pid_lon_controller import PidLonController
from .controllers.aeb_controller import (
    calculate_stopping_time,
    calculate_ttc,
    state_transition_logic,
    aeb_activation_logic
)

from .planners.cubic_spline_planner import Spline2D

import numpy as np


def get_driver():
    return "BaselineDriver"


class BaselineDriver():
    def __init__(self, router):
        self.router = router
        self.pid_lat = PidLatController()
        self.pid_lon = PidLonController()
        self.aeb_state = 0  # Initial state

    def run(
        self, ego_pose, ego_dimension, ego_dynamics,
        npcs, timestamp, horizon=30
    ):
        ref_path = self.router.get_reference_path(
            ego_pose, ego_dynamics, timestamp, horizon)

        seed_trajectory = self.plan_trajectory(ref_path)

        target_waypoint = seed_trajectory[0]
        steering = self.pid_lat.pid_control(ego_pose, target_waypoint)

        # AEB System Integration
        ego_speed = ego_dynamics.get_speed()
        lead_vehicle = self.get_lead_vehicle(ego_pose, npcs)

        # throt = self.pid_lon.pid_control(
        #     target_speed=10,
        #     current_speed=ego_dynamics.get_speed()
        # )

        if lead_vehicle:
            relative_velocity = ego_speed - lead_vehicle.get_velocity().length()
            relative_distance = self.calculate_distance(ego_pose, lead_vehicle)

            t_fcw = calculate_stopping_time(ego_speed, 3.8) + calculate_stopping_time(ego_speed, 9.8)
            t_pb1 = calculate_stopping_time(ego_speed, 3.8)
            t_pb2 = calculate_stopping_time(ego_speed, 5.8)
            t_fb = calculate_stopping_time(ego_speed, 9.8)
            ttc = calculate_ttc(relative_velocity, relative_distance)
            
            self.aeb_state = state_transition_logic(ttc, t_fcw, t_pb1, t_pb2, t_fb)
            deceleration = aeb_activation_logic(self.aeb_state, ego_speed)
            
            throttle, brake = self.calculate_throttle_brake(deceleration)
        else:
            throttle = self.pid_lon.pid_control(
                target_speed=10,
                current_speed=ego_speed
            )
            brake = 0.0

        # return VehicleControl(steer=steering, throttle=throt, brake=0.)
        return VehicleControl(steer=steering, throttle=throttle, brake=brake)
    
    def get_lead_vehicle(self, ego_pose, npcs):
        """
        Get the nearest lead vehicle in front of the ego vehicle.
        """
        # Simplified example to get the closest vehicle ahead
        closest_vehicle = None
        min_distance = float('inf')
        for npc in npcs:
            distance = self.calculate_distance(ego_pose, npc)
            if distance < min_distance and self.is_ahead(ego_pose, npc):
                min_distance = distance
                closest_vehicle = npc
        return closest_vehicle

    def calculate_distance(self, ego_pose, npc):
        """
        Calculate the distance between the ego vehicle and an NPC vehicle.
        """
        dx = npc.get_location().x - ego_pose.location.x
        dy = npc.get_location().y - ego_pose.location.y
        return np.sqrt(dx**2 + dy**2)

    def is_ahead(self, ego_pose, npc):
        """
        Determine if an NPC vehicle is ahead of the ego vehicle.
        """
        ego_heading = np.array([np.cos(np.radians(ego_pose.rotation.yaw)), np.sin(np.radians(ego_pose.rotation.yaw))])
        npc_direction = np.array([npc.get_location().x - ego_pose.location.x, npc.get_location().y - ego_pose.location.y])
        return np.dot(ego_heading, npc_direction) > 0

    def calculate_throttle_brake(self, deceleration):
        """
        Calculate the throttle and brake values based on the required deceleration.
        """
        if deceleration == 0:
            return 0.5, 0.0  # Maintain speed
        elif deceleration > 0:
            return 0.0, deceleration / 9.8  # Apply brake proportional to max deceleration
        else:
            return -deceleration / 9.8, 0.0  # Apply throttle proportional to negative deceleration

    def plan_trajectory(self, ref_path, ds=1.):
        # NOTE: This is NOT a real planner
        traj = []
        # sp = Spline2D(
        #     [wp.transform.location.x for wp in ref_path],
        #     [wp.transform.location.y for wp in ref_path]
        # )

        sp = Spline2D(
            [wp[0] for wp in ref_path],
            [wp[1] for wp in ref_path]
        )

        s = np.arange(0, sp.s[-1], ds)
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            if ix is None:
                break
            iyaw = sp.calc_yaw(i_s)
            traj.append([ix, iy, iyaw])

        return traj
