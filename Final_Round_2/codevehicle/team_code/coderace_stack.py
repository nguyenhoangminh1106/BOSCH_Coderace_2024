import carla
import importlib
import traceback
from common.utils.stopwatch import Stopwatch


class CodeRaceStack:
    def __init__(self, config, perception, localization, router):
        self.config = config
        self.object_detection = perception[0]
        self.localization = localization
        self.router = router

        driver_module = importlib.import_module(self.config.driver)
        driver_obj = getattr(driver_module, 'get_driver')()
        self.driver = getattr(driver_module, driver_obj)(router)

    def run(self, inputs, timestamp):
        vehicle_control = None
        try:
            # --- localization ---
            with Stopwatch("localization"):
                self.localization.run(input_data=inputs)
                ego_pose = self.localization.agent.state
                ego_dimensions = self.localization.agent.dimensions
                ego_dynamics = self.localization.agent.dynamics
            # --- end localization ---

            # --- object detection ---
            with Stopwatch("obj_detection"):
                self.object_detection.run(
                    input_data=inputs,
                    ego_in_world=self.localization.agent.state
                )
                npcs = self.object_detection.agents
            # --- end object detection ---

            # --- driving ---
            with Stopwatch("driving"):
                vehicle_control = self.driver.run(
                    ego_pose, ego_dimensions, ego_dynamics,
                    npcs, timestamp
                )
            # --- end driving ---

        except Exception:
            vehicle_control = carla.VehicleControl(
                steer=0, throttle=0, brake=-1)
            traceback.print_exc()

        return vehicle_control

    def stay_still(self):
        return carla.VehicleControl(steer=0.0, throttle=0.0)

    def destroy(self):
        self.object_detection.destroy()
