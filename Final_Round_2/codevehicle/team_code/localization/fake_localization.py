from common.dynamics.vehicle_dimensions import VehicleDimensions
from common.dynamics.vehicle_dynamics import VehicleDynamics
from common.dynamics.agent import Agent


def get_localization():
    return 'FakeLocalization'


class FakeLocalization:
    def __init__(self, config):
        self.config = config
        self.agent = None

    def cheat(self, world, ego_actor):
        self.agent = Agent(
            state=ego_actor.get_transform(),
            dynamics=VehicleDynamics(
                velocity=ego_actor.get_velocity(),
                acceleration=ego_actor.get_acceleration(),
                angular_velocity=ego_actor.get_angular_velocity()
            ),
            dimensions=VehicleDimensions(ego_actor.bounding_box.extent)
        )

    def run(self, input_data=None):
        """Cheat by peeking at the world. This is to bypass not having
        a localization module right now.
        """
        self.agent.dynamics.speedometer = input_data["SPEED"][1]["speed"]
