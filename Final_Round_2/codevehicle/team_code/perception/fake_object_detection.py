from common.dynamics.vehicle_dimensions import VehicleDimensions
from common.dynamics.vehicle_dynamics import VehicleDynamics
from common.dynamics.agent import Agent, ObjectType

DETECTION_RADIUS = 60


def get_object_detection():
    return "FakeObjectDetection"


class FakeObjectDetection:

    def __init__(self, config) -> None:
        self.config = config
        self.agents = []

    def cheat(
            self,
            world,
            ego_vehicle
    ):
        # re-init the containers
        self.agents = []

        # get ego vehicle's location
        ego_loc = ego_vehicle.get_location()

        # get all actors in world within radius
        for actor in world.get_actors():
            if actor.attributes.get('role_name') == 'hero':
                continue

            if "vehicle" in actor.type_id:
                object_type = ObjectType.VEHICLE
            elif "pedestrian" in actor.type_id:
                object_type = ObjectType.PEDESTRIAN
            else:
                # object_type = ObjectType.UNKNOWN
                continue

            # vehicle state
            veh_state = actor.get_transform()

            squared_distance = (
                (veh_state.location.x - ego_loc.x)**2 +
                (veh_state.location.y - ego_loc.y)**2 +
                (veh_state.location.z - ego_loc.z)**2
            )
            if squared_distance <= DETECTION_RADIUS**2:
                # wrap-up into Agent object
                agent = Agent(
                    object_id=actor.id,
                    object_type = object_type,
                    state=veh_state,
                    dynamics=VehicleDynamics(
                        velocity=actor.get_velocity(),
                        acceleration=actor.get_acceleration(),
                        angular_velocity=actor.get_angular_velocity()
                    ),
                    dimensions=VehicleDimensions(actor.bounding_box.extent)
                )
                self.agents.append(agent)

    def run(self, input_data=None, ego_in_world=None):
        pass

    def destroy(self):
        pass
