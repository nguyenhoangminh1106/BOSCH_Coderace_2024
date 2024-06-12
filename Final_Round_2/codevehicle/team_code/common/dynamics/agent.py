from common.dynamics.vehicle_state import VehicleState
from common.dynamics.vehicle_dimensions import VehicleDimensions
from common.dynamics.vehicle_dynamics import VehicleDynamics


class ObjectType:
    UNKNOWN = 0
    VEHICLE = 1
    PEDESTRIAN = 2


class MovingObject:
    def __init__(
        self,
        state=VehicleState(),
        dynamics=VehicleDynamics()
    ):
        self._state = state
        self._dynamics = dynamics

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, var):
        self._state = var

    @property
    def dynamics(self):
        return self._dynamics

    @dynamics.setter
    def dynamics(self, var):
        self._dynamics = var


class Agent(MovingObject):
    def __init__(
        self,
        object_id=-1,
        object_type=ObjectType.UNKNOWN,
        state=VehicleState(),
        dynamics=VehicleDynamics(),
        dimensions=VehicleDimensions()
    ):
        super().__init__(state=state, dynamics=dynamics)
        self._dimensions = dimensions
        self._id = object_id
        self._type = object_type

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, var):
        self._id = var

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, var):
        self._type = var

    @property
    def dimensions(self):
        return self._dimensions

    @dimensions.setter
    def dimensions(self, var):
        self._dimensions = var

    def __repr__(self):
        return f"id: {self._id}\n" \
               f"type: {self._type}\n" \
               f"dimensions: {self._dimensions}\n" \
               f"state:\n{self._state}\n" \
               f"dynamics:\n{self._dynamics}\n"
