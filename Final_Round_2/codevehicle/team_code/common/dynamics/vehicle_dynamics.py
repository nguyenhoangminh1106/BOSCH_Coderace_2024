import numpy as np
from carla import Vector3D


class VehicleDynamics:
    def __init__(
        self,
        velocity=Vector3D(),
        angular_velocity=Vector3D,
        acceleration=Vector3D,
        speedometer=float("inf"),
    ):
        self._velocity = velocity
        self._angular_velocity = angular_velocity
        self._acceleration = acceleration
        self._speedometer = speedometer

    @property
    def speedometer(self):
        return self._speedometer

    @speedometer.setter
    def speedometer(self, var):
        self._speedometer = var

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, var):
        self._velocity = var

    @property
    def acceleration(self):
        return self._acceleration

    @acceleration.setter
    def acceleration(self, var):
        self._acceleration = var

    @property
    def angular_velocity(self):
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, var):
        self._angular_velocity = var

    def get_speed(self):
        return np.linalg.norm([self.velocity.x, self.velocity.y])

    def __repr__(self):
        return f"velocity: {self._velocity}\n" \
               f"angular_vel: {self._angular_velocity}\n" \
               f"acceleration: {self._acceleration}"
