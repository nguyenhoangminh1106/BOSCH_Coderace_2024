from collections import deque
from carla import Location
import numpy as np
import math


class PidLatController:
    """
    PidLatController implements lateral control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=0, K_I=0, dt=0.03):
        """
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=20)

    def pid_control(self, vehicle_transform, waypoint):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param vehicle_transform: current transform of the vehicle
        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1]
        """
        v_begin = vehicle_transform.location

        yaw_in_rads = np.radians(vehicle_transform.rotation.yaw)
        v_offset = Location(
            x=math.cos(yaw_in_rads),
            y=math.sin(yaw_in_rads)
        )

        v_end = v_begin + v_offset

        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array(
            [
                waypoint[0] - v_begin.x,
                waypoint[1] - v_begin.y,
                0.0,
            ]
        )
        _dot = math.acos(
            np.clip(
                np.dot(w_vec, v_vec) /
                (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)),
                -1.0,
                1.0,
            )
        )

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer)>=5:
            _de=(self._e_buffer[-5]-8*self._e_buffer[-4]+8*self._e_buffer[-2]-self._e_buffer[-1])/(12*self._dt)
            _ie=self._e_buffer[0]+self._e_buffer[-1]
            for i in range(1,len(self._e_buffer)-1):
                if i %2==1:
                    _ie+=4*self._e_buffer[i]
                else:
                    _ie+=2*self._e_buffer[i]
            _ie=_ie*self._dt/3
        elif len(self._e_buffer)>=3:
            _de=(self._e_buffer[-1]-self._e_buffer[-3])/(2*self._dt)
            _ie=self._e_buffer[0]+self._e_buffer[-1]
            for i in range(1,len(self._e_buffer)-1):
                if i %2==1:
                    _ie+=4*self._e_buffer[i]
                else:
                    _ie+=2*self._e_buffer[i]
            _ie=_ie*self._dt/3
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip(
            (self._K_P * _dot)
            + (self._K_D * _de / self._dt)
            + (self._K_I * _ie * self._dt),
            -1.0,
            1.0,
        )
