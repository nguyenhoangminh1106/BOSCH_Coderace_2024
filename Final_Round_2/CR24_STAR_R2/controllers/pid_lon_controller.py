from collections import deque
import numpy as np


class PidLonController:
    """
    PidLonController implements longitudinal control using a PID.
    """

    def __init__(self, K_P=1.0, K_D=1, K_I=1, dt=0.03):
        """
        :param vehicle: actor to apply to local planner logic onto
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

    def pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        _e = target_speed - current_speed
        self._e_buffer.append(_e)
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
            (self._K_P * _e)
            + (self._K_D * _de / self._dt)
            + (self._K_I * _ie * self._dt),
            0.0,
            1.0,
        )
