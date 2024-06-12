import math
import numpy as np

def calculate_stopping_time(velocity, deceleration):
    """
    Calculate the stopping time given velocity and deceleration.
    
    Args:
        velocity (float): The current speed of the vehicle (m/s).
        deceleration (float): The deceleration rate (m/s^2).
        
    Returns:
        float: The stopping time (seconds).
    """
    if deceleration == 0:
        return float('inf')
    return velocity / deceleration

def calculate_ttc(relative_velocity, relative_distance):
    """
    Calculate the Time to Collision (TTC).
    
    Args:
        relative_velocity (float): The relative speed between the ego vehicle and the leading vehicle (m/s).
        relative_distance (float): The distance between the ego vehicle and the leading vehicle (m).
        
    Returns:
        float: The time to collision (seconds).
    """
    if relative_velocity == 0:
        return float('inf')  # To handle division by zero
    return relative_distance / relative_velocity

def state_transition_logic(ttc, t_fcw, t_pb1, t_pb2, t_fb):
    """
    Determine the state transition based on TTC and stopping times.
    
    Args:
        ttc (float): Time to Collision (seconds).
        t_fcw (float): Forward Collision Warning time (seconds).
        t_pb1 (float): Partial Braking 1 time (seconds).
        t_pb2 (float): Partial Braking 2 time (seconds).
        t_fb (float): Full Braking time (seconds).
        
    Returns:
        int: The state of the AEB system.
    """
    if ttc < 0 and t_fcw > abs(ttc):
        return 1  # FCW
    elif ttc < 0 and t_pb1 > abs(ttc):
        return 2  # PB1
    elif ttc < 0 and t_pb2 > abs(ttc):
        return 3  # PB2
    elif ttc < 0 and t_fb > abs(ttc):
        return 4  # FB
    else:
        return 0  # Default state

def aeb_activation_logic(state, velocity):
    """
    Determine the braking force based on the current state of the AEB system.
    
    Args:
        state (int): The current state of the AEB system.
        velocity (float): The current speed of the vehicle (m/s).
        
    Returns:
        float: The required deceleration (m/s^2).
    """
    if state == 0:
        return 0  # Default state, maintain speed
    elif state == 1:
        return 0  # FCW, alert driver (handled separately)
    elif state == 2:
        return 3.8  # PB1
    elif state == 3:
        return 5.8  # PB2
    elif state == 4:
        return 9.8  # FB
    else:
        return 0  # Default