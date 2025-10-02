from math import cos, sin
from datetime import datetime

WHEEL_DISTANCE = 159.40 # in mm

def direct_kinematics(v_droit, v_gauche) -> tuple[float, float]:
    linear_speed = (v_droit + v_gauche) / 2
    angular_speed = (v_droit - v_gauche) / WHEEL_DISTANCE
    return (linear_speed, angular_speed)

def odom(linear_speed, angular_speed, delta_time) -> tuple[float, float, float]:
    delta_theta = angular_speed * delta_time.microseconds / 1_000_000.
    if (delta_theta == 0):
        return (linear_speed * delta_time.microseconds / 1_000_000., 0., 0.)
    delta_x = (linear_speed * sin(delta_theta)) / delta_theta
    delta_y = (linear_speed *(1-cos(delta_theta)) ) / delta_theta
    return (delta_x, delta_y, delta_theta)

def tick_odom(prev_x, prev_y, prev_theta, linear_speed, angular_speed, delta_time) -> tuple[float, float, float]:
    (delta_x_local, delta_y_local, delta_theta) = odom(linear_speed, angular_speed, delta_time)
    delta_x_world = delta_x_local * cos(prev_theta) - delta_y_local * sin(prev_theta)
    delta_y_world = delta_x_local * sin(prev_theta) + delta_y_local * cos(prev_theta)
    return (prev_x + delta_x_world, prev_y + delta_y_world, prev_theta + delta_theta)                                                                                                                                                                                                                                                                                                                           