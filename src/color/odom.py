from math import cos, sin
from datetime import datetime

WHEEL_DISTANCE = 159.40 # in mm

def direct_kinematics(v_droit, v_gauche) -> tuple[float, float]:
    linear_speed = (v_droit + v_gauche) / 2  # in mm/s
    angular_speed = (v_droit - v_gauche) / (WHEEL_DISTANCE)  # in rad/s
    return (linear_speed, angular_speed)

def odom(linear_speed, angular_speed, delta_time) -> tuple[float, float, float]:
    delta_theta = angular_speed * delta_time
    if (delta_theta == 0):
        return (linear_speed * delta_time, 0., 0.)
    delta_x = (linear_speed * delta_time * sin(delta_theta)) / delta_theta
    delta_y = (linear_speed * delta_time * (1-cos(delta_theta))) / delta_theta
    return (delta_x, delta_y, delta_theta)

def tick_odom(prev_x, prev_y, prev_theta, linear_speed, angular_speed, delta_time) -> tuple[float, float, float]:
    (delta_x_local, delta_y_local, delta_theta) = odom(linear_speed, angular_speed, delta_time)
    delta_x_world = delta_x_local * cos(prev_theta) - delta_y_local * sin(prev_theta)
    delta_y_world = delta_x_local * sin(prev_theta) + delta_y_local * cos(prev_theta)
    return (prev_x + delta_x_world, prev_y + delta_y_world, prev_theta + delta_theta)

def rotation_speed_to_linear_speed(rotation_speed) -> float:
    return (rotation_speed * 3.14159 * 51 / 360)  # in mm/s

def odom_mapping(prev_x, prev_y, prev_theta, dxl_io, delta_time) -> tuple[float, float, float]:
    real_v_droit = -rotation_speed_to_linear_speed(dxl_io.get_present_speed([1])[0])
    real_v_gauche = rotation_speed_to_linear_speed(dxl_io.get_present_speed([2])[0])
    # print(f"real_v_droit = {real_v_droit}, real_v_gauche = {real_v_gauche}")
    (real_linear_speed, real_angular_speed) = direct_kinematics(real_v_droit, real_v_gauche)
    # print(f"real_linear_speed = {real_linear_speed:.2f}, real_angular_speed = {real_angular_speed:.3f}")
    return tick_odom(prev_x, prev_y, prev_theta, real_linear_speed, real_angular_speed, delta_time)