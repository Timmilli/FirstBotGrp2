from math import cos, sin

wheel_distance = 159.40 # in mm

def direct_kinematics(v_gauche, v_droit) -> (float, float):
    linear_speed = (v_gauche + v_droit) / 2
    angular_speed = (v_gauche - v_droit) / wheel_distance
    return (linear_speed, angular_speed)

def odom(linear_speed, angular_speed, delta_time) -> (float, float, float):
    delta_theta = angular_speed * delta_time
    delta_x = (linear_speed * sin(delta_theta)) / delta_theta
    delta_y = (linear_speed *(1-cos(delta_theta)) ) / delta_theta
    return (delta_x, delta_y, delta_theta)

def tick_odom(prev_x, prev_y, prev_theta, linear_speed, angular_speed, delta_time) -> (float, float, float):
    (delta_x, delta_y, delta_theta) = odom(linear_speed, angular_speed, delta_time)
    return (prev_x + delta_x, prev_y + delta_y, prev_theta + delta_theta)
