from math import cos, sin, sqrt, atan2, pi
from datetime import datetime
from odom import direct_kinematics, tick_odom
from pypot import dynamixel 
import cv2
import numpy as np

WHEEL_DISTANCE = 159.40 ## in mm
WHEEL_SIZE = 51.
DIST_TOLERANCE = 5. # in mm
ANGLE_TOLERANCE = pi/15. # in radians
SPEED_RATIO = 0.1
SPEED_START_ROTATION = 1 ## in mm/s
DEBUG = True

def inverse_kinematics(linear_speed, angular_speed) -> tuple[float, float]:
    v_droit  = linear_speed + (angular_speed * WHEEL_DISTANCE / 2)
    v_gauche = linear_speed - (angular_speed * WHEEL_DISTANCE / 2)
    return (v_droit, v_gauche)

def rotation_speed_to_linear_speed(rotation_speed) -> float: #in degres/s
    perimeter = 51*pi
    return perimeter*rotation_speed/360

def go_to_xya(x, y, theta):
    ports = dynamixel.get_available_ports()
    if not ports:
        exit('No port')

    dxl_io = dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1])

    (curr_x, curr_y, curr_theta) = (0., 0., 0.)
    (prev_x, prev_y, prev_theta) = (None, None, None)
    delta_time = 0.
    tolerance_time = 1_000_000. # in microseconds
    start = datetime.now()
    while(True):
        if DEBUG:
            print(f"Currently at {curr_x}, {curr_y}, {curr_theta}")
            print(f"Distances to target: {x - curr_x}, {y - curr_y}, {theta - curr_theta}")

        dx_to_target = (x - curr_x)
        dy_to_target = (y - curr_y)
        goal_linear_speed = SPEED_RATIO * sqrt(dx_to_target**2 + dy_to_target**2)
        goal_angular_speed = SPEED_RATIO * (atan2(dy_to_target, dx_to_target) - curr_theta)
        
        # If don't want to advance anymore, try to match target theta
        if (goal_linear_speed < SPEED_START_ROTATION):
            goal_angular_speed += SPEED_RATIO * (theta-curr_theta)
        
        if DEBUG:
            print(f"wanting to go at linear_speed = {goal_linear_speed} and angular_speed = {goal_angular_speed}")

        (goal_v_droit, goal_v_gauche) = inverse_kinematics(goal_linear_speed, goal_angular_speed)
        
        delta_time = start - datetime.now()
        
        dxl_io.set_moving_speed({1: goal_v_droit})
        dxl_io.set_moving_speed({2: goal_v_gauche})
         
        start = datetime.now()

        real_v_droit =  rotation_speed_to_linear_speed(dxl_io.get_moving_speed(1))
        real_v_gauche = rotation_speed_to_linear_speed(dxl_io.get_moving_speed(2))
        (real_linear_speed, real_angular_speed) = direct_kinematics(0., 0.) #TODO: Replace zeros by read from motors
        dxw = real_linear_speed*cos(curr_theta+real_angular_speed)
        dyw = real_linear_speed*sin(curr_theta+real_angular_speed)
        
        (prev_x, prev_y, prev_theta) = (curr_x, curr_y, curr_theta)
        (curr_x, curr_y, curr_theta) = tick_odom(prev_x, prev_y, prev_theta, real_linear_speed, real_angular_speed, delta_time)

        if (x - curr_x < DIST_TOLERANCE and y - curr_y < DIST_TOLERANCE and theta - curr_theta < ANGLE_TOLERANCE):
            tolerance_time -= delta_time.microseconds

        if (tolerance_time <= 0.):
            return
        
        




def pixel_to_robot(u, v, pts_image, pts_robot):
    H, status = cv2.findHomography(pts_image, pts_robot)
    pixel = np.array([ [u, v] ], dtype=np.float32)
    robot_point = cv2.perspectiveTransform(np.array([pixel]), H)
    x_robot, y_robot = robot_point[0][0]
    return x_robot, y_robot

x, y = pixel_to_robot(320, 240)
print(f"Pixel (320,240) correspond à position robot ({x:.2f}, {y:.2f})")


