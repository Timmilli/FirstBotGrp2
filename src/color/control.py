from math import cos, sin, sqrt, atan2, pi
from datetime import datetime
from odom import direct_kinematics, tick_odom

wheel_distance = 159.40 ## in mm

def inverse_kinematics(linear_speed, angular_speed) -> tuple[float, float]:
    v_droit  = linear_speed + (angular_speed * wheel_distance / 2)
    v_gauche = linear_speed - (angular_speed * wheel_distance / 2)
    return (v_droit, v_gauche)

DIST_TOLERANCE = 5. # in mm
ANGLE_TOLERANCE = pi/15. # in radians
SPEED_RATIO = 0.1
SPEED_START_ROTATION = 1 ## in mm/s
def go_to_xya(x, y, theta):
    (curr_x, curr_y, curr_theta) = (0., 0., 0.)
    (prev_x, prev_y, prev_theta) = (None, None, None)
    delta_time = 0.
    tolerance_time = 1_000_000. # in microseconds
    start = datetime.now()
    while(True):

        dx_to_target = (x - curr_x)
        dy_to_target = (y - curr_y)
        goal_linear_speed = SPEED_RATIO * sqrt(dx_to_target**2 + dy_to_target**2)
        goal_angular_speed = SPEED_RATIO * (atan2(dy_to_target, dx_to_target) - curr_theta)
        
        # If don't want to advance anymore, try to match target theta
        if (goal_linear_speed < SPEED_START_ROTATION):
            goal_angular_speed += SPEED_RATIO * (theta-curr_theta)
        
        (goal_v_droit, goal_v_gauche) = inverse_kinematics(goal_linear_speed, goal_angular_speed)
        delta_time = start - datetime.now()
        start = datetime.now()
        #TODO: Envoyer les commandes au roues 

        (real_linear_speed, real_angular_speed) = direct_kinematics(0., 0.) #TODO: Replace zeros by read from motors
        dxw = real_linear_speed*cos(curr_theta+real_angular_speed)
        dyw = real_linear_speed*sin(curr_theta+real_angular_speed)
        
        (prev_x, prev_y, prev_theta) = (curr_x, curr_y, curr_theta)
        (curr_x, curr_y, curr_theta) = tick_odom(prev_x, prev_y, prev_theta, real_linear_speed, real_angular_speed, delta_time)

        if (x - curr_x < DIST_TOLERANCE and y - curr_y < DIST_TOLERANCE and theta - curr_theta < ANGLE_TOLERANCE):
            tolerance_time -= delta_time.microseconds
        
        if (tolerance_time <= 0.):
            return