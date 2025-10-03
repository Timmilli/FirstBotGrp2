from math import cos, sin, sqrt, atan2, pi
from datetime import datetime
from odom import direct_kinematics, tick_odom, odom_mapping, WHEEL_DISTANCE
from pypot import dynamixel
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

WHEEL_SIZE = 51.
DIST_TOLERANCE = 5.  # in mm
ANGLE_TOLERANCE = pi/15.  # in radians
SPEED_RATIO = 5
SPEED_START_ROTATION = 50  # in mm/s
DEBUG = False

# Ptn pixel de l'image
pts_image = np.array([
    [0, 479],
    [639, 479],
    [0, 0],
    [639, 0]

], dtype=np.float32)

# Points robot correspondants en cm
pts_robot = np.array([
    [-49, 82],
    [41, 85],
    [-62, 162],
    [57, 173]
], dtype=np.float32)


# Calculates the inverse kinematic of the bot
# Takes the linear and angular speeds of the bot
# and returns the speed of both motors
def inverse_kinematics(linear_speed, angular_speed) -> tuple[float, float]:
    v_droit = linear_speed + (angular_speed * WHEEL_DISTANCE / 2)
    v_gauche = linear_speed - (angular_speed * WHEEL_DISTANCE / 2)
    return (v_droit, v_gauche)


# Calculates the linear speed from a rotational speed
def rotation_speed_to_linear_speed(rotation_speed) -> float:  # in degres/s
    perimeter = 51*pi
    return perimeter*rotation_speed/360


# Goes to to the world point (x,y) with the orientation theta
# x, y are in millimeters
# theta in radian
def go_to_xya(x, y, theta):
    y = -y
    ports = dynamixel.get_available_ports()
    if not ports:
        exit('No port')

    dxl_io = dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1])

    (curr_x, curr_y, curr_theta) = (0., 0., 0.)
    (prev_x, prev_y, prev_theta) = (None, None, None)
    delta_time = 0.
    tolerance_time = 1_000_000.  # in microseconds
    start = datetime.now()

    rear_mode = False

    while (True):
        if DEBUG:
            print(f"Currently at {curr_x}, {curr_y}, {curr_theta}")
            print(
                f"Distances to target: {x - curr_x}, {y - curr_y}, {theta - curr_theta}")

        dx_to_target = (x - curr_x)
        dy_to_target = (y - curr_y)

        d_to_target = sqrt(dx_to_target**2 + dy_to_target**2)
        angle_to_target = atan2(dy_to_target, dx_to_target) - curr_theta

        goal_linear_speed = SPEED_RATIO * d_to_target
        # goal_linear_speed = 400
        goal_angular_speed = SPEED_RATIO * angle_to_target

        dx_to_target_robot = d_to_target * cos(angle_to_target)

        if ((dx_to_target_robot < -100 and not rear_mode) or (dx_to_target_robot > 100 and rear_mode)):
            rear_mode = not rear_mode
            goal_linear_speed /= 10
        if rear_mode:
            goal_linear_speed *= -1

        (goal_v_droit, goal_v_gauche) = inverse_kinematics(
            goal_linear_speed, goal_angular_speed)
        while (abs(goal_v_droit) > 600 or abs(goal_v_gauche) > 600):
            goal_v_droit *= 0.9
            goal_v_gauche *= 0.9

        if DEBUG:
            print(
                f"wanting to go at linear_speed = {goal_linear_speed} and angular_speed = {goal_angular_speed}")
            print(
                f"making it goal_v_droit = {goal_v_droit} and goal_v_gauche = {goal_v_gauche}")

        delta_time = datetime.now() - start

        dxl_io.set_moving_speed({1: -goal_v_droit})
        dxl_io.set_moving_speed({2: goal_v_gauche})

        start = datetime.now()
        time.sleep(0.1)

        real_v_droit = - \
            rotation_speed_to_linear_speed(dxl_io.get_moving_speed([1])[0])
        real_v_gauche = rotation_speed_to_linear_speed(
            dxl_io.get_moving_speed([2])[0])
        (real_linear_speed, real_angular_speed) = direct_kinematics(
            real_v_droit, real_v_gauche)
        norm = real_linear_speed * delta_time.microseconds/1_000_000
        angle = real_angular_speed * delta_time.microseconds/1_000_000
        dxw = norm*cos(curr_theta+angle)
        dyw = norm*sin(curr_theta+angle)
        print(f"real_v_droit = {real_v_droit}, real_v_gauche = {real_v_gauche}, real_linear_speed = {real_linear_speed}, real_angular_speed = {real_angular_speed}, norm = {norm}, angle = {angle}, dxw = {dxw}, dyw = {dyw}")
        print(f"deltatime = {delta_time.microseconds/1_000_000}\n")
        (prev_x, prev_y, prev_theta) = (curr_x, curr_y, curr_theta)
        (curr_x, curr_y, curr_theta) = (
            prev_x+dxw, prev_y+dyw, (prev_theta+angle))

        if ((abs(x - curr_x) < 10 and abs(y - curr_y) < 10) or tolerance_time <= 0.):
            break

        if (abs(x - curr_x) < DIST_TOLERANCE and abs(y - curr_y) < DIST_TOLERANCE):
            tolerance_time -= delta_time.microseconds

    # While the bot doesn't have a correct orientation
    while abs(theta - curr_theta) > pi/16:
        print(f"Currently at {curr_x}, {curr_y}, {curr_theta}")
        print(
            f"Distances to target: {x - curr_x}, {y - curr_y}, {theta - curr_theta}")
        delta = theta - curr_theta
        speed = -   100*delta
        while abs(speed) < 50:
            speed *= 2

        delta_time = datetime.now() - start

        dxl_io.set_moving_speed({1: speed})
        dxl_io.set_moving_speed({2: speed})

        start = datetime.now()
        time.sleep(0.1)

        real_v_droit = - \
            rotation_speed_to_linear_speed(dxl_io.get_moving_speed([1])[0])
        real_v_gauche = rotation_speed_to_linear_speed(
            dxl_io.get_moving_speed([2])[0])
        (real_linear_speed, real_angular_speed) = direct_kinematics(
            real_v_droit, real_v_gauche)
        norm = real_linear_speed * delta_time.microseconds/1_000_000
        angle = real_angular_speed * delta_time.microseconds/1_000_000
        dxw = norm*cos(curr_theta+angle)
        dyw = norm*sin(curr_theta+angle)
        # dyw = norm*sin(curr_theta+angle)/10 #TODO: Suppr the 10x (normalement)
        print(f"real_v_droit = {real_v_droit}, real_v_gauche = {real_v_gauche}, real_linear_speed = {real_linear_speed}, real_angular_speed = {real_angular_speed}, norm = {norm}, angle = {angle}, dxw = {dxw}, dyw = {dyw}")
        print(f"deltatime = {delta_time.microseconds/1_000_000}\n")
        (prev_x, prev_y, prev_theta) = (curr_x, curr_y, curr_theta)
        (curr_x, curr_y, curr_theta) = (prev_x+dxw, prev_y+dyw, prev_theta+angle)
        if curr_theta > pi:
            curr_theta -= 2*pi
        elif curr_theta < -pi:
            curr_theta += 2*pi

    dxl_io.set_moving_speed({1: 0})
    dxl_io.set_moving_speed({2: 0})


# Goes to to the world point (x,y) with the orientation theta,
# but using another strategy
# x, y are in millimeters
# theta in radian
def go_to_xya_v2(x, y, theta):
    ports = dynamixel.get_available_ports()
    if not ports:
        exit('No port')

    dxl_io = dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1])

    (curr_x, curr_y, curr_theta) = (0., 0., 0.)

    start = datetime.now()
    delta_time = 0.

    # Returns the squared distance to the destination
    def distance_to_dest_sqrd():
        return (x-curr_x)**2 + (y-curr_y)**2

    # Returns the distance of the target from the bot
    def dest_from_robot():
        dx_world = (x-curr_x)
        dy_world = (y-curr_y)
        dx_robot = dx_world*cos(curr_theta)+dy_world*sin(curr_theta)
        dy_robot = (-dx_robot)*sin(curr_theta)+dy_world*cos(curr_theta)
        return (dx_robot, dy_robot)

    # Returns the angle between the target and the bot orientation
    def angle_to_dest():
        (x, y) = dest_from_robot()
        return atan2(y, x)

    # While the robot is neither facing or backing the target
    print(f"Angle to dest = {angle_to_dest()}")
    while angle_to_dest() > pi/8 or angle_to_dest() < -pi/8:
        print(f"Angle to dest = {angle_to_dest()}")
        speed = 100*angle_to_dest()
        while abs(speed) < 50:
            speed *= 1.1
        while abs(speed) > 600:
            speed *= 0.9

        delta_time = (datetime.now() - start).microseconds/1_000_000

        dxl_io.set_moving_speed({1: speed})
        dxl_io.set_moving_speed({2: speed})

        start = datetime.now()
        time.sleep(0.1)

        (curr_x, curr_y, curr_theta) = odom_mapping(
            curr_x, curr_y, curr_theta, dxl_io, delta_time)

    # While the robot is too far from the target
    print(f"sqrt(distance_to_dest_sqrd()) = {sqrt(distance_to_dest_sqrd())}")
    while sqrt(distance_to_dest_sqrd()) > DIST_TOLERANCE:
        goal_angular_speed = angle_to_dest() if abs(angle_to_dest()) > 0.5 else 0
        goal_linear_speed = 600 if goal_angular_speed == 0 else abs(
            sqrt(distance_to_dest_sqrd()) * goal_angular_speed)
        (goal_v_droit, goal_v_gauche) = inverse_kinematics(
            goal_linear_speed, goal_angular_speed)
        while (abs(goal_v_droit) < 100 or abs(goal_v_gauche) < 100):
            goal_v_droit *= 10
            goal_v_gauche *= 10
        while abs(goal_v_droit) > 600 or abs(goal_v_gauche) > 600:
            goal_v_droit *= 0.9
            goal_v_gauche *= 0.9
        if DEBUG:
            print(f"Want to go to ({x}, {y}, {theta})")
            print(f"Currently at  ({curr_x}, {curr_y}, {curr_theta})")
            print(f"distance_to_dest_sqrd = {sqrt(distance_to_dest_sqrd())}")
            print(
                f"goal_linear_speed = {goal_linear_speed} | goal_angular_speed = {goal_angular_speed}")
            print(
                f"goal_v_droit = {goal_v_droit} | goal_v_gauche = {goal_v_gauche}")
            print()

        dxl_io.set_moving_speed({1: -goal_v_droit})
        dxl_io.set_moving_speed({2: goal_v_gauche})

        delta_time = (datetime.now() - start).microseconds/1_000_000
        time.sleep(0.1)
        start = datetime.now()

        (curr_x, curr_y, curr_theta) = odom_mapping(
            curr_x, curr_y, curr_theta, dxl_io, delta_time)

    # While the robot doesn't have a correct orientation
    while abs(theta - curr_theta) > pi/16:
        delta = theta - curr_theta
        speed = - 100*delta
        while abs(speed) < 50:
            speed *= 1.1
        while abs(speed) > 600:
            speed *= 0.9

        delta_time = (datetime.now() - start).microseconds/1_000_000

        dxl_io.set_moving_speed({1: speed})
        dxl_io.set_moving_speed({2: speed})

        start = datetime.now()
        time.sleep(0.1)

        (curr_x, curr_y, curr_theta) = odom_mapping(
            curr_x, curr_y, curr_theta, dxl_io, delta_time)

    dxl_io.set_moving_speed({1: 0})
    dxl_io.set_moving_speed({2: 0})


# Calculates the go_to function for a single frame
# Returns the target speed of both motors
def go_to_one_frame(x, y, dxlio):
    y = -y
    goal_linear_speed = SPEED_RATIO * sqrt(x**2 + y**2)
    goal_angular_speed = SPEED_RATIO * atan2(y, x)
    (goal_v_droit, goal_v_gauche) = inverse_kinematics(
        goal_linear_speed, goal_angular_speed)
    while (abs(goal_v_droit) > 150 or abs(goal_v_gauche) > 150):
        goal_v_droit *= 0.9
        goal_v_gauche *= 0.9
    return (goal_v_droit, goal_v_gauche)


# Ptn pixel de l'image
pts_image = np.array([
    [0, 0],
    [639, 0],
    [0, 479],
    [639, 479]
], dtype=np.float32)


# Calculates the position of a pixel on the frame but in the bot view
def pixel_to_robot(u, v):
    H, status = cv2.findHomography(pts_image, pts_robot)
    pixel = np.array([[u, v]], dtype=np.float32)
    robot_point = cv2.perspectiveTransform(np.array([pixel]), H)
    x_robot, y_robot = robot_point[0][0]
    return x_robot, y_robot


# Calculates the position of a pixel on the frame but in the world view
def pixel_to_world(u, v, curr_x, curr_y, curr_theta):
    x_robot, y_robot = pixel_to_robot(u, v)
    x_world = curr_x + x_robot * cos(curr_theta) - y_robot * sin(curr_theta)
    y_world = curr_y + x_robot * sin(curr_theta) + y_robot * cos(curr_theta)
    return x_world, y_world


detected_world_points = []


# Plots the trajectory traveled by the bot
# trajectory is either shape(N,2) or shape(3,N,2)
# The shape choice is determined by the color Boolean
def plot_trajectory(trajectory, color=False):
    if not trajectory:
        print("There is no trajectory to plot.")
        return

    match color:
        case True:
            colors = ['yellow', 'blue', 'red']
            plt.figure(figsize=(8, 8))
            for i in range(len(trajectory)):
                if len(trajectory[i]) == 0:
                    trajectory[i] = [(0, 0)]
                xs, ys = zip(*trajectory[i])
                plt.plot(xs, ys, '-o', color=colors[i], label="Trajectoire")
                plt.plot(xs[-1], ys[-1], 'black', label="Position finale")
            plt.title("Trajectoire du robot")
            plt.xlabel("X (cm)")
            plt.ylabel("Y (cm)")
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.savefig("bot_trajectory.png")
            print("Trajectory saved in 'bot_trajectory.png'")
            plt.show()

        case False:
            xs, ys = zip(*trajectory)
            plt.figure(figsize=(8, 8))
            plt.plot(xs, ys, '-o', color='blue', label="Trajectoire")
            plt.plot(xs[-1], ys[-1], 'ro', label="Position finale")
            plt.title("Trajectoire du robot")
            plt.xlabel("X (cm)")
            plt.ylabel("Y (cm)")
            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.savefig("bot_trajectory.png")
            print("Trajectory saved in 'bot_trajectory.png'")
            plt.show()
