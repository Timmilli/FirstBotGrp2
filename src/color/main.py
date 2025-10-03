import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys
import time

from image_processing import next_color, process_frame_hsv, process_frame_rgb
from control import go_to_xya, pixel_to_robot, go_to_one_frame, rotation_speed_to_linear_speed, current_position, plot_trajectory
from odom import odom_mapping

parser = argparse.ArgumentParser(
    prog='Main file',
    description='To launch the robot movements')
parser.add_argument('-c', '--computer_used', action='store_true',
                    help='Defines if the program is launched on a computer.')
parser.add_argument('-m', '--motor_used', action='store_true',
                    help='Defines if the motors needs to be used.')
parser.add_argument('-b', '--brown_detection', action='store_true',
                    help='Defines if the motors needs to be used.')
parser.add_argument('-r', '--rgb_used', action='store_true',
                    help='Defines if the RGB is used over the HSV.')
parser.add_argument('-p', '--pid_used', action='store_true',
                    help='Use a simple PID as the motor control.')
parser.add_argument('-o', '--odom_used', action='store_true',
                    help='Using the odometry to map a path.')

parser.add_argument('-col', '--color',
                    default=0, type=int,
                    help='Defines the color of the line to follow.')
parser.add_argument('-s', '--speed',
                    default=360, type=int,
                    help='Defines the standard speed of the wheels. Default is 360.')
parser.add_argument('-g', '--goto',
                    default=(0, 0, 0), type=int, nargs=3,
                    help='Goes to a specific position given by x (mm), y (mm), theta (rad).')

args = parser.parse_args()

if(args.odom_used):
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        sys.exit("Motors are used but are not detected. Exiting...")
    else:
        dxl_io = pypot.dynamixel.DxlIO(ports[0])
        dxl_io.set_wheel_mode([1, 2])
        dxl_io.disable_torque([1, 2])
    current_time = time.time()  # in milliseconds
    curr_x, curr_y, curr_theta = 0., 0., 0.
    while True:
        delta_time = time.time() - current_time
        current_time = time.time()
        curr_x, curr_y, curr_theta = odom_mapping(curr_x, curr_y, curr_theta, dxl_io, delta_time)
        print(f"Robot position: x={curr_x:.2f} mm, y={curr_y:.2f} mm, theta={curr_theta:.2f} rad")
        # print(delta_time, curr_x, curr_y, curr_theta)

if(args.goto != (0, 0, 0)):
    go_to_xya(args.goto[0], args.goto[1], args.goto[2])
    print("Arrived at destination. Exiting...")
    sys.exit()


MOTOR_USED = args.motor_used
COMPUTER_USED = args.computer_used
BROWN_USED = args.brown_detection
RGB_USED = args.rgb_used
PID_USED = args.pid_used

print(
    f"Motor used:{MOTOR_USED}; Computer used:{COMPUTER_USED}; Brown used:{BROWN_USED}; Hsv used:{RGB_USED}")

# Coded in HSV
# Maroon has to be the last color
hsv_boundaries = [
    ([10, 80, 100], [40, 200, 255]),  # A yellow tape (to be reworked)
    ([90, 120, 200], [110, 255, 255]),  # A blue tape
    ([110, 100, 200], [180, 200, 255]),  # A red tape
    ([0, 0, 60], [180, 90, 140])  # A maroon tape (to be reworked)

]
# Coded in BGR
# Maroon has to be the last color
rgb_boundaries = [
    ([100, 170, 150], [150, 220, 200]),  # A yellow tape (to be reworked on)
    ([190, 150, 0], [255, 200, 50]),  # A blue tape
    ([140, 70, 130], [100, 100, 255]),  # A red tape (to be reworked on)
    ([110, 90, 70], [130, 110, 90]),  # A brown tape (to be reworked on)
]

color_string = [ "Yellow Tape", "Blue Tape", "Red Tape", "Maroon Tape"]
current_color = int(args.color)

if RGB_USED:
    boundaries = rgb_boundaries
else:
    boundaries = hsv_boundaries

lower, upper = boundaries[current_color]
# print(color_string[current_color])
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")
brown_lower, brown_upper = boundaries[-1]
brown_lower = np.array(brown_lower, dtype="uint8")
brown_upper = np.array(brown_upper, dtype="uint8")
switch_ready = True


top_band = 400
bot_band = 410


if MOTOR_USED:
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        sys.exit("Motors are used but are not detected. Exiting...")
    else:
        dxl_io = pypot.dynamixel.DxlIO(ports[0])
        dxl_io.set_wheel_mode([1])
        STANDARD_SPEED = args.speed  # values best between 300 and 700
        print(
            f"Motors detected and used. Setting standard speed at {STANDARD_SPEED}.")


video_capture = cv2.VideoCapture(0)

width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video_capture.get(cv2.CAP_PROP_FPS))
print(f"width:{width}; height:{height}; fps:{fps}")

pid = PID(2, 0.4, 0.1, setpoint=0)


def exit_program():
    video_capture.release()
    cv2.destroyAllWindows()
    if MOTOR_USED:
        dxl_io.set_moving_speed(
            {1: 0})  # Degrees / s
        dxl_io.set_moving_speed(
            {2: 0})  # Degrees / s
    sys.exit()


try:
    dico = {
        "width": width,
        "height": height,
        "fps": fps,
        "top_band": top_band,
        "bot_band": bot_band,
        "MOTOR_USED": MOTOR_USED,
        "COMPUTER_USED": COMPUTER_USED,
        "BROWN_USED": BROWN_USED,
        "RGB_USED": RGB_USED,
        "boundaries": boundaries,
        "lower": lower,
        "upper": upper,
        "brown_lower": brown_lower,
        "brown_upper": brown_upper,
        "current_color": current_color,
        "color_string": color_string,
        "switch_ready": switch_ready
    }
    speed = 0
    color_search = False
    current_time = time.time()  # in milliseconds
    curr_x, curr_y, curr_theta = 0., 0., 0.
    trajectory = []
    while True:
        result, frame = video_capture.read()  # read frames from the video
        if result is False:
            print("Capture has failed. Exiting...")
            exit_program()

        print(color_string[current_color])
        if RGB_USED:
            absisse, color_detected = process_frame_rgb(frame, dico)
        else:
            bypass = False
            absisse, color_detected, other_color_detected, bypass = process_frame_hsv(frame, dico)
            if bypass and MOTOR_USED:
                # print(bypass)
                dxl_io.set_moving_speed({1: -300})  # Degrees / s
                dxl_io.set_moving_speed({2: 300})  # Degrees / s
                time.sleep(0.05)


        v_mot_droit, v_mot_gauche = 0, 0
        if PID_USED:
            if color_detected:
                speed = pid((absisse-width/2)/(width/2))
                # color_search = False
            else:
                speed = 0
                # if not other_color_detected:dv
                #     color_search = True
                # if not color_search:
            if curr_x < 0 and curr_x > -0.03 and curr_y < 0.04 and curr_y > -0.04 and color_search == True:
                color_search = False
                next_color(dico)
            if curr_x > 0.2:
                color_search = True

            v_mot_droit = STANDARD_SPEED - ((4/5)*STANDARD_SPEED*abs(speed)) + speed*STANDARD_SPEED
            v_mot_gauche = STANDARD_SPEED - ((4/5)*STANDARD_SPEED*abs(speed)) - speed*STANDARD_SPEED
            # print(round(speed, 2))
        else:
            if not color_detected :
                absisse = width/2
            x_robot, y_robot = pixel_to_robot(absisse, 480 - (top_band+bot_band)/2)
            v_mot_droit, v_mot_gauche = go_to_one_frame(y_robot, 40*(x_robot + 5), dxl_io)
            print(round(absisse, 2), (top_band+bot_band)/2, round(x_robot, 2), round(y_robot, 2), v_mot_droit, v_mot_gauche)

        # print(round(absisse, 2))

        if MOTOR_USED:
            dxl_io.set_moving_speed({1: -v_mot_droit})  # Degrees / s
            dxl_io.set_moving_speed({2: v_mot_gauche})  # Degrees / s

        if COMPUTER_USED:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit_program()

        if COMPUTER_USED:
            x_robot, y_robot = pixel_to_robot(320, 240)
            print(f"Pixel (320,240) → Robot ({x_robot:.2f}, {y_robot:.2f}) cm")
        delta_time = time.time() - current_time
        current_time = time.time()
        # Lecture de la position courante
        curr_x, curr_y, curr_theta= odom_mapping(curr_x, curr_y, curr_theta, dxl_io, delta_time)

        print(f"[Position] x: {curr_x:.2f}, y: {curr_y:.2f}, θ: {curr_theta:.2f}")

        trajectory.append((curr_x, curr_y))

        # Pause courte pour limiter la charge CPU
        time.sleep(0.02)

        # Si tu utilises OpenCV, permettre la fermeture propre via 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Fermeture demandée via 'q'")
            break
        
except KeyboardInterrupt:
    print("KeyboardInterrupt. Exiting...")
    plot_trajectory(trajectory)
    exit_program()
