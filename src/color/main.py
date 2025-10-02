import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys
import time

from image_processing import next_color, process_frame_hsv, process_frame_rgb
from control import pixel_to_robot, go_to_one_frame

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

parser.add_argument('-col', '--color', nargs=1,
                    default=0, type=int,
                    help='Defines the color of the line to follow.')
parser.add_argument('-s', '--speed', nargs=1,
                    default=360, type=float,
                    help='Defines the standard speed of the wheels. Default is 360.')

args = parser.parse_args()

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
current_color = int(args.color[0])

if RGB_USED:
    boundaries = rgb_boundaries
else:
    boundaries = hsv_boundaries

lower, upper = boundaries[current_color]
print(color_string[current_color])
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
        STANDARD_SPEED = 300
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
    while True:
        result, frame = video_capture.read()  # read frames from the video
        if result is False:
            print("Capture has failed. Exiting...")
            exit_program()

        if RGB_USED:
            absisse, color_detected = process_frame_rgb(frame, dico)
        else:
            bypass = False
            absisse, color_detected, bypass = process_frame_hsv(frame, dico)
            if bypass and MOTOR_USED:
                # print(bypass)
                dxl_io.set_moving_speed({1: -300})  # Degrees / s
                dxl_io.set_moving_speed({2: 300})  # Degrees / s
                time.sleep(0.05)


        v_mot_droit, v_mot_gauche = 0, 0
        if PID_USED:
            if color_detected:
                speed = pid((absisse-width/2)/(width/2))
            else:
                speed = 0
            v_mot_droit = (STANDARD_SPEED - ((4/5)*STANDARD_SPEED*abs(speed)) + speed*STANDARD_SPEED)
            v_mot_gauche = STANDARD_SPEED - ((4/5)*STANDARD_SPEED*abs(speed)) - speed*STANDARD_SPEED
            # print(round(speed, 2))
        else:
            if not color_detected :
                absisse = width/2
            x_robot, y_robot = pixel_to_robot(absisse, (top_band+bot_band)/2)
            v_mot_droit, v_mot_gauche = go_to_one_frame(x_robot, y_robot, dxl_io)
            # print(round(absisse, 2), (top_band+bot_band)/2, round(x_robot, 2), round(y_robot, 2), v_mot_droit, v_mot_gauche)

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

except KeyboardInterrupt:
    print("KeyboardInterrupt. Exiting...")
    exit_program()
