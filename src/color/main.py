import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys

from image_processing import process_frame, next_color, process_frame_hsv, process_frame_rgb

parser = argparse.ArgumentParser(
    prog='Main file',
    description='To launch the robot movements')
parser.add_argument('-c', '--computer_used', action='store_true',
                    help='Defines if the program is launched on a computer.')
parser.add_argument('-m', '--motor_used', action='store_true',
                    help='Defines if the motors needs to be used.')
parser.add_argument('-b', '--brown_detection', action='store_false',
                    help='Defines if the motors needs to be used.')
parser.add_argument('-h', '--hsv_used', action='store_false',
                    help='Defines if the HSV is used over the RGB.')
# parser.add_argument('-rgb', '--rgb_used', action='store_false',
#                     help='Defines if the HSV is used over the RGB.')

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
HSV_USED = args.hsv_used
# RGB_USED = args.rgb_used

# Coded in HSV
# Maroon has to be the last color
hsv_boundaries = [
    ([20, 0, 160], [140, 40, 255]),  # A yellow tape
    ([80, 210, 210], [100, 255, 255]),  # A blue tape
    ([160, 80, 110], [180, 255, 255]),  # A red tape
    ([100, 70, 80], [130, 150, 140])  # A maroon tape
]
# Coded in BGR
# Maroon has to be the last color
rgb_boundaries = [
    ([100, 170, 150], [150, 220, 200]),  # A yellow tape (to be reworked on)
    ([190, 150, 0], [255, 200, 50]),  # A blue tape
    ([140, 70, 130], [100, 100, 255]),  # A red tape (to be reworked on)
    ([110, 90, 70], [130, 110, 90]),  # A brown tape (to be reworked on)
]

color_string = ["Yellow Tape", "Blue Tape", "Red Tape", "Maroon Tape"]
current_color = 0

if HSV_USED:
    boundaries = hsv_boundaries
else:
    boundaries = rgb_boundaries

lower, upper = boundaries[next_color(current_color, boundaries, color_string)]
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
        STANDARD_SPEED = 360
        print(
            f"Motors detected and used. Setting standard speed at {STANDARD_SPEED}.")


video_capture = cv2.VideoCapture(0)

width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video_capture.get(cv2.CAP_PROP_FPS))
print(f"width:{width}; height:{height}; fps:{fps}")

pid = PID(0.5, 0.05, 0.01, setpoint=0)


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
        "motor_used": MOTOR_USED,
        "computer_used": COMPUTER_USED,
        "brown_used": BROWN_USED,
        "hsv_used": HSV_USED,
        # "rgb_used": RGB_USED,
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

        if HSV_USED:
            absisse, color_detected = process_frame_hsv(frame, dico)
        else:
            absisse, color_detected = process_frame_rgb(frame, dico)

        if color_detected:
            speed = pid(absisse-width/2)
            print("blue detected")
        else:
            speed = 0
            print("color not detected")

        if MOTOR_USED:
            dxl_io.set_moving_speed(
                {1: -(STANDARD_SPEED - speed*STANDARD_SPEED)})  # Degrees / s
            dxl_io.set_moving_speed(
                {2: STANDARD_SPEED + speed*STANDARD_SPEED})  # Degrees / s

        if COMPUTER_USED:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit_program()

except KeyboardInterrupt:
    print("KeyboardInterrupt. Exiting...")
    exit_program()
