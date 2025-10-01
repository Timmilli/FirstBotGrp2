import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys

parser = argparse.ArgumentParser(
    prog='Main file',
    description='To launch the robot movements')
parser.add_argument('-c', '--computer_used', action='store_true',
                    help='Defines if the program is launched on a computer.')
parser.add_argument('-m', '--motor_used', action='store_true',
                    help='Defines if the motors needs to be used.')
parser.add_argument('-b', '--brown_detection', action='store_false',
                    help='Defines if the motors needs to be used.')

parser.add_argument('-s', '--speed', nargs=1,
                    default=360, type=float,
                    help='Defines the standard speed of the wheels. Default is 360.')

args = parser.parse_args()

MOTOR_USED = args.motor_used
COMPUTER_USED = args.computer_used
BROWN_USED = args.brown_detection


if MOTOR_USED:
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        sys.exit("Motors are used but are not detected. Exiting...")
    else:
        dxl_io = pypot.dynamixel.DxlIO(ports[0])
        dxl_io.set_wheel_mode([1])
        STANDARD_SPEED = 360
        print(
            f"Motors detected and used. Setting standard speed at {STANDARD_SPEED:%02f}.")


video_capture = cv2.VideoCapture(0)

width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video_capture.get(cv2.CAP_PROP_FPS))
print(f"width:{width}; height:{height}; fps:{fps}")

pid = PID(4, 0.2, 0.2, setpoint=0)

# Coded in HSV
# Maroon has to be the last color
boundaries = [
    ([20, 0, 160], [140, 40, 255]),  # A yellow tape
    ([80, 210, 210], [100, 255, 255]),  # A blue tape
    ([160, 80, 110], [180, 255, 255]),  # A red tape
    ([100, 70, 80], [130, 150, 140])  # A maroon tape
]

color_string = ["Yellow Tape", "Blue Tape", "Red Tape", "Maroon Tape"]
current_color = 0


def next_color():
    global current_color
    if current_color < len(boundaries)-2:
        current_color += 1
    else:
        current_color = 0
    print(color_string[current_color])
    return current_color


lower, upper = boundaries[next_color()]
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")
brown_lower, brown_upper = boundaries[-1]
brown_lower = np.array(brown_lower, dtype="uint8")
brown_upper = np.array(brown_upper, dtype="uint8")
switch_ready = True


def exit_program():
    video_capture.release()
    cv2.destroyAllWindows()
    if MOTOR_USED:
        dxl_io.set_moving_speed(
            {1: 0})  # Degrees / s
        dxl_io.set_moving_speed(
            {2: 0})  # Degrees / s
    sys.exit()


top_band = 400
bot_band = 410

try:
    speed = 0
    while True:
        result, frame = video_capture.read()  # read frames from the video
        if result is False:
            print("Capture has failed. Exiting...")
            exit_program()

        if COMPUTER_USED:
            full_frame = deepcopy(frame)

        # frame = frame[0:height, int(width/2)-5:int(width/2)+5]
        frame = frame[top_band:bot_band, 0:width]

        # Transform from RGB to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mask and output for color to be followed
        mask = cv2.inRange(frame, lower, upper)
        if BROWN_USED:
            brown_mask = cv2.inRange(hsv_frame, brown_lower, brown_upper)
        if COMPUTER_USED:
            output = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
            if BROWN_USED:
                brown_output = cv2.bitwise_and(
                    hsv_frame, hsv_frame, mask=brown_mask)
        # Find all pixels detected in the mask
        coords = cv2.findNonZero(mask)
        if BROWN_USED:
            brown_coords = cv2.findNonZero(brown_mask)
        nb_center = 0
        # In case if nothing is detected
        if coords is not None:
            for coord in coords:
                nb_center += coord[0][1]
                if COMPUTER_USED:
                    output = cv2.circle(output, coord[0], radius=0,
                                        color=(0, 0, 255), thickness=-1)
            speed = pid(-(nb_center/len(coords))/width + 0.5)
            # print(nb_center/len(coords)/width - 0.5, ", speed = ", speed)

        else:
            coords = [0]
            speed = 0
            # print("color not detected, speed = ", speed)

        # Mask and output for color to be followed
        if BROWN_USED:
            brown_mask = cv2.inRange(hsv_frame, brown_lower, brown_upper)
            if COMPUTER_USED:
                brown_output = cv2.bitwise_and(
                    hsv_frame, hsv_frame, mask=brown_mask)

            brown_nb_left = 0
            brown_nb_center = 0
            brown_nb_right = 0
            if brown_coords is not None:
                for coord in brown_coords:
                    # If the coordinate is in the first (left) third
                    if coord_is_in_left(coord[0]):
                        # Show the coordinate as red
                        if COMPUTER_USED:
                            output = cv2.circle(output, coord[0], radius=0,
                                                color=(255, 255, 255), thickness=-1)
                        # Add the pixel to the left counter
                        brown_nb_left += 1
                    elif coord_is_in_center(coord[0]):
                        if COMPUTER_USED:
                            output = cv2.circle(output, coord[0], radius=0,
                                                color=(255, 255, 255), thickness=-1)
                        brown_nb_center += 1
                    elif coord_is_in_right(coord[0]):
                        if COMPUTER_USED:
                            output = cv2.circle(output, coord[0], radius=0,
                                                color=(255, 255, 255), thickness=-1)
                        brown_nb_right += 1
            else:
                brown_coords = [0]
            brown_center_percentage = brown_nb_center/len(brown_coords)
            if not switch_ready and brown_center_percentage < 0.2:
                switch_ready = True
            if switch_ready and 0.7 <= brown_center_percentage and brown_center_percentage <= 1:
                lower, upper = boundaries[next_color()]
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                switch_ready = False

        if MOTOR_USED:
            dxl_io.set_moving_speed(
                {2: -(STANDARD_SPEED + speed*STANDARD_SPEED)})  # Degrees / s
            dxl_io.set_moving_speed(
                {1: STANDARD_SPEED - speed*STANDARD_SPEED})  # Degrees / s

        if COMPUTER_USED:
            # Visual line, not necessary for computing
            cv2.rectangle(output, (int(width/3), bot_band),
                          (int(width*2/3), top_band), (255, 0,  0), 2)

            # cv2.rectangle(output, (int(width/2)-5, 0),
            #              (int(width/2)+5, height), (0, 255, 0), 2)

            # Showing images
            cv2.imshow("images", np.hstack([frame, output]))

            if cv2.waitKey(1) & 0xFF == ord("q"):
                exit_program()

        if cv2.waitKey(1) & 0xFE == ord("n"):
            lower, upper = boundaries[next_color()]
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

except KeyboardInterrupt:
    print("KeyboardInterrupt. Exiting...")
    exit_program()
