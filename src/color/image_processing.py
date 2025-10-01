import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys

def next_color(current_color, boundaries, color_string):
    if current_color < len(boundaries)-2:
        current_color += 1
    else:
        current_color = 0
    print(color_string[current_color])
    return current_color

def coord_is_in_left(coord, width):
    x, y = coord
    if x < width / 3:
        return True
    return False


def coord_is_in_center(coord, width):
    x, y = coord
    if width / 3 <= x and x <= width * 2 / 3:
        return True
    return False

def coord_is_in_right(coord, width):
    x, y = coord
    if width * 2 / 3 < x:
        return True
    return False

def process_frame(frame, lower, upper, brown_lower, brown_upper, width, height, top_band, bot_band, BROWN_USED, COMPUTER_USED, switch_ready, boundaries, current_color, color_string):
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
    color_detected = True
    if coords is not None:
        color_detected = True
        for coord in coords:
            nb_center += coord[0][1]
            if COMPUTER_USED:
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
        # print(nb_center/len(coords)/width - 0.5, ", speed = ", speed)

    else:
        coords = [0]
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
                if coord_is_in_left(coord[0], width):
                    # Show the coordinate as red
                    if COMPUTER_USED:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    # Add the pixel to the left counter
                    brown_nb_left += 1
                elif coord_is_in_center(coord[0], width):
                    if COMPUTER_USED:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    brown_nb_center += 1
                elif coord_is_in_right(coord[0], width):
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
            lower, upper = boundaries[next_color(current_color, boundaries, color_string)]
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            switch_ready = False

    if COMPUTER_USED:
        # Visual line, not necessary for computing
        cv2.rectangle(output, (int(width/3), bot_band),
                        (int(width*2/3), top_band), (255, 0,  0), 2)

        # cv2.rectangle(output, (int(width/2)-5, 0),
        #              (int(width/2)+5, height), (0, 255, 0), 2)

        # Showing images
        cv2.imshow("images", np.hstack([frame, output]))


    if cv2.waitKey(1) & 0xFE == ord("n"):
        lower, upper = boundaries[next_color()]
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

    return (-(nb_center/len(coords))/width + 0.5, color_detected)