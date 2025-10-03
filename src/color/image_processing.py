import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
from daytime import time


# Switches to the next color
def next_color(dico):
    boundaries = dico["boundaries"]
    color_string = dico["color_string"]
    if dico["current_color"] < len(boundaries)-2:
        dico["current_color"] += 1
    else:
        dico["current_color"] = 0
    dico["lower"], dico["upper"] = dico["boundaries"][dico["current_color"]]
    dico["lower"] = np.array(dico["lower"], dtype="uint8")
    dico["upper"] = np.array(dico["upper"], dtype="uint8")


# Checks if the coordinate is on the left third of the image
def coord_is_in_left(coord, width):
    x, y = coord
    if x < width / 3:
        return True
    return False


# Checks if the coordinate is on the center third of the image
def coord_is_in_center(coord, width):
    x, y = coord
    if width / 3 <= x and x <= width * 2 / 3:
        return True
    return False


# Checks if the coordinate is on the right third of the image
def coord_is_in_right(coord, width):
    x, y = coord
    if width * 2 / 3 < x:
        return True
    return False


# Processes a frame using HSV color codes
def process_frame_hsv(frame, dico):
    # Cropping the frame to a band
    if not dico["COMPUTER_USED"]:
        frame = frame[dico["top_band"]:dico["bot_band"], 0:dico["width"]]

    # Transforming from BRG to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Setting mask and output for color currently followed
    mask = cv2.inRange(hsv_frame, dico["lower"], dico["upper"])
    if dico["COMPUTER_USED"]:
        output = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)

    # Finding all pixels detected in the mask
    coords = cv2.findNonZero(mask)

    nb_center = 0
    color_detected = False
    other_color_detected = False
    bypass = False
    # If the color is detected
    if coords is not None:
        color_detected = True

        # Using a bypass for the red loops which makes the robot go straight
        if len(coords) > 3500 and dico["current_color"] == 2:
            bypass = True

        for coord in coords:
            nb_center += coord[0][0]
            if dico["COMPUTER_USED"]:
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
    # If the color wasn't found
    else:
        coords = [0]

    if dico["BROWN_USED"]:
        # Setting mask and output for the brown tape
        # Here two masks are used because the brown is detected at the end of
        # the spectrum (140 <- 180/0 -> 20)
        brown_mask_low = cv2.inRange(
            hsv_frame, dico["brown_lower_low"], dico["brown_upper_low"])
        brown_mask_high = cv2.inRange(
            hsv_frame, dico["brown_lower_high"], dico["brown_upper_high"])
        brown_mask = cv2.bitwise_or(brown_mask_low, brown_mask_high)
        if dico["COMPUTER_USED"] or dico["VIDEO_FEEDBACK"]:
            brown_output = cv2.bitwise_and(
                hsv_frame, hsv_frame, mask=brown_mask)

        # Finding all pixels detected in the mask
        brown_coords = cv2.findNonZero(brown_mask)
        brown_nb_left = 0
        brown_nb_center = 0
        brown_nb_right = 0
        if brown_coords is not None:
            for coord in brown_coords:
                # If the coordinate is in the first (left) third
                if coord_is_in_left(coord[0], dico["width"]):
                    # Showing the coordinate as red
                    if dico["COMPUTER_USED"]:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    # Adding the pixel to the left counter
                    brown_nb_left += 1
                elif coord_is_in_center(coord[0], dico["width"]):
                    if dico["COMPUTER_USED"]:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    brown_nb_center += 1
                elif coord_is_in_right(coord[0], dico["width"]):
                    if dico["COMPUTER_USED"]:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    brown_nb_right += 1
        else:
            # If no brown was detected
            brown_coords = [0]
        # Calculating the percentage of brown detected in the center
        # over the amount of brown detected overall
        brown_center_percentage = brown_nb_center/len(brown_coords)
        # If the percentage is low enough
        if not dico["switch_ready"] and brown_center_percentage < 0.2:
            # The bot is ready to switch color
            dico["switch_ready"] = True
        # If the bot is ready to switch and if there are enough brown pixels in the center
        if dico["switch_ready"] and 0.7 <= brown_center_percentage and brown_center_percentage <= 1:
            # The bot switches the color to be detected
            next_color(dico)
            switch_ready = False

    if dico["COMPUTER_USED"]:
        # Showing the frame and the masks
        cv2.imshow("images", np.hstack([frame, output]))

    if dico["VIDEO_FEEDBACK"]:
        # Showing the masks
        cv2.imshow("images", output)

    return ((nb_center/len(coords)), color_detected, other_color_detected, bypass)
