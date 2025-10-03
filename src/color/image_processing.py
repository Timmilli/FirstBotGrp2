import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
from daytime import time


def next_color(dico):
    boundaries = dico["boundaries"]
    color_string = dico["color_string"]
    if dico["current_color"] < len(boundaries)-2:
        dico["current_color"] += 1
    else:
        dico["current_color"] = 0
    # print("next", color_string[dico["current_color"]])
    dico["lower"], dico["upper"] = dico["boundaries"][dico["current_color"]]
    dico["lower"] = np.array(dico["lower"], dtype="uint8")
    dico["upper"] = np.array(dico["upper"], dtype="uint8")


def coord_is_in_left(coord, left_bar):
    x, y = coord
    if x < left_bar:
        return True
    return False


def coord_is_in_center(coord, left_bar, right_bar):
    x, y = coord
    if left_bar <= x and x <= right_bar:
        return True
    return False


def coord_is_in_right(coord, right_bar):
    x, y = coord
    if right_bar < x:
        return True
    return False


def process_frame_hsv(full_frame, dico):
    if not dico["COMPUTER_USED"]:
        # Keeping the whole frame for computer usage
        frame = full_frame[dico["top_band"]:dico["bot_band"], 0:dico["width"]]
    else:
        frame = full_frame

    # Transform from RGB to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask and output for color to be followed
    mask = cv2.inRange(hsv_frame, dico["lower"], dico["upper"])
    if dico["COMPUTER_USED"] or dico["VIDEO_FEEDBACK"]:
        output = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    nb_center = 0
    # In case if nothing is detected
    color_detected = False
    other_color_detected = False
    bypass = False
    if coords is not None:
        if len(coords) > 3500 and dico["current_color"] == 2:
            bypass = True
        color_detected = True
        for coord in coords:
            nb_center += coord[0][0]
            if dico["COMPUTER_USED"] or dico["VIDEO_FEEDBACK"]:
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
    else:
        coords = [0]
        nb_center = 0

    # Repeating the code for brown tape detection

    # Mask and output for color to be followed
    if dico["BROWN_USED"]:
        brown_hsv_frame = full_frame[dico["brown_top_band"]:dico["brown_bot_band"], int(
            dico["width"]/3):int(dico["width"]*2/3)]
        brown_mask_low = cv2.inRange(
            brown_hsv_frame, dico["brown_lower_low"], dico["brown_upper_low"])
        brown_mask_high = cv2.inRange(
            brown_hsv_frame, dico["brown_lower_high"], dico["brown_upper_high"])
        brown_mask = cv2.bitwise_or(brown_mask_low, brown_mask_high)

        brown_coords = cv2.findNonZero(brown_mask)
        if brown_coords is not None:
            brown_nb_coords = len(brown_coords)
        else:
            brown_nb_coords = 0
        brown_center_percentage = brown_nb_coords / \
            ((dico['width']/3)*abs(dico['brown_top_band']-dico['brown_bot_band']))
        print(
            f"{dico['current_color']} \
            center:{brown_nb_coords} \
           percentage:{brown_center_percentage}")
        if not dico["switch_ready"] and brown_center_percentage < 0.7:
            dico["switch_ready"] = True
        if dico["switch_ready"] and 0.6 <= brown_center_percentage:
            next_color(dico)
            dico["switch_ready"] = False

    if dico["COMPUTER_USED"]:
        # Showing images
        cv2.imshow("images", np.hstack([frame, output]))
    if dico["VIDEO_FEEDBACK"]:
        # Showing images
        cv2.imshow("images", output)

    if cv2.waitKey(1) & 0xFE == ord("n"):
        next_color(dico)

    return ((nb_center/len(coords)), color_detected, other_color_detected, bypass)
