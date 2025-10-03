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
    print("next", color_string[dico["current_color"]])
    dico["lower"], dico["upper"] = dico["boundaries"][dico["current_color"]]
    dico["lower"] = np.array(dico["lower"], dtype="uint8")
    dico["upper"] = np.array(dico["upper"], dtype="uint8")


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


def process_frame_hsv(frame, dico):
    if dico["COMPUTER_USED"]:
        full_frame = deepcopy(frame)

    # frame = frame[0:height, int(width/2)-5:int(width/2)+5]
    if not dico["COMPUTER_USED"]:
        frame = frame[dico["top_band"]:dico["bot_band"], 0:dico["width"]]

    # Transform from RGB to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask and output for color to be followed
    mask = cv2.inRange(hsv_frame, dico["lower"], dico["upper"])
    if dico["BROWN_USED"]:
        brown_mask = cv2.inRange(
            hsv_frame, dico["brown_lower"], dico["brown_upper"])
    if dico["COMPUTER_USED"]:
        output = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)
        if dico["BROWN_USED"]:
            brown_output = cv2.bitwise_and(
                hsv_frame, hsv_frame, mask=brown_mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    if dico["BROWN_USED"]:
        brown_coords = cv2.findNonZero(brown_mask)
    nb_center = 0
    # In case if nothing is detected
    color_detected = False
    other_color_detected = False
    bypass = False
    if coords is not None:
        # print("nb_coords ", len(coords))
        if len(coords) > 3500 and dico["current_color"] == 2:
            bypass = True
        color_detected = True
        for coord in coords:
            nb_center += coord[0][0]
            if dico["COMPUTER_USED"]:
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
        # print(nb_center/len(coords)/width - 0.5, ", speed = ", speed)

    else:
        # next_color(dico)
        # hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_frame, dico["lower"], dico["upper"])
        # coords = cv2.findNonZero(mask)
        # # if coords is not None:
        # #     if len(coords) > 300:
        #         # other_color_detected = True

        # next_color(dico)
        # hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv_frame, dico["lower"], dico["upper"])
        # coords = cv2.findNonZero(mask)
        # # if coords is not None:
        # #     if len(coords) > 300:
        #         # other_color_detected = True
        
        coords = [0]
        # next_color(dico)

    # Mask and output for color to be followed
    if dico["BROWN_USED"]:
        brown_mask = cv2.inRange(
            hsv_frame, dico["brown_lower"], dico["brown_upper"])
        if dico["COMPUTER_USED"]:
            brown_output = cv2.bitwise_and(
                hsv_frame, hsv_frame, mask=brown_mask)

        brown_nb_left = 0
        brown_nb_center = 0
        brown_nb_right = 0
        if brown_coords is not None:
            for coord in brown_coords:
                # If the coordinate is in the first (left) third
                if coord_is_in_left(coord[0], dico["width"]):
                    # Show the coordinate as red
                    if dico["COMPUTER_USED"]:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    # Add the pixel to the left counter
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
            brown_coords = [0]
        brown_center_percentage = brown_nb_center/len(brown_coords)
        if not dico["switch_ready"] and brown_center_percentage < 0.2:
            dico["switch_ready"] = True
        if dico["switch_ready"] and 0.7 <= brown_center_percentage and brown_center_percentage <= 1:
            next_color(dico)
            switch_ready = False

    if dico["COMPUTER_USED"]:
        # Visual line, not necessary for computing
        cv2.rectangle(output, (int(dico["width"]/3), dico["bot_band"]),
                      (int(dico["width"]*2/3), dico["top_band"]), (255, 0,  0), 2)

        # cv2.rectangle(output, (int(width/2)-5, 0),
        #              (int(width/2)+5, height), (0, 255, 0), 2)

        # Showing images
        cv2.imshow("images", np.hstack([frame, output]))

    if cv2.waitKey(1) & 0xFE == ord("n"):
        next_color(dico)

    return ((nb_center/len(coords)), color_detected, other_color_detected, bypass)


def process_frame_rgb(frame, dico):
    if dico["COMPUTER_USED"]:
        full_frame = deepcopy(frame)

    # frame = frame[0:height, int(width/2)-5:int(width/2)+5]
    if not dico["COMPUTER_USED"]:
        frame = frame[dico["top_band"]:dico["bot_band"], 0:dico["width"]]

    # Mask and output for color to be followed
    mask = cv2.inRange(frame, dico["lower"], dico["upper"])
    if dico["BROWN_USED"]:
        brown_mask = cv2.inRange(
            frame, dico["brown_lower"], dico["brown_upper"])
    if dico["COMPUTER_USED"]:
        output = cv2.bitwise_and(frame, frame, mask=mask)
        if dico["BROWN_USED"]:
            brown_output = cv2.bitwise_and(
                frame, frame, mask=brown_mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    if dico["BROWN_USED"]:
        brown_coords = cv2.findNonZero(brown_mask)
    nb_center = 0
    # In case if nothing is detected
    color_detected = False
    if coords is not None:
        color_detected = True
        for coord in coords:
            nb_center += coord[0][0]
            if dico["COMPUTER_USED"]:
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
        # print(nb_center/len(coords)/width - 0.5, ", speed = ", speed)

    else:
        coords = [0]
        # print("color not detected, speed = ", speed)

    # Mask and output for color to be followed
    if dico["BROWN_USED"]:
        brown_mask = cv2.inRange(
            frame, dico["brown_lower"], dico["brown_upper"])
        if dico["COMPUTER_USED"]:
            brown_output = cv2.bitwise_and(
                frame, frame, mask=brown_mask)

        brown_nb_left = 0
        brown_nb_center = 0
        brown_nb_right = 0
        if brown_coords is not None:
            for coord in brown_coords:
                # If the coordinate is in the first (left) third
                if coord_is_in_left(coord[0], dico["width"]):
                    # Show the coordinate as red
                    if dico["COMPUTER_USED"]:
                        output = cv2.circle(output, coord[0], radius=0,
                                            color=(255, 255, 255), thickness=-1)
                    # Add the pixel to the left counter
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
            brown_coords = [0]
        brown_center_percentage = brown_nb_center/len(brown_coords)
        if not dico["switch_ready"] and dico["brown_center_percentage"] < 0.2:
            dico["switch_ready"] = True
        if dico["switch_ready"] and 0.7 <= dico["brown_center_percentage"] and dico["brown_center_percentage"] <= 1:
            dico["lower"], dico["upper"] = dico["boundaries"][next_color(
                dico["current_color"], dico["boundaries"], dico["color_string"])]
            lower = np.array(dico["lower"], dtype="uint8")
            upper = np.array(dico["upper"], dtype="uint8")
            switch_ready = False

    if dico["COMPUTER_USED"]:
        # Visual line, not necessary for computing
        cv2.rectangle(output, (int(dico["width"]/3), dico["bot_band"]),
                      (int(dico["width"]*2/3), dico["top_band"]), (255, 0,  0), 2)

        # cv2.rectangle(output, (int(width/2)-5, 0),
        #              (int(width/2)+5, height), (0, 255, 0), 2)

        # Showing images
        cv2.imshow("images", np.hstack([frame, output]))

    if cv2.waitKey(1) & 0xFE == ord("n"):
        dico["lower"], dico["upper"] = dico["boundaries"][next_color(
            dico["current_color"], dico["boundaries"], dico["color_string"])]
        dico["lower"] = np.array(dico["lower"], dtype="uint8")
        dico["upper"] = np.array(dico["upper"], dtype="uint8")

    return ((nb_center/len(coords)), color_detected)
