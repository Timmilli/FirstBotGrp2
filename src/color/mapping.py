import numpy as np
import cv2
import pypot.dynamixel
from simple_pid import PID
from copy import deepcopy
import argparse
import sys


def mapping(dxl_io, VIDEO_CAPTURE):
    if VIDEO_CAPTURE:
        video_capture = cv2.VideoCapture(0)

        width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(video_capture.get(cv2.CAP_PROP_FPS))
        print(f"width:{width}; height:{height}; fps:{fps}")
    v_gauche, v_droit = dxl_io.get_moving_speed([1, 2])


if __name__ == '__main__':
    print("Starting mapping")
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        sys.exit("Motors are used but are not detected. Exiting...")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.disable_torque([1, 2])
    mapping(dxl_io)
