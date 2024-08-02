# initialize
import sys
import math
import copy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum
import matplotlib.pyplot as plt

from wallfollow import *
from linefollow import *
from mode_manage import *
#from get_stopsign import *
from elevator_controller import *


sys.path.insert(1, '/Users/AT/Desktop/racecar-neo-installer/racecar-student/library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#################
#variables
#################

#wallfollow
kp_angle = - 0.3
ki_angle = 0
kd_angle = - 0.001
kp_dist = - 0.01
ki_dist = 0
kd_dist = 0.005

goal_dist = 30
wall_speed = 0.8

#linefollow
kp_insec = 0.01
ki_insec = 0.005
kd_insec = 0.005
kp_gap = -0.8
ki_gap= 0.0
kd_gap = 0.0

line_speed = 0.8
"""
BLUE_LINE = ((63, 34, 143), (129, 206, 229))
#BLUE_LINE = ((58, 98, 143), (158, 255, 255))
GREEN_LINE = ((50, 131, 173), (103, 255, 255))  # The HSV range for the color green
RED_LINE = ((0, 105, 94), (18, 255,255))
LINE_PRIORITY = (GREEN_LINE, BLUE_LINE, RED_LINE)
"""

ORANGE_LINE = ((11, 64, 143), (16, 255, 255))
RED_LINE = ((0, 64, 140), (10, 255, 255))
BLUE_LINE = ((63, 34, 143), (129, 206, 229))
GREEN_LINE = ((45, 11, 105), (61, 255, 255))
YELLOW_LINE = ((24, 53, 98), (30, 255, 255))
LINE_PRIORITY = (GREEN_LINE, RED_LINE, BLUE_LINE, ORANGE_LINE, YELLOW_LINE)


#ELEVATOR OBJECT
RED1 = ((0, 100, 100), (18, 255, 255))  # Lower red range
RED2 = ((160, 100, 100), (180, 255, 255))
BLUE = ((68, 101, 124), (129, 229, 206))

UPPER_CROP = ((360,0),(420, 640))
LOWER_CROP = ((420, 0), (480, 640))

################
#PID object
################
wallfollow = WallFollow(kp_angle, ki_angle, kd_angle, kp_dist, ki_dist, kd_dist, wall_speed, goal_dist)
#wallfollow = WallFollow2(kp_angle, ki_angle, kd_angle, wall_speed)
linefollow = LineFollow(kp_insec, ki_insec, kd_insec, kp_gap, ki_gap, kd_gap, line_speed, LINE_PRIORITY, UPPER_CROP, LOWER_CROP)

elevator_controller = elevatorController(0.01, 0.0, 0.005)
mode_manager = ModeManager()

#detector = EdgeTPUDetector('/Users/nakajimamotoki/Downloads/onesixty_integer_quant.tflite', (160,160))

emergencySTOP = False
prevSTOP = False


def start():
    elevator_controller.reset()


def update():

    global emergencySTOP

    #mode_dict = {0:"wallfollow",2:"linefollow", 99:"No AR Marker"}
    image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    mode = mode_manager.update(image)

    speed, angle, state = elevator_controller.update(image, scan, [BLUE], [RED1, RED2])

    print(state)

    if state == 0:
        angle = np.clip(angle, -1, 1)
        rc.drive.set_speed_angle(speed, angle)
        return
    else:
        mode = 0
        print("=======cannot detect marker; using default driving mode=========")

    if mode == 99:
        speed, angle = wallfollow.update(scan)
        print()
        print("wall following")
        print()

    elif mode == 0:
        speed, angle = wallfollow.update(scan)

    elif mode == 2:
        speed, angle = linefollow.update(image)

    else:
        speed = 0
        angle = 0

    #print(detector.get_best_coordinate(image))

    
    angle = np.clip(angle, -1, 1)


    print()
    print(f"speed:{speed}")
    print(f"angle:{angle}")
    print()

    if emergencySTOP:
        print("EMERGENCY STOP!!!")
        speed = 0



    rc.drive.set_speed_angle(speed, angle)

    #print(f"mode: {mode_dict[mode]}")

# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
