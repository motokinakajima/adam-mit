# initialize
import sys
import math
import copy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
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


sys.path.insert(1, '../../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#################
#variables
#################

#wallfollow
kp1 = - 0.0025
ki1 = 0
kd1 = - 0.00
kp2 = - 0.01
ki2 = 0
kd2 = -0.00
kp3 = -0.01
ki3 =  0
kd3 = -0.00

normal_ratio = [0.4,0.2,0.4]
abnormal_ratio = [0,0.8,1]

limit_dist = 250
wall_speed = 0.8

#linefollow
kp_insec = 0.1
ki_insec = 0.005
kd_insec = 0.005
kp_gap = -1.6
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

ORANGE_LINE = ((11, 116, 236), (21, 255, 255))
RED_LINE = ((0, 64, 0), (8, 255,255))
BLUE_LINE = ((76, 116, 161), (121, 255, 255))
GREEN_LINE = ((47, 83, 0), (61, 255, 255))
YELLOW_LINE = ((24, 79, 0), (32, 255, 255))
LINE_PRIORITY = (YELLOW_LINE, GREEN_LINE, BLUE_LINE, RED_LINE, ORANGE_LINE)


#ELEVATOR OBJECT
RED1 = ((0, 100, 100), (18, 255, 255))  # Lower red range
RED2 = ((160, 100, 100), (180, 255, 255))
BLUE = ((68, 101, 124), (129, 229, 206))

UPPER_CROP = ((360,0),(420, 640))
LOWER_CROP = ((420, 0), (480, 640))

################
#PID object
################
#wallfollow = WallFollow2(kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3, wall_speed, normal_ratio,abnormal_ratio,limit_dist)
simple_wall_follow = SimpleWallFollow(0.052, 0.0, 0.005)
wallfollow = WallFollow3(0.006, 0.0, 0.002)
#wallfollow = WallFollow2(kp_angle, ki_angle, kd_angle, wall_speed))
linefollow = LineFollow(kp_insec, ki_insec, kd_insec, kp_gap, ki_gap, kd_gap, line_speed, LINE_PRIORITY, UPPER_CROP, LOWER_CROP)

elevator_controller = elevatorController(0.008, 0.0, 0.005)
mode_manager = ModeManager()

#detector = EdgeTPUDetector('/Users/nakajimamotoki/Downloads/onesixty_integer_quant.tflite', (160,160))

emergencySTOP = False
prevSTOP = False
prev_color_index = None
last_color_detected = None
update_last_color = True
emergencyTurn = 0
counter = 0

def start():
    elevator_controller.reset()

def update():

    global emergencySTOP, emergencyTurn, prev_color_index, last_color_detected, update_last_color, counter
    print()
    print()

    #mode_dict = {0:"wallfollow",2:"linefollow", 99:"No AR Marker"}
    image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    mode = mode_manager.update(image)

    _, forward_dist = rc_utils.get_lidar_closest_point(scan, (-15, 15))
    left_front_dist = rc_utils.get_lidar_average_distance(scan, -15, 5)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 15, 5)

    speed, angle, state = elevator_controller.update(image, scan, [BLUE], [RED1, RED2])

    print(state)

    if state == 0:
        angle = np.clip(angle, -1, 1)
        rc.drive.set_speed_angle(speed, angle)
        return
    else:
        mode = 99
        print("=======cannot detect marker; using default driving mode=========")

    if mode == 99:
        speed, angle = simple_wall_follow.update(scan)
    if mode == 8:
        speed, angle = linefollow.update(image)
        speed, angle2 = wallfollow.update(scan)
        angle = np.clip(angle, -1, 1) * 0.0 + np.clip(angle2, -1, 1) * 1.0
        """
        speed, angle = linefollow.update(image)

        current_index = linefollow.get_current_color_index()

        #print(f"forward_dist: {forward_dist}")
        #print(f"left_front_dist: {left_front_dist}")
        #print(f"right_front_dist: {right_front_dist}")

        #print()
        #print("line following")
        #print()
        #print(f"The color index is :{current_index}")
        if current_index != prev_color_index:
            update_last_color = False
            print("===========emergency rotate!===========")
            if current_index is not None and emergencyTurn == 0:
                update_last_color = False
                counter = 0
                if current_index % 2 == 0:
                    emergencyTurn =  -1.0
                else:
                    emergencyTurn = 1.0
        if forward_dist < 100:
            print("===========emergency rotate!===========")
            if current_index is not None and emergencyTurn == 0:
                update_last_color = False
                counter = 0
                if current_index % 2 == 0:
                    emergencyTurn =  -1.0
                else:
                    emergencyTurn = 1.0
            #elif current_index is None:
                #speed = -1.0
            if left_front_dist > right_front_dist:
                angle = -1.0
            else:
                angle = 1.0
        contour_width_ratio = linefollow.get_contour_width_ratio()
        print(f"found color index {current_index} with contour width radio of {contour_width_ratio}")
        if contour_width_ratio < 0.7 and contour_width_ratio != 0 and current_index != None:
            if current_index != last_color_detected and counter >= 60:
                counter = 0
                emergencyTurn = 0
                print(f"following new color index {current_index} with contour width radio of {contour_width_ratio}")
                update_last_color = True

        if emergencyTurn != 0:
            counter += 1
            angle = emergencyTurn
            print(f"emergencyTurn: {emergencyTurn}")
        else:
            print("emergencyTurn is off")

        prev_color_index = current_index
        if update_last_color:
            last_color_detected = current_index
            """

    elif mode == 0:
        speed, angle = wallfollow.update(scan)

    elif mode == 2:
        speed, angle = linefollow.update(image)

    elif mode == 7 and mode_manager.get_ar_distance() < 40:
        angle += 0.3
        angle = np.clip(angle,-1,1)

        if mode_manager.detect_marker() == None:
            mode_manager.mode = 99


    else:
        #speed = 0
        #angle = 0
        passs

    #print(detector.get_best_coordinate(image))


    angle = np.clip(angle, -1, 1)


    #print(f"mode: {mode}")
    #print(f"speed:{speed}")
    #print(f"angle:{angle}")
    print()
    print()
    """
    if emergencySTOP:
        print("EMERGENCY STOP!!!")
        speed = 0
    """



    rc.drive.set_speed_angle(speed, angle)

    #print(f"mode: {mode_dict[mode]}")

# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
