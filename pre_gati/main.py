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


sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#################
#variables
#################

#wallfollow
kp1 = - 0.0025
ki1 = 0
kd1 = - 0.001
kp2 = - 0.01
ki2 = 0
kd2 = -0.001
kp3 = -0.05
ki3 =  0
kd3 = -0.001

goal_dist = 30

#normal_ratio = [0.1,0.3,0.6]
#normal_ratio = [0.5,0.2,0.3]
#abnormal_ratio = [0.4,0.5,0.1]
normal_ratio = [0.5,0.2,0.3]
abnormal_ratio = [0,0.8,1]

wall_speed = 0.8
limit_dist =250
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

UPPER_CROP = ((360,0),(420, 640))
LOWER_CROP = ((420, 0), (480, 640))

################
#PID object
################
wallfollow = WallFollow2(kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3, wall_speed)
#wallfollow = WallFollow3(kp_angle, ki_angle, kd_angle, wall_speed)
linefollow = LineFollow(kp_insec, ki_insec, kd_insec, kp_gap, ki_gap, kd_gap, line_speed, LINE_PRIORITY, UPPER_CROP, LOWER_CROP)

mode_manager = ModeManager()

#detector = EdgeTPUDetector('/Users/nakajimamotoki/Downloads/onesixty_integer_quant.tflite', (160,160))


def start():
    pass

def update():

    mode_dict = {0:"wallfollow",2:"linefollow", 99:"No AR Marker"}
    image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    mode = mode_manager.update(image)
    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
    if forward_wall_dist < limit_dist:
        ratio = abnormal_ratio
        print("abnormal")
    else:
        ratio = normal_ratio
    wallfollow.set_ratio(ratio)
    print(f"mode: {mode}")

    if mode == 99:
        print("No AR marker: wallfollowing")
        speed, angle = wallfollow.update(scan)
    elif mode == 0:
        print("wallfollowing")
        speed, angle = wallfollow.update(scan)

    elif mode == 2:
        print("linefollowing")
        speed, angle = linefollow.update(image)

    else:
        speed,angle = wallfollow.update(scan)

    
    #print(detector.get_best_coordinate(image))

    
    print()
    print(f"speed:{speed}")
    print(f"angle:{angle}")
    print(wallfollow.pid3.kp)
    

    
    rc.drive.set_speed_angle(speed, angle)

    #print(f"mode: {mode_dict[mode]}")
    
# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()



