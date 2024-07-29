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
from marker import * 


sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#################
#variables
#################

#wallfollow
kp_angle = 0.01
ki_angle = 0.005
kd_angle = 0.005
kp_dist = 0.01
ki_dist = 0.005
kd_dist = 0.005

goal_dist = 50
wall_speed = 0.8

#linefollow
kp_line = 0.01
ki_line = 0.005
kd_line = 0.005

line_speed = 0.8

BLUE_LINE = ((76, 53, 26), (126, 255, 255))
LINE_CROP_FLOOR = ((180, 0), (240, 320))

################
#PID object
################
wallfollow = WallFollow(kp_angle, ki_angle, kd_angle, kp_dist, ki_dist, kd_dist, wall_speed, goal_dist)
linefollow = LineFollow(kp_line, ki_line, kd_line, line_speed)


def start():
    pass

def update():
    mode_dict = {0:"wallfollow",2:"linefollow"}
    image = rc.camera.get_color_image()
    line_center = None
    line_area = 0.0
    line_center,line_area = update_contour(image,BLUE_LINE,LINE_CROP_FLOOR)
    mode, square = detect_marker(image)

    if image == None:
        print("no image")
        speed, angle = 0,0

    else:
        if mode == 0:
            speed, angle = wallfollow.update()

        elif mode == 2:
            speed, angle = linefollow.update(line_center)
    
        print(f"mode:{mode_dict[mode]}")
        print(f"marker square{square}")

    
    print()
    print(f"speed:{speed}")
    print(f"angle:{angle}")
            
    

    speed, angle = wallfollow.update()
    rc.drive.set_speed_angle(speed, angle)
    
# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()



