import numpy as np
from pid import PIDController
import os
from dotenv import load_dotenv
from pathlib import Path
import sys

dotenv_path = Path('../.env')
load_dotenv(dotenv_path=dotenv_path)
LIBRARY_PASS = os.getenv('LIBRARY_PASS')
sys.path.insert(1, LIBRARY_PASS)
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()


def update_contour(_image,color,crop_floor):
    center = None
    area = None
    _image = rc_utils.crop(_image, crop_floor[0], crop_floor[1])
    # print(image.shape)
    # print(CROP_FLOOR)
    if _image is None:
        center = None
        area = 0
    else:  
        contours = rc_utils.find_contours(_image, color[0],color[1])
        tmp_contour_area = rc_utils.get_largest_contour(contours)
        if tmp_contour_area is not None:
            area = rc_utils.get_contour_area(tmp_contour_area)
            center = rc_utils.get_contour_center(tmp_contour_area)
        
        else:
            center = None
            area = 0
    return center, area


class LineFollow:
    def __init__(self, kp, ki, kd, speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.pid = PIDController(kp, ki, kd)
        self.pid.start()

        self.speed = speed
    
    def update(self, center):
        if center is not None:
            error = rc_utils.remap_range(center[1], 0, 320, -1, 1)
        else:
            error = 0.0
            print("NO CONTOUR")

        angle = self.pid.update(0,error)
        
        # angle = kp * error
        angle = np.clip(angle, -1, 1)

        return self.speed, angle