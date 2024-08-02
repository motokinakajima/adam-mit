import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys

sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils


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
    def __init__(self, kp_insec, ki_insec, kd_insec, kp_gap, ki_gap, kd_gap, speed, color, upper_crop, lower_crop):
        self.kp_insec = kp_insec
        self.ki_insec = ki_insec
        self.kd_insec = kd_insec

        self.pid_insec = PIDController(kp_insec, ki_insec, kd_insec)
        self.pid_gap = PIDController(kp_gap, ki_gap, kd_gap)

        self.pid_insec.start()
        self.pid_gap.start()

        self.color = color
        self.upper_crop = upper_crop
        self.lower_crop = lower_crop

        self.speed = speed
    
    def update(self, image):
        middle_crop = (self.upper_crop[0],self.lower_crop[1])


        upper_center, _ = update_contour(image, self.color, self.upper_crop)
        lower_center, _ = update_contour(image, self.color, self.lower_crop)
        center, _ = update_contour(image, self.color, middle_crop)

        if upper_center is not None and lower_center is not None:
        
            intersection = lower_center[0] + (upper_center[0] - lower_center[0]) * (lower_center[1] - upper_center[1]) / (lower_center[0] - self.lower_crop[1][0] / 2)
            insec_gap = intersection - self.lower_crop[1][1]


            if center is not None:
                insec_error = rc_utils.remap_range(insec_gap, 0, 320, -1, 1)
                gap_error = rc_utils.remap_range(center[1], 0, 320, -1, 1)
            else:
                insec_error = 0.0
                gap_error = 0.0
                print("NO CONTOUR")

            insec_angle = self.pid_insec.update(0,insec_error)
            gap_angle = self.pid_gap.update(0,gap_error)

            angle = insec_angle + gap_angle
            angle = np.clip(angle, -1, 1)

        else:
            angle = 0

        return self.speed, angle
