import numpy as np
import cv2 as cv
from pid import PIDController
import os
from pathlib import Path
import sys

sys.path.insert(1, '../../library')
import racecar_utils as rc_utils


def update_contour(_image,priority,crop_floor):
    contour_center = None
    contour_area = 0
    MIN_CONTOUR_AREA = 500
    current_color_index = None
    

    # print(CROP_FLOOR)
    if _image is None:
        contour_center = None
        contour_area = 0
    else:  
        _image = rc_utils.crop(_image, crop_floor[0], crop_floor[1])
        for i,color in enumerate(priority):
            contours = rc_utils.find_contours(_image, *color)
            tmp_contour_area = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if tmp_contour_area is not None:
                contour_area = tmp_contour_area
                contour_center = rc_utils.get_contour_center(contour_area)
                #print(f"{i}th loop")
                current_color_index = i
                break
            
        else:
            # if any color's contours is None, variables become init value
            contour_center = None
            contour_area = 0

    return contour_center, contour_area, current_color_index


class LineFollow:
    def __init__(self, kp_insec, ki_insec, kd_insec, kp_gap, ki_gap, kd_gap, speed, priority, upper_crop, lower_crop):
        self.kp_insec = kp_insec
        self.ki_insec = ki_insec
        self.kd_insec = kd_insec

        self.pid_insec = PIDController(kp_insec, ki_insec, kd_insec)
        self.pid_gap = PIDController(kp_gap, ki_gap, kd_gap)

        self.pid_insec.start()
        self.pid_gap.start()

        self.priority = priority
        self.upper_crop = upper_crop
        self.lower_crop = lower_crop

        self.speed = speed
    
    def update(self, image):
        middle_crop = (self.upper_crop[0],self.lower_crop[1])

        upper_center, _ , self.current_color_index = update_contour(image, self.priority, self.upper_crop)
        lower_center, _ = update_contour(image, self.priority, self.lower_crop)
        center, _ = update_contour(image, self.priority, middle_crop)
    
        #intersection = lower_center[0] + (upper_center[0] - lower_center[0]) * (lower_center[1] - upper_center[1]) / (lower_center[0] - self.lower_crop[1][0] / 2)
        #insec_gap = intersection - self.lower_crop[1][1]


        #if center is not None:
        if (center is not None) and (upper_center is not None) and (lower_center is not None):
            intersection = lower_center[0] + (upper_center[0] - lower_center[0]) * (lower_center[1] - upper_center[1]) / (lower_center[0] - self.lower_crop[1][0] / 2)
            insec_gap = intersection - self.lower_crop[1][1]
            insec_error = rc_utils.remap_range(insec_gap, 0, 640, -1, 1)
            gap_error = rc_utils.remap_range(center[1], 0, 640, -1, 1)

            print(f"insec error: {insec_error}, gap error: {gap_error}")
        elif center is not None:
            print("NO INTERSECTION")
            insec_error = 0.0
            gap_error  = rc_utils.remap_range(center[1], 0, 640, -1, 1)
            #print(f"Center of LINE: {center[1]}")
            #print(f"gap error: {gap_error}")
        else:
            insec_error = 0.0
            gap_error = 0.0
            print("NO CONTOUR")

        insec_angle = self.pid_insec.update(0,insec_error)
        gap_angle = self.pid_gap.update(0,gap_error)

        print(f"gap angle: {gap_angle}")
        print(f"insec angle: {insec_angle}")

        angle = insec_angle + gap_angle
        angle = gap_angle
        angle = np.clip(angle, -1, 1)

        return self.speed, angle
    
    def get_current_color_index(self):
        return self.current_color_index