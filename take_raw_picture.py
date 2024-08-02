########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
import matplotlib.pyplot as plt

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
CROP_FLOOR = ((180, 0), (240,320))
BLUE = ((74, 98, 0), (118, 255, 255))  # The HSV range for the color blue
counter1 = 0

########################################################################################
# Functions
########################################################################################
def start():
    pass
def update():

# Use the triggers to control the car's spee
    global counter1
    if rc.controller.was_pressed(rc.controller.Button.X):
        image = rc.camera.get_color_image()
        #image = rc_utils.crop(image, (60,0), (240,320))
        #contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        #largest_contour = rc_utils.get_largest_contour(contours)
        #if largest_contour is not None:
        #    rc_utils.draw_contour(image, largest_contour)
        #area = round(cv.contourArea(largest_contour))
        #image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        filename = "./raw_images/image" + str(counter1) + ".jpg"
        counter1 += 1
        print("wrote file:",filename)
        #print("area:", area)
        cv.imwrite(filename,image)
        
def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


