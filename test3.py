"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: lab_f.py
Title: Lab F - Line Follower
Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""

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

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 1000

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (480,640))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!

BLUE = ((63, 34, 143), (129, 206, 229))  # The HSV range for the color blue
GREEN = ((68, 49, 131), (100, 184, 255))  # The HSV range for the color green
RED = ((0, 50, 50), (10, 255,255))  # The HSV range for the color red


# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (BLUE, RED, GREEN)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0 # The area of contour
pre_error = 0.0
integral = 0.0
kp = 1.0
ki = 0.1
kd = 0.2
angle_values = []
image2 = None

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
counter1 = 0
def update_contour():
    global contour_center
    global contour_area
    image = rc.camera.get_color_image()
    global image2
    image2 = image
    #print(image.shape) 
    #print(CROP_FLOOR)
    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        for i,color in enumerate(COLOR_PRIORITY):
            contours = rc_utils.find_contours(image, *color)
            tmp_contour_area = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
            if tmp_contour_area is not None:
                contour_area = tmp_contour_area
                contour_center = rc_utils.get_contour_center(contour_area)
                break
        else:
            # if any color's contours is None, variables become init value
            contour_center = None
            contour_area = 0

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        # Display the image to the screen


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    image = rc.camera.get_color_image()
    height, width, channels = image.shape[:3]
    print("width: "+str(width))
    print("height: "+str(height))
    contours = rc_utils.find_contours(image,BLUE[0], BLUE[1])
    largest_contour = rc_utils.get_largest_contour(contours)
    if largest_contour is not None:
        rc_utils.draw_contour(image, largest_contour)
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
    #cv.imwrite("image.jpg",image)
    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(3)

    # Print start message
    print(
        ">> Lab 2A - Color Image Line Following\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global pre_error
    global integral
    global counter1

    # Search for contours in the current color image
    #update_contour()

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        #error = contour_center[1] - rc.camera.get_width() / 2
        
        error = rc_utils.remap_range(contour_center[1], 0, 640, -1, 1)
    else:
        error = 0.0
    
    integral += error
    derivative = error - pre_error
    angle = kp * error + ki * integral + kd * derivative
    pre_error = error
    angle_values.append(angle)
    angle = np.clip(angle, -1, 1)


# Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    if rc.controller.was_pressed(rc.controller.Button.X):
        image = rc.camera.get_color_image()
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        largest_contour = rc_utils.get_largest_contour(contours)
        if largest_contour is not None:
            rc_utils.draw_contour(image, largest_contour)
        area = round(cv.contourArea(largest_contour))
        #image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        #image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        filename = "./test3_images/image" + str(counter1) + ".jpg"
        counter1 += 1
        print("wrote file:",filename)
        print("area:", area)
        cv.imwrite(filename,image)

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global image2
    #rc.display.show_color_image(image2)
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        print(contour_center)
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()

    #plt.plot(angle_values)
    #plt.xlabel('Time')
    #plt.ylabel('Angle')
    #plt.title('Angle over Time')
    #plt.show()



