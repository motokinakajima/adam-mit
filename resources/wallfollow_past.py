"""
MIT BWSI Autonomous RACECAR
MIT License
bwsix RC101 - Fall 2023

File Name: lab_g.py

Title: Lab G - Line Follower with Safety Stop

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. When the RACECAR sees
a white cone at the end of the course, it must stop automatically before hitting the cone.

Note: You may copy code from Lab F to complete Lab G. You are expected to build on top
of your progress and add state machine logic and safety stop features to your code.
There is no template code in this document to follow except for the RACECAR script structure
found in template.py. The Grand Prix will be very similar to this lab. If you have time,
try optimizing your line following algorithm. Good luck!

Expected Outcome: When the user runs the script, they must not be able to manually control
the RACECAR. The RACECAR must move forward on its own, traverse through the course, and then
stop on its own.
- The speed of the RACECAR can be controlled by a state machine or script, but not by the user
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
- The RACECAR must stop before the white cone at the end of the course. The RACECAR must stop
close enough to the cone such that it does not see the entirety of the white cone at the end
of the race. (less than 30 cm). The RACECAR must not hit the white cone.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
#FRONT_WINDOW = (-10, 10)
#REAR_WINDOW = (170, 190)
angle_values = []
kp = 0.01
ki = 0
kd = 0.002
pre_error = 0.0
integral = 0.0


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Remove 'pass' and write your source code for the start() function here
    rc.drive.stop()


    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():

    angle = 0.0
    speed = 0
    LEFT = -1
    RIGHT = 1
    mode = 0
    global pre_error
    global integral
    before = 0
    
    # Use the triggers to control the car's speed
    """
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    if rc.controller.is_down(rc.controller.Button.X):
        speed = 0.5
    """


    scan = rc.lidar.get_samples()

    left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, 128, 38)
    left_error = left_front_dist - left_rear_dist
    right_error = right_front_dist - right_rear_dist
    #error =  right_front_dist - right_rear_dist - left_front_dist + left_rear_dist
    print("lf: ",left_front_dist," rf: ",right_front_dist," lr: ",left_rear_dist," rr: ",right_rear_dist)
    error = 0
    if left_error >= right_error:
        error = left_error * -1
    else:
        error = right_error
    print("left_error: ",left_error," right_error: ",right_error)
    integral += error
    derivative = error - pre_error
    before = kp * error + ki * integral + kd * derivative
    pre_error = error

    if rc.controller.is_down(rc.controller.Button.A):
        

        """
        _, right_wall_dist = rc_utils.get_lidar_closest_point(scan, (10, 75))
        _, left_wall_dist = rc_utils.get_lidar_closest_point(scan, (-75, -10))

        left_front_dist = rc_utils.get_lidar_average_distance(scan, -38, 76)
        right_front_dist = rc_utils.get_lidar_average_distance(scan, 38, 76)
        speed = 0.3
        if right_wall_dist < 30:
            angle = LEFT
            mode = 0
        elif left_wall_dist < 30:
            angle = RIGHT
            mode = 1
        elif right_front_dist >70:
            angle = RIGHT
            mode = 2
        elif left_front_dist > 70:
            angle = LEFT
            mode = 3
        else:
            speed = 0.2
            angle = 0
            mode = 4
        """
        
        angle = before
        if right_front_dist == 0 or right_rear_dist == 0 or right_front_dist >= 400:
            angle = 2 * kp * (- left_front_dist + left_rear_dist)
            mode = 2
        if left_front_dist == 0 or left_rear_dist == 0 or left_front_dist >= 400:
            angle = 2 * kp * (right_front_dist - right_rear_dist)
            mode = 3

        angle_values.append(angle)
            
        angle = np.clip(angle, -1, 1)
        speed = 0.8
        
    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-45, 45))
    print("Forward distance:", forward_wall_dist)
    if forward_wall_dist < 27:
        if right_front_dist - right_rear_dist - left_front_dist + left_rear_dist >= 0:
            angle = LEFT
        else:
            angle = RIGHT
        speed = -0.7
        mode = 4
    
    #print(f"mode {mode}")

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    

    # Print the center and area of the largest contour when B is held down
    # Remove 'pass' and write your source code for the update() function here

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

    #plt.plot(angle_values, color='blue')
    #plt.xlabel('Time')
    #plt.ylabel('Angle')
    #plt.title('Angle over Time')
    #plt.show()

