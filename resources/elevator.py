# initialize
import sys
import math
import copy
import cv2 as cv
import numpy as np
import ipywidgets as widgets
import statistics
from nptyping import NDArray
from typing import Any, Tuple, List, Optional
from enum import Enum


print(np.__version__)
sys.path.insert(1, '/Users/AT/Desktop/racecar-neo-installer/racecar-student/library')
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

#const variables
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
MIN_CONTOUR_AREA = 30
image = None

LINE_CROP_FLOOR = ((180, 0), (240, 320))
CONE_CROP_FLOOR = ((60, 0), (240, 320))

# The HSV range for the color blue
#BLUE_LINE = ((68, 49, 131), (100, 184, 255))
ORANGE_LINE = ((11, 64, 143), (16, 255, 255))
RED_LINE = ((0, 64, 140), (10, 255, 255))
BLUE_LINE = ((63, 34, 143), (129, 206, 229))

prev_id = None
first_id = None

elevator_set = True

elevator_wait = True

RED_ORANGE_SEEN = False
prev_light = None

#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


# classes
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
class PID:
    def __init__(self, Kp, Ki, Kd, dt=1/60):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.previous_error = 0
        self.previous_d_error = 0
        self.integral = 0
        self.dt = dt
    
    def update(self, error):
        dt = self.dt
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral - self.Kd * derivative
        self.previous_error = error
        self.previous_d_error = derivative
        return output


class Adam:
    def __init__(self,alpha,beta):
        self.alpha = alpha
        self.beta = beta
        self.epsilon = 0.00001
        self.v = 0
        self.s = 0

    def update(self, error):
        alpha = self.alpha
        beta = self.beta
        epsilon = self.epsilon

        self.v = beta * self.v + (1 - beta) * error
        self.s = beta * self.s + (1 - beta) * error ** 2
        output = alpha * (self.v / np.sqrt(self.s + epsilon))

        return output
    

class Momentum:
    def __init__(self,alpha,beta):
        self.alpha = alpha
        self.beta = beta
        self.v = 0

    def update(self, error):
        alpha = self.alpha
        beta = self.beta

        self.v = beta * self.v + (1 - beta) * error
        output = alpha * self.v

        return output
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


#cone slalom variables
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
cone_slalom_speed = 0.3
cone_slalom_integral_dif = 0
cone_slalom_spent_time = 0
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


#wall_follow_speed = -0.3
#wall_follow_integral = 0.0
#wall_follow_pre_error = 0.0
#wall_follow_kp = 0.005
#wall_follow_ki = 0
#wall_follow_kd = 0.0001


#line follow variables
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
line_follow_speed = 0.1
line_follow_contour_center = None
line_follow_contour_area = 0
line_follow_integral = 0.0
line_follow_pre_error = 0.0
line_follow_kp = 0.4
line_follow_ki = 0
line_follow_kd = 0.01 #0.2
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


#wallfollow variables
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

kp_a1 = 1 / 300
ki_a1 = 0
kd_a1 = 1 / 1000


kp_dist = 1 / 500
ki_dist = 0
kd_dist = 1 / 2000


pid_angle_1 = PID(kp_a1,ki_a1,kd_a1)
pid_dist = PID(kp_dist,ki_dist,kd_dist)

goal_dis = 75
limit_dist = 250
angle = 0
speed = 0
turn_dis = 250
#turn_dis = 300
angle_values = []
prev = 0
cnt = 0
normal_speed = 1.0
turn_speed = 0.2
turn_angle = -1
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    
#functions
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
def sigmoid(x):
    a = 1 + np.exp(-x)
    return (1/a) * 2 - 1

def lidar_angle(scan,angle):
    front = 90 - angle
    rear = 90 + angle
    forward_point = np.array([scan[front * 2] * np.cos(np.pi * angle / 180), scan[front * 2] * np.sin(np.pi * angle / 180 )])
    rear_point = np.array([scan[rear * 2] * np.cos(-np.pi * angle / 180), scan[rear * 2] * np.sin(-np.pi * angle / 180)])
    dif = forward_point - rear_point
    tan = dif[1] / dif[0]
    angle = 90 - abs(np.arctan(tan) * 180 / np.pi)
    if forward_point[0] < rear_point[0]:
        angle *= -1




def update_contour(_image,color,crop_floor):
    center = None
    area = 0
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
    return (center,area)



def wall_follow():
    global goal_dis, limit_dist, turn_dis, prev, angle, speed
    global normal_speed, turn_speed, turn_angle
    
    scan = rc.lidar.get_samples()
    
    #left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    #left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    right_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)

    _, r_min = rc_utils.get_lidar_closest_point(scan, (90, 270))

    #for i in r_list:
    #    if i != 0 ad 
    
    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))

    error_a1 = lidar_angle(scan, 45)

    if abs(error_a1 - prev) > 20:
        error_a1 = prev
    error_a1 = np.nan_to_num(error_a1)

    error_a2 = 0
    error_d = 0
    #error_a2 = right_front_dist - right_rear_dist

    error_d = r_min - goal_dis

    #angle_1 = momentum_a1.update(error_a1)
    #angle_2 = momentum_a2.update(error_a2)
    #dist = momentum_d.update(error_d)

    angle_1 = error_a1
    angle_2 = error_a2
    dist = error_d

    angle_1 = pid_angle_1.update(angle_1)

    #angle_1 = np.tanh(angle_1)
    angle_1 = np.clip(angle_1, -1, 1)

    angle_dist = pid_dist.update(error_d)

    angle_dist = np.tanh(angle_dist)

    angle = np.clip(angle_1, -1, 1)    
    
    if forward_wall_dist < limit_dist:
        pass
        #goal_dis = 10

    #angle = momentum.update(angle)
    #angle = pid_angle.update(angle)
    #angle = adam.update(angle)

    #print(error_a2)
    #print(pid_angle_2.Kp,pid_angle_2.Kd,pid_angle_2.Ki)

    speed = 0

    speed = normal_speed

    """
        angle_values.append(angle)
    if len(angle_values) > 5:
        angle_values = angle_values[1:]
    angle = average(angle_values)
    """
    if forward_wall_dist < turn_dis:
        angle = turn_angle
        speed = turn_speed
        print("teurning")

    else:
        print("normal")

    if forward_wall_dist < 30:
        pass
        #speed = 0.1
        #angle = 0

    """ 
    if angle != 0:
        pid_angle_1.Kp = 1 / (speed * 1000)
    else:
        pid_angle_1.Kp = 1 / 300
    angle_1 = pid_angle_1.update(angle_1)
    """
    angle = np.clip(angle,-1,1)
    angle = np.nan_to_num(angle)

    rc.drive.set_speed_angle(speed, angle)
        
    print(f"angle:{error_a1}")
    print("angle",angle)
    print("==========================================")
    print()

def wall_follow2():
    global goal_dis, limit_dist, turn_dis, prev, angle, speed
    global normal_speed, turn_speed, turn_angle
    
    scan = rc.lidar.get_samples()
    
    left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
    #right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
    left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
    #right_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)

    _, l_min = rc_utils.get_lidar_closest_point(scan, (-60, -120))

    print(f"left_front_dist: {left_front_dist}")
    print(f"left_rear_dist: {left_rear_dist}")
    print(f"l_min: {l_min}")

    #for i in r_list:
    #    if i != 0 ad 
    
    _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))

    error_1 = left_rear_dist - left_front_dist
    error_2 = goal_dis - l_min

    angle_1 = pid_angle_1.update(error_1)
    angle_2 = pid_dist.update(error_2)

    angle = np.clip(angle_1 + angle_2, -1, 1)
    speed = normal_speed
    rc.drive.set_speed_angle(speed, angle)


def detect_marker():
    global image
    global ar_markers
    global prev_id
    first_id = 0
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters =  cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
    if markerIds is not None and markerCorners is not None and len(markerCorners) > 0:
        ids = markerIds
        corner = markerCorners[0]
        print(ids)
        print(corner)
        
        # Ensure there are enough corners to calculate the area
        if len(corner[0]) == 4:
            square = (abs((corner[0][2][0] - corner[0][1][0]) * (corner[0][0][1] - corner[0][1][1])
                          - (corner[0][0][0] - corner[0][1][0]) * (corner[0][2][1] - corner[0][1][1]) +
                          abs((corner[0][0][1] - corner[0][3][0]) * (corner[0][2][1] - corner[0][3][1])
                              - (corner[0][2][0] - corner[0][3][0]) * (corner[0][0][1] - corner[0][3][1])))) / 2
            print(square)
            if square > 30:
                print("DETECTED")
                first_id = ids[0][0]
            else:
                print("AR too small")
        else:
            print("Not enough corners detected")

    prev_id = first_id
    if markerIds is None:
        print("NO MARKER")

    return first_id

def AR32():
    global image
    global ar_markers
    global prev_id
    global prev_light
    first_id = 0
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters =  cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    center = None
    
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
    if markerIds is not None and markerCorners is not None and len(markerCorners) > 0:
        ids = markerIds
        corner = markerCorners[0]
        print(ids)
        print(corner)
        
        # Ensure there are enough corners to calculate the area
        if len(corner[0]) == 4:
            square = (abs((corner[0][2][0] - corner[0][1][0]) * (corner[0][0][1] - corner[0][1][1])
                          - (corner[0][0][0] - corner[0][1][0]) * (corner[0][2][1] - corner[0][1][1]) +
                          abs((corner[0][0][1] - corner[0][3][0]) * (corner[0][2][1] - corner[0][3][1])
                              - (corner[0][2][0] - corner[0][3][0]) * (corner[0][0][1] - corner[0][3][1])))) / 2
            print(square)
            center_x = np.mean(corner[:, 0])
            center_y = np.mean(corner[:, 1])
            center = (center_x, center_y)
            print("Center:", center)
            if square > 30:
                print("DETECTED")
                first_id = ids[0][0]
            else:
                print("AR too small")
        else:
            print("Not enough corners detected")
    
    _, blue_area = update_contour(image, BLUE_LINE, ((0, 0), (480, 640)))
    _, red_area = update_contour(image, RED_LINE, ((0, 0), (480, 640)))
    _, orange_area = update_contour(image, ORANGE_LINE, ((0, 0), (480, 640)))
    detected_light = max([blue_area, red_area, orange_area])

    prev_id = first_id
    prev_light = detected_light
    if markerIds is None:
        print("NO MARKER")

    return center, detected_light
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


# start
def start():
    pass
prev_cone_detected = False

# update
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
def update():
    global elevator_wait
    global elevator_set
    global RED_ORANGE_SEEN
    global prev_id
    global first_id
    global ar_markers
    global image
    global prev_cone_detected
    image = rc.camera.get_color_image()
    line_center = None
    line_area = 0.0
    cone_center = None
    cone_area = 0.0
    (line_center,line_area) = update_contour(image,BLUE_LINE,LINE_CROP_FLOOR)
    #(cone_center,cone_area) = update_contour(image,BLUE_CONE,CONE_CROP_FLOOR)

    first_id = detect_marker()
    
    mode = 99
    
    if image is None:
        print("NO IMAGE")
        mode = None
    """
    if prev_id == 0:
        print("mode changed to 0: wall following")
        wall_follow()
        mode = 0
        
    elif prev_id == 2:
        print("mode changed to 2: line following")
        if line_center is not None:
            line_follow(line_center,line_area)
            print(line_center)
        else:
            print("NO CONTOUR DETECTED")
            wall_follow
        mode = 2

    elif prev_id == 1:
        print("mode chnaged to 1: cone slalom")
        if cone_center is not None:
            cone_slalom(cone_center,cone_area)
            print(cone_center)
            prev_cone_detected = True
        else:
            if prev_cone_detected == True:
                mode = 0
                print("NO CONE DETECTED")
                prev_id = 0
                return
            wall_follow()
        mode = 1
    """
    if prev_id == 3:
        wall_follow2()

        mode = 3

        elevator_set = True

    else:
        pass

    if elevator_wait:
        print("======waiting for the elevator...=========")
        center, light = AR32()
        if RED_ORANGE_SEEN:
            if prev_light == RED_LINE and light == BLUE_LINE:
                print("Move into the elevator")
                wall_follow2()
            elif light != BLUE_LINE:
                RED_ORANGE_SEEN = True



    """
    elif rc.controller.is_down(rc.controller.Button.X):
        print("mode changed to 3")
        speed2 = -0.1
        print("你好")
        mode = 3
C
    elif rc.controller.is_down(rc.controller.Button.Y):
        print("mode changed to 4")
        speed2 = 0.1
        print("Wazzup")
        mode = 4
        rc.drive.set_speed_angle(speed2, 0)
    """
    print("mode :",mode)

    prev_id = first_id
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


# update slow
def update_slow():
    pass


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()


