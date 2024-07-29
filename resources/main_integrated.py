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

sys.path.insert(1, '../../../library')
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
BLUE_LINE = ((76, 53, 26), (126, 255, 255))
BLUE_CONE = ((74, 98, 0), (118, 255, 255))

prev_id = None
first_id = None
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
cone_slalom_speed = -0.3
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
line_follow_speed = -0.1
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
#kp_a1 = 1 / 300
kp_a1 = 1 / 300
ki_a1 = 0
kd_a1 = 1 / 1000
kd_a1 = 0

kp_dist = 1 / 100
ki_dist = 0
kd_dist = 1 / 1000


pid_angle_1 = PID(kp_a1,ki_a1,kd_a1)
pid_dist = PID(kp_dist,ki_dist,kd_dist)

goal_dis = 50
limit_dist = 250
angle = 0
speed = 0
turn_dis = 250
#turn_dis = 300
angle_values = []
prev = 0
cnt = 0
normal_speed = -0.3
turn_speed = -0.2
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

    return angle

def cone_distance(area_cone, area_cone_min):
    a_fit = 2174.29
    b_fit = 0.07838
    if area_cone >= area_cone_min:
        dist = a_fit / area_cone ** 0.5 + b_fit
    else:
        dist = 500
    return dist

def cone_get_angle(y, danger_distance, safe_distance, yellow_arrow):
    if y <= danger_distance:
        yellow_arrow -= 300
        angle = yellow_arrow*(0.3/320)
        print("danger_dist",yellow_arrow)
        #「0.6」は要調整
    elif y <= safe_distance:
        angle = yellow_arrow*(0.4/320)
        print("safe_dist")
        #「0.3」は要調整
    else:
        angle = 0
    angle = np.clip(angle, -1, 1) 
    return angle

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
    return (center,area)

def cone_slalom(center, area):
    global cone_slalom_speed
    global cone_slalom_integral_dif
    global cone_slalom_spent_time
    blue_area = 0
    blue_center = None
    blue_area = area
    blue_center = center
    y = 0
    gap = 290
    danger_zone = 70
    safe_zone = 200
    _angle = 0.0
    if not blue_area == None:
        print("blue found!!")
    print(" blue:",blue_area)
    if not blue_center == None:
        print("blue[0]:",blue_center[0],"blue[1]:",blue_center[1])

    if (not blue_area == None):
        y = cone_distance(blue_area,30)
        print("distance: ",y)
        if not blue_center == None:
            print("angle: ",cone_get_angle(y,danger_zone,safe_zone,blue_center[1] - 160))
            _angle = cone_get_angle(y,danger_zone,safe_zone,blue_center[1] - 160)
            print("Center:", blue_center, "Area:", blue_area)
    rc.drive.set_speed_angle(cone_slalom_speed, _angle)

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

def line_follow(center,area):
    global image
    global line_follow_speed
    global line_follow_pre_error
    global line_follow_integral
    if center is not None:
        error = rc_utils.remap_range(center[1], 0, 320, -1, 1)
    else:
        error = 0.0
        print("NO CONTOUR")
    line_follow_integral += error
    if center is None:
        line_follow_integal = 0
    derivative = error - line_follow_pre_error
    angle = line_follow_kp * error + line_follow_ki * line_follow_integral + line_follow_kd * derivative
    # angle = kp * error
    print("angle:", angle)
    line_follow_pre_error = error
    angle = np.clip(angle, -1, 1)


    #rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    #lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    #speed = rt - lt

    rc.drive.set_speed_angle(line_follow_speed, angle)

def detect_marker():
    global image
    global ar_markers
    global prev_id
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters =  cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
    ar_markers = cv.aruco.detectMarkers(image, cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250),
                                        parameters=cv.aruco.DetectorParameters_create())
    if markerIds is not None:
        if markerCorners is not None:
            ids = markerIds
            corner = markerCorners[0]
            # square = abs((corner[0][0]- corner[2][0]) * (corner[0][1]- corner[2][1]))
            square = (abs((corner[2][0] - corner[1][0]) * (corner[0][1] - corner[1][1])
                          - (corner[0][0] - corner[1][0]) * (corner[2][1] - corner[1][1]) +
                          abs((corner[0][1] - corner[3][0]) * (corner[2][1] - corner[3][1])
                              - (corner[2][0] - corner[3][0]) * (corner[0][1] - corner[3][1])))) / 2
            print(square)
            if square > 30:
                print("DETECTED")
                first_id = ids[0][0]
            else:
                first_id = prev_id
                print("AR too samll")

        else:
            first_id = prev_id
    else:
        first_id = prev_id

    prev_id = first_id
    if ar_markers is None:
        print("NO MARKER")

    return first_id
#↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑


# start
def start():
    pass
prev_cone_detected = False

# update
#↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
def update():
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
    (cone_center,cone_area) = update_contour(image,BLUE_CONE,CONE_CROP_FLOOR)

    first_id = detect_marker()
    
    mode = 99
    
    if image is None:
        print("NO IMAGE")
        mode = None
        
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


