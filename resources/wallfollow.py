import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys


sys.path.insert(1, '../../../../racecar-neo-installer/racecar-student/library')
import racecar_core
import racecar_utils as rc_utils

#rc = racecar_core.create_racecar()

class WallFollow:
    def __init__(self, kp_angle, ki_angle , kd_angle, kp_dist, ki_dist ,kd_dist, speed, goal_dist):
        self.kp_angle = kp_angle
        self.ki_angle = ki_angle
        self.kd_angle = kd_angle
        self.kp_dist = kp_dist
        self.ki_dist = ki_dist
        self.kd_dist = kd_dist
        self.angle_pid = PIDController(kp_angle, ki_angle, kd_angle)
        self.dist_pid = PIDController(kp_dist, ki_dist, kd_dist)
        
        self.angle_pid.start()
        self.dist_pid.start()

        self.speed = speed
        self.goal_dist = goal_dist
        self.prev = 0

    

    def update(self, scan):

        _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
        _, r_min = rc_utils.get_lidar_closest_point(scan, (90, 270))

        error_angle = self.lidar_angle(scan, 45)
        
        """
        if abs(error_angle - self.prev) > 20:
            error_a1 = self.prev
        """
        
        error_angle = np.nan_to_num(error_angle)
        self.prev = error_angle
        print(error_angle)
        
        angle_a = self.angle_pid.update(0,error_angle)
        angle_d = self.dist_pid.update(self.goal_dist,r_min)

        print(f"r_min : {r_min}")

        angle_d = np.tanh(angle_d)

        angle = angle_a + angle_d
        angle = np.clip(angle, -1, 1)

        return self.speed, angle
    

    def lidar_angle(self,scan,angle,right=True):
        #self.scan = scan
        if right:
            target = 90
        else:
            target = 270

        front = target - angle
        rear = target + angle

        #print(self.scan)


        forward_point = np.array([scan[front * 2] * np.cos(np.pi * angle / 180), scan[front * 2] * np.sin(np.pi * angle / 180 )])
        rear_point = np.array([scan[rear * 2] * np.cos(-np.pi * angle / 180), scan[rear * 2] * np.sin(-np.pi * angle / 180)])
        dif = forward_point - rear_point
        tan = dif[1] / dif[0]
        angle = 90 - abs(np.arctan(tan) * 180 / np.pi)
        if forward_point[0] < rear_point[0]:
            angle *= -1

        return angle



class WallFollow2:
    def __init__(self, kp, ki , kd, speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.pid = PIDController(kp, ki, kd)
    
        self.pid.start()

        self.speed = speed

    def update(self, scan):

        left_front_dist = rc_utils.get_lidar_average_distance(scan, -52, 38)
        right_front_dist = rc_utils.get_lidar_average_distance(scan, 52, 38)
        left_rear_dist = rc_utils.get_lidar_average_distance(scan, -128, 38)
        right_rear_dist = rc_utils.get_lidar_average_distance(scan, 128, 38)
        left_error = left_front_dist - left_rear_dist
        right_error = right_front_dist - right_rear_dist
        #error =  right_front_dist - right_rear_dist - left_front_dist + left_rear_dist
        error = 0
        if left_error >= right_error:
            error = left_error * -1
        else:
            error = right_error

        angle = self.pid.update(0,error)
        angle = np.clip(angle, -1, 1)

        return self.speed, angle
    
