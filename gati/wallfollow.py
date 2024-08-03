import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys


sys.path.insert(1, '../../../library')
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
    def __init__(self, kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3, speed, ratio,abnormal_ratio, limit_dist):
        self.kp1 = kp1
        self.ki1 = ki1
        self.kd1 = kd1
        self.kp2 = kp2
        self.ki2 = ki2
        self.kd2 = kd2
        self.kp3 = kp3
        self.ki3 = ki3
        self.kd3 = kd3
                
        self.pid1 = PIDController(kp1, ki1, kd1)
        self.pid2 = PIDController(kp2, ki2, kd2)
        self.pid3 = PIDController(kp3, ki3, kd3)
    
        self.pid1.start()
        self.pid2.start()
        self.pid3.start()

        self.speed = speed
        self.ratio = ratio
        self.abnormal_ratio = abnormal_ratio
        self.limit_dist = limit_dist

    def update(self, scan,):
        _, forward_scan = rc_utils.get_lidar_closest_point(scan, (-5, 5))
        if forward_scan > self.limit_dist:
            ratio = self.abnormal_ratio
        else:
            ratio = self.ratio

        left_front_dist = rc_utils.get_lidar_average_distance(scan, -45, 10)
        right_front_dist = rc_utils.get_lidar_average_distance(scan, 45, 10)
        left_rear_dist = rc_utils.get_lidar_average_distance(scan, -135, 10)
        right_rear_dist = rc_utils.get_lidar_average_distance(scan, 135, 10)
        #left_error = left_front_dist - left_rear_dist
        #right_error = right_front_dist - right_rear_dist
        error1 =  right_front_dist - right_rear_dist - left_front_dist + left_rear_dist
        """
        error = 0
        if left_error >= right_error:
            error = left_error * -1
        else:
            error = right_error
        """

        angle1 = self.pid1.update(0,error1)
        angle1 = np.clip(angle1, -1, 1)

        forward_scan = scan[-180:] + scan[:180]
        index = np.argmax(forward_scan)

        error2 = index - 90

        angle2 = self.pid2.update(0,error2)
        angle2 = np.clip(angle2, -1, 1)

        r_dist = rc_utils.get_lidar_average_distance(scan, 90, 10)
        l_dist = rc_utils.get_lidar_average_distance(scan, -90, 10)
        error3 = r_dist - l_dist

        angle3 = self.pid3.update(0,error3)
        angle3 = np.clip(angle3,-1,1)

        angle = ratio[0] * angle1 + ratio[1] * angle2 + ratio[2] * angle3
        angle = np.clip(angle,-1,1)
        print(angle1,angle2,angle3)

        left_limit_dist = rc_utils.get_lidar_average_distance(scan, -30, 20)
        right_limit_dist = rc_utils.get_lidar_average_distance(scan, 30, 20)
        """
        if left_limit_dist < self.limit_dist / 3:
            angle += 0.4

        if right_limit_dist < self.limit_dist / 3:
            angle -= 0.4
        """        
        return self.speed, angle


    def set_ratio(self, ratio):
        self.ratio = ratio