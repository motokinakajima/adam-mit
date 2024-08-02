import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys


sys.path.insert(1, '../../library')
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


class WallFollow2:
    def __init__(self, kp1, ki1 , kd1, kp2, ki2, kd2, kp3, ki3,kd3, speed):
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

    def update(self, scan):

        left_front_dist = rc_utils.get_lidar_average_distance(scan, -45, 10)
        right_front_dist = rc_utils.get_lidar_average_distance(scan, 45, 10)
        left_rear_dist = rc_utils.get_lidar_average_distance(scan, -135, 10)
        right_rear_dist = rc_utils.get_lidar_average_distance(scan, 135, 10)
        left_error = left_front_dist - left_rear_dist
        right_error = right_front_dist - right_rear_dist
        error1 =  right_front_dist - right_rear_dist - left_front_dist + left_rear_dist
        """
        error = 0
        if left_error >= right_error:
            error1 = left_error * -1
        else:
            error1 = right_error
        """

        angle1 = self.pid1.update(0,error1)
        angle1 = np.clip(angle1, -1, 1)

        forward_scan = np.append(scan[-180:],scan[:180])
        index = np.argmax(forward_scan)

        error2 = index - 90
        angle2 = self.pid2.update(0,error2)
        angle2 = np.clip(angle2, -1, 1)

        _, r_min = rc_utils.get_lidar_closest_point(scan, (45, 135)) 
        _, l_min = rc_utils.get_lidar_closest_point(scan, (225, 315))
        print("r_min",r_min)
        print("l_min",l_min)
        error3 = r_min - l_min
        angle3 = self.pid3.update(0,error3)
        print("error",error3)
        angle3 = np.clip(angle3, -1, 1)

        angle = self.ratio[0] * angle1 + self.ratio[1] * angle2 + self.ratio[2] * angle3
        angle = np.clip(angle, -1, 1)
        print(angle1,angle2,angle3)
        return self.speed, angle

    def set_ratio(self,ratio):
        self.ratio = ratio
