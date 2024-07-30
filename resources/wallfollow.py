import numpy as np
from pid import PIDController
import os
from dotenv import load_dotenv
from pathlib import Path
import sys

dotenv_path = Path('../.env')
load_dotenv(dotenv_path=dotenv_path)
LIBRARY_PASS = os.getenv('LIBRARY_PASS')
sys.path.insert(1, LIBRARY_PASS)
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

def lidar_angle(scan,angle,right=True):
    if right:
        target = 90
    else:
        targert = 270

    front = target - angle
    rear = target + angle

    forward_point = np.array([scan[front * 2] * np.cos(np.pi * angle / 180), scan[front * 2] * np.sin(np.pi * angle / 180 )])
    rear_point = np.array([scan[rear * 2] * np.cos(-np.pi * angle / 180), scan[rear * 2] * np.sin(-np.pi * angle / 180)])
    dif = forward_point - rear_point
    tan = dif[1] / dif[0]
    angle = 90 - abs(np.arctan(tan) * 180 / np.pi)
    if forward_point[0] < rear_point[0]:
        angle *= -1

    return angle


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

    def update(self):

        scan = rc.lidar.get_samples()

        _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
        _, r_min = rc_utils.get_lidar_closest_point(scan, (90, 270))

        error_angle = self.angle_pid.update(0,lidar_angle(scan, 45))
        
        """
        if abs(error_angle - self.prev) > 20:
            error_a1 = self.prev
        """
        
        error_angle = np.nan_to_num(error_angle)
        self.prev = error_angle
        
        angle_a = self.angle_pid.update(0,lidar_angle(scan, 45))
        angle_d = self.dist_pid.update(self.goal_dist,r_min)

        angle_d = np.tnah(angle_d)

        angle = angle_a + angle_d
        angle = np.clip(angle, -1, 1)

        return self.speed, angle

