import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys


sys.path.insert(1, '../../../library')
import racecar_core
import racecar_utils as rc_utils

def group_lidar_data(lidar_data, max_diff=10, min_group_size=5):
    groups = []
    current_group = []
    start_index = 0
    group_indices = []

    for i in range(len(lidar_data)):
        if i == 0 or abs(lidar_data[i] - lidar_data[i - 1]) <= max_diff or lidar_data[i] == 0 or lidar_data[i - 1] == 0:
            if not current_group:
                start_index = i
            current_group.append(lidar_data[i])
        else:
            if len(current_group) >= min_group_size:
                groups.append(current_group)
                group_indices.append((start_index, i - 1))
            current_group = [lidar_data[i]]
            start_index = i
    if len(current_group) >= min_group_size:
        groups.append(current_group)
        group_indices.append((start_index, len(lidar_data) - 1))

    #print(f'Grouped data: {groups}')
    #print(f'Group indices: {group_indices}')

    return groups, group_indices


def find_furthest_group(groups, group_indices):
    if not groups:
        raise ValueError("No groups found. Ensure your grouping parameters are correct.")

    if not group_indices:
        raise ValueError("No group indices found. Ensure your grouping function is working properly.")

    # Find the index of the group with the maximum mean distance
    furthest_group_index = max(range(len(groups)), key=lambda i: np.mean(groups[i]) if groups[i] else 0)
    furthest_group = groups[furthest_group_index]
    start_index, end_index = group_indices[furthest_group_index]
    return furthest_group, start_index, end_index

def find_best_group(groups, group_indices, lidar_data):
    if not groups or not group_indices:
        raise ValueError("No groups or indices found. Ensure your grouping function is working properly.")

    # Calculate average distance
    all_distances = [np.mean(group) for group in groups if group]
    average_distance = np.mean(all_distances) if all_distances else 0

    # Define a function to evaluate the suitability of a group
    def evaluate_group(i):
        group = groups[i]
        start_index, end_index = group_indices[i]
        mean_distance = np.mean(group) if group else 0
        group_length = end_index - start_index + 1

        # Calculate distance from the middle index (180 degrees)
        middle_index = len(lidar_data) // 2
        distance_from_middle = abs((start_index + end_index) / 2 - middle_index)

        # Evaluate the group: prefer wide groups with distances higher than average
        score = mean_distance * 0.5 - distance_from_middle * 0.3
        return score, mean_distance, group_length

    # Find the best group based on the evaluation
    best_group_index = max(range(len(groups)), key=lambda i: evaluate_group(i)[0])
    best_group = groups[best_group_index]
    start_index, end_index = group_indices[best_group_index]

    return best_group, start_index, end_index
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

class WallFollow3:
    def __init__(self, kp, ki, kd):
        self.pid = PIDController(kp, ki, kd)
        self.pid.start()

    def update(self, scan):
        # Extract data from -90 to 90 degrees
        lidar_data = np.append(scan[540:720], scan[0:180])
        #print(f'lidar_data shape: {lidar_data.shape}')

        #print(f'LiDAR Data: {lidar_data}')

        # Group the LiDAR data
        groups, group_indices = group_lidar_data(lidar_data, 20, 10)

        #print(f'Groups: {groups}')
        #print(f'Group Indices: {group_indices}')

        try:
            # Find the furthest group
            furthest_group, start_index, end_index = find_best_group(groups, group_indices, lidar_data)

            #print(f'Furthest Group: {furthest_group}')
            print(f'Start Index: {start_index}, End Index: {end_index}')

            # Calculate angle
            angle = self.pid.update((start_index + end_index) / 2, 180)
            return 1, angle
        except ValueError as e:
            print(f'Error: {e}')
            return 1, 0  # Default angle when there's an error

class SimpleWallFollow():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pid = PIDController(kp, ki, kd)
        self.pid.start()
    def update(self, scan):
        right_distance = rc_utils.get_lidar_average_distance(scan, 60)
        left_distance = rc_utils.get_lidar_average_distance(scan, 300)
        setpoint = right_distance - left_distance
        angle = self.pid.update(setpoint, 0)
        return 1, angle
