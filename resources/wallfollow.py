import numpy as np
from pid import PIDController



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


class WallFollow:
    def __init__(self, angle_kp, angle_ki ,angle_kd, dist_kp, dist_ki ,dist_kd, speed, goal_dist):
        self.angle_pid = PIDController(angle_kp, angle_ki, angle_kd)
        self.dist_pid = PIDController(dist_kp, dist_ki, dist_kd)

        self.speed = speed
        self.goal_dist = goal_dist
        self.prev = 0

    def update(self):

        scan = rc.lidar.get_samples()

        _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
        _, r_min = rc_utils.get_lidar_closest_point(scan, (90, 270))

        error_angle = lidar_angle(scan, 45)

        if abs(error_angle - self.prev) > 20:
            error_a1 = self.prev 
        error_angle = np.nan_to_num(error_angle)
        self.prev = error_angle

        error_dist = (r_min - self.goal_dist) / self.goal_dist

        angle_a = np.tanh(error_angle)
        angle_d = np.tnah(error_dist)

        angle = angle_a + angle_d
        angle = np.clip(angle, -1, 1)

        return self.speed, angle

