import cv2
from mode_manage import *
from pid import *
import sys

sys.path.insert(1, '../../../libraryvim')
import racecar_core
import racecar_utils as rc_utils


def _find_contours(image, lower_hsv, upper_hsv):
    # Create a mask for the given HSV range
    mask = cv2.inRange(image, lower_hsv, upper_hsv)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours

def _update_contour(_image, color_ranges, MIN_CONTOUR_AREA=200):
    hsv_image = cv2.cvtColor(_image, cv2.COLOR_BGR2HSV)

    all_contours = []

    for lower_hsv, upper_hsv in color_ranges:
        # Find contours for each color range
        contours = _find_contours(hsv_image, lower_hsv, upper_hsv)
        all_contours.extend(contours)  # Combine all contours

    # Filter out small contours
    large_contours = [cnt for cnt in all_contours if cv2.contourArea(cnt) >= MIN_CONTOUR_AREA]

    if large_contours:
        # Find the largest contour
        largest_contour = max(large_contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(largest_contour)

        # Compute the center of the largest contour
        moments = cv2.moments(largest_contour)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            contour_center = (cx, cy)
        else:
            contour_center = None

    else:
        contour_center = None
        contour_area = 0

    return contour_center, contour_area

class ArFollow:
    def __init__(self, kp, ki, kd, target_id):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pid = PIDController(kp, ki, kd)
        self.target_id = target_id
        self.pid.start()

    def update(self, image, target_x=320):
        detector = ModeManager()

        # Detect marker and get its corners
        marker_id, marker_corner = detector.find_marker(image, self.target_id)

        # Debug prints for detected marker ID and corners
        print(f"Detected Marker ID: {marker_id}")
        print(f"Detected Marker Corners: {marker_corner}")

        if marker_id is not None and marker_corner is not None:
            # Calculate the center of the detected marker
            center = detector.get_center(marker_corner)

            # Debug print for center coordinates
            print(f"Calculated Center: {center}")

            if center is not None:
                x, y = center
                print(f"Marker Center - x: {x}, y: {y}")
                return self.pid.update(target_x, x)
            else:
                print("Invalid marker corners detected at update")
        else:
            print("Marker not found or invalid marker ID")

        # Handle cases where marker is not found or center is invalid
        print("Using default values due to invalid marker or center")
        return self.pid.update(target_x, target_x)
        # Return a default value or handle accordingly

class elevatorController:
    def __init__(self, kp, ki, kd, wait_distance=150, elevator_distance=80, bypass_speed=1.0, ar_follow_speed=0.5, get_onto_speed=1.0, divider_target_x = 400):
        self.phase = 0
        self.wait_distance = wait_distance
        self.elevator_distance = elevator_distance
        self.bypass_speed = bypass_speed
        self.ar_follow_speed = ar_follow_speed
        self.get_onto_speed = get_onto_speed
        self.target_x = divider_target_x
        self.red_seen = False
        self.detector = ModeManager()
        self.elevator_ar_follow = ArFollow(kp, ki, kd, 32)
        self.divider_ar_follow = ArFollow(kp, ki, kd, 3)

    def reset(self):
        self.phase = 0
        self.red_seen = False
        self.prev_state = -1

    def update(self, image, scan, blueHSV, redHSV):
        speed = 0
        angle = 0
        _, forward_wall_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
        id3_marker, _ = self.detector.find_marker(image, 3)
        if self.phase == 0:
            if id3_marker == None and self.prev_state == -1:
                self.prev_state = -1
                return speed, angle, -1
            if self.detector.get_biggest_marker(image) == 3:
                speed = self.bypass_speed
                angle = self.divider_ar_follow.update(image, self.target_x) * -1
            else:
                self.phase = 1
        elif self.phase == 1:
            if forward_wall_dist > self.wait_distance:
                speed = self.ar_follow_speed
                angle = self.elevator_ar_follow.update(image) * -1
            else:
                self.phase = 2
        elif self.phase == 2:
            blue_center, _ = _update_contour(image, blueHSV)
            red_center, _ = _update_contour(image, redHSV)
            #print(f"blue:center: {blue_center}")
            #print(f"red_center: {red_center}")
            #print(f"red seen: {self.red_seen}")
            #print()
            speed = 0.0
            angle = 0.0
            if self.red_seen == True:
                if blue_center is not None:
                    self.phase = 3
            else:
                if red_center is not None:
                    self.red_seen = True
        elif self.phase == 3:
            if forward_wall_dist > self.elevator_distance:
                speed = self.get_onto_speed
                angle = self.elevator_ar_follow.update(image) * -1
            else:
                self.phase = 4
        elif self.phase == 4:
            if forward_wall_dist < 100:
                speed = 0
                angle = 0
            else:
                self.phase = 5
        elif self.phase == 5:
            self.prev_state = 1
            return speed, angle, 1
        self.prev_state = 0
        return speed, angle, 0
