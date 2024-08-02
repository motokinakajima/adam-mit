import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys
import cv2 as cv

sys.path.insert(1, '/Users/nakajimamotoki/racecar-neo-installer/racecar-student/library')
import racecar_core
import racecar_utils as rc_utils

class ModeManager:
    def __init__(self):
        self.mode = 99

    def update(self, image):
        scan_mode, _ = self.detect_marker(image)

        if scan_mode != 99:
            self.mode = scan_mode

        return self.mode
    
    def detect_marker(self, image):
        first_id = 99
        square = None

        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
        if markerIds is not None and markerCorners is not None and len(markerCorners) > 0:
            ids = markerIds
            corner = markerCorners[0]
            
            # Ensure there are enough corners to calculate the area
            if len(corner[0]) == 4:
                square = (abs((corner[0][2][0] - corner[0][1][0]) * (corner[0][0][1] - corner[0][1][1])
                            - (corner[0][0][0] - corner[0][1][0]) * (corner[0][2][1] - corner[0][1][1]) +
                            abs((corner[0][0][1] - corner[0][3][0]) * (corner[0][2][1] - corner[0][3][1])
                                - (corner[0][2][0] - corner[0][3][0]) * (corner[0][0][1] - corner[0][3][1])))) / 2
                if square > 30:
                    first_id = ids[0][0]
                else:
                    print("AR too small")
            else:
                print("Not enough corners detected")

        if markerIds is None:
            print("NO MARKER")

        return first_id, square

    def get_all_markers(self, image):
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)
        
        return (markerCorners, markerIds)
    
    def get_biggest_marker(self, image):
        markerIds, markerCorners = self.get_all_markers(image)
        biggest_index = None
        max_area = 0  # Initialize max_area to track the largest marker

        if markerIds is not None and markerCorners is not None and len(markerCorners) > 0:
            for i, corners in enumerate(markerCorners):
                # Ensure there are enough corners to calculate the area
                if len(corners[0]) == 4:
                    # Calculate the area of the marker using the shoelace formula
                    x0, y0 = corners[0][0]
                    x1, y1 = corners[0][1]
                    x2, y2 = corners[0][2]
                    x3, y3 = corners[0][3]

                    area = abs((x0 * y1 + x1 * y2 + x2 * y3 + x3 * y0) - (y0 * x1 + y1 * x2 + y2 * x3 + y3 * x0)) / 2

                    if area > max_area:
                        max_area = area
                        biggest_index = i

            if biggest_index is not None:
                biggest_marker_id = markerIds[biggest_index][0]
                print(f"Largest marker ID: {biggest_marker_id}, Area: {max_area}")
                return biggest_marker_id
            else:
                print("No sufficiently large marker found")
                return None
        else:
            print("No markers detected")
            return None
    
    def find_marker(self, image, targetId):
        markerIds, markerCorners = self.get_all_markers(image)
        for i, markerId in enumerate(markerIds):
            # Handle markerId as array or list
            if isinstance(markerId, np.ndarray):
                markerId = markerId.flatten()  # Flatten in case of multi-dimensional array
            if isinstance(markerId, (list, np.ndarray)):
                markerId = markerId[0] if len(markerId) > 0 else None

            # Handle the case where markerId is still not an integer
            if markerId is None:
                print("markerId is None or empty after extraction")
                continue
            
            if markerId == targetId:
                return markerId, markerCorners[i]
        return None, None




    
    def get_center(self, markerCorner):
        if markerCorner is None or len(markerCorner) != 4:
            print("Invalid marker corners")
            return None, None
        
        # Extracting the coordinates
        x_coords = [corner[0][0] for corner in markerCorner]
        y_coords = [corner[0][1] for corner in markerCorner]
    
        # Calculate the center coordinates
        x_center = sum(x_coords) / len(x_coords)
        y_center = sum(y_coords) / len(y_coords)
    
        return x_center, y_center
