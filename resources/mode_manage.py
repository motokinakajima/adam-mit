import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys
import cv2 as cv

sys.path.insert(1, '../../library')
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
        parameters = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)

        # Ensure markerCorners and markerIds are not None and have the expected shapes
        if markerCorners is None:
            markerCorners = []
        if markerIds is None:
            markerIds = []

        return markerCorners, markerIds


    def get_biggest_marker(self, image):
        markerCorners, markerIds = self.get_all_markers(image)
        biggest_index = None
        max_area = 0  # Initialize max_area to track the largest marker

        if markerIds is None or markerCorners is None:
            print("Illegal input")
            return None

        #print("markerCorners:", markerCorners)  # Print entire markerCorners to debug

        if len(markerCorners) > 0:
            for i, corners in enumerate(markerCorners):
                #print(f"corners[{i}]: {corners}")

                # Handle the shape of corners
                if corners.shape == (1, 4, 2):
                    corners = corners[0]  # Extract the first element (4 corners)

                    # Ensure corners has exactly 4 points
                    if corners.shape == (4, 2):
                        # Calculate the area of the marker using the shoelace formula
                        x0, y0 = corners[0]
                        x1, y1 = corners[1]
                        x2, y2 = corners[2]
                        x3, y3 = corners[3]

                        area = abs((x0 * y1 + x1 * y2 + x2 * y3 + x3 * y0) - (y0 * x1 + y1 * x2 + y2 * x3 + y3 * x0)) / 2

                        if area > max_area:
                            max_area = area
                            biggest_index = i
                    else:
                        print(f"Invalid number of corners: {corners.shape}")
                else:
                    print(f"Unexpected corners shape: {corners.shape}")
                    continue

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
        markerCorners, markerIds = self.get_all_markers(image)

        # Print types and contents for debugging
        #print(f"Type of markerIds: {type(markerIds)}")
        #print(f"Raw markerIds: {markerIds}")
        #print(f"Type of markerCorners: {type(markerCorners)}")
        #print(f"Raw markerCorners: {markerCorners}")

        # Convert markerIds from tuple if necessary
        if isinstance(markerIds, tuple):
            markerIds = markerIds[0] if len(markerIds) > 0 else np.array([])

        markerIds = np.array(markerIds).flatten() if markerIds is not None else np.array([])

        # Convert markerCorners from tuple if necessary
        if isinstance(markerCorners, tuple):
            markerCorners = markerCorners[0] if len(markerCorners) > 0 else np.array([])

        # Ensure markerCorners is a NumPy array
        if not isinstance(markerCorners, np.ndarray):
            markerCorners = np.array(markerCorners)

        # Check for empty markerCorners
        if len(markerCorners) == 0:
            print("No marker corners detected")
            return None, None

        # Iterate over markerIds to find the targetId
        for i, markerId in enumerate(markerIds):
            print(f"Raw markerId at index {i}: {markerId}")

            if markerId == targetId:
                # Ensure the index exists in markerCorners
                if i < len(markerCorners):
                    print(f"Found marker ID: {markerId} with corners: {markerCorners[i]}")
                    return markerId, markerCorners[i]
                else:
                    print(f"Index {i} out of bounds for markerCorners")
                    return None, None

        print(f"Marker ID {targetId} not found")
        return None, None

    def get_center(self, markerCorner):
        if markerCorner is None or len(markerCorner) != 4:
            print("Invalid marker corners")
            return None, None

        try:
            # Ensure markerCorner is a NumPy array for consistent indexing
            if isinstance(markerCorner, tuple):
                markerCorner = markerCorner[0]

            # Extracting the coordinates
            x_coords = [corner[0] for corner in markerCorner]
            y_coords = [corner[1] for corner in markerCorner]

            # Calculate the center coordinates
            x_center = sum(x_coords) / len(x_coords)
            y_center = sum(y_coords) / len(y_coords)

            print(x_center)

            return x_center, y_center
        except Exception as e:
            print(f"Error calculating center: {e}")
            return None, None

    def get_ar_distance(self, area, a_fit=3611.56, b_fit=17.91):
        return a_fit / area**0.5 + b_fit
