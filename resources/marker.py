import numpy as np
from pid import PIDController
import os
from pathlib import Path
import sys
import cv2 as cv

sys.path.insert(1, '../../../library')
import racecar_core
import racecar_utils as rc_utils



def detect_marker(image):
    first_id = None
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
                pass
        else:
            pass
    if markerIds is None:
        pass
    return first_id, square
