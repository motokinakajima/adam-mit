import cv2
import numpy as np

class ArrowDetector:
    def __init__(self, min_contour=1000):
        self.min_contour = min_contour

    def get_arrow_detection(self, image, hsv_range):
        lower_color, upper_color = hsv_range

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create a mask for the arrow color
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours on the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours by area
        filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= self.min_contour]

        # Find the largest contour by area
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)

            # Create a mask for the largest contour
            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

            # Apply the mask to the original image
            masked_image = cv2.bitwise_and(image, image, mask=contour_mask)

            # Convert masked image to grayscale
            gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to the grayscale image
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Apply edge detection
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours again on the edge-detected image
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Find the largest contour by area in the edge-detected image
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)

                # Approximate the contour
                epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                approx = cv2.approxPolyDP(largest_contour, epsilon, True)

                # Determine the direction
                direction, _ = self.determine_direction(image, approx)
                return direction
        return 0  # Return 0 if no arrow is detected

    def determine_direction(self, image, approx):
        # Calculate the centroid of the contour
        M = cv2.moments(approx)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            return 0, None

        # Calculate the angles and find the point with the smallest angle in the range 15 to 60 degrees
        angles = []
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0]
            angle = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            angle_degrees = np.degrees(angle)
            if 15 <= abs(angle_degrees) <= 60:
                angles.append((abs(angle_degrees), p1))

        # Print all angles and their corresponding points
        for angle, point in angles:
            print(f"Angle: {angle:.2f} degrees, Point: {point}")

        if not angles:
            return 0, None

        smallest_angle_point = min(angles, key=lambda x: x[0])[1]

        # Draw a red line from the centroid to the smallest angle point
        cv2.line(image, (cx, cy), tuple(smallest_angle_point), (0, 0, 255), 2)

        # Determine the direction based on the relative position of the smallest angle point to the centroid
        if smallest_angle_point[0] > cx:
            direction = 1  # Right
        else:
            direction = -1  # Left

        return direction, cv2.contourArea(approx)
