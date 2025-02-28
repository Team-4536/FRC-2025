import cv2
import numpy as np


def drawDecorations(image):
    cv2.putText(
        image,
        "Purple Pipe Detector",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def runPipeline(image, llrobot):
    # Initialize variables
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define range of purple color in HSV
    lower_purple = np.array([130, 50, 50])
    upper_purple = np.array([160, 255, 255])

    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Apply morphological operations to reduce noise and fill gaps
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area and aspect ratio
    min_area = 500  # Minimum area of the contour
    min_aspect_ratio = 2.0  # Minimum height/width ratio for a vertical pipe

    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(h) / w
            if aspect_ratio > min_aspect_ratio:
                valid_contours.append(contour)

    # Draw contours and bounding boxes
    for contour in valid_contours:
        cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Find the largest valid contour
    if valid_contours:
        largestContour = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largestContour)

        # Calculate center point of the largest contour
        center_x = x + w // 2
        center_y = y + h // 2

        # Update llpython with information about the largest contour
        llpython = [1, center_x, center_y, w, h, 0, 0, 0]

    # Add decorations to the image
    drawDecorations(image)

    # Return the largest contour, the modified image, and the llpython data
    return largestContour, image, llpython
