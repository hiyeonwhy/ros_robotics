#!/usr/bin/env python3
import cv2

image_path = '/home/hiyeon/ros_ws/src/camera_calibration_node/cali/calib0000.jpg'
img = cv2.imread(image_path)
points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked point: ({x}, {y})")
        points.append((x, y))
        cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow('image', img)

cv2.imshow('image', img)
cv2.setMouseCallback('image', mouse_callback)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("Final points:", points)

