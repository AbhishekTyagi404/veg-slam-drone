
# File: camera_occupancy_overlay.py
# Description: Live camera feed with simulated occupancy grid overlay (for Veg Drone visualization)
# Author: Abhishek Tyagi
# Version: 2.3

import cv2
import numpy as np

def generate_fake_occupancy(img_shape):
    occ = np.zeros(img_shape[:2], dtype=np.uint8)
    cv2.rectangle(occ, (100, 100), (200, 200), 255, -1)
    cv2.circle(occ, (300, 150), 40, 255, -1)
    return occ

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (480, 360))
    occ_map = generate_fake_occupancy(frame.shape)

    occ_colored = cv2.applyColorMap(occ_map, cv2.COLORMAP_JET)
    overlay = cv2.addWeighted(frame, 0.8, occ_colored, 0.5, 0)

    cv2.imshow("Veg Drone Live Feed with Occupancy Overlay", overlay)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
