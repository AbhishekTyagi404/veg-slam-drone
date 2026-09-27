"""
vision_multitask_node.py

Version: 2.4
Author: Abhishek Tyagi (mechatronics.abhishek@gmail.com)

Runs object detection and PCA face recognition on one camera feed (Raspberry Pi 4).
Now reuses ObjectDetector / FaceRecognizer instead of duplicating their code, so
fixes in one place apply to both scripts.
"""

import time

import cv2

from face_recognition_node import FaceRecognizer
from object_detection_node import ObjectDetector


def main(model='yolov5n.onnx', face_db='face_db'):
    print("[INFO] Initializing system...")
    detector = ObjectDetector(model)
    recognizer = FaceRecognizer(face_db)
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)
    print("[INFO] Object detection and face recognition initialized.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        detections = detector.detect(frame)          # detect on the clean frame
        recognizer.annotate(frame)
        detector.annotate(frame, detections)
        cv2.imshow("Veg Drone - Object + Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
