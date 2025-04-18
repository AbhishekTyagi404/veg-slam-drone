"""
vision_multitask_node.py

Version: 2.3
Author: Abhishek Tyagi (mechatronics.abhishek@gmail.com)

Description:
This script runs both object detection and PCA-based face recognition 
simultaneously using a single Pi Camera feed. Designed for embedded 
deployment on Raspberry Pi 4 as part of the "Veg" autonomous surveillance 
quadcopter platform. Compatible with OpenCV and ONNX-based lightweight CNN.
"""

import cv2
import numpy as np
import os
import time

# ---------- Face Recognition Setup ----------
FACE_WIDTH = 128
FACE_HEIGHT = 128
THRESHOLD = 1800.0

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def load_training_faces(path='face_db'):
    labels, faces, label_map = [], [], {}
    idx = 0
    for name in os.listdir(path):
        person_path = os.path.join(path, name)
        if not os.path.isdir(person_path): continue
        label_map[idx] = name
        for img_name in os.listdir(person_path):
            img_path = os.path.join(person_path, img_name)
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            img = cv2.resize(img, (FACE_WIDTH, FACE_HEIGHT))
            faces.append(img.flatten())
            labels.append(idx)
        idx += 1
    return np.array(faces), np.array(labels), label_map

def train_pca(X, k=50):
    mean = np.mean(X, axis=0)
    X_centered = X - mean
    cov = np.cov(X_centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    top_k = eigvecs[:, -k:]
    return mean, top_k

def project_face(face, mean, eigvecs):
    return np.dot(face - mean, eigvecs)

def recognize(face_vec, projections, labels, label_map):
    dists = [np.linalg.norm(face_vec - p) for p in projections]
    idx = np.argmin(dists)
    return (label_map[labels[idx]], dists[idx]) if dists[idx] < THRESHOLD else ("Unknown", dists[idx])

# ---------- Object Detection Setup ----------
net = cv2.dnn.readNetFromONNX('yolov4-tiny.onnx')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
labels = ["person", "car", "bike", "dog", "cat"]

def preprocess(frame):
    return cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)

def detect_objects(frame):
    blob = preprocess(frame)
    net.setInput(blob)
    return net.forward()

def draw_object_boxes(frame, outputs, conf_thresh=0.5):
    h, w = frame.shape[:2]
    for output in outputs[0]:
        conf = float(output[4])
        if conf < conf_thresh:
            continue
        class_scores = output[5:]
        class_id = np.argmax(class_scores)
        confidence = class_scores[class_id]
        if confidence > conf_thresh:
            cx, cy, bw, bh = (output[0:4] * [w, h, w, h]).astype(int)
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            label = f"{labels[class_id]}: {confidence:.2f}"
            cv2.rectangle(frame, (x1, y1), (x1 + bw, y1 + bh), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# ---------- Main Vision Fusion Loop ----------
def main():
    print("[INFO] Initializing system...")
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)

    X, y, label_map = load_training_faces()
    mean, eigvecs = train_pca(X, k=50)
    projections = [project_face(f, mean, eigvecs) for f in X]
    print("[INFO] Object detection and face recognition initialized.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Object Detection
        outputs = detect_objects(frame)
        draw_object_boxes(frame, outputs)

        # Face Recognition
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.2, 5)
        for (x, y0, w, h) in faces:
            roi = gray[y0:y0+h, x:x+w]
            roi_resized = cv2.resize(roi, (FACE_WIDTH, FACE_HEIGHT)).flatten()
            vec = project_face(roi_resized, mean, eigvecs)
            label, dist = recognize(vec, projections, y, label_map)
            color = (0, 255, 0) if label != "Unknown" else (0, 0, 255)
            cv2.rectangle(frame, (x, y0), (x+w, y0+h), color, 2)
            cv2.putText(frame, f"{label} ({dist:.0f})", (x, y0-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("Veg Drone - Object + Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
