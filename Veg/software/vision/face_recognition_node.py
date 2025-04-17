
# face_recognition_node.py
# PCA-based face recognition using OpenCV on Raspberry Pi

import cv2
import numpy as np
import os
import time

# Parameters
FACE_WIDTH = 128
FACE_HEIGHT = 128
THRESHOLD = 1800.0  # Euclidean distance threshold

# Load face cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Load training data
def load_training_faces(path='face_db'):
    labels = []
    faces = []
    label_map = {}
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

# PCA training
def train_pca(X, k=50):
    mean = np.mean(X, axis=0)
    X_centered = X - mean
    cov = np.cov(X_centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    top_k = eigvecs[:, -k:]
    return mean, top_k

# Project face into PCA space
def project_face(face, mean, eigvecs):
    return np.dot(face - mean, eigvecs)

def recognize(face_vec, projections, labels, label_map):
    dists = [np.linalg.norm(face_vec - p) for p in projections]
    idx = np.argmin(dists)
    if dists[idx] < THRESHOLD:
        return label_map[labels[idx]], dists[idx]
    else:
        return "Unknown", dists[idx]

# Main loop
def main():
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)

    X, y, label_map = load_training_faces()
    mean, eigvecs = train_pca(X, k=50)
    projections = [project_face(f, mean, eigvecs) for f in X]

    print("[INFO] Face recognition system initialized.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
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

        cv2.imshow("Veg Drone - Face Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
