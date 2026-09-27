"""
face_recognition_node.py — PCA (eigenface) face recognition for the Veg drone (Raspberry Pi).

Fix vs. the original: PCA is computed with an SVD of the (n_images x 16384) data
matrix instead of eigh() on a 16384 x 16384 covariance matrix (~2.1 GB of float64,
which does not fit on a 4 GB Pi 4 alongside SLAM). Unreadable images are skipped.

face_db layout:  face_db/<person_name>/<image>.jpg
"""

import os
import time

import cv2
import numpy as np

FACE_WIDTH = 128
FACE_HEIGHT = 128
THRESHOLD = 1800.0  # distance in PCA space; tune on your own data


class FaceRecognizer:
    def __init__(self, db_path='face_db', k=50, threshold=THRESHOLD):
        self.threshold = threshold
        self.cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        X, self.labels, self.label_map = load_training_faces(db_path)
        self.mean, self.eigvecs = train_pca(X, k)
        self.projections = project_face(X, self.mean, self.eigvecs)   # (n, k) in one matrix product

    def recognize(self, face_vec):
        dists = np.linalg.norm(self.projections - face_vec, axis=1)
        idx = int(np.argmin(dists))
        if dists[idx] < self.threshold:
            return self.label_map[int(self.labels[idx])], float(dists[idx])
        return "Unknown", float(dists[idx])

    def annotate(self, frame, gray=None):
        if gray is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for (x, y0, w, h) in self.cascade.detectMultiScale(gray, 1.2, 5):
            roi = cv2.resize(gray[y0:y0 + h, x:x + w], (FACE_WIDTH, FACE_HEIGHT)).flatten().astype(np.float32)
            label, dist = self.recognize(project_face(roi, self.mean, self.eigvecs))
            color = (0, 255, 0) if label != "Unknown" else (0, 0, 255)
            cv2.rectangle(frame, (x, y0), (x + w, y0 + h), color, 2)
            cv2.putText(frame, f"{label} ({dist:.0f})", (x, max(y0 - 10, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame


def load_training_faces(path='face_db'):
    if not os.path.isdir(path):
        raise FileNotFoundError(f"Face database '{path}' not found (expected {path}/<person>/<images>)")
    labels, faces, label_map = [], [], {}
    idx = 0
    for name in sorted(os.listdir(path)):
        person_path = os.path.join(path, name)
        if not os.path.isdir(person_path):
            continue
        n_before = len(faces)
        for img_name in sorted(os.listdir(person_path)):
            img = cv2.imread(os.path.join(person_path, img_name), cv2.IMREAD_GRAYSCALE)
            if img is None:                      # not an image / unreadable -> skip instead of crashing
                continue
            faces.append(cv2.resize(img, (FACE_WIDTH, FACE_HEIGHT)).flatten())
            labels.append(idx)
        if len(faces) > n_before:
            label_map[idx] = name
            idx += 1
    if len(faces) < 2:
        raise ValueError(f"Need at least 2 face images in '{path}', found {len(faces)}")
    return np.asarray(faces, dtype=np.float32), np.asarray(labels), label_map


def train_pca(X, k=50):
    """Eigenfaces via thin SVD: memory O(n*d), not O(d^2)."""
    mean = X.mean(axis=0)
    _, _, vt = np.linalg.svd(X - mean, full_matrices=False)
    k = min(k, vt.shape[0])
    return mean, vt[:k].T            # (d, k), strongest components first


def project_face(face, mean, eigvecs):
    return (face - mean) @ eigvecs


def main():
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)
    rec = FaceRecognizer()
    print("[INFO] Face recognition system initialized.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Veg Drone - Face Recognition", rec.annotate(frame))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
