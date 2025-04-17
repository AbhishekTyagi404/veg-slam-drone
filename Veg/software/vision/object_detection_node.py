
# object_detection_node.py
# Real-time object detection using OpenCV DNN on Raspberry Pi camera

import cv2
import time

# Load YOLO-style network using OpenCV DNN (Tiny model recommended for Pi)
net = cv2.dnn.readNetFromONNX('yolov4-tiny.onnx')  # Provide your ONNX model
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

labels = ["person", "car", "bike", "dog", "cat"]  # Adjust as per training

def preprocess(frame):
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    return blob

def detect(frame):
    blob = preprocess(frame)
    net.setInput(blob)
    outputs = net.forward()
    return outputs

def draw_boxes(frame, outputs, conf_thresh=0.5):
    h, w = frame.shape[:2]
    for output in outputs[0]:
        conf = float(output[4])
        if conf < conf_thresh:
            continue
        class_scores = output[5:]
        class_id = class_scores.argmax()
        confidence = class_scores[class_id]
        if confidence > conf_thresh:
            cx, cy, bw, bh = (output[0:4] * [w, h, w, h]).astype(int)
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            label = f"{labels[class_id]}: {confidence:.2f}"
            cv2.rectangle(frame, (x1, y1), (x1 + bw, y1 + bh), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def main():
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        outputs = detect(frame)
        draw_boxes(frame, outputs)
        cv2.imshow("Veg Drone - Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
