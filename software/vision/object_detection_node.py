"""
object_detection_node.py — YOLO object detection with OpenCV DNN (Raspberry Pi).

Fixes vs. the original:
  * the model is loaded when you create ObjectDetector (not at import), with a clear error if missing
  * output decoding auto-detects the two common ONNX layouts
        YOLOv5-style  (1, N, 5 + C)  [cx, cy, w, h, objectness, class scores...]
        YOLOv8-style  (1, 4 + C, N)  [cx, cy, w, h, class scores...]
    and scales boxes from network-input pixels to the camera frame correctly
  * non-maximum suppression (no more stacks of duplicate boxes)
  * full 80-class COCO label list (the old 5-name list crashed on class ids > 4)

Recommended Pi model: yolov5n.onnx or yolov8n.onnx exported at 320 or 416 px.
"""

import time

import cv2
import numpy as np

COCO_LABELS = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush']


def decode_yolo(output, frame_w, frame_h, input_size, conf_thresh=0.5, nms_thresh=0.45):
    """Returns a list of (class_id, confidence, (x, y, w, h)) in frame pixels."""
    out = np.squeeze(np.asarray(output))
    if out.ndim != 2:
        raise ValueError(f"Unsupported YOLO output shape {np.shape(output)}")
    if out.shape[0] < out.shape[1] and out.shape[0] <= 128:   # (4 + C, N) -> YOLOv8 layout
        out = out.T
        boxes, scores = out[:, :4], out[:, 4:]
    else:                                                       # (N, 5 + C) -> YOLOv5 layout
        boxes, scores = out[:, :4], out[:, 5:] * out[:, 4:5]
    class_ids = scores.argmax(axis=1)
    confs = scores[np.arange(len(scores)), class_ids]
    keep = confs >= conf_thresh
    boxes, confs, class_ids = boxes[keep], confs[keep], class_ids[keep]
    if len(boxes) == 0:
        return []
    # Coordinates are in network-input pixels (or 0..1 for some exports)
    scale = 1.0 if boxes.max() <= 1.5 else float(input_size)
    sx, sy = frame_w / scale, frame_h / scale
    xywh = [[int((cx - w / 2) * sx), int((cy - h / 2) * sy), int(w * sx), int(h * sy)] for cx, cy, w, h in boxes]
    idx = cv2.dnn.NMSBoxes(xywh, confs.astype(float).tolist(), conf_thresh, nms_thresh)
    return [(int(class_ids[i]), float(confs[i]), tuple(xywh[i])) for i in np.array(idx).flatten()]


class ObjectDetector:
    def __init__(self, model='yolov5n.onnx', input_size=416, labels=COCO_LABELS, conf_thresh=0.5):
        try:
            self.net = cv2.dnn.readNetFromONNX(model)
        except cv2.error as e:
            raise FileNotFoundError(f"Could not load ONNX model '{model}': {e}") from None
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.input_size, self.labels, self.conf_thresh = input_size, labels, conf_thresh

    def detect(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (self.input_size, self.input_size), swapRB=True, crop=False)
        self.net.setInput(blob)
        h, w = frame.shape[:2]
        return decode_yolo(self.net.forward(), w, h, self.input_size, self.conf_thresh)

    def label(self, class_id):
        return self.labels[class_id] if class_id < len(self.labels) else f"class{class_id}"

    def annotate(self, frame, detections=None):
        for cid, conf, (x, y, w, h) in (self.detect(frame) if detections is None else detections):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{self.label(cid)}: {conf:.2f}", (x, max(y - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame


def main():
    det = ObjectDetector()
    cap = cv2.VideoCapture(0)
    time.sleep(2.0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Veg Drone - Object Detection", det.annotate(frame))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
