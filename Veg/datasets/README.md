
# Veg Drone – Dataset Guide

This folder contains datasets used for training and testing Veg Drone's onboard AI systems.

---

## 📁 training_images/

Images used for training the object detection model (YOLO-like architecture).

### Structure
```
training_images/
├── person/
│   ├── img001.jpg
│   └── ...
├── car/
│   ├── car1.jpg
│   └── ...
```

Each subfolder represents a class. Annotate images using tools like LabelImg or Roboflow and export in YOLO or Pascal VOC format.

---

## 📁 face_db/

Contains labeled face images used by the PCA-based face recognition module.

### Structure
```
face_db/
├── abhishek/
│   ├── img1.jpg
│   └── img2.jpg
├── ramesh/
│   ├── img1.jpg
│   └── img2.jpg
```

Each folder name corresponds to a person. Images must be grayscale or convertible, and ideally cropped frontal faces sized 128×128.

---

## 🧪 Notes

- Total training size should be limited if running inference onboard (Raspberry Pi)
- Use grayscale or low-res images for faster PCA projection and detection on edge devices

---

## 🛠 Tools Recommended

- [LabelImg](https://github.com/tzutalin/labelImg)
- [Roboflow](https://roboflow.com/)
- [OpenCV Tools](https://docs.opencv.org/)
