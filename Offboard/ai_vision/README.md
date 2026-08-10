# 👁️ AI Vision — Object Detection Pipeline

> **Purpose:** Train and export a YOLO-based computer vision model to detect opposing robots on the battlefield. The model is trained offline in the cloud and exported as PyTorch weights (`.pt`) for real-time inference onboard the robot.

---

## Table of Contents

1. [Overview](#overview)
2. [Workflow](#workflow)
3. [Dataset](#dataset)
4. [Training Notebook](#training-notebook)
5. [Model Deployment](#model-deployment)

---

## Overview

The `ai_vision` directory contains the offline training pipeline for the robot's primary object detection system. Using the Ultralytics YOLO framework, we train a custom lightweight model (`YOLO26n` architecture) to identify hostile robots from the live camera feed.

The workflow relies on **Roboflow** for dataset management and annotation, and **Kaggle** (or any cloud GPU) for model training, keeping the computationally heavy tasks offboard.

---

## Workflow

1. **Collect Images:** Capture images from the robot's camera (`/camera/image_raw`) during manual driving or from simulation.
2. **Annotate (Roboflow):** Upload images to Roboflow, draw bounding boxes around target robots, and generate a dataset version.
3. **Export Dataset:** Export the dataset from Roboflow in YOLO PyTorch format (e.g., `Eye To Zion - Detection Robots.v4i.yolo26(1).zip`).
4. **Train (Kaggle):** Use `eye-to-zion-ai-vision.ipynb` to train the YOLO model on the exported dataset.
5. **Deploy:** Transfer the exported weights (`best.pt`) to the onboard ROS 2 vision stack.

---

## Dataset

The directory contains an example dataset export from Roboflow:
`Eye To Zion - Detection Robots.v4i.yolo26(1).zip`

**Structure of Exported YOLO Dataset:**
- `/train/` — Images and `.txt` bounding box labels for training.
- `/valid/` — Images and `.txt` labels for validation.
- `/test/` — Images and `.txt` labels for final testing.
- `data.yaml` — Class configuration and dataset paths.

*Note: Ensure the dataset ZIP is unpacked or loaded properly within your Kaggle/Colab notebook environment before starting training.*

---

## Training Notebook

| | |
|---|---|
| **File** | `eye-to-zion-ai-vision.ipynb` |
| **Framework** | Ultralytics YOLO (PyTorch) |
| **Hardware** | Kaggle (Tesla T4 GPU recommended) |

### Key Training Parameters

The notebook initializes a Nano-scale YOLO model and trains it using extensive data augmentation to handle varied lighting and camera angles on the battlefield:

```python
results = model.train(
    data=f"{dataset.location}/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    project='EyeToZion_AI',
    name='yolo_robot_detect',
    hsv_h=0.015,   # Minor hue shift
    hsv_s=0.7,     # Saturation for variable lighting
    hsv_v=0.4,     # Exposure for shadows/highlights
    translate=0.1, # Image shift
    scale=0.5,     # Zoom for varying distances
    fliplr=0.5     # Horizontal flip
)
```

### Outputs

After training completes, Ultralytics strips the optimizer from the final weights to reduce file size. The primary output required for deployment is:
`/runs/detect/EyeToZion_AI/yolo_robot_detect/weights/best.pt`

---

## Model Deployment

1. Download the `best.pt` weights file from the notebook outputs.
2. Transfer it to the Raspberry Pi (e.g., into a `weights/` folder within your ROS 2 vision package).
3. The onboard vision node (using `ultralytics` package) will load these weights, subscribe to the `/camera/image_raw` topic, and publish bounding box coordinates or tracking data to guide the robot's turret/navigation system.
