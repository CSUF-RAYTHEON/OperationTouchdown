# Object Detection Model (YOLO-Style)

## Overview

This project implements a **YOLO-style object detection model** in TensorFlow (`tf_keras`) to detect:

* Buckets
* Traffic Cones
* Cardboard Boxes
* Ramps

The model predicts bounding boxes using a **7×7 grid output**.

---

## Key Files

```text
train.py              # main file to train the model
object_detector.py   # model architecture + decoding logic
fix_labels.py        # optional dataset cleanup
data.yaml            # dataset config (reference)
train.record         # training dataset (TFRecord)
val.record           # validation dataset (TFRecord)
ugv_object_detect_model.fbz # final trained model
```

---

## Requirements

Install the following before running:

```bash
pip install tensorflow==2.19.1 tf-keras==2.19.0
pip install numpy matplotlib
```

If using Google Colab, also run:

```python
from google.colab import drive
drive.mount('/content/drive')
```

---

## Running Inference

Inside object_detector.py or raytheon_ugv_model.py:

preds = det_model.predict(images)
boxes = decode_predictions(preds[0])
show_prediction(image, preds[0])
```

---

## How to Run the Model

### 1. Set Dataset Paths

Inside `train.py`, make sure these paths are correct:

```python
DATASET_DIR = "path/to/your/dataset"
TRAIN_RECORD = DATASET_DIR + "/train.record"
VAL_RECORD = DATASET_DIR + "/val.record"
```

---

### 2. Train the Model

Run:

```bash
python train.py
```

This will:

* Load TFRecord data
* Build the model
* Train for specified epochs

---

### 3. Run Inference

Inside `object_detector.py`, use:

```python
preds = det_model.predict(images)
boxes = decode_predictions(preds[0])
show_prediction(image, preds[0])
```

This will:

* Generate predictions
* Convert grid outputs → bounding boxes
* Display detections

---

## Model Details

* Input: `224 × 224 × 3`
* Output: `7 × 7 × 9`

Each grid cell predicts:

```text
[x, y, w, h, objectness, class1, class2, class3, class4]
```

---

## Decoder Logic

The model output is converted to usable detections by:

1. Applying sigmoid to objectness
2. Applying softmax to class probabilities
3. Computing confidence score
4. Filtering by threshold
5. Applying Non-Max Suppression (NMS)

---


## Quick Start

```bash
pip install tensorflow==2.19.1 tf-keras==2.19.0 numpy matplotlib
python train.py
```

---

## Summary

```text
Load TFRecord → Train model → Predict → Decode → Visualize
```

---
