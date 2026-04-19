from ultralytics import YOLO

# load small pretrained model
model = YOLO("yolo11n.pt")

# train on your dataset
model.train(
    data="data.yaml",
    epochs=50,
    imgsz=640
)