from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
# Train the model

results = model.train(data="/home/park/PycharmProjects/Yolo/connector/data.yaml", epochs=500, imgsz=1280)