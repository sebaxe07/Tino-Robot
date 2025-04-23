from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("yolo11n-pose.pt")

# Export the model to TensorRT format
model.export(format="engine")  # creates 'yolo11n.engine'
