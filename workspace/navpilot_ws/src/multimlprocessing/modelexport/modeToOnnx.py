from ultralytics import YOLO

model = YOLO("yolo26n.pt")
model.export(format="onnx")


# yolo export model=yolo26l-pose.pt format=onnx