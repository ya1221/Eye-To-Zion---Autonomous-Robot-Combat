from ultralytics import YOLO

model = YOLO('best.pt')

print("Starting export to NCNN format...")

model.export(
    format='ncnn',
    imgsz=320
)

print("Export completed. A new folder (e.g., 'best_ncnn_model') was created.")
