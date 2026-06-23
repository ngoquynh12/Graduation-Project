from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# Kh?i t?o camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# Load m� h�nh YOLOv8 c?a b?n
model = YOLO("/home/pi/human_sound_model/best.pt")  # du?ng d?n t?i file best.pt

while True:
    frame = picam2.capture_array()
    results = model(frame)
    annotated = results[0].plot()
    cv2.imshow("YOLOv8 Camera", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
