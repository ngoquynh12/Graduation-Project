import cv2
import time
from ultralytics import YOLO
from picamera2 import Picamera2

YOLO_MODEL_PATH = "/home/pi/human_sound_model/best.pt"

CAM_W, CAM_H = 640, 480
FPS = 30

IMG_SIZE = 320
CONF_THRESHOLD = 0.25
INFER_INTERVAL = 0.20

running = True

def camera_yolo_test():
    global running

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (CAM_W, CAM_H), "format": "RGB888"},  # gi? nhu b?n
        controls={"FrameRate": FPS}
    )
    picam2.configure(config)
    picam2.start()

    yolo_model = YOLO(YOLO_MODEL_PATH)

    last_infer = 0.0
    last_annotated = None

    try:
        while running:
            frame = picam2.capture_array()

            # ? KH�NG convert n?a (v� m�y b?n dang b? swap s?n)
            frame_bgr = frame

            now = time.time()
            if now - last_infer >= INFER_INTERVAL:
                last_infer = now
                results = yolo_model.predict(
                    frame_bgr, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False
                )
                last_annotated = results[0].plot()

            show = last_annotated if last_annotated is not None else frame_bgr
            cv2.imshow("YOLO Camera Test", show)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                running = False

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_yolo_test()