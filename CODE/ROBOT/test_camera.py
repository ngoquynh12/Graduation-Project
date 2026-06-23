import cv2
import time
from ultralytics import YOLO
from picamera2 import Picamera2

YOLO_MODEL_PATH = "/home/pi/human_sound_model/best.pt"

# d�ng full FOV c?a IMX219
CAM_W, CAM_H = 1640, 1232
FPS = 30

# k�ch thu?c hi?n th? / dua v�o YOLO
SHOW_W, SHOW_H = 640, 480

IMG_SIZE = 320
CONF_THRESHOLD = 0.25
INFER_INTERVAL = 0.20

running = True

def camera_yolo_test():
    global running

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "RGB888"},
        raw={"size": (CAM_W, CAM_H)}
    )
    picam2.configure(config)
    picam2.start()

    yolo_model = YOLO(YOLO_MODEL_PATH)

    last_infer = 0.0
    last_annotated = None

    try:
        while running:
            frame = picam2.capture_array()

            # resize xu?ng cho nh? nhung v?n gi? full g�c nh�n
            frame_small = cv2.resize(frame, (SHOW_W, SHOW_H))

            # gi? nguy�n nhu code �ng, kh�ng convert m�u
            frame_bgr = frame_small

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