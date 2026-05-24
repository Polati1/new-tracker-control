import cv2
import numpy as np
from threading import Thread
from pypylon import pylon
import time
import serial
import struct
import ctypes

try:
    winmm = ctypes.WinDLL('winmm.dll')
    winmm.timeBeginPeriod(1) 
except Exception as e:
    print(f"Timer fix failed: {e}")

last_time = time.time()
alive = True
new_bbox = None
reset_bbox = False
video_width = 900
video_height = 900
actual_w = 900
actual_h = 900
seeker_size = 300
tracker_radius = 75
seek_radius = 150
display_width = 300
midx = actual_w // 2
midy = actual_h // 2
prev_h_error = 0
prev_y_error = 0
vx_filt = 0.0
vy_filt = 0.0
lpf_alpha = 0.91
text_x = 30
timer = 0
last_lock_time = 0
lock_active = False
first_lock = True
tracker = None
frame_global = None

def drawRectangle(frame, bbox):
    x, y, w, h = map(int, bbox)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
    cv2.circle(frame, (x + w//2, y + h//2), 3, (0, 0, 255), -1)

def drawCircle(frame, coord):
    cv2.circle(frame, (int(coord[0]), int(coord[1])), 2, (0, 255, 0), 1)

def drawText(frame, txt, location, color=(50, 170, 50)):
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

def draw_new_bbox(event, x, y, flags, param):
    global new_bbox, reset_bbox, lock_active, last_lock_time, tracker, frame_global, first_lock
    global prev_h_error, prev_y_error, vx_filt, vy_filt

    if event == cv2.EVENT_LBUTTONDOWN:

        box_size = tracker_radius
        x0 = max(x - box_size, 0)
        y0 = max(y - box_size, 0)
        w = h = box_size * 2

        new_bbox = (x0, y0, w, h)
        reset_bbox = True

        tracker = cv2.legacy.TrackerMOSSE.create()
        if frame_global is not None:
            tracker.init(frame_global, new_bbox)
            
            x_new, y_new, w_new, h_new = new_bbox
            prev_h_error = (x_new + w_new//2) - (actual_w // 2)
            prev_y_error = -((y_new + h_new//2) - (actual_h // 2))
            vx_filt = 0.0
            vy_filt = 0.0

        lock_active = True
        first_lock = True
        last_lock_time = time.time()

class BaslerVideoCaptureAsync:
    def __init__(self, width, height):
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.Value = width
        self.camera.Height.Value = height
        self.running = False
        self.grabbed = False
        self.frame = None

    def start(self):
        self.running = True
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.thread = Thread(target=self.update, daemon=True)
        self.thread.start()
        return self

    def update(self):
        while self.running and self.camera.IsGrabbing():

            res = self.camera.RetrieveResult(200)
            if res.GrabSucceeded():
                img = res.Array
                if len(img.shape) == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)
                self.frame = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            res.Release()

    def read(self):
        if self.frame is not None:
            return True, self.frame
        return False, None

    def stop(self):
        self.running = False
        self.thread.join()
        self.camera.StopGrabbing()
        self.camera.Close()
        winmm.timeEndPeriod(1)

def start_opencv_thread():

    global alive, reset_bbox, new_bbox, fps_smooth, last_time, midx, midy, frame_global, lock_active, tracker, text_x, prev_h_error, prev_y_error, vx_filt, vy_filt, first_lock

    video = BaslerVideoCaptureAsync(video_width, video_height).start()
    cv2.namedWindow("Camera/Tracker View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Camera/Tracker View", draw_new_bbox)

    # y or pitch motor serial
    sery = serial.Serial(port='COM9', baudrate=115200, timeout=0)
    bufy = bytearray(9)
    bufy[0] = 0xAA

    # x or yaw motor serial
    serx = serial.Serial(port='COM1', baudrate=115200, timeout=0)
    bufx = bytearray(9)
    bufx[0] = 0xAA

    _, frame = video.read()
    bbox = (0, 0, 0, 0)

    prev_time = time.perf_counter()
  
    while alive:

        ret, frame = video.read()
        if not ret: continue

        frame_global = frame.copy()

        if lock_active and tracker is not None:
            ok, bbox = tracker.update(frame)
        else:
            ok = False

        seeker_view = None

        if ok:

            drawRectangle(frame, bbox)
            x, y, w, h = map(int, bbox)
            midx = x + w//2
            midy = y + h//2

            h_error = (int(bbox[0] + bbox[2]//2)) - (actual_w // 2)
            y_error = -((int(bbox[1] + bbox[3]//2)) - (actual_h // 2))

            y1 = max(0, midy - seek_radius)
            y2 = min(actual_h, midy + seek_radius)
            x1 = max(0, midx - seek_radius)
            x2 = min(actual_w, midx + seek_radius)
            seeker_view = frame[y1:y2, x1:x2]

            cv2.line(frame, (0, midy), (actual_w, midy), (0, 0, 255), 1)
            cv2.line(frame, (midx, 0), (midx, actual_h), (0, 0, 255), 1)

        else:

            if lock_active:
                lock_active = False
                tracker = None

            first_lock = True
            midx, midy = actual_w//2, actual_h//2
            drawText(frame, "NO LOCK", (text_x, 30), (0, 0, 255))

            h_error = 999.0
            y_error = 999.0
            vx_filt = 0.0  
            vy_filt = 0.0
            vx = 0.0
            vy = 0.0

        curr_time = time.perf_counter()
        dt = curr_time - prev_time

        if dt > 0:

            fps = 1.0 / dt

            # Normal velocity calculation
            vx = (h_error - prev_h_error) / dt
            vy = (y_error - prev_y_error) / dt
            
            vx_filt = lpf_alpha * vx_filt + (1 - lpf_alpha) * vx
            vy_filt = lpf_alpha * vy_filt + (1 - lpf_alpha) * vy

        else:

            fps = 0
            vx = 0
            vy = 0

        if not ok:

            struct.pack_into('<f', bufy, 1, 999.0)
            struct.pack_into('<f', bufy, 5, 0.0)
            sery.write(bufy)

            struct.pack_into('<f', bufx, 1, 999.0)
            struct.pack_into('<f', bufx, 5, 0.0)
            serx.write(bufx)

        else:

            struct.pack_into('<f', bufy, 1, y_error)
            struct.pack_into('<f', bufy, 5, vy_filt)
            sery.write(bufy)

            struct.pack_into('<f', bufx, 1, h_error)
            struct.pack_into('<f', bufx, 5, vx_filt)
            serx.write(bufx)

        prev_time = curr_time
        prev_h_error = h_error
        prev_y_error = y_error

        if seeker_view is not None and seeker_view.size > 0:
            inset = cv2.resize(seeker_view, (display_width, display_width), interpolation=cv2.INTER_NEAREST)
            y_start, x_start = actual_h - 300, actual_w - 300
            frame[y_start:y_start+300, x_start:x_start+300] = inset
            cv2.rectangle(frame, (x_start, y_start), (x_start+300, y_start+300), (0, 255, 255), 2)

        drawText(frame, f"FPS: {fps:.1f}", (text_x, 60))
        drawText(frame, f"Co-ords: {int(midx)}, {int(midy)}", (text_x, 90))
        drawText(frame, f"Horz. Error: {h_error}", (text_x, 120))
        drawText(frame, f"Vert. Error: {y_error}", (text_x, 150))
        drawText(frame, f"Vx: {vx_filt:.1f}", (text_x, 180))
        drawText(frame, f"Vy: {vy_filt:.1f}", (text_x, 210))
        drawCircle(frame, (actual_w//2, actual_h//2))

        cv2.imshow("Camera/Tracker View", frame)

        if cv2.waitKey(1) == 27:
            alive = False

    video.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_opencv_thread()