import cv2
import numpy as np
import time
import serial
import struct
from pypylon import pylon
from threading import Thread, Event, Lock
import ctypes

try:
    winmm = ctypes.WinDLL('winmm.dll')
    winmm.timeBeginPeriod(1) 
except Exception as e:
    print(f"Timer fix failed: {e}")

alive = True
prev_blob = None
prev_h_error = 0
prev_y_error = 0
vx_filt = 0.0
vy_filt = 0.0

shared_h_error = 999.0
shared_y_error = 999.0
shared_vx = 0.0
shared_vy = 0.0

TEXT_X = 30
MAX_JUMP = 100
TRACKER_RADIUS = 25
LPF_ALPHA = 0.98

CAMERA_ROI_SIZE = 900
CAMERA_CENTER_X = CAMERA_ROI_SIZE // 2
CAMERA_CENTER_Y = CAMERA_ROI_SIZE // 2

ROI_SIZE = 500 
ROI_X1 = (CAMERA_ROI_SIZE - ROI_SIZE) // 2 
ROI_Y1 = (CAMERA_ROI_SIZE - ROI_SIZE) // 2 
ROI_X2 = ROI_X1 + ROI_SIZE 
ROI_Y2 = ROI_Y1 + ROI_SIZE

BLOB_ROI_SIZE = 250
#array of zeros for the flood mask
flood = np.zeros((BLOB_ROI_SIZE + 2, BLOB_ROI_SIZE + 2), np.uint8)

BRIGHTNESS_THRESHOLD = 245
CENTROID_THRESHOLD = 235

PITCH_MOTOR_SER = 'COM9'
YAW_MOTOR_SER = 'COM14'
LOOPBACK = 'loop://'

def safe_serial(port):
    try:
        return serial.Serial(port=port, baudrate=115200, timeout=0)
    except Exception:
        return None

sery = safe_serial(PITCH_MOTOR_SER)
serx = safe_serial(YAW_MOTOR_SER)

bufy = bytearray(9)
bufx = bytearray(9)

bufy[0] = 0xAA
bufx[0] = 0xAA

lock = Lock()


def serial_thread():

    global shared_h_error, shared_y_error, shared_vx, shared_vy

    while True:

        lock.acquire()

        struct.pack_into('<f', bufy, 1, shared_y_error)
        struct.pack_into('<f', bufy, 5, shared_vy)

        struct.pack_into('<f', bufx, 1, shared_h_error)
        struct.pack_into('<f', bufx, 5, shared_vx)

        lock.release()

        sery.write(bufy)
        serx.write(bufx)

Thread(target=serial_thread, daemon=True).start()

def safe_serial(port):
    try:
        return serial.Serial(port=port, baudrate=115200, timeout=0)
    except Exception:
        return None

def drawTrackBox(frame, bbox):
    x, y, w, h = bbox
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0), 2)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255), 1)
    cv2.circle(frame, (x + w//2, y + h//2), 3, (255), -1)

def drawTrackLine(frame, bbox):
    x, y, w, h = bbox
    cv2.line(frame, (0, y + h//2), (CAMERA_ROI_SIZE, y + h//2), (0), 2)
    cv2.line(frame, (x + w//2, 0), (x + w//2, CAMERA_ROI_SIZE), (0), 2)
    cv2.line(frame, (0, y + h//2), (CAMERA_ROI_SIZE, y + h//2), (255), 1)
    cv2.line(frame, (x + w//2, 0), (x + w//2, CAMERA_ROI_SIZE), (255), 1)

def drawCircle(frame, coord):
    cv2.circle(frame, (int(coord[0]), int(coord[1])), 10, (255), 1, cv2.LINE_AA)

def drawText(frame, txt, location):
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255), 1)

def find_bright_centroid(frame):

    _, maxVal, _, maxLoc = cv2.minMaxLoc(frame[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]) #find brightest pixel val and location in x,y coords
    if maxVal < BRIGHTNESS_THRESHOLD: #immediately return None for pixel array and None for location
        return None, None
    fx = maxLoc[0] + ROI_X1
    fy = maxLoc[1] + ROI_Y1
    #find bounding box for blob region around brightest pixel, ensuring it stays within the ROI bounds
    x1 = max(fx - (BLOB_ROI_SIZE//2), 0) #find left edge of box, ensuring it doesn't go below 0
    y1 = max(fy - (BLOB_ROI_SIZE//2), 0) #find top edge of box, ensuring it doesn't go below 0
    x2 = min(fx + (BLOB_ROI_SIZE//2), CAMERA_ROI_SIZE) #find right edge of box, ensuring it doesn't go above ROI size
    y2 = min(fy + (BLOB_ROI_SIZE//2), CAMERA_ROI_SIZE) #find bottom edge of box, ensuring it doesn't go above ROI size

    #get tracking roi array
    crop = frame[y1:y2, x1:x2] 
    crop_loc = (fx - x1, fy - y1) #get maxLoc coordinates in tracking roi coords

    #mask is a binary image where everything above threshold is 255, else 0
    _, mask = cv2.threshold(crop, CENTROID_THRESHOLD, 255, cv2.THRESH_BINARY)

    flood[:] = 0 #clear the flood mask (allocate all 0s)
    flood_view = flood[:mask.shape[0]+2, :mask.shape[1]+2]     #bound it the tracking mask goes near the edges near the edges

    cv2.floodFill(mask, flood_view, crop_loc, 128) #starting from cropLoc, fill everything connected with 128. one single 128 blob
    spot_mask = (mask == 128).astype(np.uint8) #remove everything which remains as not 128
    weighted = (crop * spot_mask).astype(np.float32) #multiply original intensity with mask for weighted sum
    M = cv2.moments(weighted) #find moments of the remaining blob to find centroid location
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"]) + x1 #back into frame coords
        cy = int(M["m01"] / M["m00"]) + y1 #back into frame coords
        return (cx, cy), spot_mask * 255 #return tracking coords and mask array
    return None, None

class BaslerVideoCaptureAsync:
    def __init__(self, width, height):
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.Value = width
        self.camera.Height.Value = height
        self.running = False
        self.grabbed = False
        self.frame = None
        self.frame_ready = Event()

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
                self.frame = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                self.frame_ready.set()
            res.Release()

    def read(self):
        self.frame_ready.wait()
        self.frame_ready.clear()
        f = self.frame
        self.frame = None
        return f

    def stop(self):
        self.running = False
        self.frame_ready.set()
        self.thread.join()
        self.camera.StopGrabbing()
        self.camera.Close()
        winmm.timeEndPeriod(1)

def start_opencv_thread():

    global alive, prev_blob, prev_h_error, prev_y_error, vx_filt, vy_filt
    global shared_h_error, shared_y_error, shared_vx, shared_vy

    video = BaslerVideoCaptureAsync(CAMERA_ROI_SIZE, CAMERA_ROI_SIZE).start()
    cv2.namedWindow("Camera/Tracker View", cv2.WINDOW_NORMAL)

    prev_time = time.perf_counter()

    while alive:

        frame = video.read()

        valid, roi_vis = find_bright_centroid(frame)

        if (valid is not None):

            roi_cx, roi_cy = valid
            candidate_x = roi_cx
            candidate_y = roi_cy

            if prev_blob is None:
                blob_x, blob_y = candidate_x, candidate_y
                prev_blob = (blob_x, blob_y)

            else:
                px, py = prev_blob
                dx = candidate_x - px
                dy = candidate_y - py

                if (dx*dx + dy*dy) <= (MAX_JUMP * MAX_JUMP):
                    blob_x, blob_y = candidate_x, candidate_y
                else:
                    blob_x, blob_y = px, py

        else:

            roi_vis = np.zeros((BLOB_ROI_SIZE, BLOB_ROI_SIZE), np.uint8)
            blob_x, blob_y = CAMERA_CENTER_X, CAMERA_CENTER_Y

        prev_blob = (blob_x, blob_y)

        x = max(blob_x - TRACKER_RADIUS, 0)
        y = max(blob_y - TRACKER_RADIUS, 0)
        w = min(TRACKER_RADIUS * 2, CAMERA_ROI_SIZE - x)
        h = min(TRACKER_RADIUS * 2, CAMERA_ROI_SIZE - y)


        bbox = (x, y, w, h)

        midx = x + w//2
        midy = y + h//2

        h_error = midx - CAMERA_CENTER_X
        y_error = CAMERA_CENTER_Y - midy

        curr_time = time.perf_counter()
        dt = curr_time - prev_time

        if dt > 0:

            fps = 1.0 / dt

            vx = (h_error - prev_h_error) / dt
            vy = (y_error - prev_y_error) / dt
            
            vx_filt = LPF_ALPHA * vx_filt + (1 - LPF_ALPHA) * vx
            vy_filt = LPF_ALPHA * vy_filt + (1 - LPF_ALPHA) * vy

        else:

            fps = 0
            vx = 0
            vy = 0

        prev_time = curr_time
        prev_h_error = h_error
        prev_y_error = y_error

        lock.acquire()
        if (valid is not None and not (h_error == 0 and y_error == 0)):
            shared_h_error = h_error
            shared_y_error = y_error
            shared_vx = vx_filt
            shared_vy = vy_filt
        else:
            shared_h_error = 999
            shared_y_error = 999
            shared_vx = 0
            shared_vy = 0
        lock.release()


        drawTrackBox(frame, bbox)
        drawTrackLine(frame, bbox)
        drawCircle(frame, (CAMERA_CENTER_X, CAMERA_CENTER_Y))
        drawText(frame, f"FPS: {int(fps)}", (TEXT_X, 60))
        drawText(frame, f"Co-ords: {midx}, {midy}", (TEXT_X, 90))
        drawText(frame, f"Horz. Error: {h_error}", (TEXT_X, 120))
        drawText(frame, f"Vert. Error: {y_error}", (TEXT_X, 150))
        drawText(frame, f"Vx: {int(vx_filt)}", (TEXT_X, 180))
        drawText(frame, f"Vy: {int(vy_filt)}", (TEXT_X, 210))

        print(f"Co-ords: {midx}, {midy} FPS: {int(fps)}")

        cv2.imshow("Camera/Tracker View", frame)
        cv2.imshow("ROI Process View", roi_vis)

        if cv2.waitKey(1) == 27:
            alive = False

    video.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_opencv_thread()
    