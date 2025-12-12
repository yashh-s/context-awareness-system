
#!/usr/bin/env python3

import threading
import serial
import time
import cv2
import numpy as np
import sys
import math
from collections import deque
try:
    from picamera2 import Picamera2
except ImportError:
    print("CRITICAL: picamera2 library not found. Please install it.")
    sys.exit(1)

# ---------------- USER TUNABLE ----------------
test_debug = False
SMOOTHING_FRAMES = 8
SMOOTHING_REQUIRED = 5
SMOOTHING_MIN_CONF = 0.5

# ---------------- SensorReader (optional) ----------------
class SensorReader:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, reconnect_interval=2.0):
        self.port = port; self.baud = baud; self.reconnect_interval = reconnect_interval
        self._stop = threading.Event(); self._thread = None; self._lock = threading.Lock()
        self.dht_temp = None; self.humidity = None; self.mlx_ambient = None; self.mlx_object = None
        self.rfid1 = None; self.rfid2 = None; self.last_update = None; self._ser = None

    def start(self):
        if self._thread and self._thread.is_alive(): return
        self._stop.clear(); self._thread = threading.Thread(target=self._run, daemon=True); self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread: self._thread.join(timeout=1.0)
        self._close_serial()

    def _open_serial(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
            try: self._ser.reset_input_buffer()
            except Exception: pass
            return True
        except Exception:
            self._ser = None; return False

    def _close_serial(self):
        try:
            if self._ser: self._ser.close()
        except Exception:
            pass
        self._ser = None

    def _safe_readline(self):
        try:
            if not self._ser: return ""
            raw = self._ser.readline()
            if not raw: return ""
            return raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            return ""

    def _run(self):
        while not self._stop.is_set():
            if self._ser is None:
                ok = self._open_serial()
                if not ok:
                    time.sleep(self.reconnect_interval)
                    continue
            try:
                line = self._safe_readline()
                if not line:
                    time.sleep(0.01); continue
                if line == "[TEMP]":
                    l1 = self._safe_readline(); l2 = self._safe_readline(); l3 = self._safe_readline(); l4 = self._safe_readline()
                    _end = self._safe_readline()
                    dht=None; hum=None; amb=None; obj=None
                    if "=" in l1:
                        try: dht = float(l1.split("=",1)[1])
                        except Exception: dht=None
                    if "=" in l2:
                        try: hum = float(l2.split("=",1)[1])
                        except Exception: hum=None
                    if "=" in l3:
                        try: amb = float(l3.split("=",1)[1])
                        except Exception: amb=None
                    if "=" in l4:
                        try: obj = float(l4.split("=",1)[1])
                        except Exception: obj=None
                    with self._lock:
                        if dht is not None: self.dht_temp = dht
                        if hum is not None: self.humidity = hum
                        if amb is not None: self.mlx_ambient = amb
                        if obj is not None: self.mlx_object = obj
                        self.last_update = time.time()
                elif line.startswith("[RFID1] UID="):
                    uid = line.replace("[RFID1] UID=","").strip()
                    with self._lock: self.rfid1 = uid; self.last_update = time.time()
                elif line.startswith("[RFID2] UID="):
                    uid = line.replace("[RFID2] UID=","").strip()
                    with self._lock: self.rfid2 = uid; self.last_update = time.time()
            except serial.SerialException:
                self._close_serial(); time.sleep(self.reconnect_interval)
            except Exception:
                time.sleep(0.01)
        self._close_serial()

    def get(self):
        with self._lock:
            return {
                "dht_temp": self.dht_temp,
                "humidity": self.humidity,
                "mlx_ambient": self.mlx_ambient,
                "mlx_object": self.mlx_object,
                "rfid1": self.rfid1,
                "rfid2": self.rfid2,
                "last_update": self.last_update
            }

# ---------------- Camera init (for modular use) ----------------
def initialize_camera():
    """Initializes and returns a Picamera2 object."""
    try:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "XRGB8888"})
        picam2.configure(config)
        picam2.start()
        return picam2
    except Exception as e:
        return None

# ---------------- Detection config (tuned ~30cm) ----------------
roi_scale = 0.30
min_area = 250      
use_gamma = True
center_only_mode = False

color_map = {
    "Red1":    ([0, 10],    [100, 255], [100, 255]),
    "Red2":    ([160, 180], [100, 255], [100, 255]),
    "Orange":  ([10, 25],   [100, 255], [100, 255]),
    "Yellow":  ([25, 35],   [100, 255], [100, 255]),
    "Green":   ([36, 85],   [60, 255],  [60, 255]),
    "Blue":    ([86, 125],  [60, 255],  [60, 255]),
    "Purple":  ([126, 145], [60, 255],  [60, 255]),
    "Pink":    ([146, 159], [80, 255],  [80, 255]),
    "Black":   ([0, 180],   [0, 255],   [0, 60]),
    "Brown":   ([5, 25],   [100, 255], [20, 120])
}

# ---------------- Utilities ----------------
def bbox_intersection(a,b):
    ax1,ay1,ax2,ay2 = a; bx1,by1,bx2,by2 = b
    ix1,iy1 = max(ax1,bx1), max(ay1,by1); ix2,iy2 = min(ax2,bx2), min(ay2,by2)
    if ix2<=ix1 or iy2<=iy1: return 0
    return (ix2-ix1)*(iy2-iy1)

def preprocess(frame):
    img = frame.copy()
    if use_gamma:
        gamma = 1.2; inv = 1.0/gamma
        table = (np.linspace(0,1,256)**inv * 255).astype(np.uint8)
        img = cv2.LUT(img, table)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    v = clahe.apply(v)
    hsv = cv2.merge([h,s,v])
    return img, hsv

def get_masks(hsv):
    masks = {}
    for cname, ((hl, hh), (sl, sh), (vl, vh)) in color_map.items():
        mask = cv2.inRange(hsv, (hl, sl, vl), (hh, sh, vh))
        if cname == "Red1":
            masks["Red"] = mask.copy()
        elif cname == "Red2":
            masks["Red"] = cv2.bitwise_or(masks.get("Red", np.zeros_like(mask)), mask)
        else:
            masks[cname] = mask
    return masks

def shape_of(cnt):
    area = cv2.contourArea(cnt)
    if area <= 0: return "Unknown"
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    v = len(approx)
    x, y, w, h = cv2.boundingRect(approx)
    ar = w / float(h) if h > 0 else 999.0
    rectangularity = area / (w * h) if w * h > 0 else 0
    if v == 4 or (v > 4 and rectangularity > 0.85) or (w>5*h or h>5*w): return "Rectangle"
    circularity = (4.0 * np.pi * area) / (peri * peri) if peri > 0 else 0
    if circularity > 0.67: return "Circle"
    return "Oval"

def choose_best_contour(center, roi_box, masks, min_area, roi_fallback_overlap=True):
    cx,cy = center; x1,y1,x2,y2 = roi_box
    best_center=None; best_overlap=None; combined_mask=None
    for cname, mask in masks.items():
        mask = cv2.medianBlur(mask,5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        mask = cv2.dilate(mask, np.ones((3,3), np.uint8), iterations=1)
        if combined_mask is None: combined_mask = mask 
        else: combined_mask = cv2.bitwise_or(combined_mask, mask)
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area: continue
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            ccx,ccy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
            label = "Red" if cname in ("Red1","Red2") else cname
            if cv2.pointPolygonTest(c, (cx,cy), False) >= 0:
                if best_center is None or area > best_center[0]: best_center = (area, label, c, (ccx,ccy))
                continue
            if roi_fallback_overlap:
                x,y,w,h = cv2.boundingRect(c)
                inter = bbox_intersection((x,y,x+w,y+h),(x1,y1,x2,y2))
                if inter > 0:
                    if best_overlap is None or area > best_overlap[4]: best_overlap = (inter, label, c, (ccx,ccy), area)
    if best_center: return best_center[1], best_center[2], best_center[3], best_center[0], combined_mask
    if best_overlap: return best_overlap[1], best_overlap[2], best_overlap[3], best_overlap[4], combined_mask
    return None, None, None, None, combined_mask

def compute_contour_extent(contour):
    area = cv2.contourArea(contour)
    x,y,w,h = cv2.boundingRect(contour)
    rect_area = max(1, w*h)
    return float(area)/float(rect_area), (x,y,w,h)

MM_PER_PIXEL = 0.65

def get_real_world_dims(w, h): return w * MM_PER_PIXEL, h * MM_PER_PIXEL
def get_real_world_area(area_pixels): return (area_pixels * (MM_PER_PIXEL ** 2)) / 100.0

def classify_pen_book_bottle(color_label, shape, area, bbox, frame_size, contour=None, obj_temp=None, ambient_temp=None, humidity=None):
    fw, fh = frame_size; x, y, w, h = bbox
    scores = {"Pen": 0.0, "Book": 0.0, "Bottle": 0.0, "Cup": 0.0, "Unknown": 1.0}
    real_width, real_height = get_real_world_dims(w, h)
    real_area = get_real_world_area(area)
    
    if shape in ("Rectangle", "Square", "Oval", "Circle") and w > 0 and h > 0:
        if (max(real_width, real_height) > 3 * min(real_width, real_height)) and (max(real_width, real_height) >= 80.0):
            scores["Pen"] = 2.8
        if (real_area <= 10):
            scores["Pen"] += 2.8

    if shape in ("Rectangle", "Square") and real_width > 0 and real_height > 0:
        if (150 <= real_width <= 450 or 150 <= real_height <= 450) and (70 <= real_width <= 230 or 70 <= real_height <= 230):
            scores["Book"] = 2.0

    if shape in ("Circle", "Oval") and real_area >= 10.0:
        scores["Cup"] = 1
        if obj_temp is not None and ambient_temp is not None:
            try:
                dt = float(obj_temp) - float(ambient_temp)
                if abs(dt) >= 1.0: scores["Cup"] += 1.5
            except: pass

    total = sum(scores.values())
    if total <= 1.0: return "Unknown", 1.0, {}
    confs = {k: scores[k] / total for k in scores}
    best = max(confs, key=confs.get)
    return best, float(min(1.0, confs[best])), {}


# ---------------- MAIN EXECUTION (for standalone testing) ----------------
if __name__ == "__main__":
    print(" Starting camp.py in standalone test mode...")
    picam2 = initialize_camera()
    if picam2 is None:
        sys.exit(1)

    sensor = SensorReader(port="/dev/ttyUSB0", baud=115200)
    sensor.start()
    time.sleep(0.2)
    recent_labels = deque(maxlen=SMOOTHING_FRAMES)
    last_printed_mode = None; last_known_label = None

    try:
        while True:
            frame = picam2.capture_array()[:, :, :3]
            H, W = frame.shape[:2]; cx, cy = W // 2, H // 2
            rw, rh = int(W * roi_scale), int(H * roi_scale)
            x1, y1, x2, y2 = cx - rw // 2, cy - rh // 2, cx + rw // 2, cy + rh // 2

            guide = frame.copy()
            cv2.rectangle(guide, (x1, y1), (x2, y2), (60, 255, 60), 2)
            cv2.circle(guide, (cx, cy), 6, (60, 255, 60), 2)

            _, hsv = preprocess(frame)
            masks = get_masks(hsv)
            label, contour, centroid, area, combined_mask = choose_best_contour((cx, cy), (x1, y1, x2, y2), masks, min_area, roi_fallback_overlap=not center_only_mode)
            
            s = sensor.get()
            result = guide.copy()

            if contour is not None and centroid is not None:
                ccx, ccy = centroid
                cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                shp = shape_of(contour); bx, by, bw, bh = cv2.boundingRect(contour)
                current_label, current_conf, _ = classify_pen_book_bottle(
                    color_label=label, shape=shp, area=area, bbox=(bx, by, bw, bh), frame_size=(W, H), contour=contour,
                    obj_temp=s.get("mlx_object"), ambient_temp=s.get("mlx_ambient"), humidity=s.get("humidity")
                )
                recent_labels.append((current_label, current_conf))
                
                chosen_label, chosen_conf = current_label, current_conf
                if len(recent_labels) > 0:
                    counts = {lab: [item[0] for item in recent_labels].count(lab) for lab, _ in recent_labels}
                    best_lab = max(counts, key=counts.get)
                    if counts[best_lab] >= SMOOTHING_REQUIRED:
                        chosen_label = best_lab

                cv2.putText(result, f"{chosen_label} ({current_conf:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
                cv2.putText(result, f"{label}, {shp}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                

                current_mode = None
                if chosen_label in ("Pen", "Book"): current_mode = "S1" # Study
                elif chosen_label in ("Cup", "Bottle"): current_mode = "R2" # Relax
                if current_mode and (current_mode != last_printed_mode or chosen_label != last_known_label):
                    print(f"MODE: {current_mode} (Object: {chosen_label})")
                    last_printed_mode = current_mode
                    last_known_label = chosen_label
            else:
                recent_labels.clear()

            cv2.imshow("Center Target Detection", result)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    finally:
        print("Stopping...")
        sensor.stop()
        picam2.stop()
        cv2.destroyAllWindows()
        sys.exit(0)
                                                                              
