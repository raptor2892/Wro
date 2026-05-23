from __future__ import annotations
import cv2
import numpy as np
import serial
import time
import sys
import threading
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from line_constants import (
    ALIGNMENT_THRESHOLD_PX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DEFAULT_BAUD_RATE,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_SERIAL_PORT,
    EXIT_KEY,
    MASK_KERNEL_SIZE,
)

# ── ROI seguimiento de línea ───────────────────────────────────────────────────
ROI_TOP_MARGIN    = 0.30
ROI_BOTTOM_MARGIN = 0.70

# ── Detección de intersecciones ────────────────────────────────────────────────
MARK_LATERAL_WIDTH   = 0.14
MARK_CENTER_EXCLUDE  = 0.46
MARK_WARN_TOP        = 0.35
MARK_WARN_BOTTOM     = 0.48
MARK_CONFIRM_TOP     = 0.48
MARK_CONFIRM_BOTTOM  = 0.70

MARK_WARN_DENSITY    = 0.20
MARK_CONFIRM_DENSITY = 0.35
MARK_CONFIRM_FRAMES  = 4
MARK_COOLDOWN_MS     = 1000

HEADLESS = True  # True = sin monitor, False = mostrar ventanas


# ══════════════════════════════════════════════════════════════════════════════
#  HILO DE CÁMARA
# ══════════════════════════════════════════════════════════════════════════════
class CameraStream:
    def __init__(self, index, width, height, fps):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("Reintentando sin backend forzado...")
            self.cap = cv2.VideoCapture(index)

        if not self.cap.isOpened():
            print(f"ERROR: No se pudo abrir la cámara {index}.")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE,     -5)

        self.frame   = None
        self.lock    = threading.Lock()
        self.running = True

        # Leer primer frame para confirmar que funciona
        ret, frame = self.cap.read()
        if not ret:
            print("ERROR: No se pudo leer el primer frame.")
            sys.exit(1)
        self.frame = frame
        print("Cámara lista.")

        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()

    def release(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


# ══════════════════════════════════════════════════════════════════════════════
#  LINE FOLLOWER
# ══════════════════════════════════════════════════════════════════════════════
class LineFollower:
    def __init__(self, camera_index=DEFAULT_CAMERA_INDEX,
                 serial_port=DEFAULT_SERIAL_PORT, baud_rate=DEFAULT_BAUD_RATE):
        self.camera_index = camera_index
        self.serial_port  = serial_port
        self.baud_rate    = baud_rate

        print(f"Abriendo cámara {self.camera_index}...")
        self.camera = CameraStream(self.camera_index, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS)

        self.serial_conectado    = False
        self.arduino             = None
        self.alignment_threshold = ALIGNMENT_THRESHOLD_PX

        self._left_buf       = deque(maxlen=MARK_CONFIRM_FRAMES)
        self._right_buf      = deque(maxlen=MARK_CONFIRM_FRAMES)
        self._last_mark_time = 0.0
        self._warn_sent      = False

        self._setup_serial()

    def _setup_serial(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            time.sleep(1.5)
            self.serial_conectado = True
            print(f"ESP32 conectado en {self.serial_port} @ {self.baud_rate} baud")
        except Exception as e:
            self.serial_conectado = False
            print(f"ESP32 no detectado: {e}\nModo simulacion activo...")

    def send_command(self, command: str):
        if self.serial_conectado and self.arduino:
            try:
                self.arduino.write((command + "\n").encode("utf-8"))
            except Exception as e:
                print(f"Error serial: {e}")
        else:
            print(f"[SIM] Comando: {command}")

    def send_error(self, error: float):
        if self.serial_conectado and self.arduino:
            try:
                self.arduino.write(f"ERROR:{float(error):.2f}\n".encode("utf-8"))
            except Exception as e:
                print(f"[SERIAL ERROR] {e}")
        else:
            print(f"[SIM] Error: {error:.2f} px")

    def process_frame(self, frame):
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)  # reducido de 15x15
        _, mask = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY_INV)
        kernel  = np.ones(MASK_KERNEL_SIZE, np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def get_contour_center(self, mask, frame_width, frame_height):
        y_start = int(frame_height * ROI_TOP_MARGIN)
        y_end   = int(frame_height * ROI_BOTTOM_MARGIN)

        roi_mask = np.zeros_like(mask)
        roi_mask[y_start:y_end, :] = mask[y_start:y_end, :]

        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, []

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200:
            return None, None, []

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None, []

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return float(cx - frame_width // 2), (cx, cy), largest

    def detect_lateral_markers(self, mask):
        height, width = mask.shape
        lateral_w = int(width * MARK_LATERAL_WIDTH)
        center_x  = width // 2
        excl_w    = int(width * MARK_CENTER_EXCLUDE)

        x_left_end    = min(lateral_w, center_x - excl_w)
        x_right_start = max(width - lateral_w, center_x + excl_w)

        if x_left_end <= 0 or x_right_start >= width:
            return False, False, int(height * MARK_WARN_TOP), int(height * MARK_CONFIRM_BOTTOM), lateral_w

        yw_start = int(height * MARK_WARN_TOP)
        yw_end   = int(height * MARK_WARN_BOTTOM)
        yc_start = int(height * MARK_CONFIRM_TOP)
        yc_end   = int(height * MARK_CONFIRM_BOTTOM)

        roi_warn_l    = mask[yw_start:yw_end, 0:x_left_end]
        roi_warn_r    = mask[yw_start:yw_end, x_right_start:width]
        roi_confirm_l = mask[yc_start:yc_end, 0:x_left_end]
        roi_confirm_r = mask[yc_start:yc_end, x_right_start:width]

        warn_left  = (np.sum(roi_warn_l > 0)    / roi_warn_l.size)    > MARK_WARN_DENSITY
        warn_right = (np.sum(roi_warn_r > 0)    / roi_warn_r.size)    > MARK_WARN_DENSITY
        conf_left  = (np.sum(roi_confirm_l > 0) / roi_confirm_l.size) > MARK_CONFIRM_DENSITY
        conf_right = (np.sum(roi_confirm_r > 0) / roi_confirm_r.size) > MARK_CONFIRM_DENSITY

        self._left_buf.append(conf_left)
        self._right_buf.append(conf_right)

        left_confirmed  = len(self._left_buf)  == MARK_CONFIRM_FRAMES and all(self._left_buf)
        right_confirmed = len(self._right_buf) == MARK_CONFIRM_FRAMES and all(self._right_buf)

        warn      = (warn_left or warn_right) and not (left_confirmed or right_confirmed)
        confirmed = left_confirmed or right_confirmed

        return warn, confirmed, yw_start, yc_end, lateral_w

    def _cooldown_ok(self) -> bool:
        return (time.time() * 1000 - self._last_mark_time) > MARK_COOLDOWN_MS

    def run(self):
        print("\n--- Seguidor de Linea activo ---")
        while True:
            ret, frame = self.camera.read()
            if not ret or frame is None:
                continue

            frame         = cv2.flip(frame, 1)
            mask          = self.process_frame(frame)
            height, width = frame.shape[:2]
            center_x      = width // 2

            # ── 1. Error de línea → PID ───────────────────────────────────────
            error, centroid, main_contour = self.get_contour_center(mask, width, height)

            if error is not None:
                self.send_error(error)

            # ── 2. Detección de intersecciones ────────────────────────────────
            warn, confirmed, y_start, y_end, lateral_w = self.detect_lateral_markers(mask)

            if warn and not self._warn_sent:
                self.send_command("SLOW")
                self._warn_sent = True
                print("-> [VISION] SLOW")

            if confirmed and self._cooldown_ok():
                self._last_mark_time = time.time() * 1000
                self._warn_sent      = False
                self.send_command("MARK")
                print("-> [VISION] MARK")

            if not warn and not confirmed:
                self._warn_sent = False

            # ── 3. HUD — solo si no es headless ──────────────────────────────
            if not HEADLESS:
                warn_y    = int(height * MARK_WARN_BOTTOM)
                color_txt = (0, 255, 0) if (error is not None and abs(error) <= self.alignment_threshold) else (0, 165, 255)

                if error is not None:
                    cv2.putText(frame, f"Error: {error:+.1f}px", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_txt, 2)
                else:
                    cv2.putText(frame, "LINEA PERDIDA", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                col = (0, 255, 0) if confirmed else ((0, 255, 255) if warn else (0, 0, 255))
                cv2.line(frame, (0, y_start),  (width, y_start),  (0, 255, 255), 1)
                cv2.line(frame, (0, y_end),    (width, y_end),    (0, 255, 255), 1)
                cv2.line(frame, (0, warn_y),   (width, warn_y),   (0, 200, 200), 1)
                cv2.line(frame, (center_x, 0), (center_x, height),(255, 0, 0),   1)
                cv2.rectangle(frame, (0, y_start),                (lateral_w, y_end), col, 2)
                cv2.rectangle(frame, (width - lateral_w, y_start),(width, y_end),     col, 2)

                if confirmed:
                    cv2.putText(frame, "INTERSECCION", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                elif warn:
                    cv2.putText(frame, "FRENANDO", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if len(main_contour) > 0 and centroid:
                    cv2.drawContours(frame, [main_contour], -1, (255, 0, 255), 2)
                    cv2.circle(frame, centroid, 6, (255, 255, 0), -1)

                cv2.imshow("Camara Robot", frame)
                cv2.imshow("Mascara",      mask)

                key      = cv2.waitKey(1) & 0xFF
                exit_val = ord(EXIT_KEY) if isinstance(EXIT_KEY, str) else EXIT_KEY
                if key == exit_val:
                    self.send_command("S")
                    break

        self._release()

    def _release(self):
        self.camera.release()
        if not HEADLESS:
            cv2.destroyAllWindows()
        if self.serial_conectado and self.arduino:
            self.send_command("S")
            self.arduino.close()
            print("Conexiones cerradas.")


if __name__ == "__main__":
    port  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SERIAL_PORT
    baud  = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD_RATE
    index = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_CAMERA_INDEX

    robot = LineFollower(camera_index=index, serial_port=port, baud_rate=baud)
    robot.run()