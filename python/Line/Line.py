import cv2
import numpy as np
import serial
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from line_constants import (
    ALIGNMENT_THRESHOLD_PX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DARK_LINE_HSV_MAX,
    DARK_LINE_HSV_MIN,
    DEFAULT_BAUD_RATE,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_SERIAL_PORT,
    EXIT_KEY,
    LEFT_TURN_ERROR_THRESHOLD_PX,
    LINE_DETECTION_RATIO,
    MASK_KERNEL_SIZE,
    MASK_POINTS,
    MAX_FRAMES_WITHOUT_LINE,
    RIGHT_TURN_ERROR_THRESHOLD_PX,
    SIGNAL_ROW_FRACTION,
)

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURACION PID  <- ajusta estos valores
# ══════════════════════════════════════════════════════════════════════════════
PID_KP          = 0.4    # Proporcional  — cuanto corrige por pixel de error
PID_KI          = 0.001  # Integral      — corrige error acumulado (empieza en 0)
PID_KD          = 0.9    # Derivativo    — amortigua oscilaciones

BASE_SPEED      = 220    # Velocidad base de ambos motores (0-255)
MAX_SPEED       = 255    # Limite superior
MIN_SPEED       = 80     # Limite inferior (evita que el motor se detenga)
# ══════════════════════════════════════════════════════════════════════════════


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()

    def compute(self, error: float) -> float:
        """Devuelve la correccion PID dado el error en pixeles."""
        now = time.time()
        dt  = now - self._prev_time
        if dt <= 0:
            dt = 1e-3

        self._integral  += error * dt
        # Anti-windup: limita el integral para que no explote
        self._integral   = max(-500, min(500, self._integral))

        derivative       = (error - self._prev_error) / dt
        correction       = self.kp * error + self.ki * self._integral + self.kd * derivative

        self._prev_error = error
        self._prev_time  = now
        return correction


class LineFollower:
    def __init__(self, camera_index=DEFAULT_CAMERA_INDEX,
                 serial_port=DEFAULT_SERIAL_PORT, baud_rate=DEFAULT_BAUD_RATE):
        self.camera_index = camera_index
        self.serial_port  = serial_port
        self.baud_rate    = baud_rate

        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            print(f"ERROR: No se pudo abrir la camara {self.camera_index}.")
            sys.exit(1)

        self.serial_conectado = False
        self.arduino          = None

        self.skip_mode             = False
        self.skip_frames           = 0
        self.last_movement         = "A"
        self.frames_without_line   = 0
        self.max_frames_without_line = MAX_FRAMES_WITHOUT_LINE
        self.alignment_threshold   = ALIGNMENT_THRESHOLD_PX

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)

        self.lower_hsv = np.array(DARK_LINE_HSV_MIN)
        self.upper_hsv = np.array(DARK_LINE_HSV_MAX)

        self.pid = PIDController(PID_KP, PID_KI, PID_KD)

        self._setup_serial()

    # ── Serial ─────────────────────────────────────────────────────────────────

    def _setup_serial(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            time.sleep(2)
            self.serial_conectado = True
            print(f"ESP32 conectado en {self.serial_port} @ {self.baud_rate} baud")
        except Exception as e:
            self.serial_conectado = False
            print(f"ESP32 no detectado en {self.serial_port}: {e}")
            print("Modo simulacion (solo video)...")

    def send_command(self, command: str):
        """Envia cualquier comando terminado en \\n."""
        if self.serial_conectado and self.arduino:
            try:
                self.arduino.write((command + "\n").encode("utf-8"))
                if self.arduino.in_waiting > 0:
                    resp = self.arduino.readline().decode("utf-8", errors="ignore").strip()
                    if resp:
                        print(f"<- ESP32: {resp}")
            except Exception as e:
                print(f"Error serial: {e}")
        else:
            print(f"[SIM] {command}")

    def send_pid(self, vel_izq: int, vel_der: int):
        """Envia velocidades PID: PID:velIzq,velDer"""
        vel_izq = max(MIN_SPEED, min(MAX_SPEED, vel_izq))
        vel_der = max(MIN_SPEED, min(MAX_SPEED, vel_der))
        self.send_command(f"PID:{vel_izq},{vel_der}")

    # ── Vision ─────────────────────────────────────────────────────────────────

    def process_frame(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        kernel = np.ones(MASK_KERNEL_SIZE, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        return mask

    def get_line_centers(self, mask, points=MASK_POINTS):
        height, width = mask.shape
        step = height // (points + 1)
        positions = []
        for i in range(1, points + 1):
            y   = i * step
            idx = np.where(mask[y, :] > 0)[0]
            positions.append((int(np.mean(idx)), y) if len(idx) > 0 else None)
        return positions

    def get_error(self, positions, frame_width) -> float | None:
        """Error en pixeles del punto 3 respecto al centro del frame."""
        center_x = frame_width // 2
        point3   = positions[2] if len(positions) > 2 else None
        if point3 is None:
            return None
        return float(point3[0] - center_x)

    def detect_horizontal_line(self, mask) -> bool:
        height, width = mask.shape
        y   = int(height * SIGNAL_ROW_FRACTION)
        return np.sum(mask[y, :] > 0) > (LINE_DETECTION_RATIO * width)

    # ── Loop principal ──────────────────────────────────────────────────────────

    def run(self):
        print("\n--- Seguidor de Linea con PID ---")
        print(f"Puerto: {self.serial_port} | Baud: {self.baud_rate}")
        print(f"KP={PID_KP}  KI={PID_KI}  KD={PID_KD}  Base={BASE_SPEED}")
        print("Presiona 'q' para salir.\n")

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("Error capturando frame.")
                break

            frame     = cv2.flip(frame, 1)
            mask      = self.process_frame(frame)
            positions = self.get_line_centers(mask)
            error     = self.get_error(positions, frame.shape[1])
            center_x  = frame.shape[1] // 2

            # ── Deteccion de interseccion ───────────────────────────────────
            if self.detect_horizontal_line(mask):
                self.skip_mode   = True
                self.skip_frames = 20
                self.pid.reset()
                cv2.putText(frame, "INTERSECCION", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # ── SKIP MODE: avanza recto ignorando la interseccion ───────────
            if self.skip_mode:
                self.send_pid(BASE_SPEED, BASE_SPEED)
                self.skip_frames -= 1
                if self.skip_frames <= 0:
                    self.skip_mode = False
                cv2.putText(frame, "Ignorando cruce", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ── SIN LINEA ───────────────────────────────────────────────────
            elif error is None:
                self.frames_without_line += 1
                if self.frames_without_line > self.max_frames_without_line:
                    # Reversa suave
                    self.send_command("R")
                    cv2.putText(frame, "Sin linea - Reversa", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    # Mantiene ultimo comando
                    cv2.putText(frame, "Buscando linea...", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # ── MODO PID NORMAL ─────────────────────────────────────────────
            else:
                self.frames_without_line = 0
                correction = self.pid.compute(error)

                # Error > 0 → linea a la derecha → motor izq mas rapido
                # Error < 0 → linea a la izquierda → motor der mas rapido
                vel_izq = int(BASE_SPEED + correction)
                vel_der = int(BASE_SPEED - correction)

                self.send_pid(vel_izq, vel_der)

                # UI
                alineado = abs(error) <= self.alignment_threshold
                color    = (0, 255, 0) if alineado else (0, 165, 255)
                estado   = "ALINEADO" if alineado else f"Error: {error:+.0f}px"
                cv2.putText(frame, estado, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(frame, f"PID: {correction:+.1f}  L:{int(max(MIN_SPEED,min(MAX_SPEED,vel_izq)))} R:{int(max(MIN_SPEED,min(MAX_SPEED,vel_der)))}", 
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            # ── Dibujar puntos ──────────────────────────────────────────────
            for i, point in enumerate(positions):
                if point is None:
                    continue
                if i == 2:
                    col = (0, 255, 0) if (error is not None and abs(error) <= self.alignment_threshold) else (0, 0, 255)
                    cv2.circle(frame, point, 8, col, -1)
                    cv2.putText(frame, "P3", (point[0] + 10, point[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                else:
                    cv2.circle(frame, point, 5, (0, 255, 255), -1)

            # Lineas de referencia
            cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)
            cv2.line(frame, (center_x - self.alignment_threshold, 0),
                     (center_x - self.alignment_threshold, frame.shape[0]), (0, 200, 0), 1)
            cv2.line(frame, (center_x + self.alignment_threshold, 0),
                     (center_x + self.alignment_threshold, frame.shape[0]), (0, 200, 0), 1)

            cv2.imshow("Seguidor de Linea - PID", frame)
            cv2.imshow("Mascara HSV", mask)

            if cv2.waitKey(1) & 0xFF == EXIT_KEY:
                self.send_command("S")
                break

        self._release()

    def _release(self):
        self.camera.release()
        cv2.destroyAllWindows()
        if self.serial_conectado and self.arduino:
            self.send_command("S")
            self.arduino.close()
            print("ESP32 desconectado.")


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Lanzado por el daemon:  python Line.py COM3 9600
    # Lanzado solo:           python Line.py           (usa defaults de line_constants)
    port  = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SERIAL_PORT
    baud  = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD_RATE
    index = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_CAMERA_INDEX

    robot = LineFollower(camera_index=index, serial_port=port, baud_rate=baud)
    robot.run()