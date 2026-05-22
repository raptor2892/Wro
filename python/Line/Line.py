from __future__ import annotations
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
    DEFAULT_BAUD_RATE,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_SERIAL_PORT,
    EXIT_KEY,
    MASK_KERNEL_SIZE,
)

# ── Configuración de la Zona de Visión (ROI Central) ─────────────────────────
ROI_TOP_MARGIN    = 0.30  # Ignora el 30% superior
ROI_BOTTOM_MARGIN = 0.70  # Ignora a partir del 70%


class LineFollower:
    def __init__(self, camera_index=DEFAULT_CAMERA_INDEX,
                 serial_port=DEFAULT_SERIAL_PORT, baud_rate=DEFAULT_BAUD_RATE):
        self.camera_index = camera_index
        self.serial_port  = serial_port
        self.baud_rate    = baud_rate

        print(f"Abriendo cámara {self.camera_index} (Modo de Selección Automática)...")
        # ── SOLUCIÓN CÁMARA USB: Quitamos el backend rígido para forzar compatibilidad ──
        self.camera = cv2.VideoCapture(self.camera_index)
        
        if not self.camera.isOpened():
            print(f"ERROR: No se pudo abrir la cámara {self.camera_index}.")
            sys.exit(1)

        # Configuración de rendimiento de la cámara
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS,          CAMERA_FPS)
        self.camera.set(cv2.CAP_PROP_EXPOSURE, -5) # Forzar exposición baja anti-reflejos

        print("Calentando sensor dinámicamente...")
        for _ in range(20):
            ret, _ = self.camera.read()
            if ret: 
                break 

        self.serial_conectado = False
        self.arduino          = None
        self.alignment_threshold = ALIGNMENT_THRESHOLD_PX

        # Estado para evitar ráfagas repetidas del mismo cruce (Mapeo por flancos)
        self.mark_left_active = False
        self.mark_right_active = False

        self._setup_serial()

    def _setup_serial(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            time.sleep(2)
            self.serial_conectado = True
            print(f"ESP32 conectado en {self.serial_port} @ {self.baud_rate} baud")
        except Exception as e:
            self.serial_conectado = False
            print(f"ESP32 no detectado en {self.serial_port}: {e}\nModo simulacion activo...")

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
                # 1. Aseguramos que el error sea tratado rigurosamente como flotante
                val_error = float(error)
                
                # 2. Construimos el string con una estructura limpia
                mensaje = f"ERROR:{val_error:.2f}\n"
                
                # 3. Enviamos al puerto serial de forma segura
                self.arduino.write(mensaje.encode("utf-8"))
                
                # 4. Print de diagnóstico limpio para telemetría
                print(f"-> [SERIAL] Error enviado: {val_error:+.2f} px")
                
            except (ValueError, TypeError) as e:
                # Captura si por alguna razón 'error' no era un número válido
                print(f"[VISIÓN ERROR] Dato de error inválido recibido: {error} ({e})")
            except Exception as e:
                # Captura problemas físicos del puerto (desconexiones o saturación)
                print(f"[SERIAL ERROR] No se pudo escribir en el puerto: {e}")
        else:
            print(f"[SIM] Error simulado: {error:.2f} px")

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)
        _, mask = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY_INV)
        
        kernel = np.ones(MASK_KERNEL_SIZE, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) 
        return mask

    def get_contour_center(self, mask, frame_width, frame_height) -> tuple[float | None, tuple[int, int] | None, list]:
        y_start = int(frame_height * ROI_TOP_MARGIN)
        y_end   = int(frame_height * ROI_BOTTOM_MARGIN)
        
        roi_mask = np.zeros_like(mask)
        roi_mask[y_start:y_end, :] = mask[y_start:y_end, :]
        
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, []
            
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 200:
            return None, None, []
            
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None, None, []
            
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        center_x = frame_width // 2
        error = float(cx - center_x)
        return error, (cx, cy), largest_contour

    def detect_lateral_markers(self, mask):
        height, width = mask.shape
        y_start = int(height * ROI_TOP_MARGIN)
        y_end   = int(height * ROI_BOTTOM_MARGIN)
        margin_x = int(width * 0.15)
        
        roi_left  = mask[y_start:y_end, 0:margin_x]
        roi_right = mask[y_start:y_end, width-margin_x:width]
        
        left_detected  = (np.sum(roi_left > 0) / roi_left.size) > 0.35
        right_detected = (np.sum(roi_right > 0) / roi_right.size) > 0.35
        return left_detected, right_detected, y_start, y_end, margin_x

    def run(self):
        print("\n--- Seguidor de Linea Optimizado (Modo Transmisión Pura) ---")
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            mask  = self.process_frame(frame)
            height, width = frame.shape[:2]
            center_x = width // 2

            # 1. Obtener error de la línea central
            error, centroid, main_contour = self.get_contour_center(mask, width, height)

            # 2. Obtener estado de los ROIs laterales para intersecciones
            left_on, right_on, y_start, y_end, margin_x = self.detect_lateral_markers(mask)

            # Enviar marcas laterales al ESP32 solo en el flanco de subida (detección inicial)
            if left_on and not self.mark_left_active:
                self.send_command("MARK:L")
                print("-> [VISIÓN] MARK:L")
            self.mark_left_active = left_on

            if right_on and not self.mark_right_active:
                self.send_command("MARK:R")
                print("-> [VISIÓN] MARK:R")
            self.mark_right_active = right_on

            # 3. ── FLUJO CORRECTO: Envío de error directo sin importar las marcas ──
            if error is not None:
                self.send_error(error)
                
                # HUD en pantalla: Estado del error
                alineado = abs(error) <= self.alignment_threshold
                color_txt = (0, 255, 0) if alineado else (0, 165, 255)
                cv2.putText(frame, f"Error: {error:+.1f}px", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_txt, 2)
            else:
                cv2.putText(frame, "LINEA PERDIDA", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # ── Dibujar HUD Visual de Competencia ──
            cv2.line(frame, (0, y_start), (width, y_start), (0, 255, 255), 1) 
            cv2.line(frame, (0, y_end), (width, y_end), (0, 255, 255), 1) 
            cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 1)

            # Cajas de detección lateral (Verde si detecta marca, Rojo si está vacía)
            cv2.rectangle(frame, (0, y_start), (margin_x, y_end), (0, 255, 0) if left_on else (0, 0, 255), 2)
            cv2.rectangle(frame, (width - margin_x, y_start), (width, y_end), (0, 255, 0) if right_on else (0, 0, 255), 2)

            if len(main_contour) > 0 and centroid:
                cv2.drawContours(frame, [main_contour], -1, (255, 0, 255), 2)
                cv2.circle(frame, centroid, 6, (255, 255, 0), -1)

            cv2.imshow("Camara Robot - Vista Principal", frame)
            cv2.imshow("Mascara Threshold", mask)

            # Salida limpia con la tecla de escape configurada
            key = cv2.waitKey(1) & 0xFF
            exit_val = ord(EXIT_KEY) if isinstance(EXIT_KEY, str) else EXIT_KEY
            if key == exit_val:
                self.send_command("S")
                break

        self._release()

    def _release(self):
        self.camera.release()
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