import cv2
import numpy as np
import argparse
import sys
import time

from pathlib import Path

# Buscamos la carpeta raíz 'Wro' (subiendo dos niveles desde Line.py)
ROOT = Path(__file__).resolve().parent.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


from python.Line.line_constants import (
    ALIGNMENT_THRESHOLD_PX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DARK_LINE_HSV_MAX,
    DARK_LINE_HSV_MIN,
    DEFAULT_ALIGNMENT_MARGIN_PX,
    DEFAULT_CAMERA_INDEX,
    LINE_DETECTION_RATIO,
    MASK_KERNEL_SIZE,
    MASK_POINTS,
    SIGNAL_ROW_FRACTION,
)




# Test script: captura cámara y prueba la detección de línea sin conectar Arduino
# Uso: python3 test_line.py --cam 0 --umbral 15


def procesar_frame(frame, limite_inferior, limite_superior):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, limite_inferior, limite_superior)
    kernel = np.ones(MASK_KERNEL_SIZE, np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
    return mascara


def obtener_centros_linea(mascara, puntos=MASK_POINTS):
    alto, ancho = mascara.shape
    posiciones = []
    paso = alto // (puntos + 1)
    for i in range(1, puntos + 1):
        y = i * paso
        fila = mascara[y, :]
        indices = np.where(fila > 0)[0]
        if len(indices) > 0:
            cx = int(np.mean(indices))
            posiciones.append((cx, y))
        else:
            posiciones.append(None)
    return posiciones


def verificar_alineacion_punto3(posiciones, ancho_frame, umbral_alineacion=ALIGNMENT_THRESHOLD_PX):
    centro_x = ancho_frame // 2
    punto3 = posiciones[2] if len(posiciones) > 2 else None
    if punto3 is None:
        return False, None
    error_pixeles = punto3[0] - centro_x
    alineado = abs(error_pixeles) <= umbral_alineacion
    return alineado, error_pixeles


def decidir_movimiento_con_alineacion(posiciones, ancho_frame, umbral_alineacion=ALIGNMENT_THRESHOLD_PX):
    alineado, error_px = verificar_alineacion_punto3(posiciones, ancho_frame, umbral_alineacion)
    if error_px is None:
        return None, None, None
    if alineado:
        return "A", error_px, True
    else:
        if error_px < -5:
            return "I", error_px, False
        elif error_px > 5:
            return "D", error_px, False
        else:
            return "A", error_px, False


def detectar_linea_horizontal(mascara):
    alto, ancho = mascara.shape
    y = int(alto * SIGNAL_ROW_FRACTION)
    fila = mascara[y, :]
    cantidad_blanco = np.sum(fila > 0)
    return cantidad_blanco > (LINE_DETECTION_RATIO * ancho)


def main():
    parser = argparse.ArgumentParser(description="Test de línea con cámara (sin Arduino)")
    parser.add_argument("--cam", type=int, default=DEFAULT_CAMERA_INDEX, help="Índice de la cámara (por defecto 0)")
    parser.add_argument("--umbral", type=int, default=DEFAULT_ALIGNMENT_MARGIN_PX, help="Umbral de alineación en píxeles")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    # Rango HSV para detectar negro (ajusta si tu línea tiene otro color)
    limite_inferior = np.array(DARK_LINE_HSV_MIN)
    limite_superior = np.array(DARK_LINE_HSV_MAX)

    print("Iniciando test de línea (sin Arduino). Presiona 'q' para salir.")
    ultimo_mov = "A"

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer de la cámara")
            break

        frame = cv2.flip(frame, 1)
        mascara = procesar_frame(frame, limite_inferior, limite_superior)
        posiciones = obtener_centros_linea(mascara, puntos=5)

        # Detectar línea horizontal (señal)
        if detectar_linea_horizontal(mascara):
            comando = "A"
            estado = "LINEA HORIZONTAL - AVANZANDO (simulado)"
        else:
            comando, error_px, alineado = decidir_movimiento_con_alineacion(posiciones, frame.shape[1], args.umbral)
            if comando is None:
                comando = ultimo_mov
                estado = "Sin punto 3 detectado - esperando (simulado)"
            else:
                if alineado:
                    estado = "ALINEADO (simulado)"
                else:
                    estado = f"Corrigiendo {comando} ({error_px:+.0f}px) (simulado)"

        # Mostrar resultados en frame
        centro_x = frame.shape[1] // 2
        for i, p in enumerate(posiciones):
            if p is not None:
                if i == 2:
                    _, error_px_draw = verificar_alineacion_punto3(posiciones, frame.shape[1], args.umbral)
                    if error_px_draw is not None and abs(error_px_draw) <= args.umbral:
                        color = (0, 255, 0)
                        grosor = 8
                    else:
                        color = (0, 0, 255)
                        grosor = 8
                    cv2.circle(frame, p, grosor, color, -1)
                    cv2.putText(frame, "P3", (p[0]+12, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    color = (0, 255, 255)
                    cv2.circle(frame, p, 5, color, -1)
                    cv2.putText(frame, str(i+1), (p[0]+10, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        cv2.line(frame, (centro_x, 0), (centro_x, frame.shape[0]), (255, 0, 0), 2)
        cv2.line(frame, (centro_x - args.umbral, 0), (centro_x - args.umbral, frame.shape[0]), (0, 255, 0), 1)
        cv2.line(frame, (centro_x + args.umbral, 0), (centro_x + args.umbral, frame.shape[0]), (0, 255, 0), 1)

        cv2.putText(frame, f"Comando simulado: {comando}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, estado, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Test Line - Frame", frame)
        cv2.imshow("Test Line - Mascara", mascara)

        # Imprimir comando simulado en consola una vez cada cierto tiempo para no spamear
        print(f"Comando simulado: {comando} - {estado}", end='\r')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print()  # nueva línea después del \r
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
