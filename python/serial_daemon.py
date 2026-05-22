"""
Daemon serial — intermediario entre ESP32 y módulos Python.

ARQUITECTURA:
  - El daemon es el ÚNICO dueño del puerto serial.
  - FOLLOW_LINE  -> cierra el puerto, lanza Line.py en ventana propia, espera pasiva.
  - STOP_LINE    -> mata Line.py, recupera el puerto, manda "S\n".
  - DETECT_MOSAIC-> si Line.py está activo lo detiene, luego procesa visión.
  - Mensajes de boot del ESP32 se ignoran silenciosamente.

MODO SIMULADOR (SIMULATE = True):
  Sin hardware. Escribe en consola:
    > FOLLOW_LINE  |  > STOP_LINE  |  > DETECT_MOSAIC  |  > exit

ESTRUCTURA:
  Wro/python/
  ├── serial_daemon.py
  ├── Line/
  │   ├── Line.py
  │   └── line_constants.py
  └── Vision/
      ├── constants.py
      └── grid-detector   (archivo sin extension .py)
"""

import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from tkinter import TRUE

import serial

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURACION  <- edita solo esta seccion
# ══════════════════════════════════════════════════════════════════════════════
PORT         = "COM3"
BAUD_RATE    = 115200
SIMULATE     = False # True = sin hardware | False = con ESP32
WINDOW_TITLE = "LineFollowerWRO"   # sin espacios ni caracteres especiales
# ══════════════════════════════════════════════════════════════════════════════

ROOT        = Path(__file__).resolve().parent   # .../Wro/python/
LINE_SCRIPT = ROOT / "Line" / "Line.py"
GRID_FILE   = ROOT / "Vision" / "grid-detector"

# ── Estado global ──────────────────────────────────────────────────────────────
_ser: serial.Serial | None = None
_line_proc: subprocess.Popen | None = None
_serial_lock = threading.Lock()
_daemon_activo = True


# ══════════════════════════════════════════════════════════════════════════════
#  MANEJO DEL PUERTO SERIAL
# ══════════════════════════════════════════════════════════════════════════════

def _abrir_puerto() -> bool:
    global _ser
    if SIMULATE:
        return True
    with _serial_lock:
        if _ser and _ser.is_open:
            return True
        try:
            _ser = serial.Serial(PORT, BAUD_RATE, timeout=0.5)
            time.sleep(5)   # estabilizacion del ESP32 tras reset
            log.info("Puerto %s abierto por el Daemon.", PORT)
            return True
        except serial.SerialException as e:
            log.error("No se pudo abrir %s: %s", PORT, e)
            return False

def _cerrar_puerto():
    global _ser
    with _serial_lock:
        if _ser and _ser.is_open:
            try:
                _ser.close()
                log.info("Puerto %s liberado por el Daemon.", PORT)
            except Exception as e:
                log.error("Error cerrando el puerto: %s", e)

def serial_send(data: str):
    """Envia texto al ESP32. En simulacion solo loguea."""
    if SIMULATE:
        log.info("[SIM ->] %s", data.strip())
        return
    with _serial_lock:
        if _ser and _ser.is_open:
            try:
                _ser.write(data.encode("utf-8"))
            except serial.SerialException as e:
                log.error("Error escribiendo en serial: %s", e)
        else:
            log.warning("Intento de envio con puerto cerrado: %s", data.strip())


# ══════════════════════════════════════════════════════════════════════════════
#  FILTRO DE MENSAJES DEL ESP32
# ══════════════════════════════════════════════════════════════════════════════

# Prefijos de mensajes de boot/diagnostico — se ignoran silenciosamente
_IGNORED_PREFIXES = (
    "ets ", "rst:", "configsip", "clk_drv", "mode:DIO", "load:0x",
    "entry 0x", "Iniciando", "Bus I2C", "Escaneando", "Dispositivo",
    "BNO08", "Reportes", "System ready", ">>>", "ACK",
)

def _es_basura(line: str) -> bool:
    """Descarta lineas con demasiados bytes no imprimibles (basura de boot)."""
    if not line or len(line) < 3:
        return True
    no_printable = sum(1 for c in line if not c.isprintable())
    return no_printable > len(line) * 0.3

def _es_ignorado(line: str) -> bool:
    return any(line.startswith(p) for p in _IGNORED_PREFIXES)


# ══════════════════════════════════════════════════════════════════════════════
#  CARGA DINAMICA DE MODULOS DE VISION
# ══════════════════════════════════════════════════════════════════════════════

def _load_grid_modules():
    import importlib.util
    from importlib.machinery import SourceFileLoader

    vision_dir = str(GRID_FILE.parent)
    if vision_dir not in sys.path:
        sys.path.insert(0, vision_dir)

    loader = SourceFileLoader("grid_detector", str(GRID_FILE))
    spec   = importlib.util.spec_from_file_location(
        "grid_detector", str(GRID_FILE), loader=loader
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.GridDetector, mod.build_color_map, mod.send_grid_serial


# ══════════════════════════════════════════════════════════════════════════════
#  HANDLERS
# ══════════════════════════════════════════════════════════════════════════════

def handle_follow_line(_: str):
    global _line_proc, _daemon_activo

    if _line_proc is not None and _line_proc.poll() is None:
        log.info("FOLLOW_LINE ignorado: Line.py ya esta activo (PID %s).", _line_proc.pid)
        return

    log.info("FOLLOW_LINE -> cediendo UART y lanzando Line.py...")

    if not SIMULATE:
        _cerrar_puerto()
        time.sleep(0.3)

    try:
        if os.name == "nt":  # Windows
            # Pasa PORT y BAUD_RATE como argumentos para que Line.py use el mismo puerto
            _line_proc = subprocess.Popen(
                [sys.executable, str(LINE_SCRIPT), PORT, str(BAUD_RATE)],
                creationflags=subprocess.CREATE_NEW_CONSOLE,
            )
        else:
            _line_proc = subprocess.Popen(
                ["x-terminal-emulator", "-e",
                 sys.executable, str(LINE_SCRIPT), PORT, str(BAUD_RATE)]
            )
        log.info("Line.py iniciado (PID %s).", _line_proc.pid)

        # Espera pasiva mientras Line.py corre
        _daemon_activo = False
        while _line_proc is not None and _line_proc.poll() is None:
            time.sleep(0.5)

        log.info("Line.py termino. Recuperando control serial...")
        if not SIMULATE:
            time.sleep(0.3)
            _abrir_puerto()
        _daemon_activo = True

    except Exception as e:
        log.error("No se pudo lanzar Line.py: %s", e)
        if not SIMULATE:
            _abrir_puerto()
        _daemon_activo = True


def handle_stop_line(_: str):
    global _line_proc

    if _line_proc is None or _line_proc.poll() is not None:
        log.info("STOP_LINE: Line.py no estaba corriendo.")
    else:
        log.info("STOP_LINE -> terminando Line.py (PID %s).", _line_proc.pid)
        try:
            _line_proc.terminate()
            _line_proc.wait(timeout=2)
        except Exception:
            try:
                _line_proc.kill()
            except Exception:
                pass

        # taskkill por titulo (sin caracteres especiales)
        if os.name == "nt":
            os.system(f"taskkill /F /FI \"WINDOWTITLE eq {WINDOW_TITLE}\" >nul 2>&1")

        _line_proc = None
        log.info("Line.py terminado.")

    if not SIMULATE:
        time.sleep(0.15)
        if _abrir_puerto():
            time.sleep(0.1)
            serial_send("S\n")
            log.info("Comando de parada 'S' enviado al robot.")


def handle_detect_mosaic(_: str):
    global _line_proc

    if _line_proc is not None and _line_proc.poll() is None:
        log.warning("Mosaico solicitado con linea activa. Forzando STOP_LINE...")
        handle_stop_line(_)
        time.sleep(0.5)

    log.info("DETECT_MOSAIC -> procesando vision...")

    try:
        GridDetector, build_color_map, _ = _load_grid_modules()
    except Exception as e:
        log.error("No se pudo cargar GridDetector: %s", e)
        serial_send("MOSAIC_ERROR:load_failed\n")
        return

    try:
        detector      = GridDetector()
        frame, result = detector.process_frame()

        if result is None or not result.get("grid_detected"):
            log.warning("Mosaico no detectado.")
            serial_send("MOSAIC_ERROR:not_detected\n")
            return

        palette   = result.get("palette", [])
        color_map = build_color_map(palette)
        matrix    = result.get("grid", {}).get("matrix", [])
        entries, serial_str = _build_serial_string(matrix, color_map)

        serial_send(serial_str)
        log.info("Mosaico enviado (%d celdas).", len(entries))

        detector.save_grid_capture(frame, result)
        detector.release()

    except Exception as e:
        log.error("Error procesando mosaico: %s", e)
        serial_send("MOSAIC_ERROR:runtime\n")


def _build_serial_string(grid_matrix, color_map):
    entries = []
    for r, row in enumerate(grid_matrix):
        for c, cell in enumerate(row):
            color = color_map.get(cell["color"], "desconocido") if cell else "desconocido"
            entries.append(f"{r},{c}={color}")
    return entries, ";".join(entries) + "\n"


def handle_ack(line: str):
    log.info("ACK: %s", line.split(":", 1)[-1].strip())

def handle_error(line: str):
    log.error("ERROR ESP32: %s", line.split(":", 1)[-1].strip() if ":" in line else line)


# ── Tabla de despacho ──────────────────────────────────────────────────────────
DISPATCH = {
    "FOLLOW_LINE":   handle_follow_line,
    "STOP_LINE":     handle_stop_line,
    "DETECT_MOSAIC": handle_detect_mosaic,
    "ACK:":          handle_ack,
    "ERROR":         handle_error,
}

def _dispatch(line: str):
    # 1. Descartar basura binaria
    if _es_basura(line):
        return
    # 2. Ignorar mensajes de boot/diagnostico
    if _es_ignorado(line):
        log.debug("[boot] %s", line)
        return
    # 3. Despachar comandos reales
    for prefix, handler in DISPATCH.items():
        if line.startswith(prefix):
            threading.Thread(target=handler, args=(line,), daemon=True).start()
            return
    # 4. Cualquier otra cosa: debug silencioso
    log.debug("Ignorado: %r", line)


# ══════════════════════════════════════════════════════════════════════════════
#  BUCLES PRINCIPALES
# ══════════════════════════════════════════════════════════════════════════════

def _run_simulator():
    print("\n+--------------------------------------+")
    print("|   MODO SIMULADOR  (SIMULATE=True)    |")
    print("+--------------------------------------+")
    print("|  Comandos disponibles:               |")
    print("|    > FOLLOW_LINE                     |")
    print("|    > STOP_LINE                       |")
    print("|    > DETECT_MOSAIC                   |")
    print("|    > exit                            |")
    print("+--------------------------------------+\n")

    while True:
        try:
            line = input("[SIM] > ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line.lower() in ("exit", "quit", "q"):
            break
        log.info("[SIM <-] %s", line)
        _dispatch(line)

    _cleanup()


def _run_serial():
    global _daemon_activo

    if not _abrir_puerto():
        sys.exit(1)

    log.info("Daemon escuchando en %s @ %d baud  (Ctrl+C para salir)", PORT, BAUD_RATE)

    try:
        while True:
            # Espera pasiva mientras Line.py tiene el control
            if not _daemon_activo:
                time.sleep(0.2)
                continue

            with _serial_lock:
                puerto_ok = _ser is not None and _ser.is_open

            if not puerto_ok:
                time.sleep(0.2)
                continue

            try:
                with _serial_lock:
                    raw = _ser.readline()
            except serial.SerialException as e:
                if _daemon_activo:
                    log.error("Error de lectura serial: %s", e)
                time.sleep(0.5)
                continue

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                log.debug("<- %s", line)
                _dispatch(line)

    except KeyboardInterrupt:
        log.info("Interrumpido por el usuario.")
    finally:
        _cleanup()


def _cleanup():
    global _line_proc
    if _line_proc and _line_proc.poll() is None:
        log.info("Cerrando Line.py (PID %s)...", _line_proc.pid)
        _line_proc.terminate()
        if os.name == "nt":
            os.system(f"taskkill /F /FI \"WINDOWTITLE eq {WINDOW_TITLE}\" >nul 2>&1")
    _cerrar_puerto()
    log.info("Daemon fuera de linea.")


def main():
    if SIMULATE:
        _run_simulator()
    else:
        _run_serial()


if __name__ == "__main__":
    main()