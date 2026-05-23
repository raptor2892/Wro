# Configuración de puertos — Line Follower

## 1. Puerto Serial (ESP32)

### Windows
1. Conecta el ESP32 por USB
2. Abre el **Administrador de dispositivos** (`Win + X → Administrador de dispositivos`)
3. Busca **Puertos (COM y LPT)**
4. Aparece algo como `USB-SERIAL CH340 (COM3)` — ese es tu puerto
5. En `line_constants.py` pon:
```python
   DEFAULT_SERIAL_PORT = "COM3"  # el que viste
```

### Linux / Raspberry Pi
```bash
# Antes de conectar el ESP32
ls /dev/tty*

# Conecta el ESP32 y vuelve a correr
ls /dev/tty*

# El que apareció nuevo es tu puerto, normalmente:
# /dev/ttyUSB0  → cable USB con chip CH340/CP2102
# /dev/ttyACM0  → cable USB con chip nativo
```
En `line_constants.py` pon:
```python
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"  # el que apareció
```

#### Si da error de permisos en Linux
```bash
sudo usermod -aG dialout $USER
# Cierra sesión y vuelve a entrar para que aplique
```

---

## 2. Índice de cámara

### Windows y Linux
```bash
# Prueba índices del 0 al 3 hasta que funcione
python3 -c "
import cv2
for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camara encontrada en indice: {i}')
        cap.release()
"
```
En `line_constants.py` pon:
```python
DEFAULT_CAMERA_INDEX = 0  # el índice que funcionó
```

### Raspberry Pi — cámara USB
Igual que arriba, normalmente es índice `0`.

Verifica que el sistema la ve:
```bash
ls /dev/video*
# Si aparece /dev/video0 → índice 0
# Si aparece /dev/video1 → índice 1
```

### Raspberry Pi — módulo de cámara oficial (ribbon cable)
El módulo oficial **no funciona con OpenCV directamente**.
Necesitas habilitarlo primero:
```bash
sudo raspi-config
# Interface Options → Camera → Enable → Reboot
```
Luego instala `picamera2`:
```bash
pip install picamera2
```
Y en `Line.py` cambia `CameraStream` para usar `picamera2` en vez de `cv2.VideoCapture`.

---

## 3. Verificar la conexión serial

```bash
# Linux/Pi — ver mensajes del ESP32 en tiempo real
cat /dev/ttyUSB0

# O con Python
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)
while True:
    line = s.readline()
    if line:
        print(line.decode('utf-8', errors='replace').strip())
"
```

### Windows
Usa el **Monitor Serial de Arduino IDE**:
1. Abre Arduino IDE
2. `Herramientas → Monitor Serial`
3. Selecciona el mismo COM y baud rate `115200`

---

## 4. Arrancar automático en la Raspberry Pi

Para que `Line.py` corra solo al encender:

```bash
# Editar crontab
crontab -e

# Agregar esta línea al final
@reboot sleep 10 && python3 /home/pi/tu_carpeta/Line/Line.py >> /home/pi/line.log 2>&1 &
```

El `sleep 10` da tiempo a que el sistema levante los drivers USB antes de intentar abrir la cámara y el serial.

Ver el log si algo falla:
```bash
cat /home/pi/line.log
```

---

## 5. Resumen de variables en `line_constants.py`

| Variable | Windows ejemplo | Raspberry Pi ejemplo |
|---|---|---|
| `DEFAULT_SERIAL_PORT` | `"COM3"` | `"/dev/ttyUSB0"` |
| `DEFAULT_CAMERA_INDEX` | `0` | `0` |
| `DEFAULT_BAUD_RATE` | `115200` | `115200` |

---

## 6. Correr manualmente

```bash
# Con valores por defecto de line_constants.py
python3 Line.py

# Sobrescribir puerto, baud y cámara desde la terminal
python3 Line.py /dev/ttyUSB0 115200 0
```