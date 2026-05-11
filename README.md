# Wro
prepa 8 torneo de wro

## Grid detector

El script `grid-detector` ahora acepta dos modos desde terminal:

- `get grid` para capturar desde la cámara.
- `get grid /ruta/a/imagen.jpg` para procesar una imagen ya subida al proyecto.

Cuando detecta la cuadrícula, abre una ventana con la imagen marcada, guarda la foto y guarda un JSON con los datos detectados en `captures/`.
