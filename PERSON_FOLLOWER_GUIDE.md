# Person Follower - Guía de Uso

## Descripción
Sistema de seguimiento de persona usando Kinect. El robot te mantiene en cuadro y a una distancia constante mientras caminas de espalda.

## Características
- ✅ Detección basada en profundidad (depth camera)
- ✅ Control de distancia objetivo
- ✅ Centrado automático (mantiene persona en centro del cuadro)
- ✅ Control proporcional suave
- ✅ Imagen de debug para visualizar detección
- ✅ Seguridad: se detiene si no detecta persona

## Lanzamiento Rápido

### Modo Prueba (sin Arduino - solo visualización)
```bash
source install/setup.bash
ros2 launch smart_t_ai_v2 person_follower.launch.py use_arduino:=false
```

### Modo Real (con Arduino - robot se mueve)
```bash
source install/setup.bash
ros2 launch smart_t_ai_v2 person_follower.launch.py use_arduino:=true
```

### Sin RViz (solo terminal)
```bash
ros2 launch smart_t_ai_v2 person_follower.launch.py use_rviz:=false use_arduino:=true
```

## Visualización en RViz

1. **LaserScan** - No aplica para person follower
2. **Kinect RGB** - Imagen de la cámara
3. **Kinect Depth** - Imagen de profundidad
4. **Debug Image** - Añadir manualmente:
   - Click en "Add" → "By topic"
   - Seleccionar `/person_follower/debug_image`
   - Tipo: Image
   
La imagen debug muestra:
- **Verde**: ROI (región de interés)
- **Rojo**: Píxeles detectados como persona
- **Azul**: Centroide de la persona
- **Amarillo**: Línea del centro al centroide (error lateral)

## Parámetros Configurables

Editar: `config/person_follower.yaml`

### Distancias (metros)
```yaml
target_distance: 1.2      # Distancia objetivo (ajusta según preferencia)
min_distance: 0.6         # Distancia mínima de seguridad
max_distance: 2.5         # Rango máximo de detección
```

### Velocidades (ajustar según terreno)
```yaml
max_linear_speed: 0.35    # Velocidad máxima hacia adelante (m/s)
max_angular_speed: 0.9    # Velocidad máxima de giro (rad/s)
```

### Control (ajuste fino del comportamiento)
```yaml
linear_kp: 0.6            # Mayor = más agresivo corrigiendo distancia
angular_kp: 1.4           # Mayor = más agresivo al centrar
```

### Procesamiento
```yaml
roi_width_ratio: 0.65     # Ancho del área central (65%)
roi_height_ratio: 0.75    # Alto del área central (75%)
min_detection_area: 600   # Píxeles mínimos para detección válida
```

## Cómo Usar

1. **Posicionamiento inicial:**
   - Colócate frente al robot a ~1.5m
   - Asegúrate de estar en el campo de visión del Kinect
   - El Kinect debe estar a altura del torso/pecho

2. **Iniciar seguimiento:**
   - Lanza el sistema con `use_arduino:=true`
   - Espera a ver en los logs: `Person detected | Distance: X.XXm`
   - El robot comenzará a ajustar su posición

3. **Caminar:**
   - Camina **de espalda** hacia donde quieras que te siga el robot
   - Mantén velocidad moderada (~0.5 m/s)
   - El robot te seguirá manteniendo la distancia objetivo

4. **Detener:**
   - Sal del campo de visión o aléjate >2.5m
   - El robot se detendrá automáticamente
   - O presiona `Ctrl+C` en la terminal

## Troubleshooting

### El robot no me detecta
- Verifica que el Kinect esté encendido (LED verde)
- Comprueba que publicas depth: `ros2 topic echo /kinect/depth/image_raw`
- Acércate más (< 2.5m)
- Asegúrate de estar en el centro del campo de visión

### El robot se mueve muy lento/rápido
- Ajusta `max_linear_speed` en `person_follower.yaml`
- Ajusta `linear_kp` (ganancia proporcional)

### El robot gira demasiado al centrarme
- Reduce `angular_kp` en `person_follower.yaml`
- Reduce `max_angular_speed`

### El robot se detiene constantemente
- Reduce `min_detection_area` (menos píxeles requeridos)
- Aumenta `roi_width_ratio` y `roi_height_ratio` (ROI más grande)
- Verifica que no haya obstáculos entre tú y el robot

### Imagen debug no aparece
- Verifica: `ros2 topic list | grep debug_image`
- En RViz: Add → By topic → `/person_follower/debug_image`
- Asegúrate de que `enable_debug: true` en config

## Topics Importantes

### Subscriptions
- `/kinect/depth/image_raw` - Imagen de profundidad del Kinect

### Publications
- `/cmd_vel` - Comandos de velocidad al robot
- `/person_follower/debug_image` - Imagen de debug (opcional)

## Arquitectura del Sistema

```
Kinect → /kinect/depth/image_raw → Person Follower → /cmd_vel → Arduino Bridge → Motores
                                          ↓
                                 /person_follower/debug_image → RViz
```

## Seguridad

⚠️ **IMPORTANTE:**
- Siempre prueba primero con `use_arduino:=false`
- Verifica en RViz que la detección funciona correctamente
- Ten espacio libre de obstáculos (2-3 metros)
- Mantén el botón de emergencia a mano
- No uses en escaleras o cerca de bordes

## Mejoras Futuras

- [ ] Filtro de Kalman para suavizar movimiento
- [ ] Predicción de trayectoria
- [ ] Detección de múltiples personas (elegir la más cercana)
- [ ] Integración con SLAM para mapeo simultáneo
- [ ] Control PID completo (no solo P)
- [ ] Detección de gestos para comandos (parar, seguir, etc.)

## Autor
Smart Trolley Team - 2026
