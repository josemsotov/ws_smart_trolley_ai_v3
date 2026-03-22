# Tools — Smart Trolley AI v2

Scripts de utilidad y configuración del sistema. No son nodos ROS2.

## Bluetooth

| Script | Descripción |
|--------|-------------|
| `connect_hc05.py` | Conectar módulo BT HC-05 al Arduino |
| `quick_bt_test.sh` | Test rápido de conexión Bluetooth |

## Hailo AI HAT2+

| Script | Descripción |
|--------|-------------|
| `download_hailo_models.sh` | Descarga modelos HEF (SCRFD 2.5G) + Vosk ES + verifica espeak |

## Uso — Descarga de modelos Hailo

```bash
# Ejecutar UNA VEZ antes del primer uso del sistema de interacción
chmod +x tools/download_hailo_models.sh
./tools/download_hailo_models.sh
```

Los modelos se instalan en:
- `src/smart_t_ai_v2/models/scrfd_2.5g.hef` — detección de caras Hailo
- `~/.vosk/models/vosk-model-small-es-0.42/` — ASR español offline
