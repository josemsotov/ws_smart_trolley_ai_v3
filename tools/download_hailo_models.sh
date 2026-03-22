#!/bin/bash
# ============================================================
#  download_hailo_models.sh
#  Descarga modelos HEF del Hailo Model Zoo para Smart Trolley V5
# ============================================================
#
#  Modelos descargados:
#    - scrfd_10g.hef           : Detección de caras precisa para HAILO10H (recomendado)
#    - scrfd_2.5g.hef          : Detección de caras ligera (alternativa)
#    - yolov8s_pose.hef        : Detección de poses humanas (opcional)
#
#  Uso:
#    chmod +x download_hailo_models.sh
#    ./download_hailo_models.sh
#
#  Los modelos se guardan en:
#    src/smart_t_ai_v2/models/
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$WORKSPACE_DIR/src/smart_t_ai_v2/models"

echo ""
echo "══════════════════════════════════════════════════════"
echo "  📥 Descarga de modelos Hailo AI para Smart Trolley"
echo "══════════════════════════════════════════════════════"
echo "  Destino: $MODELS_DIR"
echo ""

mkdir -p "$MODELS_DIR"

# ── Función de descarga con verificación ──────────────────────────────────────
download_model() {
    local url="$1"
    local filename="$2"
    local description="$3"
    local dest="$MODELS_DIR/$filename"

    if [ -f "$dest" ]; then
        echo "  ✅ Ya existe: $filename (saltando)"
        return 0
    fi

    echo "  📦 Descargando: $description"
    echo "     → $filename"
    echo "     URL: $url"

    if command -v wget &>/dev/null; then
        wget -q --show-progress -O "$dest" "$url" || {
            echo "  ❌ Error descargando $filename"
            rm -f "$dest"
            return 1
        }
    elif command -v curl &>/dev/null; then
        curl -L --progress-bar -o "$dest" "$url" || {
            echo "  ❌ Error descargando $filename"
            rm -f "$dest"
            return 1
        }
    else
        echo "  ❌ wget o curl no disponible"
        return 1
    fi

    local size
    size=$(du -sh "$dest" 2>/dev/null | cut -f1)
    echo "  ✅ Descargado: $filename ($size)"
}

# ── Modelos Hailo Model Zoo ────────────────────────────────────────────────────
# Fuente: https://github.com/hailo-ai/hailo_model_zoo
# HEFs compilados para HAILO10H (Hailo AI HAT2+)

echo "  🔍 Verificando dispositivo Hailo..."
if command -v hailortcli &>/dev/null; then
    ARCH=$(hailortcli fw-control identify 2>/dev/null | grep "Device Architecture" | awk '{print $NF}')
    echo "  📟 Arquitectura detectada: ${ARCH:-desconocida}"
else
    ARCH="hailo10h"
    echo "  ⚠️  hailortcli no encontrado, asumiendo HAILO10H"
fi

echo ""
echo "  Descargando modelos para ${ARCH}..."
echo ""

# ─── SCRFD 10G — Detección de caras (recomendado para HAILO10H) ─────────────
# Input: 640x640 RGB | ~10 GFLOPs | 40+ FPS en Hailo10H
# Fuente: Hailo Model Zoo v5.2.0 — https://hailo-model-zoo.s3.eu-west-2.amazonaws.com
MZ_BASE="https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled"
MZ_VERSION="v5.2.0"
MZ_ARCH="hailo10h"

SCRFD10_URL="$MZ_BASE/$MZ_VERSION/$MZ_ARCH/scrfd_10g.hef"
download_model "$SCRFD10_URL" "scrfd_10g.hef" "SCRFD 10G - Detección de caras (HAILO10H, recomendado)"

# ─── SCRFD 2.5G — Detección de caras ligera (alternativa) ────────────────────
# Input: 640x640 RGB | ~2.5 GFLOPs | 60+ FPS en Hailo10H
SCRFD_URL="$MZ_BASE/$MZ_VERSION/$MZ_ARCH/scrfd_2.5g.hef"
echo ""
read -p "  ¿Descargar también SCRFD 2.5G (más ligero, ~1.5MB)? [s/N] " resp
if [[ "$resp" =~ ^[Ss]$ ]]; then
    download_model "$SCRFD_URL" "scrfd_2.5g.hef" "SCRFD 2.5G - Detección de caras (ligero)"
fi

# ─── Vosk Modelo Español ──────────────────────────────────────────────────────
VOSK_DIR="$HOME/.vosk/models"
VOSK_MODEL_DIR="$VOSK_DIR/vosk-model-small-es"
VOSK_URL="https://alphacephei.com/vosk/models/vosk-model-small-es-0.42.zip"

echo ""
echo "──────────────────────────────────────────────────────"
echo "  🎤 Modelo Vosk Español (reconocimiento de voz)"
echo "──────────────────────────────────────────────────────"

if [ -d "$VOSK_MODEL_DIR" ]; then
    echo "  ✅ Ya existe: $VOSK_MODEL_DIR"
else
    mkdir -p "$VOSK_DIR"
    echo "  📦 Descargando modelo Vosk ES (~50MB)..."
    ZIP_PATH="$VOSK_DIR/vosk-model-small-es.zip"

    if command -v wget &>/dev/null; then
        wget -q --show-progress -O "$ZIP_PATH" "$VOSK_URL"
    else
        curl -L --progress-bar -o "$ZIP_PATH" "$VOSK_URL"
    fi

    echo "  📂 Descomprimiendo..."
    cd "$VOSK_DIR"
    unzip -q "$ZIP_PATH"
    # Renombrar al nombre estándar
    for d in vosk-model-small-es-*/; do
        if [ -d "$d" ]; then
            mv "$d" vosk-model-small-es
            break
        fi
    done
    rm -f "$ZIP_PATH"
    cd - > /dev/null

    if [ -d "$VOSK_MODEL_DIR" ]; then
        echo "  ✅ Modelo Vosk instalado en: $VOSK_MODEL_DIR"
    else
        echo "  ❌ Error instalando modelo Vosk"
    fi
fi

# ─── espeak (TTS) ────────────────────────────────────────────────────────────
echo ""
echo "──────────────────────────────────────────────────────"
echo "  🔊 Verificando espeak (TTS)..."
if command -v espeak &>/dev/null; then
    echo "  ✅ espeak disponible"
else
    echo "  ⚠️  espeak no encontrado. Instalando..."
    sudo apt-get install -y espeak espeak-data 2>/dev/null && \
        echo "  ✅ espeak instalado" || \
        echo "  ❌ No se pudo instalar espeak"
fi

# ─── Resumen ─────────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════════"
echo "  📋 Resumen de modelos instalados:"
echo ""
echo "  HEF (Hailo):"
for f in "$MODELS_DIR"/*.hef; do
    [ -f "$f" ] && echo "    ✅ $(basename "$f") ($(du -sh "$f" | cut -f1))"
done

echo ""
echo "  Vosk (voz):"
[ -d "$VOSK_MODEL_DIR" ] && echo "    ✅ vosk-model-small-es" || echo "    ❌ no instalado"

echo ""
echo "  Para arrancar el sistema de interacción:"
echo "    source install/setup.bash"
echo "    ros2 launch smart_t_ai_v2 interaction.launch.py"
echo ""
echo "  O junto con el robot completo:"
echo "    ros2 launch smart_t_ai_v2 launch_robot.launch.py use_interaction:=true"
echo "══════════════════════════════════════════════════════"
