# EMPAREJAR CONTROL INALÁMBRICO CON DONGLE

## 📋 TU SITUACIÓN ACTUAL
- ✅ Dongle USB conectado y reconocido ("Twin USB Joystick")
- ✅ LED del control encendido
- ❌ Control NO transmite datos al dongle

## 🔧 MÉTODOS DE EMPAREJAMIENTO

### Método 1: Botón MODE (Más común)
1. **Con el dongle conectado**, presiona y mantén **MODE** en el control
2. Mantén presionado por **5-10 segundos** 
3. El LED debería **parpadear rápidamente**
4. Suelta cuando el LED se quede **fijo**
5. Probamos si responde

### Método 2: Combinación de botones
Algunos controles requieren:
- **START + SELECT** (simultáneamente por 3 segundos)
- **MODE + START** (simultáneamente)
- **L1 + R1 + START** (simultáneamente)

### Método 3: Botón físico en el dongle
- Algunos dongles tienen un **botón pequeño de pairing**
- Presiónalo y luego presiona MODE en el control

### Método 4: Reiniciar control
1. Apaga el control (mantén MODE 10+ segundos)
2. Enciéndelo de nuevo (presiona START o MODE brevemente)
3. Debería auto-emparejar con el dongle

## 🔍 CÓMO SABER SI FUNCIONÓ
El LED del control:
- **Parpadeando lento** = buscando dongle
- **Parpadeando rápido** = modo pairing
- **FIJO** = conectado correctamente

## ⚡ PRÓXIMO PASO
Una vez emparejado, ejecutaremos este test:
```bash
timeout 5 evtest /dev/input/event0
```

Y presionarás botones para confirmar que funciona.
