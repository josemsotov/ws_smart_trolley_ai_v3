/**
 * StatusBar — Barra superior: conexión Pi5, estado Arduino, modo activo
 */
import { Wifi, WifiOff, Cpu, AlertCircle } from 'lucide-react'

export default function StatusBar({ connected, arduinoStatus, activeMode }) {
  const modeLabel = {
    follower:         '👤 Follower',
    face_recognition: '🎭 Cara',
    gesture_control:  '✋ Gestos',
    teleop:           '🕹 Manual',
    idle:             '⏸ Idle',
  }[activeMode] ?? '—'

  const arduino_ok = arduinoStatus && !arduinoStatus.includes('DESCONECTADO')

  return (
    <div className="flex items-center justify-between px-4 py-2 bg-slate-900/80 border-b border-trolley-border text-xs">
      {/* Conexión Pi5 */}
      <div className={`flex items-center gap-1 ${connected ? 'text-green-400' : 'text-red-400'}`}>
        {connected
          ? <Wifi size={14} />
          : <WifiOff size={14} />
        }
        <span>{connected ? 'Pi5 OK' : 'Sin conexión'}</span>
      </div>

      {/* Modo activo */}
      <span className="text-slate-300 font-medium">{modeLabel}</span>

      {/* Arduino */}
      <div className={`flex items-center gap-1 ${arduino_ok ? 'text-green-400' : 'text-yellow-400'}`}>
        {arduino_ok
          ? <Cpu size={14} />
          : <AlertCircle size={14} />
        }
        <span>{arduino_ok ? 'Arduino' : 'Arduino?'}</span>
      </div>
    </div>
  )
}
