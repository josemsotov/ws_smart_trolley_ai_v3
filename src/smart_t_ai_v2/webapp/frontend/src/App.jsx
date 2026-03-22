/**
 * App.jsx — SmartTrolley PWA
 *
 * Gestiona el estado global y el enrutamiento entre pantallas:
 *  - dashboard  → botones de modo + telemetría
 *  - camera     → stream MJPEG de la cámara Kinect
 *  - scores     → scorecard de 18 hoyos
 *
 * La comunicación con el Pi5 se hace exclusivamente a través del WebSocket
 * en useWebSocket().
 */
import { useState, useEffect } from 'react'
// Lucide icons usados en StatusBar/Dashboard (importados en subcomponentes)

import useWebSocket   from './hooks/useWebSocket'
import StatusBar      from './components/StatusBar'
import Dashboard      from './components/Dashboard'
import CameraView     from './components/CameraView'
import ShotCounter    from './components/ShotCounter'

// Estado inicial de modos (se sobreescribe con full_state del servidor)
const DEFAULT_MODES = {
  follower_enabled:          false,
  face_recognition_enabled:  false,
  gesture_control_enabled:   false,
  teleop_enabled:            false,
}

const DEFAULT_SCORES = Object.fromEntries(
  Array.from({ length: 18 }, (_, i) => [String(i + 1), 0])
)
const DEFAULT_PAR = Object.fromEntries(
  Array.from({ length: 18 }, (_, i) => [String(i + 1), 4])
)

export default function App() {
  const { connected, lastMessage, send } = useWebSocket()
  const [tab,          setTab]          = useState('dashboard')
  const [modes,        setModes]        = useState(DEFAULT_MODES)
  const [telemetry,    setTelemetry]    = useState({})
  const [gestureStatus, setGestureStatus] = useState('idle')
  const [arduinoStatus, setArduinoStatus] = useState('DESCONECTADO')
  const [scores,       setScores]       = useState(DEFAULT_SCORES)
  const [par,          setPar]          = useState(DEFAULT_PAR)
  const [currentHole,  setCurrentHole]  = useState(1)

  // ── Procesar mensajes entrantes del Pi5 ──────────────────────────────────
  useEffect(() => {
    if (!lastMessage) return
    const { type, data } = lastMessage

    switch (type) {
      case 'full_state':
        setModes({
          follower_enabled:         data.follower_enabled,
          face_recognition_enabled: data.face_recognition_enabled,
          gesture_control_enabled:  data.gesture_control_enabled,
          teleop_enabled:           false,
        })
        setScores(data.scores ?? DEFAULT_SCORES)
        setPar(data.par ?? DEFAULT_PAR)
        setCurrentHole(data.current_hole ?? 1)
        setArduinoStatus(data.arduino_status ?? 'DESCONECTADO')
        break

      case 'mode_update':
        setModes(prev => ({ ...prev, [data.mode + '_enabled']: data.enabled }))
        break

      case 'telemetry':
        setTelemetry(data)
        break

      case 'gesture_status':
        setGestureStatus(data)
        break

      case 'arduino_status':
        setArduinoStatus(data)
        break

      case 'score_update':
        setScores(data.scores)
        break

      case 'hole_update':
        setCurrentHole(data.current_hole)
        break

      case 'scores_reset':
        setScores(data)
        break

      default:
        break
    }
  }, [lastMessage])

  // ── Manejadores de acciones ───────────────────────────────────────────────
  const handleModeToggle = (mode, enabled) => {
    // Sólo un modo activo a la vez (desactivar los demás)
    if (enabled) {
      setModes({
        follower_enabled:         mode === 'follower'         ? true : false,
        face_recognition_enabled: mode === 'face_recognition' ? true : false,
        gesture_control_enabled:  mode === 'gesture_control'  ? true : false,
        teleop_enabled:           mode === 'teleop'           ? true : false,
      })
      // Desactivar los demás en el servidor
      const allModes = ['follower', 'face_recognition', 'gesture_control', 'teleop']
      allModes.forEach(m => send('set_mode', { mode: m, enabled: m === mode }))
    } else {
      setModes(prev => ({ ...prev, [mode + '_enabled']: false }))
      send('set_mode', { mode, enabled: false })
    }
  }

  const handleStop = () => {
    send('stop')
    setModes({
      follower_enabled:         false,
      face_recognition_enabled: false,
      gesture_control_enabled:  false,
      teleop_enabled:           false,
    })
  }

  const handleScoreChange = (hole, delta) =>
    send('score_update', { hole, delta })

  const handleHoleChange = (hole) => {
    setCurrentHole(hole)
    send('set_hole', { hole })
  }

  // ── Determinar modo activo para StatusBar ────────────────────────────────
  const activeMode =
    modes.follower_enabled         ? 'follower' :
    modes.face_recognition_enabled ? 'face_recognition' :
    modes.gesture_control_enabled  ? 'gesture_control' :
    modes.teleop_enabled           ? 'teleop' :
    'idle'

  // ── Navegación tabs ───────────────────────────────────────────────────────
  const tabs = [
    { id: 'dashboard', icon: '🏠', label: 'Control' },
    { id: 'camera',    icon: '📷', label: 'Cámara' },
    { id: 'scores',    icon: '⛳', label: 'Score' },
  ]

  return (
    <div className="flex flex-col h-full bg-trolley-dark select-none">

      {/* ── Barra de estado ─────────────────────────────────── */}
      <StatusBar
        connected={connected}
        arduinoStatus={arduinoStatus}
        activeMode={activeMode}
      />

      {/* ── Contenido principal ─────────────────────────────── */}
      <div className="flex-1 overflow-hidden">
        {tab === 'dashboard' && (
          <Dashboard
            modes={modes}
            telemetry={telemetry}
            gestureStatus={gestureStatus}
            onModeToggle={handleModeToggle}
            onStop={handleStop}
            onTabChange={setTab}
          />
        )}
        {tab === 'camera' && <CameraView />}
        {tab === 'scores' && (
          <ShotCounter
            scores={scores}
            par={par}
            currentHole={currentHole}
            onScoreChange={handleScoreChange}
            onHoleChange={handleHoleChange}
          />
        )}
      </div>

      {/* ── Tab bar inferior (estilo iOS) ────────────────────── */}
      <nav className="flex border-t border-trolley-border bg-slate-900/95 backdrop-blur-sm
                      safe-area-inset-bottom">
        {tabs.map(t => (
          <button
            key={t.id}
            onClick={() => setTab(t.id)}
            className={`
              flex-1 flex flex-col items-center justify-center py-3 gap-0.5
              text-xs transition-colors
              ${tab === t.id ? 'text-green-400' : 'text-slate-500'}
            `}
          >
            <span className="text-xl leading-none">{t.icon}</span>
            <span>{t.label}</span>
          </button>
        ))}
      </nav>
    </div>
  )
}
