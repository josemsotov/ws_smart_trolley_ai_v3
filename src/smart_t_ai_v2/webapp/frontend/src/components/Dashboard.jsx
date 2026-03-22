/**
 * Dashboard — Pantalla principal del SmartTrolley
 *
 * Secciones:
 *  - Modos (Follower, Face, Gestos, Manual)
 *  - Telemetría rápida (vel, posición)
 *  - Acceso a cámara y scorecard
 */
import { useState } from 'react'
import { UserRound, Smile, Hand, Gamepad2, Square } from 'lucide-react'
import ModeButton from './ModeButton'

export default function Dashboard({
  modes,
  telemetry,
  gestureStatus,
  onModeToggle,
  onStop,
  onTabChange,
}) {
  return (
    <div className="flex flex-col gap-4 p-4 h-full overflow-y-auto">

      {/* ── Sección modos ─────────────────────────────────────── */}
      <h2 className="text-xs uppercase tracking-widest text-slate-500 font-semibold">Modos</h2>
      <div className="grid grid-cols-2 gap-3">
        <ModeButton
          icon={UserRound}
          label="Seguidor"
          sublabel="Sigue a una persona"
          active={modes.follower_enabled}
          onToggle={() => onModeToggle('follower', !modes.follower_enabled)}
          color="green"
        />
        <ModeButton
          icon={Smile}
          label="Reconocimiento"
          sublabel="Identifica al jugador"
          active={modes.face_recognition_enabled}
          onToggle={() => onModeToggle('face_recognition', !modes.face_recognition_enabled)}
          color="blue"
        />
        <ModeButton
          icon={Hand}
          label="Gestos"
          sublabel={gestureStatus !== 'idle' ? gestureStatus : 'Control por mano'}
          active={modes.gesture_control_enabled}
          onToggle={() => onModeToggle('gesture_control', !modes.gesture_control_enabled)}
          color="purple"
        />
        <ModeButton
          icon={Gamepad2}
          label="Manual"
          sublabel="Joystick virtual"
          active={modes.teleop_enabled}
          onToggle={() => onModeToggle('teleop', !modes.teleop_enabled)}
          color="orange"
        />
      </div>

      {/* ── Parada de emergencia ──────────────────────────────── */}
      <button
        onClick={onStop}
        className="w-full flex items-center justify-center gap-2
                   bg-red-600/20 border border-red-600 text-red-400
                   rounded-2xl py-3 font-bold active:scale-95 transition-all"
      >
        <Square size={18} fill="currentColor" />
        PARAR TODO
      </button>

      {/* ── Telemetría ────────────────────────────────────────── */}
      <h2 className="text-xs uppercase tracking-widest text-slate-500 font-semibold">Telemetría</h2>
      <div className="grid grid-cols-3 gap-2 text-center">
        {[
          { label: 'Vel (m/s)',  value: telemetry.linear_vel?.toFixed(2) ?? '0.00' },
          { label: 'Giro (r/s)', value: telemetry.angular_vel?.toFixed(2) ?? '0.00' },
          { label: 'Pos X',      value: telemetry.x?.toFixed(2) ?? '0.00' },
        ].map(({ label, value }) => (
          <div key={label} className="bg-trolley-card rounded-xl p-3 border border-trolley-border">
            <p className="text-xs text-slate-400">{label}</p>
            <p className="text-lg font-mono font-bold text-white">{value}</p>
          </div>
        ))}
      </div>

      {/* ── Accesos rápidos ───────────────────────────────────── */}
      <div className="grid grid-cols-2 gap-3">
        <button
          onClick={() => onTabChange('camera')}
          className="bg-trolley-card border border-trolley-border rounded-xl py-4
                     text-slate-300 text-sm font-medium active:scale-95 transition-all"
        >
          📷 Cámara en vivo
        </button>
        <button
          onClick={() => onTabChange('scores')}
          className="bg-trolley-card border border-trolley-border rounded-xl py-4
                     text-slate-300 text-sm font-medium active:scale-95 transition-all"
        >
          ⛳ Scorecard
        </button>
      </div>
    </div>
  )
}
