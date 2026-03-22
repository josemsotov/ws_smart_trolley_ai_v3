/**
 * ModeButton — Botón grande de toggle para activar/desactivar modos del trolley
 *
 * Props:
 *   icon      — Componente Lucide
 *   label     — Texto principal
 *   sublabel  — Texto secundario (descripción corta)
 *   active    — boolean
 *   onToggle  — () => void
 *   color     — tailwind color class (default: green)
 */
export default function ModeButton({
  icon: Icon,
  label,
  sublabel,
  active,
  onToggle,
  color = 'green',
}) {
  const colorMap = {
    green:  { ring: 'ring-green-500',  bg: 'bg-green-500/20',  text: 'text-green-400',  dot: 'bg-green-500' },
    blue:   { ring: 'ring-blue-500',   bg: 'bg-blue-500/20',   text: 'text-blue-400',   dot: 'bg-blue-500' },
    purple: { ring: 'ring-purple-500', bg: 'bg-purple-500/20', text: 'text-purple-400', dot: 'bg-purple-500' },
    orange: { ring: 'ring-orange-500', bg: 'bg-orange-500/20', text: 'text-orange-400', dot: 'bg-orange-500' },
  }
  const c = colorMap[color] ?? colorMap.green

  return (
    <button
      onClick={onToggle}
      className={`
        relative flex flex-col items-center justify-center gap-2
        w-full rounded-2xl p-5 transition-all duration-200 active:scale-95
        bg-trolley-card border
        ${active
          ? `${c.ring} ring-2 ${c.bg} border-transparent`
          : 'border-trolley-border hover:border-slate-500'
        }
      `}
    >
      {/* Estado ON/OFF — indicador */}
      <span className={`
        absolute top-3 right-3 w-2.5 h-2.5 rounded-full
        ${active ? `${c.dot} shadow-lg` : 'bg-slate-600'}
      `} />

      <Icon size={32} className={active ? c.text : 'text-slate-400'} />
      <span className={`text-sm font-semibold ${active ? c.text : 'text-slate-300'}`}>
        {label}
      </span>
      {sublabel && (
        <span className="text-xs text-slate-500 text-center leading-tight">
          {sublabel}
        </span>
      )}
    </button>
  )
}
