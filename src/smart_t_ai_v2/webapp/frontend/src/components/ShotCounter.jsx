/**
 * ShotCounter — Scorecard de golf (18 hoyos)
 * Permite incrementar / decrementar golpes por hoyo con botones táctiles grandes.
 */
export default function ShotCounter({ scores, par, currentHole, onScoreChange, onHoleChange }) {
  const total   = Object.values(scores).reduce((s, v) => s + v, 0)
  const parTotal = Object.values(par).reduce((s, v) => s + v, 0)
  const diff = total - parTotal

  return (
    <div className="flex flex-col h-full gap-3 p-4 overflow-y-auto">

      {/* Resumen */}
      <div className="flex justify-between items-center bg-trolley-card rounded-xl p-3 border border-trolley-border">
        <div className="text-center">
          <p className="text-xs text-slate-400">Hoyo actual</p>
          <p className="text-2xl font-bold text-white">{currentHole}</p>
        </div>
        <div className="text-center">
          <p className="text-xs text-slate-400">Total golpes</p>
          <p className="text-2xl font-bold text-white">{total}</p>
        </div>
        <div className="text-center">
          <p className="text-xs text-slate-400">vs Par {parTotal}</p>
          <p className={`text-2xl font-bold ${diff > 0 ? 'text-red-400' : diff < 0 ? 'text-green-400' : 'text-slate-300'}`}>
            {diff === 0 ? 'E' : diff > 0 ? `+${diff}` : diff}
          </p>
        </div>
      </div>

      {/* Grid de hoyos */}
      <div className="grid grid-cols-3 gap-2">
        {Array.from({ length: 18 }, (_, i) => i + 1).map(hole => {
          const key    = String(hole)
          const score  = scores[key] ?? 0
          const parVal = par[key] ?? 4
          const isCurrent = hole === currentHole
          const scoreDiff = score - parVal

          const diffColor =
            score === 0            ? 'text-slate-500' :
            scoreDiff < -1         ? 'text-yellow-300' :  // Eagle+
            scoreDiff === -1       ? 'text-green-400' :   // Birdie
            scoreDiff === 0        ? 'text-slate-300' :   // Par
            scoreDiff === 1        ? 'text-orange-400' :  // Bogey
            'text-red-400'                                 // Double+

          return (
            <div
              key={hole}
              onClick={() => onHoleChange(hole)}
              className={`
                bg-trolley-card rounded-xl p-2 border cursor-pointer transition-all
                ${isCurrent ? 'border-green-500 ring-1 ring-green-500' : 'border-trolley-border'}
              `}
            >
              <div className="flex justify-between items-center mb-1">
                <span className="text-xs text-slate-400">#{hole}</span>
                <span className="text-xs text-slate-500">P{parVal}</span>
              </div>
              <p className={`text-xl font-bold text-center ${diffColor}`}>
                {score === 0 ? '-' : score}
              </p>
              {isCurrent && (
                <div className="flex gap-1 mt-2 justify-center">
                  <button
                    onClick={e => { e.stopPropagation(); onScoreChange(hole, -1) }}
                    className="flex-1 bg-slate-700 active:bg-slate-600 rounded-lg py-1 text-lg font-bold"
                  >−</button>
                  <button
                    onClick={e => { e.stopPropagation(); onScoreChange(hole, +1) }}
                    className="flex-1 bg-green-700 active:bg-green-600 rounded-lg py-1 text-lg font-bold"
                  >+</button>
                </div>
              )}
            </div>
          )
        })}
      </div>
    </div>
  )
}
