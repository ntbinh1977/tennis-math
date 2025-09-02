import React, { useMemo, useState, useEffect } from 'react'

/** ---------- Constants & helpers ---------- */
const g = 9.81
const mphToMs = (mph) => mph * 0.44704
const deg = (r) => (r * 180) / Math.PI
const rad = (d) => (d * Math.PI) / 180

// Court constants (singles)
const BASELINE_TO_NET = 11.89 // m
const NET_H = 0.914 // m (center)
const SERVICE_DEPTH = 6.40 // m (net to service line)
const HALF_SERVICE_W = 4.115 // m (centerline to singles sideline)

// Default geometry
const DEFAULT_STEP_IN = 0.5 // m inside baseline at contact

/** z(x): height at horizontal distance x from contact */
function zAtX({ v, theta, h0, x }) {
  return h0 + x * Math.tan(theta) - (g * x * x) / (2 * v * v * Math.cos(theta) ** 2)
}

/** Solve for landing X (where z=0) via bisection */
function landingX({ v, theta, h0 }) {
  let lo = 0,
    hi = 50
  for (let i = 0; i < 120; i++) {
    const mid = 0.5 * (lo + hi)
    zAtX({ v, theta, h0, x: mid }) > 0 ? (lo = mid) : (hi = mid)
  }
  return 0.5 * (lo + hi)
}

/** theta that yields a specific landing depth after the net */
function thetaForTarget({ v, h0, xNet, dAfterNet }) {
  const targetX = xNet + dAfterNet
  const f = (t) => landingX({ v, theta: t, h0 }) - targetX

  // Find bracket
  let a = rad(-20),
    b = rad(35)
  let fa = f(a),
    fb = f(b),
    guard = 0
  while (fa * fb > 0 && guard++ < 40) {
    a -= rad(5)
    b += rad(5)
    fa = f(a)
    fb = f(b)
  }
  // Bisection
  for (let i = 0; i < 90; i++) {
    const m = 0.5 * (a + b)
    const fm = f(m)
    if (fa * fm <= 0) {
      b = m
      fb = fm
    } else {
      a = m
      fa = fm
    }
  }
  return 0.5 * (a + b)
}

/** theta that gives exactly `margin` clearance above net */
function thetaForNetClear({ v, h0, xNet, margin }) {
  const targetH = NET_H + margin
  const f = (t) => zAtX({ v, theta: t, h0, x: xNet }) - targetH

  let a = rad(-20),
    b = rad(35)
  let fa = f(a),
    fb = f(b),
    guard = 0
  while (fa * fb > 0 && guard++ < 40) {
    a -= rad(5)
    b += rad(5)
    fa = f(a)
    fb = f(b)
  }
  for (let i = 0; i < 90; i++) {
    const m = 0.5 * (a + b)
    const fm = f(m)
    if (fa * fm <= 0) {
      b = m
      fb = fm
    } else {
      a = m
      fa = fm
    }
  }
  return 0.5 * (a + b)
}

/** SVG helpers */
const polylinePath = (pts) =>
  pts.map((p, i) => (i === 0 ? `M ${p[0]} ${p[1]}` : `L ${p[0]} ${p[1]}`)).join(' ')
function arcPath(cx, cy, r, a0, a1) {
  const large = Math.abs(a1 - a0) > Math.PI ? 1 : 0
  const sweep = a1 > a0 ? 1 : 0
  const x0 = cx + r * Math.cos(a0),
    y0 = cy + r * Math.sin(a0)
  const x1 = cx + r * Math.cos(a1),
    y1 = cy + r * Math.sin(a1)
  return `M ${x0} ${y0} A ${r} ${r} 0 ${large} ${sweep} ${x1} ${y1}`
}

/** Main computation.
 *  Uses BOTH:
 *    - your desired net-clearance margin (exactly or as feasible), and
 *    - a deep-in target depth (Wide: 5.8 m, T: 5.4 m after net).
 *
 *  We pick theta = clamp( max(theta_for_target, theta_for_margin), <= theta_at_service_line )
 *  If even theta_at_service_line can't meet the margin, we report the maximum achievable clearance.
 */
function computeAngles({ mph, feet, inches, target, stepIn, margin }) {
  const v = Math.max(5, mphToMs(mph || 0))
  const h0 = Math.max(1.2, feet * 0.3048 + (inches || 0) * 0.0254)
  const xNet = BASELINE_TO_NET - stepIn

  // Preferred depth target (m after net)
  const dPref = target === 'wide' ? 5.8 : 5.4
  const thetaDepth = thetaForTarget({ v, h0, xNet, dAfterNet: dPref })
  const thetaMargin = thetaForNetClear({ v, h0, xNet, margin })
  const thetaServiceMax = thetaForTarget({
    v,
    h0,
    xNet,
    dAfterNet: SERVICE_DEPTH - 0.05,
  })

  // Enforce both: deep-ish and margin, but never beyond service line
  let theta = Math.max(thetaDepth, thetaMargin)
  let clamped = false
  if (theta > thetaServiceMax) {
    theta = thetaServiceMax
    clamped = true
  }

  const Xland = landingX({ v, theta, h0 })
  const clearance = zAtX({ v, theta, h0, x: xNet }) - NET_H
  const dAfterNet = Xland - xNet
  const marginSatisfied = !clamped ? clearance + 1e-6 >= margin : clearance + 1e-6 >= margin

  // Lateral target and azimuth
  const yTarget = target === 'wide' ? Math.min(3.85, HALF_SERVICE_W - 0.25) : 0.5
  const XforPhi = Math.min(Xland, xNet + SERVICE_DEPTH - 0.05) // keep inside box visually
  const phi = Math.atan2(yTarget, XforPhi)

  return {
    // inputs/geometry
    v,
    h0,
    xNet,
    // results
    theta,
    phi,
    clearance,
    Xland,
    dAfterNet,
    XforPhi,
    yTarget,
    // status
    clampedToService: clamped,
    marginSatisfied,
  }
}

/** ---------- Diagram Components ---------- */
function SideView({ sol, theme }) {
  const W = 680,
    H = 320,
    pad = 40
  const w = W - 2 * pad,
    h = H - 2 * pad

  const xMax = Math.max(sol.Xland + 1.0, sol.xNet + SERVICE_DEPTH + 1.0)
  const yMax = Math.max(sol.h0 + 0.8, 2.2)

  const sx = (x) => pad + (x / xMax) * w
  const sy = (z) => H - pad - (z / yMax) * h

  // Trajectory samples
  const pts = []
  for (let x = 0; x <= sol.Xland; x += sol.Xland / 160) {
    const z = zAtX({ v: sol.v, theta: sol.theta, h0: sol.h0, x })
    pts.push([sx(x), sy(z)])
  }

  const arcR = 28
  const thetaArc = arcPath(sx(0), sy(sol.h0), arcR, 0, sol.theta)

  return (
    <div className="svgCard">
      <div className="svgTitle">Elevation θ — Side View</div>
      <svg width={W} height={H}>
        {/* ground */}
        <line x1={pad} y1={sy(0)} x2={W - pad} y2={sy(0)} className="axis" />
        {/* net */}
        <line x1={sx(sol.xNet)} y1={sy(0)} x2={sx(sol.xNet)} y2={sy(NET_H)} className="net" />
        <line x1={sx(sol.xNet) - 10} y1={sy(NET_H)} x2={sx(sol.xNet) + 10} y2={sy(NET_H)} className="netTape" />
        {/* service line */}
        <line
          x1={sx(sol.xNet + SERVICE_DEPTH)}
          y1={sy(0)}
          x2={sx(sol.xNet + SERVICE_DEPTH)}
          y2={sy(2.8)}
          className="serviceLine"
        />
        {/* trajectory */}
        <path d={polylinePath(pts)} className="traj" />
        {/* contact */}
        <circle cx={sx(0)} cy={sy(sol.h0)} r="5.5" className="contact" />
        {/* theta arc */}
        <path d={thetaArc} className="arc" />
        <text x={sx(0) + 34} y={sy(sol.h0) - 8} className="labelSubtle">
          θ (elevation)
        </text>
        {/* labels */}
        <text x={sx(sol.xNet) + 6} y={sy(NET_H) - 6} className="labelNet">
          net
        </text>
        <text x={sx(sol.xNet + SERVICE_DEPTH) + 6} y={sy(0) + 14} className="labelService">
          service line
        </text>
        {/* clearance bar */}
        <line x1={sx(sol.xNet)} y1={sy(NET_H)} x2={sx(sol.xNet)} y2={sy(NET_H + sol.clearance)} className="clearBar" />
        <text x={sx(sol.xNet) + 8} y={sy(NET_H + sol.clearance / 2)} className="labelClear">
          clearance ≈ {(sol.clearance * 100).toFixed(0)} cm
        </text>
      </svg>
      <div className="footnote">Illustrative only (simple no‑air model).</div>
    </div>
  )
}

function TopView({ sol }) {
  const W = 680,
    H = 320,
    pad = 36
  const w = W - 2 * pad,
    h = H - 2 * pad

  const xMax = Math.max(sol.XforPhi + 1.0, sol.xNet + SERVICE_DEPTH + 1.0)
  const yMax = HALF_SERVICE_W + 1.0

  const sx = (x) => pad + (x / xMax) * w
  const sy = (y) => H - pad - ((y + yMax) / (2 * yMax)) * h // center y=0

  const arcR = 30
  const phiArc = arcPath(sx(0), sy(0), arcR, 0, sol.phi)

  return (
    <div className="svgCard">
      <div className="svgTitle">Azimuth φ — Top View</div>
      <svg width={W} height={H}>
        {/* service box */}
        <rect
          x={sx(sol.xNet)}
          y={sy(HALF_SERVICE_W)}
          width={sx(sol.xNet + SERVICE_DEPTH) - sx(sol.xNet)}
          height={sy(-HALF_SERVICE_W) - sy(HALF_SERVICE_W)}
          className="box"
        />
        {/* center line */}
        <line x1={sx(sol.xNet)} y1={sy(0)} x2={sx(sol.xNet + SERVICE_DEPTH)} y2={sy(0)} className="centerLine" />
        {/* net */}
        <line x1={sx(sol.xNet)} y1={sy(HALF_SERVICE_W + 0.7)} x2={sx(sol.xNet)} y2={sy(-HALF_SERVICE_W - 0.7)} className="net" />
        {/* aim arrow */}
        <defs>
          <marker id="arrow" markerWidth="8" markerHeight="8" refX="6.5" refY="3.5" orient="auto">
            <polygon points="0 0, 7 3.5, 0 7" className="arrowHead"></polygon>
          </marker>
        </defs>
        <line x1={sx(0)} y1={sy(0)} x2={sx(sol.XforPhi)} y2={sy(sol.yTarget)} className="aim" markerEnd="url(#arrow)" />
        {/* phi arc */}
        <path d={phiArc} className="arcPhi" />
        <text x={sx(0) + 36} y={sy(0) - 6} className="labelPhi">
          φ (azimuth)
        </text>
        <circle cx={sx(0)} cy={sy(0)} r="5.5" className="contact" />
        <text x={sx(sol.xNet) + 6} y={sy(HALF_SERVICE_W) + 14} className="labelNet">
          net
        </text>
      </svg>
      <div className="footnote">Arrow shows left‑right aim. Flip direction for deuce/ad as needed.</div>
    </div>
  )
}

/** ---------- App UI ---------- */
export default function App() {
  // Inputs
  const [mph, setMph] = useState(50)
  const [feet, setFeet] = useState(5)
  const [inches, setInches] = useState(5)
  const [pref, setPref] = useState('wide') // 'wide' | 't'
  const [stepIn, setStepIn] = useState(DEFAULT_STEP_IN)
  const [marginCm, setMarginCm] = useState(20) // user input in cm

  // Theme
  const [theme, setTheme] = useState(() => {
    const saved = localStorage.getItem('theme')
    if (saved) return saved
    return window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark'
  })
  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme)
    localStorage.setItem('theme', theme)
  }, [theme])

  // Compute solution (uses desired net clearance IN the formula)
  const sol = useMemo(
    () =>
      computeAngles({
        mph,
        feet,
        inches,
        target: pref,
        stepIn,
        margin: Math.max(0.05, (marginCm || 0) / 100), // meters
      }),
    [mph, feet, inches, pref, stepIn, marginCm]
  )

  const thetaDeg = deg(sol.theta).toFixed(1)
  const phiDeg = Math.abs(deg(sol.phi)).toFixed(1)

  return (
    <div className="wrap">
      <header>
        <div className="dot" />
        <h1>Tennis Serve Angle Helper</h1>
        <div className="spacer" />
        <button className="btn" onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}>
          {theme === 'dark' ? '🌞 Bright' : '🌙 Dark'}
        </button>
      </header>

      <div className="grid">
        <div className="card">
          <h2>Inputs</h2>
          <div className="row">
            <div>
              <label>Typical serve speed (mph)</label>
              <input type="number" value={mph} min="20" max="150" step="1" onChange={(e) => setMph(+e.target.value || 0)} />
            </div>
            <div>
              <label>Contact height — feet</label>
              <input type="number" value={feet} min="4" max="8" step="1" onChange={(e) => setFeet(+e.target.value || 0)} />
            </div>
          </div>
          <div className="row">
            <div>
              <label>Contact height — inches</label>
              <input type="number" value={inches} min="0" max="11" step="1" onChange={(e) => setInches(+e.target.value || 0)} />
            </div>
            <div>
              <label>Target</label>
              <div className="radio">
                <label>
                  <input type="radio" name="t" checked={pref === 'wide'} onChange={() => setPref('wide')} /> Wide
                </label>
                <label>
                  <input type="radio" name="t" checked={pref === 't'} onChange={() => setPref('t')} /> T
                </label>
              </div>
            </div>
          </div>

          <div className="divider" />

          <h2>Advanced (optional)</h2>
          <div className="row">
            <div>
              <label>Step‑in distance (m)</label>
              <input type="number" value={stepIn} min="0" max="1.2" step="0.1" onChange={(e) => setStepIn(+e.target.value || 0)} />
              <div className="hint">How far inside the baseline you contact (default {DEFAULT_STEP_IN} m).</div>
            </div>
            <div>
              <label>Desired net clearance (cm)</label>
              <input type="number" value={marginCm} min="5" max="40" step="1" onChange={(e) => setMarginCm(+e.target.value || 0)} />
              <div className="hint">This clearance is enforced in the calculation.</div>
            </div>
          </div>

          <div className="divider" />

          <h2>Results</h2>
          <div className="out">
            <div className="pill">
              <b>Elevation θ:</b> <span className="strong">{thetaDeg}°</span>
            </div>
            <div className="pill">
              <b>Azimuth φ:</b> <span className="strong">{phiDeg}°</span>{' '}
              <span className="hint">({pref === 'wide' ? 'toward sideline' : 'toward center'})</span>
            </div>
            <div className="pill">
              <b>Net clearance:</b> {(sol.clearance * 100).toFixed(0)} cm
            </div>
          </div>

          {!sol.marginSatisfied && (
            <div className="warn">
              Your requested clearance may be too large at this speed/height to still land inside the box. Shown is the
              deepest legal angle (on the service line) with the achieved clearance.
            </div>
          )}
          <div className="hint">Simple ballistic model (no drag/spin); diagrams are for intuition.</div>
        </div>

        <div className="card">
          <h2>Diagrams</h2>
          <div className="canvasWrap">
            <SideView sol={sol} />
            <TopView sol={sol} />
          </div>
        </div>
      </div>

      <footer className="foot">
        Court constants (singles): baseline→net 11.89 m, net 0.914 m, service depth 6.40 m, half‑width 4.115 m.
      </footer>
    </div>
  )
}
