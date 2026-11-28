/**
 * Measurements of a part while the simulation runs: the physical quantities of a joint, a bar
 * or rack, a gear, wheel or propeller, or a spring, as functions of a state (q, q̇), so they can
 * be read now and over the recorded history (for graphs).
 *
 * Rigid bodies (uniform bars and disks): centre of mass G, angle, their rates, momentum
 * p = m v_G, angular momentum about G (I ω) and about the origin (r_G × p + I ω), kinetic energy
 * ½ m v_G² + ½ I ω² (a propeller's in its 3D form) and gravitational potential energy m g y_G.
 */
import { propellerInertia, type Design } from '../mechanics/model';
import type { NumericSystem } from '../mechanics/simulate';

type Vec = [number, number];

/** The mechanism at one state: joint positions and velocities, gear turns and rates. */
export class StateAt {
  private pos: Float64Array | null = null;
  private jac: Float64Array | null = null;
  private gear: Float64Array | null = null;
  constructor(
    readonly sys: NumericSystem,
    readonly q: ArrayLike<number>,
    readonly qd: ArrayLike<number>,
  ) {}
  point(id: number): Vec | null {
    const i = this.sys.pointIndex.get(id);
    if (i === undefined) return null;
    if (!this.pos) {
      this.pos = new Float64Array(2 * this.sys.pointIndex.size);
      this.sys.positions(this.q, this.pos);
    }
    return [this.pos[2 * i], this.pos[2 * i + 1]];
  }
  velocity(id: number): Vec | null {
    const i = this.sys.pointIndex.get(id);
    if (i === undefined) return null;
    const n = this.sys.n;
    if (!this.jac) {
      this.jac = new Float64Array(2 * n * this.sys.pointIndex.size);
      this.sys.jacobians(this.q, this.jac);
    }
    let [vx, vy] = [0, 0];
    for (let m = 0; m < n; m++) {
      vx += this.jac[i * 2 * n + m] * this.qd[m];
      vy += this.jac[i * 2 * n + n + m] * this.qd[m];
    }
    return [vx, vy];
  }
  /** A gear's turn since the drawing (a propeller's hub yaw), and its rate. */
  gearAngle(id: number): number {
    if (!this.gear) {
      this.gear = new Float64Array(this.sys.gearIds.length);
      this.sys.gearAngles(this.q, this.gear);
    }
    const k = this.sys.gearIds.indexOf(id);
    return k >= 0 ? this.gear[k] : 0;
  }
  gearRate(id: number): number {
    const k = this.sys.gearCoord.get(id) ?? -1;
    return k >= 0 ? this.qd[k] : 0;
  }
  /** A propeller's blade spin and its rate (0 if braked). */
  spin(id: number): [number, number] {
    const pr = this.sys.derivation.propellers.find((x) => x.id === id);
    return pr && pr.spin >= 0 ? [this.q[pr.spin], this.qd[pr.spin]] : [0, 0];
  }
}

export interface Measure {
  key: string;
  /** Its symbol (HTML) and what it is. */
  sym: string;
  name: string;
  unit: string;
  /** An angle (in °): its history is unwrapped so it runs on past ±180°. */
  angle?: boolean;
  value: (s: StateAt) => number;
}

const DEG = 180 / Math.PI;
const NaNv: Vec = [NaN, NaN];

/** The quantities measured on a part (kind, id), with gravity g (m/s²). */
export function measuresOf(d: Design, kind: 'point' | 'link' | 'spring' | 'gear', id: number, g: number): Measure[] {
  if (kind === 'point') {
    const p = d.points.find((x) => x.id === id);
    if (!p) return [];
    const m = p.mass;
    const r = (s: StateAt) => s.point(id) ?? NaNv;
    const v = (s: StateAt) => s.velocity(id) ?? NaNv;
    const out: Measure[] = [
      { key: 'x', sym: 'x', name: 'x position', unit: 'm', value: (s) => r(s)[0] },
      { key: 'y', sym: 'y', name: 'y position', unit: 'm', value: (s) => r(s)[1] },
      { key: 'vx', sym: 'ẋ', name: 'x velocity', unit: 'm/s', value: (s) => v(s)[0] },
      { key: 'vy', sym: 'ẏ', name: 'y velocity', unit: 'm/s', value: (s) => v(s)[1] },
      { key: 'speed', sym: '|v|', name: 'speed', unit: 'm/s', value: (s) => Math.hypot(...v(s)) },
    ];
    if (m > 0)
      out.push(
        { key: 'px', sym: 'p<sub>x</sub>', name: 'x momentum', unit: 'kg·m/s', value: (s) => m * v(s)[0] },
        { key: 'py', sym: 'p<sub>y</sub>', name: 'y momentum', unit: 'kg·m/s', value: (s) => m * v(s)[1] },
        { key: 'LO', sym: 'L<sub>O</sub>', name: 'angular momentum about the origin', unit: 'kg·m²/s', value: (s) => m * (r(s)[0] * v(s)[1] - r(s)[1] * v(s)[0]) },
        { key: 'T', sym: 'T', name: 'kinetic energy', unit: 'J', value: (s) => 0.5 * m * (v(s)[0] ** 2 + v(s)[1] ** 2) },
        { key: 'V', sym: 'V<sub>g</sub>', name: 'gravitational potential energy', unit: 'J', value: (s) => m * g * r(s)[1] },
      );
    return out;
  }
  if (kind === 'spring') {
    const sp = d.springs.find((x) => x.id === id);
    if (!sp) return [];
    const len = (s: StateAt) => {
      const [a, b] = [s.point(sp.a), s.point(sp.b)];
      return a && b ? Math.hypot(b[0] - a[0], b[1] - a[1]) : NaN;
    };
    const ext = (s: StateAt) => {
      const e = len(s) - sp.rest;
      return sp.tensionOnly ? Math.max(0, e) : e;
    };
    const rate = (s: StateAt) => {
      const [a, b, va, vb] = [s.point(sp.a), s.point(sp.b), s.velocity(sp.a) ?? [0, 0], s.velocity(sp.b) ?? [0, 0]];
      if (!a || !b) return NaN;
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      return ((b[0] - a[0]) * (vb[0] - va[0]) + (b[1] - a[1]) * (vb[1] - va[1])) / L;
    };
    return [
      { key: 'len', sym: 'ℓ', name: 'length', unit: 'm', value: len },
      { key: 'rate', sym: 'ℓ̇', name: 'rate of stretch', unit: 'm/s', value: rate },
      { key: 'ext', sym: 'ℓ − ℓ<sub>0</sub>', name: sp.tensionOnly ? 'stretch (slack is 0)' : 'extension', unit: 'm', value: ext },
      { key: 'F', sym: 'F', name: 'tension (+) or push (−)', unit: 'N', value: (s) => sp.k * ext(s) },
      { key: 'Ve', sym: 'V<sub>e</sub>', name: 'elastic potential energy', unit: 'J', value: (s) => 0.5 * sp.k * ext(s) ** 2 },
    ];
  }
  // a rigid body: centre of mass G, its angle, their rates, mass m and inertia about G
  let body: { m: number; I: (s: StateAt) => number; G: (s: StateAt) => Vec; vG: (s: StateAt) => Vec; th: (s: StateAt) => number; w: (s: StateAt) => number; extraT?: (s: StateAt) => number; turn: string } | null = null;
  const extra: Measure[] = [];
  if (kind === 'link') {
    const l = d.links.find((x) => x.id === id);
    if (!l) return [];
    const ends = (s: StateAt) => [s.point(l.a) ?? NaNv, s.point(l.b) ?? NaNv, s.velocity(l.a) ?? NaNv, s.velocity(l.b) ?? NaNv] as const;
    body = {
      m: l.mass,
      I: (s) => {
        const [a, b] = ends(s);
        return (l.mass * ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)) / 12;
      },
      G: (s) => {
        const [a, b] = ends(s);
        return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      },
      vG: (s) => {
        const [, , va, vb] = ends(s);
        return [(va[0] + vb[0]) / 2, (va[1] + vb[1]) / 2];
      },
      th: (s) => {
        const [a, b] = ends(s);
        return Math.atan2(b[1] - a[1], b[0] - a[0]);
      },
      w: (s) => {
        // ω = (r_ab × v_ab) / |r_ab|²
        const [a, b, va, vb] = ends(s);
        const [dx, dy, dvx, dvy] = [b[0] - a[0], b[1] - a[1], vb[0] - va[0], vb[1] - va[1]];
        return (dx * dvy - dy * dvx) / (dx * dx + dy * dy || 1);
      },
      turn: 'θ',
    };
  } else {
    const gr = d.gears.find((x) => x.id === id);
    if (!gr) return [];
    const prop = gr.propeller;
    const inertia = prop ? propellerInertia(gr) : null;
    body = {
      m: gr.mass,
      I: () => (inertia ? inertia.I3 : 0.5 * gr.mass * gr.radius * gr.radius),
      G: (s) => s.point(gr.center) ?? NaNv,
      vG: (s) => s.velocity(gr.center) ?? NaNv,
      th: (s) => s.gearAngle(id) + (prop ? prop.angle : 0),
      w: (s) => s.gearRate(id),
      turn: prop ? 'θ' : 'φ',
      // a propeller's rotation energy in its 3D form: ½ (I₁ φ̇² + (I₂ sin²φ + I₃ cos²φ) θ̇²)
      extraT: inertia
        ? (s) => {
            const [phi, phid] = s.spin(id);
            const w = s.gearRate(id);
            return 0.5 * (inertia.I1 * phid * phid + (inertia.I2 * Math.sin(phi) ** 2 + inertia.I3 * Math.cos(phi) ** 2) * w * w);
          }
        : undefined,
    };
    if (prop) extra.push({ key: 'spin', sym: 'φ̇', name: 'blade spin rate', unit: 'rpm', value: (s) => (s.spin(id)[1] * 60) / (2 * Math.PI) });
  }
  const B = body;
  const T = (s: StateAt) => {
    const v = B.vG(s);
    return 0.5 * B.m * (v[0] ** 2 + v[1] ** 2) + (B.extraT ? B.extraT(s) : 0.5 * B.I(s) * B.w(s) ** 2);
  };
  return [
    { key: 'xG', sym: 'x<sub>G</sub>', name: 'centre of mass, x', unit: 'm', value: (s) => B.G(s)[0] },
    { key: 'yG', sym: 'y<sub>G</sub>', name: 'centre of mass, y', unit: 'm', value: (s) => B.G(s)[1] },
    { key: 'th', sym: B.turn, name: kind === 'link' ? 'angle' : d.gears.find((x) => x.id === id)?.propeller ? 'hub yaw' : 'turn since the drawing', unit: '°', angle: true, value: (s) => B.th(s) * DEG },
    { key: 'vx', sym: 'ẋ<sub>G</sub>', name: 'velocity of G, x', unit: 'm/s', value: (s) => B.vG(s)[0] },
    { key: 'vy', sym: 'ẏ<sub>G</sub>', name: 'velocity of G, y', unit: 'm/s', value: (s) => B.vG(s)[1] },
    { key: 'w', sym: `${B.turn}̇`, name: 'angular velocity', unit: '°/s', value: (s) => B.w(s) * DEG },
    ...extra,
    { key: 'px', sym: 'p<sub>x</sub>', name: 'momentum, x', unit: 'kg·m/s', value: (s) => B.m * B.vG(s)[0] },
    { key: 'py', sym: 'p<sub>y</sub>', name: 'momentum, y', unit: 'kg·m/s', value: (s) => B.m * B.vG(s)[1] },
    { key: 'LG', sym: 'L<sub>G</sub>', name: 'angular momentum about G', unit: 'kg·m²/s', value: (s) => B.I(s) * B.w(s) },
    {
      key: 'LO',
      sym: 'L<sub>O</sub>',
      name: 'angular momentum about the origin',
      unit: 'kg·m²/s',
      value: (s) => {
        const [G, v] = [B.G(s), B.vG(s)];
        return B.m * (G[0] * v[1] - G[1] * v[0]) + B.I(s) * B.w(s);
      },
    },
    { key: 'T', sym: 'T', name: 'kinetic energy', unit: 'J', value: T },
    { key: 'V', sym: 'V<sub>g</sub>', name: 'gravitational potential energy', unit: 'J', value: (s) => B.m * g * B.G(s)[1] },
  ];
}

/** A history of values, angles unwrapped so they run on past ±180°. */
export function unwrapDegrees(vals: number[]): number[] {
  const out = vals.slice();
  for (let i = 1; i < out.length; i++) {
    let d = out[i] - out[i - 1];
    while (d > 180) {
      out[i] -= 360;
      d -= 360;
    }
    while (d < -180) {
      out[i] += 360;
      d += 360;
    }
  }
  return out;
}
