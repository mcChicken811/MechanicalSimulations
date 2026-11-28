/**
 * Internal forces along a bar: the axial force N (tension positive), shear force V and bending
 * moment M at every section, from the forces on it (mechanics/forces.ts) and its motion.
 *
 * Cut the bar at distance s from its end A. The piece [0, s] moves with the rest of the bar, so
 * (d'Alembert) the external loads on it, its weight, its air drag, minus its inertia, are balanced
 * by what the rest [s, L] does on it at the cut: a force R(s) and a couple M(s):
 *     R(s) = −[ Σ_{x_i < s} F_i + ∫₀ˢ w dx ],
 *     M(s) = −[ Σ_{x_i < s} ((r_i − P(s)) × F_i + M_i) + ∫₀ˢ (x − s) t × w dx ],
 * where the point loads F_i (joint forces, contact, friction, a slider on it, …) act at r_i, a
 * distance x_i along it, and the distributed load per length is
 *     w(x) = μ (g − c v(x) − a(x)),   μ = m / L,
 * the weight, air drag (the simulator's −c M q̇ spread over the bar) and inertia of each bit. A bar
 * is rigid, so v and a are linear along it, between its ends' values, and so is w. Then, with t
 * along the bar (A to B) and n square to it (t turned a quarter turn anticlockwise):
 *     N = R · t,   V = −R · n,   M as above (anticlockwise on the piece [0, s]),
 * the usual beam convention with A on the left: dM/ds = V, a simply supported beam under a load
 * sags with M > 0. Past the far end everything balances (Newton–Euler for the whole bar), which
 * `closure` checks.
 */
import type { BarState, ForceResult } from './forces';

type Vec = [number, number];

/** A point force on the bar, where it acts (anywhere: an offset load adds its moment), and a couple. */
export interface BeamLoad {
  at: Vec;
  F: Vec;
  M?: number;
}

export interface BeamDiagram {
  /** The bar's length (m). */
  L: number;
  /**
   * Stations along the bar (m from A) and the internal forces there. A point load's station is
   * listed twice, just before it and just after, so its jump shows as a step.
   */
  s: number[];
  N: number[];
  V: number[];
  M: number[];
  /** Where the point loads act (m from A). */
  at: number[];
  /** What is left past the far end, (N, V, M): about zero, as the bar's loads balance its motion. */
  closure: [number, number, number];
}

export interface BeamSettings {
  gravity: number;
  /** Air drag c (1/s). */
  damping?: number;
  /** Evenly spaced stations, besides those at the loads (default 120). */
  samples?: number;
}

const dot = (a: Vec, b: Vec) => a[0] * b[0] + a[1] * b[1];
const cross = (a: Vec, b: Vec) => a[0] * b[1] - a[1] * b[0];

/** The axial force, shear force and bending moment along bar `bar` under point loads `loads`. */
export function beamDiagram(bar: BarState, loads: BeamLoad[], settings: BeamSettings): BeamDiagram {
  const { A, B } = bar;
  const L = Math.hypot(B[0] - A[0], B[1] - A[1]) || 1e-12;
  const t: Vec = [(B[0] - A[0]) / L, (B[1] - A[1]) / L];
  const nrm: Vec = [-t[1], t[0]];
  const mu = bar.m / L;
  const c = settings.damping ?? 0;
  const g = settings.gravity;
  // the distributed load at A and at B (linear in between)
  const w0: Vec = [mu * (-c * bar.vA[0] - bar.aA[0]), mu * (-g - c * bar.vA[1] - bar.aA[1])];
  const w1: Vec = [mu * (-c * bar.vB[0] - bar.aB[0]), mu * (-g - c * bar.vB[1] - bar.aB[1])];
  const dw: Vec = [w1[0] - w0[0], w1[1] - w0[1]];
  const [wn0, dwn] = [cross(t, w0), cross(t, dw)];
  const pts = loads
    .map((ld) => ({ ...ld, x: Math.min(L, Math.max(0, dot([ld.at[0] - A[0], ld.at[1] - A[1]], t))) }))
    .sort((p, q) => p.x - q.x);
  // the internal forces at x: with the loads before it, or (inclusive) those at it as well
  const at = (x: number, inclusive: boolean): [number, number, number] => {
    const sig = x / L;
    const P: Vec = [A[0] + x * t[0], A[1] + x * t[1]];
    let Fx = L * (w0[0] * sig + (dw[0] * sig * sig) / 2);
    let Fy = L * (w0[1] * sig + (dw[1] * sig * sig) / 2);
    let Mz = -(wn0 * x * x) / 2 - (dwn * x * x * x) / (6 * L);
    for (const p of pts) {
      if (p.x > x || (p.x === x && !inclusive)) break;
      Fx += p.F[0];
      Fy += p.F[1];
      Mz += cross([p.at[0] - P[0], p.at[1] - P[1]], p.F) + (p.M ?? 0);
    }
    const R: Vec = [-Fx, -Fy];
    return [dot(R, t), -dot(R, nrm), -Mz];
  };
  const stations: { x: number; inclusive: boolean }[] = [];
  const S = Math.max(2, settings.samples ?? 120);
  const tol = L * 1e-9;
  for (let k = 0; k <= S; k++) {
    const x = (L * k) / S;
    if (!pts.some((p) => Math.abs(p.x - x) <= tol)) stations.push({ x, inclusive: false });
  }
  for (const x of [...new Set(pts.map((p) => p.x))]) stations.push({ x, inclusive: false }, { x, inclusive: true });
  stations.sort((p, q) => p.x - q.x || Number(p.inclusive) - Number(q.inclusive));
  const out: BeamDiagram = { L, s: [], N: [], V: [], M: [], at: [...new Set(pts.map((p) => p.x))], closure: at(L, true) };
  for (const st of stations) {
    const [N, V, M] = at(st.x, st.inclusive);
    out.s.push(st.x);
    out.N.push(N);
    out.V.push(V);
    out.M.push(M);
  }
  return out;
}

/** The value of largest size (with its sign) in a list, and where it is. */
export function peak(vals: number[]): { v: number; i: number } {
  let i = 0;
  for (let k = 1; k < vals.length; k++) if (Math.abs(vals[k]) > Math.abs(vals[i])) i = k;
  return { v: vals.length ? vals[i] : NaN, i };
}

/** The value at distance x along a diagram (the later of a step's two values). */
export function valueAt(d: BeamDiagram, vals: number[], x: number): number {
  const n = d.s.length;
  if (!n) return NaN;
  if (x <= d.s[0]) return vals[0];
  for (let k = 1; k < n; k++)
    if (d.s[k] >= x) {
      const [s0, s1] = [d.s[k - 1], d.s[k]];
      if (s1 - s0 < 1e-12) return vals[k];
      return vals[k - 1] + ((x - s0) / (s1 - s0)) * (vals[k] - vals[k - 1]);
    }
  return vals[n - 1];
}

/**
 * Bar `linkId`'s diagram from the forces at a state: every force on it where it acts (joint,
 * slider, teeth, weld, contact, friction, the pull), with its weight and air drag spread along it
 * rather than at its centre. `settings` must match the forces' own (gravity, drag).
 */
export function barDiagram(r: ForceResult, linkId: number, settings: BeamSettings): BeamDiagram | null {
  const bar = r.bars.find((b) => b.id === linkId);
  if (!bar) return null;
  const on = (x: { body: { kind: string; id: number } | null }) => x.body?.kind === 'link' && x.body.id === linkId;
  const loads: BeamLoad[] = [...r.items.filter(on), ...r.loads.filter((x) => on(x) && x.type !== 'weight' && x.type !== 'drag')];
  return beamDiagram(bar, loads, settings);
}
