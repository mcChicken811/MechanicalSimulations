/**
 * Setting a mechanism's initial condition by hand: one coordinate is driven (its value, or its
 * rate) towards where the mouse is, and every other coordinate follows so that the constraints
 * still hold, changing as little as it can (minimal-norm corrections), so coordinates the
 * constraints do not tie to the driven one stay put.
 *
 * Positions: the driven coordinate moves along the constraint manifold's tangent
 *   t = e_k − Φ_oᵀ (Φ_o Φ_oᵀ)⁻¹ Φ_k   (Φ_o: the constraint jacobian without column k),
 * each step a 1D Newton step bringing the chosen material point closest to the mouse, then a
 * Gauss–Newton projection back onto Φ(q) = 0 with q_k held.
 * Rates: q̇_k is set so the material point's velocity matches the mouse's offset over a time T,
 * then the other rates are corrected minimally to satisfy Φ_q q̇ = 0.
 */
import { solveInPlace } from './linalg';
import type { NumericSystem } from './simulate';

/** A point of a part, followed by the mouse: a joint, a point along a bar, or a point fixed in a gear's frame. */
export type MaterialPoint = { kind: 'point'; id: number } | { kind: 'bar'; a: number; b: number; s: number } | { kind: 'gear'; gear: number; center: number; u: number; v: number };

type Vec = [number, number];

export class PoseEditor {
  readonly q: Float64Array;
  readonly qd: Float64Array;
  private readonly n: number;
  private readonly nc: number;
  private readonly pos: Float64Array;
  private readonly jac: Float64Array;
  private readonly Phi: Float64Array;
  private readonly PhiQ: Float64Array;
  private readonly gearBuf: Float64Array;

  constructor(
    readonly sys: NumericSystem,
    q0: ArrayLike<number>,
    qd0: ArrayLike<number>,
  ) {
    this.n = sys.n;
    this.nc = sys.nc;
    this.q = Float64Array.from(q0);
    this.qd = Float64Array.from(qd0);
    this.pos = new Float64Array(2 * sys.pointIndex.size);
    this.jac = new Float64Array(2 * this.n * sys.pointIndex.size);
    this.Phi = new Float64Array(this.nc);
    this.PhiQ = new Float64Array(this.nc * this.n);
    this.gearBuf = new Float64Array(sys.gearIds.length);
    this.projectRates(-1);
  }

  /** Where a material point is, and ∂r/∂q (rows x and y), at q. */
  point(mp: MaterialPoint, q: Float64Array = this.q): { r: Vec; Jx: Float64Array; Jy: Float64Array } | null {
    const { n, sys, pos, jac } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    const Jx = new Float64Array(n);
    const Jy = new Float64Array(n);
    const add = (id: number, w: number): Vec | null => {
      const i = sys.pointIndex.get(id);
      if (i === undefined) return null;
      for (let m = 0; m < n; m++) {
        Jx[m] += w * jac[i * 2 * n + m];
        Jy[m] += w * jac[i * 2 * n + n + m];
      }
      return [w * pos[2 * i], w * pos[2 * i + 1]];
    };
    if (mp.kind === 'point') {
      const r = add(mp.id, 1);
      return r && { r, Jx, Jy };
    }
    if (mp.kind === 'bar') {
      const [ra, rb] = [add(mp.a, 1 - mp.s), add(mp.b, mp.s)];
      return ra && rb ? { r: [ra[0] + rb[0], ra[1] + rb[1]], Jx, Jy } : null;
    }
    // r = r_c + Rot(φ)(u, v), φ the gear's turn (one for one with coordinate k)
    const rc = add(mp.center, 1);
    if (!rc) return null;
    sys.gearAngles(q, this.gearBuf);
    const gi = sys.gearIds.indexOf(mp.gear);
    const phi = gi >= 0 ? this.gearBuf[gi] : 0;
    const [c, s] = [Math.cos(phi), Math.sin(phi)];
    const k = sys.gearCoord.get(mp.gear) ?? -1;
    if (k >= 0) {
      Jx[k] += -mp.u * s - mp.v * c;
      Jy[k] += mp.u * c - mp.v * s;
    }
    return { r: [rc[0] + mp.u * c - mp.v * s, rc[1] + mp.u * s + mp.v * c], Jx, Jy };
  }

  /** The coordinates a material point moves with directly (|∂r/∂q_k| > 0), by index. */
  related(mp: MaterialPoint): number[] {
    const p = this.point(mp);
    if (!p) return [];
    const out: number[] = [];
    for (let k = 0; k < this.n; k++) if (Math.hypot(p.Jx[k], p.Jy[k]) > 1e-9) out.push(k);
    return out;
  }

  /**
   * Minimal-norm solution of Φ_o x = rhs over every coordinate but k (k = −1: all of them):
   * x = Φ_oᵀ (Φ_o Φ_oᵀ + εI)⁻¹ rhs.
   */
  private minimal(k: number, rhs: Float64Array): Float64Array | null {
    const { n, nc, PhiQ } = this;
    const x = new Float64Array(n);
    if (nc === 0) return x;
    const A = new Float64Array(nc * nc);
    for (let a = 0; a < nc; a++)
      for (let b = 0; b < nc; b++) {
        let s = 0;
        for (let j = 0; j < n; j++) if (j !== k) s += PhiQ[a * n + j] * PhiQ[b * n + j];
        A[a * nc + b] = s + (a === b ? 1e-12 : 0);
      }
    const mu = Float64Array.from(rhs);
    if (!solveInPlace(A, mu, nc)) return null;
    for (let j = 0; j < n; j++) {
      if (j === k) continue;
      let s = 0;
      for (let a = 0; a < nc; a++) s += PhiQ[a * n + j] * mu[a];
      x[j] = s;
    }
    return x;
  }

  /** How every coordinate moves as q_k moves at unit rate, the constraints kept (to first order). */
  tangent(k: number, q: Float64Array = this.q): Float64Array | null {
    const { n, nc, sys, PhiQ } = this;
    if (nc > 0) sys.phiQ(q, PhiQ);
    const rhs = new Float64Array(nc);
    for (let a = 0; a < nc; a++) rhs[a] = -PhiQ[a * n + k];
    const t = this.minimal(k, rhs);
    if (!t) return null;
    t[k] = 1;
    return t;
  }

  /** Bring q back onto Φ(q) = 0 with q_k held; false if it cannot (a limit of the motion). */
  private project(k: number, q: Float64Array): boolean {
    const { nc, sys, Phi, PhiQ } = this;
    if (nc === 0) return true;
    for (let iter = 0; iter < 25; iter++) {
      sys.phi(q, Phi);
      let err = 0;
      for (let a = 0; a < nc; a++) err = Math.max(err, Math.abs(Phi[a]));
      if (err < 1e-11) return true;
      sys.phiQ(q, PhiQ);
      const dx = this.minimal(k, Phi);
      if (!dx) return false;
      for (let j = 0; j < q.length; j++) q[j] -= dx[j];
    }
    sys.phi(q, Phi);
    return Phi.every((v) => Math.abs(v) < 1e-8);
  }

  /**
   * Drive coordinate k so the material point comes as close to `target` as the mechanism lets it,
   * the others following. Returns the coordinate's new value.
   */
  chasePosition(k: number, mp: MaterialPoint, target: Vec, maxStep: number): number {
    for (let iter = 0; iter < 16; iter++) {
      const p = this.point(mp);
      const t = this.tangent(k);
      if (!p || !t) break;
      let [dx, dy] = [0, 0];
      for (let m = 0; m < this.n; m++) {
        dx += p.Jx[m] * t[m];
        dy += p.Jy[m] * t[m];
      }
      const g2 = dx * dx + dy * dy;
      if (g2 < 1e-14) break; // a dead point: moving q_k does not move the point
      let delta = ((target[0] - p.r[0]) * dx + (target[1] - p.r[1]) * dy) / g2;
      delta = Math.max(-maxStep, Math.min(maxStep, delta));
      if (Math.abs(delta) < 1e-10) break;
      // step along the tangent, then back onto the constraints; halve the step at a limit
      let moved = false;
      for (let tries = 0; tries < 6 && !moved; tries++, delta /= 2) {
        const trial = Float64Array.from(this.q);
        for (let m = 0; m < this.n; m++) trial[m] += delta * t[m];
        trial[k] = this.q[k] + delta;
        if (this.project(k, trial)) {
          this.q.set(trial);
          moved = true;
        }
      }
      if (!moved) break;
    }
    this.projectRates(-1);
    return this.q[k];
  }

  /**
   * Set coordinate k's rate so the material point moves towards `target` at (target − r) / T,
   * the other rates following (minimally) so the constraints still hold. Returns the rate.
   */
  chaseRate(k: number, mp: MaterialPoint, target: Vec, T: number): number {
    const p = this.point(mp);
    const t = this.tangent(k);
    if (!p || !t) return this.qd[k];
    let [dx, dy] = [0, 0];
    for (let m = 0; m < this.n; m++) {
      dx += p.Jx[m] * t[m];
      dy += p.Jy[m] * t[m];
    }
    const g2 = dx * dx + dy * dy;
    if (g2 < 1e-14) return this.qd[k];
    this.qd[k] = ((target[0] - p.r[0]) * dx + (target[1] - p.r[1]) * dy) / (g2 * T);
    this.projectRates(k);
    return this.qd[k];
  }

  /** Correct the rates minimally (q̇_k held; −1: none held) so that Φ_q q̇ = 0. */
  projectRates(k: number) {
    const { n, nc, sys, PhiQ, qd } = this;
    if (nc === 0) return;
    sys.phiQ(this.q, PhiQ);
    const rhs = new Float64Array(nc);
    for (let a = 0; a < nc; a++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += PhiQ[a * n + j] * qd[j];
      rhs[a] = s;
    }
    const dx = this.minimal(k, rhs);
    if (dx) for (let j = 0; j < n; j++) qd[j] -= dx[j];
  }

  /** Velocity of a material point at the current rates. */
  velocity(mp: MaterialPoint): Vec | null {
    const p = this.point(mp);
    if (!p) return null;
    let [vx, vy] = [0, 0];
    for (let m = 0; m < this.n; m++) {
      vx += p.Jx[m] * this.qd[m];
      vy += p.Jy[m] * this.qd[m];
    }
    return [vx, vy];
  }
}
