/**
 * Setting a mechanism's initial condition by hand.
 *
 * Positions are set by pulling: a spring-damper from the point the user clicked towards the
 * cursor drives the equations of motion, integrated with RK4, for the chosen coordinate and the
 * coordinates the constraints tie it to (directly or through others); every other coordinate is
 * held, its rate 0. Only the pull acts (gravity, springs and motors are left out while setting),
 * through the real mass matrix, velocity terms and constraints:
 *   M_FF q̈_F = f_F + J_Fᵀ F − c M_FF q̇_F + Φ_Fᵀ λ,   Φ_F q̈_F = γ
 * with F = m (ω² (r_cursor − r) − 2ω ṙ), m = 1 / λ_max(J A Jᵀ) the point's effective mass in
 * the direction it moves most easily (A the constrained inverse inertia), so any part follows
 * at about the same rate ω, critically damped that way and never faster.
 * After each step the free coordinates are projected back onto Φ(q) = 0.
 *
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
  private readonly Mbuf: Float64Array;

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
    this.Mbuf = new Float64Array(this.n * this.n);
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

  /** The coordinates the constraints tie to k (directly or through others), k included, ascending. */
  connected(k: number): number[] {
    const cols = this.sys.derivation.phiQ.map((row) => row.flatMap((p, j) => (p.isZero() ? [] : [j])));
    const seen = new Set([k]);
    const stack = [k];
    while (stack.length) {
      const j = stack.pop()!;
      for (const c of cols) if (c.includes(j)) for (const x of c) if (!seen.has(x)) seen.add(x) && stack.push(x);
    }
    return [...seen].sort((a, b) => a - b);
  }

  private drag: { mp: MaterialPoint; free: number[]; rows: number[]; v: Float64Array; off: number[]; lam: [number, number, number, number] } | null = null;

  /** Start pulling material point `mp`, coordinate k and those tied to it free to move, the rest held. */
  startDrag(k: number, mp: MaterialPoint) {
    const d = this.sys.derivation;
    const free = this.connected(k);
    const rows = d.phiQ.flatMap((row, a) => (row.some((p, j) => !p.isZero() && free.includes(j)) ? [a] : []));
    // the forces left out while setting: gravity, springs and motors
    const offSyms = new Set(['g', ...d.springs.map((sp) => sp.k), ...d.motors.flatMap((m) => [m.tau, ...(m.b ? [m.b] : [])])]);
    const off = d.params.flatMap((p, i) => (offSyms.has(p.sym) ? [i] : []));
    this.drag = { mp, free, rows, v: new Float64Array(this.n), off, lam: [0, 0, 0, 0] };
  }

  /** The coordinates moving in the current drag. */
  get dragFree(): number[] {
    return this.drag?.free ?? [];
  }

  /**
   * Solve the free coordinates' KKT system [M_FF Φ_Fᵀ; Φ_F −ε] [x; λ] = [rhs; c] (M filled);
   * returns x over all coordinates (0 for the held ones).
   */
  private kkt(M: Float64Array, rhs: Float64Array, c: Float64Array): Float64Array | null {
    const { n, PhiQ } = this;
    const { free, rows } = this.drag!;
    const m = free.length;
    const N = m + rows.length;
    for (const eps of [1e-10, 1e-6]) {
      const K = new Float64Array(N * N);
      const b = new Float64Array(N);
      free.forEach((ia, a) => {
        free.forEach((ib, bb) => (K[a * N + bb] = M[ia * n + ib]));
        b[a] = rhs[ia];
      });
      rows.forEach((row, r) => {
        free.forEach((ia, a) => (K[(m + r) * N + a] = K[a * N + m + r] = PhiQ[row * n + ia]));
        K[(m + r) * N + m + r] = -eps;
        b[m + r] = c[row];
      });
      if (!solveInPlace(K, b, N)) continue;
      const x = new Float64Array(n);
      free.forEach((ia, a) => (x[ia] = b[a]));
      return x;
    }
    return null;
  }

  /** q̈ while pulling towards `target` (held coordinates 0), or null if the system is singular. */
  private dragAccel(q: Float64Array, v: Float64Array, target: Vec, W: number): Float64Array | null {
    const { n, sys } = this;
    const dr = this.drag!;
    const M = this.Mbuf;
    sys.mass(q, M);
    const f = new Float64Array(n);
    const saved = dr.off.map((i) => sys.params[i]);
    dr.off.forEach((i) => (sys.params[i] = 0));
    sys.force(q, v, f);
    dr.off.forEach((i, j) => (sys.params[i] = saved[j]));
    const p = this.point(dr.mp, q);
    if (!p) return null;
    let [vx, vy] = [0, 0];
    for (let m = 0; m < n; m++) {
      vx += p.Jx[m] * v[m];
      vy += p.Jy[m] * v[m];
    }
    // F = Λ (ω² e − 2ω ṙ): the pull, scaled by the point's effective mass
    const [ex, ey] = [W * W * (target[0] - p.r[0]) - 2 * W * vx, W * W * (target[1] - p.r[1]) - 2 * W * vy];
    const [a, b, c, d] = dr.lam;
    const [Fx, Fy] = [a * ex + b * ey, c * ex + d * ey];
    const cDamp = 0.4 * W; // a little drag on every free coordinate, so nothing keeps swinging
    for (const i of dr.free) {
      let Mv = 0;
      for (let j = 0; j < n; j++) Mv += M[i * n + j] * v[j];
      f[i] += p.Jx[i] * Fx + p.Jy[i] * Fy - cDamp * Mv;
    }
    if (this.nc) sys.phiQ(q, this.PhiQ);
    const gam = new Float64Array(this.nc);
    if (this.nc) sys.gamma(q, v, gam);
    return this.kkt(M, f, gam);
  }

  /** The point's effective mass for the pull, from its mobility J A Jᵀ (2×2) at the current pose. */
  private updateLambda() {
    const { n, sys } = this;
    const dr = this.drag!;
    const p = this.point(dr.mp);
    if (!p) return;
    const M = this.Mbuf;
    sys.mass(this.q, M);
    if (this.nc) sys.phiQ(this.q, this.PhiQ);
    const zero = new Float64Array(this.nc);
    const ax = this.kkt(M, p.Jx, zero);
    const ay = this.kkt(M, p.Jy, zero);
    if (!ax || !ay) return;
    let [wxx, wxy, wyy] = [0, 0, 0];
    for (let m = 0; m < n; m++) {
      wxx += p.Jx[m] * ax[m];
      wxy += p.Jx[m] * ay[m];
      wyy += p.Jy[m] * ay[m];
    }
    // the most mobile direction's effective mass: every direction then responds at ω or slower
    // (a direction it can hardly move in follows more slowly, rather than being pushed without bound)
    const top = (wxx + wyy) / 2 + Math.sqrt(((wxx - wyy) / 2) ** 2 + wxy * wxy);
    const m = top > 1e-12 ? 1 / top : 0;
    dr.lam = [m, 0, 0, m];
  }

  /** Bring the free coordinates back onto the constraints (positions, then rates), the held ones untouched. */
  private projectFree(q: Float64Array, v: Float64Array) {
    const { n, nc, sys, Phi, PhiQ } = this;
    if (!nc) return;
    const { free, rows } = this.drag!;
    const solve = (rhs: Float64Array): Float64Array | null => {
      // minimal-norm x over the free coordinates with Φ_F x = rhs (on the rows touching them)
      const r = rows.length;
      const A = new Float64Array(r * r);
      rows.forEach((ra, a) =>
        rows.forEach((rb, b) => {
          let s = 0;
          for (const j of free) s += PhiQ[ra * n + j] * PhiQ[rb * n + j];
          A[a * r + b] = s + (a === b ? 1e-12 : 0);
        }),
      );
      const mu = Float64Array.from(rows.map((row) => rhs[row]));
      if (!solveInPlace(A, mu, r)) return null;
      const x = new Float64Array(n);
      for (const j of free) rows.forEach((row, a) => (x[j] += PhiQ[row * n + j] * mu[a]));
      return x;
    };
    for (let iter = 0; iter < 4; iter++) {
      sys.phi(q, Phi);
      if (rows.every((row) => Math.abs(Phi[row]) < 1e-12)) break;
      sys.phiQ(q, PhiQ);
      const dx = solve(Phi);
      if (!dx) break;
      for (const j of free) q[j] -= dx[j];
    }
    sys.phiQ(q, PhiQ);
    const c = new Float64Array(nc);
    for (const row of rows) for (let j = 0; j < n; j++) c[row] += PhiQ[row * n + j] * v[j];
    const dv = solve(c);
    if (dv) for (const j of free) v[j] -= dv[j];
  }

  /**
   * Advance the pull by dt seconds (in 1 ms RK4 steps): the clicked point is drawn towards
   * `target`, critically damped at ω rad/s. Returns false if the equations became singular.
   */
  dragStep(target: Vec, dt: number, W = 18): boolean {
    const dr = this.drag;
    if (!dr) return false;
    this.updateLambda();
    const steps = Math.max(1, Math.min(80, Math.ceil(dt / 1e-3)));
    const h = dt / steps;
    const { q } = this;
    const v = dr.v;
    for (let s = 0; s < steps; s++) {
      const a1 = this.dragAccel(q, v, target, W);
      if (!a1) return false;
      const stage = (k: Float64Array, kv: Float64Array, c: number) => {
        const qs = Float64Array.from(q);
        const vs = Float64Array.from(v);
        for (const j of dr.free) {
          qs[j] += c * h * kv[j];
          vs[j] += c * h * k[j];
        }
        return [qs, vs] as const;
      };
      const [q2, v2] = stage(a1, v, 0.5);
      const a2 = this.dragAccel(q2, v2, target, W);
      if (!a2) return false;
      const [q3, v3] = stage(a2, v2, 0.5);
      const a3 = this.dragAccel(q3, v3, target, W);
      if (!a3) return false;
      const [q4, v4] = stage(a3, v3, 1);
      const a4 = this.dragAccel(q4, v4, target, W);
      if (!a4) return false;
      for (const j of dr.free) {
        q[j] += (h / 6) * (v[j] + 2 * v2[j] + 2 * v3[j] + v4[j]);
        v[j] += (h / 6) * (a1[j] + 2 * a2[j] + 2 * a3[j] + a4[j]);
      }
      this.projectFree(q, v);
      if (!q.every(Number.isFinite) || !v.every(Number.isFinite)) return false;
    }
    return true;
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
