import { compile, type CompiledFn } from '../cas/codegen';
import type { Poly } from '../cas/poly';
import type { Derivation } from './derive';
import { solveInPlace } from './linalg';

/** The symbolic derivation compiled to fast numeric functions. */
export class NumericSystem {
  readonly n: number;
  readonly nc: number;
  readonly params: Float64Array;
  readonly pointIndex: Map<number, number>;
  readonly totalMass: number;

  private readonly massFn: CompiledFn;
  private readonly massBuf: Float64Array;
  private readonly forceFn: CompiledFn;
  private readonly phiFn: CompiledFn;
  private readonly phiQFn: CompiledFn;
  private readonly gammaFn: CompiledFn;
  private readonly posFn: CompiledFn;
  private readonly jacFn: CompiledFn;
  private readonly energyFn: CompiledFn;
  /** Per spring: [D, ½∂D/∂q_0, …, ½∂D/∂q_{n−1}]. */
  private readonly springFn: CompiledFn;
  private readonly springParams: { k: number; rest: number }[];
  private readonly springBuf: Float64Array;
  private readonly gearAngleFn: CompiledFn;
  private readonly meshGradFn: CompiledFn;
  /** Gear ids in gearAngles order, and the index of each gear's φ in q (−1 when locked). */
  readonly gearIds: number[];
  readonly gearCoord: Map<number, number>;

  constructor(readonly derivation: Derivation) {
    const d = derivation;
    const inputs = { coords: d.coords.map((c) => c.sym), vels: d.coords.map((c) => c.vel), params: d.params.map((p) => p.sym) };
    this.n = d.coords.length;
    this.nc = d.constraints.length;
    this.params = Float64Array.from(d.params.map((p) => p.value));
    this.pointIndex = new Map(d.activePoints.map((id, i) => [id, i]));
    this.totalMass = d.params.filter((p) => p.unit === 'kg').reduce((s, p) => s + p.value, 0);

    const pos: Poly[] = [];
    const jac: Poly[] = [];
    for (const id of d.activePoints) {
      const [x, y] = d.positions.get(id)!;
      pos.push(x, y);
      for (const c of inputs.coords) jac.push(x.diff(c));
      for (const c of inputs.coords) jac.push(y.diff(c));
    }
    // M is symmetric: compile the upper triangle only, mass() mirrors it
    this.massFn = compile(d.M.flatMap((row, i) => row.slice(i)), inputs);
    this.massBuf = new Float64Array((this.n * (this.n + 1)) / 2);
    this.forceFn = compile(d.f, inputs);
    this.phiFn = compile(d.constraints.map((c) => c.phi), inputs);
    this.phiQFn = compile(d.phiQ.flat(), inputs);
    this.gammaFn = compile(d.gamma, inputs);
    this.posFn = compile(pos, inputs);
    this.jacFn = compile(jac, inputs);
    this.energyFn = compile([d.T, d.V], inputs);
    const paramIndex = new Map(d.params.map((p, i) => [p.sym, i]));
    this.springParams = d.springs.map((sp) => ({ k: paramIndex.get(sp.k)!, rest: paramIndex.get(sp.rest)! }));
    this.springFn = compile(d.springs.flatMap((sp) => [sp.D, ...sp.grad]), inputs);
    this.springBuf = new Float64Array(d.springs.length * (this.n + 1));
    this.gearIds = d.gearAngles.map((ga) => ga.id);
    this.gearAngleFn = compile(d.gearAngles.map((ga) => ga.angle), inputs);
    this.gearCoord = new Map(this.gearIds.map((id) => [id, d.coords.findIndex((c) => c.sym === `ph${id}`)]));
    this.meshGradFn = compile(d.meshes.flatMap((m) => m.pitchGrad), inputs);
  }
  /** Per mesh, ∂v_pitch/∂q̇ (n entries each, concatenated). */
  meshGrads(q: ArrayLike<number>, out: Float64Array) {
    this.meshGradFn(q, EMPTY, this.params, out);
  }
  /** Rotation φ of every gear, in gearIds order. */
  gearAngles(q: ArrayLike<number>, out: Float64Array) {
    this.gearAngleFn(q, EMPTY, this.params, out);
  }

  mass(q: ArrayLike<number>, out: Float64Array) {
    const { n, massBuf: u } = this;
    this.massFn(q, EMPTY, this.params, u);
    for (let i = 0, k = 0; i < n; i++)
      for (let j = i; j < n; j++, k++) {
        out[i * n + j] = u[k];
        out[j * n + i] = u[k];
      }
  }
  /** f(q, q̇), including the spring forces −∂V_s/∂q. */
  force(q: ArrayLike<number>, qd: ArrayLike<number>, out: Float64Array) {
    this.forceFn(q, qd, this.params, out);
    const { n, springParams, springBuf: buf, params } = this;
    if (!springParams.length) return;
    this.springFn(q, EMPTY, params, buf);
    springParams.forEach((sp, j) => {
      const o = j * (n + 1);
      const len = Math.sqrt(Math.max(buf[o], 0));
      const rest = params[sp.rest];
      // −∂/∂q ½k(ℓ − ℓ₀)² = −k (1 − ℓ₀/ℓ) · ½ ∂(ℓ²)/∂q
      const c = rest === 0 ? params[sp.k] : params[sp.k] * (1 - rest / Math.max(len, 1e-9));
      for (let i = 0; i < n; i++) out[i] -= c * buf[o + 1 + i];
    });
  }
  /** Σ ½ k (ℓ − ℓ₀)² over all springs. */
  springEnergy(q: ArrayLike<number>): number {
    const { n, springParams, springBuf: buf, params } = this;
    if (!springParams.length) return 0;
    this.springFn(q, EMPTY, params, buf);
    let V = 0;
    springParams.forEach((sp, j) => {
      const stretch = Math.sqrt(Math.max(buf[j * (n + 1)], 0)) - params[sp.rest];
      V += 0.5 * params[sp.k] * stretch * stretch;
    });
    return V;
  }
  phi(q: ArrayLike<number>, out: Float64Array) {
    this.phiFn(q, EMPTY, this.params, out);
  }
  phiQ(q: ArrayLike<number>, out: Float64Array) {
    this.phiQFn(q, EMPTY, this.params, out);
  }
  gamma(q: ArrayLike<number>, qd: ArrayLike<number>, out: Float64Array) {
    this.gammaFn(q, qd, this.params, out);
  }
  /** Positions of all active joints, [x0, y0, x1, y1, …] in activePoints order. */
  positions(q: ArrayLike<number>, out: Float64Array) {
    this.posFn(q, EMPTY, this.params, out);
  }
  /** Per joint: 2×n Jacobian ∂r/∂q, row-major, concatenated in activePoints order. */
  jacobians(q: ArrayLike<number>, out: Float64Array) {
    this.jacFn(q, EMPTY, this.params, out);
  }
  energy(q: ArrayLike<number>, qd: ArrayLike<number>): { T: number; V: number } {
    const out = new Float64Array(2);
    this.energyFn(q, qd, this.params, out);
    return { T: out[0], V: out[1] + this.springEnergy(q) };
  }
}

const EMPTY = new Float64Array(0);

/**
 * A spring from the mouse to a material point of the mechanism: (1−s)·r_a + s·r_b on a bar,
 * or, when `gear` is set, the point at offset (u, v) in that gear's frame (a is its centre).
 */
export interface Grab {
  a: number;
  b: number;
  s: number;
  gear?: { id: number; u: number; v: number };
  target: [number, number];
  stiffness: number; // N/m
}

/**
 * Viscous friction as a Rayleigh dissipation function R(q, q̇); each equation gains −∂R/∂q̇ᵢ
 * and friction turns energy into heat at the rate P = 2R.
 *   pin joints:  R = ½ c_pin Σ over pairs of bodies at the joint (ω_i − ω_k)²   (N·m·s)
 *   rails:       R = ½ c_rail v²                                            (N·s/m)
 *   gear teeth:  R = ½ c_mesh v_pitch²,  v_pitch = R_a (φ̇_a − θ̇_c)            (N·s/m)
 *                (for a rack, v_pitch = R (φ̇ − θ̇_rack), the speed along the rack)
 * Each joint and mesh has a temperature rise ΔT obeying Newton's law of cooling,
 *   C dΔT/dt = P − (C/τ) ΔT.
 */
export interface FrictionSettings {
  enabled: boolean;
  pin: number;
  rail: number;
  mesh: number;
  /** Cooling time constant τ (s). */
  coolingTime: number;
  /** Heat capacity C of each joint or mesh (J/K). */
  heatCapacity: number;
}

/** Temperature rise (K) by joint id, gear-mesh id and rack-mesh id. */
export interface Heat {
  joints: Map<number, number>;
  meshes: Map<number, number>;
  racks: Map<number, number>;
}

export const defaultFriction = (): FrictionSettings => ({ enabled: false, pin: 0.02, rail: 0.5, mesh: 0.5, coolingTime: 4, heatCapacity: 0.02 });

export class Simulator {
  readonly q: Float64Array;
  readonly qd: Float64Array;
  t = 0;
  /** Mass-proportional drag coefficient c (1/s): Q_drag = −c M q̇. */
  damping = 0;
  grab: Grab | null = null;
  /** Total force of the grab, spring pull plus damping (N). */
  appliedForce: [number, number] = [0, 0];
  friction: FrictionSettings = defaultFriction();
  /** Temperature rise (K) of every joint (derivation.joints order) and mesh (derivation.meshes order). */
  readonly jointHeat: Float64Array;
  readonly meshHeat: Float64Array;
  /** Total energy turned into heat by friction (J). */
  dissipated = 0;
  /** Current friction power (W) of every joint then every mesh. */
  readonly power: Float64Array;
  private readonly stagePower: Float64Array[];
  private readonly bodyGrad: Float64Array[];
  private readonly meshBuf: Float64Array;

  private readonly n: number;
  private readonly nc: number;
  private readonly M: Float64Array;
  private readonly f: Float64Array;
  private readonly K: Float64Array;
  private readonly rhs: Float64Array;
  private readonly Phi: Float64Array;
  private readonly PhiQ: Float64Array;
  private readonly gam: Float64Array;
  private readonly pos: Float64Array;
  private readonly jac: Float64Array;
  private readonly JJt: Float64Array;
  private readonly mu: Float64Array;

  constructor(readonly sys: NumericSystem) {
    const { n, nc } = sys;
    this.n = n;
    this.nc = nc;
    this.q = Float64Array.from(sys.derivation.coords.map((c) => c.initial));
    this.qd = new Float64Array(n);
    this.M = new Float64Array(n * n);
    this.f = new Float64Array(n);
    this.K = new Float64Array((n + nc) * (n + nc));
    this.rhs = new Float64Array(n + nc);
    this.Phi = new Float64Array(nc);
    this.PhiQ = new Float64Array(nc * n);
    this.gam = new Float64Array(nc);
    this.pos = new Float64Array(2 * sys.pointIndex.size);
    this.jac = new Float64Array(2 * n * sys.pointIndex.size);
    this.JJt = new Float64Array(nc * nc);
    this.mu = new Float64Array(nc);
    const d = sys.derivation;
    this.jointHeat = new Float64Array(d.joints.length);
    this.meshHeat = new Float64Array(d.meshes.length);
    const ne = d.joints.length + d.meshes.length;
    this.power = new Float64Array(ne);
    this.stagePower = [0, 1, 2, 3].map(() => new Float64Array(ne));
    const maxBodies = d.joints.reduce((m, j) => Math.max(m, j.bars.length + j.gears.length + 1), 1);
    this.bodyGrad = Array.from({ length: maxBodies }, () => new Float64Array(n));
    this.meshBuf = new Float64Array(d.meshes.length * n);
  }

  /** q̈ from M q̈ = f + Q, Φ_q q̈ = γ (Baumgarte-stabilised), solved as one KKT system. */
  accelerations(q: Float64Array, qd: Float64Array, out: Float64Array, stage = -1): boolean {
    const { n, sys, M, f, rhs } = this;
    sys.mass(q, M);
    sys.force(q, qd, f);
    if (this.friction.enabled) this.applyFriction(q, qd, f, stage >= 0 ? this.stagePower[stage] : this.power);

    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += M[i * n + j] * qd[j];
      f[i] -= this.damping * s;
    }
    this.applyGrab(q, qd, f);

    if (this.solveKKT(q, qd, f, 1e-10) || this.solveKKT(q, qd, f, 1e-6)) {
      for (let i = 0; i < n; i++) out[i] = rhs[i];
      return true;
    }
    return false;
  }

  /** Assemble [M Φ_qᵀ; Φ_q −εI] [q̈; −λ] = [f; γ̃] and solve it; the result is left in rhs. */
  private solveKKT(q: Float64Array, qd: Float64Array, f: Float64Array, eps: number): boolean {
    const { n, nc, sys, M, K, rhs } = this;
    const N = n + nc;
    K.fill(0);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) K[i * N + j] = M[i * n + j];
      rhs[i] = f[i];
    }
    if (nc > 0) {
      const { Phi, PhiQ, gam } = this;
      sys.phi(q, Phi);
      sys.phiQ(q, PhiQ);
      sys.gamma(q, qd, gam);
      const alpha = 10;
      const beta = 10;
      for (let k = 0; k < nc; k++) {
        let phiDot = 0;
        for (let j = 0; j < n; j++) {
          const v = PhiQ[k * n + j];
          K[(n + k) * N + j] = v;
          K[j * N + n + k] = v;
          phiDot += v * qd[j];
        }
        K[(n + k) * N + n + k] = -eps; // regularises redundant constraints
        rhs[n + k] = gam[k] - 2 * alpha * phiDot - beta * beta * Phi[k];
      }
    }
    return solveInPlace(K, rhs, N);
  }

  /** Add −∂R/∂q̇ to f and record the power 2R dissipated at every joint and mesh. */
  private applyFriction(q: Float64Array, qd: Float64Array, f: Float64Array, power: Float64Array) {
    const { n, sys, pos, jac, bodyGrad, friction } = this;
    const d = sys.derivation;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    const dotQd = (g: Float64Array) => {
      let s = 0;
      for (let m = 0; m < n; m++) s += g[m] * qd[m];
      return s;
    };
    d.joints.forEach((jt, ji) => {
      let P = 0;
      const ij = sys.pointIndex.get(jt.id)!;
      // angular-velocity gradients ∂ω/∂q̇ of the bodies meeting here
      let nb = 0;
      for (const { other } of jt.bars) {
        const io = sys.pointIndex.get(other)!;
        const dx = pos[2 * io] - pos[2 * ij];
        const dy = pos[2 * io + 1] - pos[2 * ij + 1];
        const L2 = dx * dx + dy * dy || 1e-12;
        const g = bodyGrad[nb++];
        for (let m = 0; m < n; m++) {
          const ddx = jac[io * 2 * n + m] - jac[ij * 2 * n + m];
          const ddy = jac[io * 2 * n + n + m] - jac[ij * 2 * n + n + m];
          g[m] = (dx * ddy - dy * ddx) / L2; // θ = atan2(Δy, Δx)
        }
      }
      for (const gid of jt.gears) {
        const g = bodyGrad[nb++];
        g.fill(0);
        const k = sys.gearCoord.get(gid) ?? -1;
        if (k >= 0) g[k] = 1;
      }
      if (jt.ground) bodyGrad[nb++].fill(0);
      if (friction.pin > 0)
        for (let a = 0; a < nb; a++)
          for (let b = a + 1; b < nb; b++) {
            const ga = bodyGrad[a];
            const gb = bodyGrad[b];
            let w = 0;
            for (let m = 0; m < n; m++) w += (ga[m] - gb[m]) * qd[m];
            P += friction.pin * w * w;
            for (let m = 0; m < n; m++) f[m] -= friction.pin * w * (ga[m] - gb[m]);
          }
      if (jt.rail && friction.rail > 0) {
        const g = bodyGrad[0];
        const row = jt.rail === 'x' ? 0 : 1;
        for (let m = 0; m < n; m++) g[m] = jac[ij * 2 * n + row * n + m];
        const v = dotQd(g);
        P += friction.rail * v * v;
        for (let m = 0; m < n; m++) f[m] -= friction.rail * v * g[m];
      }
      power[ji] = P;
    });
    if (d.meshes.length) {
      sys.meshGrads(q, this.meshBuf);
      d.meshes.forEach((_, mi) => {
        const g = this.meshBuf.subarray(mi * n, mi * n + n);
        const v = dotQd(g);
        power[d.joints.length + mi] = friction.mesh * v * v;
        for (let m = 0; m < n; m++) f[m] -= friction.mesh * v * g[m];
      });
    }
  }

  /** Generalized force of the mouse spring: Q = Jᵀ F at the grabbed material point. */
  private applyGrab(q: Float64Array, qd: Float64Array, f: Float64Array) {
    const g = this.grab;
    if (!g) return;
    const { n, sys, pos, jac } = this;
    const ia = sys.pointIndex.get(g.a);
    const ib = sys.pointIndex.get(g.b);
    if (ia === undefined || ib === undefined) return;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    const wa = 1 - g.s;
    const wb = g.s;
    let J = (row: number, j: number) => wa * jac[ia * 2 * n + row * n + j] + wb * jac[ib * 2 * n + row * n + j];
    let rx = wa * pos[2 * ia] + wb * pos[2 * ib];
    let ry = wa * pos[2 * ia + 1] + wb * pos[2 * ib + 1];
    if (g.gear) {
      // r = r_c + Rot(φ)(u, v);  ∂r/∂q = ∂r_c/∂q + Rot'(φ)(u, v) ∂φ/∂q
      const k = sys.gearCoord.get(g.gear.id) ?? -1;
      const phi = k >= 0 ? q[k] : 0;
      const [c, s] = [Math.cos(phi), Math.sin(phi)];
      const { u, v } = g.gear;
      rx += u * c - v * s;
      ry += u * s + v * c;
      const dx = -u * s - v * c;
      const dy = u * c - v * s;
      const base = J;
      J = (row, j) => base(row, j) + (j === k ? (row === 0 ? dx : dy) : 0);
    }
    let vx = 0;
    let vy = 0;
    for (let j = 0; j < n; j++) {
      vx += J(0, j) * qd[j];
      vy += J(1, j) * qd[j];
    }
    // The spring pulls the point towards the cursor; the damper steadies it like a firm hand,
    // resisting its motion in every direction (damping only along the line to the cursor
    // leaves held mechanisms swinging for seconds). The app's arrow shows the pull alone,
    // since the damper opposes the point's velocity, which is rarely along that line.
    const c = 2 * 0.5 * Math.sqrt(g.stiffness * Math.max(sys.totalMass, 1e-3));
    const Fx = g.stiffness * (g.target[0] - rx) - c * vx;
    const Fy = g.stiffness * (g.target[1] - ry) - c * vy;
    this.appliedForce = [Fx, Fy];
    for (let j = 0; j < n; j++) f[j] += J(0, j) * Fx + J(1, j) * Fy;
  }

  /** One classical 4th-order Runge–Kutta step of size h on y = (q, q̇). */
  step(h: number): boolean {
    const { n } = this;
    if (n === 0) return true;
    const q0 = this.q.slice();
    const v0 = this.qd.slice();
    const tq = new Float64Array(n);
    const tv = new Float64Array(n);
    const a = [new Float64Array(n), new Float64Array(n), new Float64Array(n), new Float64Array(n)];
    const v = [v0, new Float64Array(n), new Float64Array(n), new Float64Array(n)];

    if (!this.accelerations(q0, v0, a[0], 0)) return false;
    const stage = (k: number, c: number) => {
      for (let i = 0; i < n; i++) {
        tq[i] = q0[i] + c * h * v[k - 1][i];
        tv[i] = v0[i] + c * h * a[k - 1][i];
      }
      v[k].set(tv);
      return this.accelerations(tq, tv, a[k], k);
    };
    if (!stage(1, 0.5) || !stage(2, 0.5) || !stage(3, 1)) return false;

    for (let i = 0; i < n; i++) {
      this.q[i] = q0[i] + (h / 6) * (v[0][i] + 2 * v[1][i] + 2 * v[2][i] + v[3][i]);
      this.qd[i] = v0[i] + (h / 6) * (a[0][i] + 2 * a[1][i] + 2 * a[2][i] + a[3][i]);
    }
    this.t += h;
    this.updateHeat(h);
    this.project();
    for (let i = 0; i < n; i++) if (!Number.isFinite(this.q[i]) || !Number.isFinite(this.qd[i])) return false;
    return true;
  }

  /**
   * Heat from friction, averaged over the RK4 stages with the RK4 weights (so the energy
   * bookkeeping matches the integrator), then Newton's law of cooling solved exactly:
   * ΔT ← ΔT e^{−h/τ} + (P̄/C) τ (1 − e^{−h/τ}).
   */
  private updateHeat(h: number) {
    const { friction, power, stagePower, jointHeat, meshHeat } = this;
    const decay = Math.exp(-h / friction.coolingTime);
    const gain = (friction.coolingTime * (1 - decay)) / friction.heatCapacity;
    const nj = jointHeat.length;
    for (let e = 0; e < power.length; e++) {
      const P = friction.enabled ? (stagePower[0][e] + 2 * stagePower[1][e] + 2 * stagePower[2][e] + stagePower[3][e]) / 6 : 0;
      power[e] = P;
      this.dissipated += P * h;
      const heat = e < nj ? jointHeat : meshHeat;
      const i = e < nj ? e : e - nj;
      heat[i] = heat[i] * decay + P * gain;
    }
  }

  /** Temperature rise (K) of every joint keyed by joint id, and of every gear or rack mesh keyed by mesh id. */
  heat(): Heat {
    const d = this.sys.derivation;
    const of = (kind: 'gear' | 'rack') => new Map(d.meshes.flatMap((m, i) => (m.kind === kind ? [[m.id, this.meshHeat[i]] as [number, number]] : [])));
    return { joints: new Map(d.joints.map((j, i) => [j.id, this.jointHeat[i]])), meshes: of('gear'), racks: of('rack') };
  }

  /** Remove constraint drift: Newton-project q onto Φ = 0 and q̇ onto Φ_q q̇ = 0. */
  project() {
    const { n, nc, sys, Phi, PhiQ, JJt, mu, q, qd } = this;
    if (nc === 0) return;
    const buildJJt = () => {
      for (let a = 0; a < nc; a++)
        for (let b = 0; b < nc; b++) {
          let s = 0;
          for (let j = 0; j < n; j++) s += PhiQ[a * n + j] * PhiQ[b * n + j];
          JJt[a * nc + b] = s + (a === b ? 1e-12 : 0);
        }
    };
    for (let iter = 0; iter < 3; iter++) {
      sys.phi(q, Phi);
      let err = 0;
      for (let k = 0; k < nc; k++) err = Math.max(err, Math.abs(Phi[k]));
      if (err < 1e-12) break;
      sys.phiQ(q, PhiQ);
      buildJJt();
      mu.set(Phi);
      if (!solveInPlace(JJt, mu, nc)) break;
      for (let j = 0; j < n; j++) for (let k = 0; k < nc; k++) q[j] -= PhiQ[k * n + j] * mu[k];
    }
    sys.phiQ(q, PhiQ);
    buildJJt();
    for (let k = 0; k < nc; k++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += PhiQ[k * n + j] * qd[j];
      mu[k] = s;
    }
    if (solveInPlace(JJt, mu, nc)) for (let j = 0; j < n; j++) for (let k = 0; k < nc; k++) qd[j] -= PhiQ[k * n + j] * mu[k];
  }

  /** Current joint positions keyed by joint id. */
  /** Position of one joint (cheaper than jointPositions when only one is needed). */
  jointPosition(id: number): [number, number] | null {
    const i = this.sys.pointIndex.get(id);
    if (i === undefined) return null;
    this.sys.positions(this.q, this.pos);
    return [this.pos[2 * i], this.pos[2 * i + 1]];
  }

  jointPositions(): Map<number, [number, number]> {
    this.sys.positions(this.q, this.pos);
    const out = new Map<number, [number, number]>();
    for (const [id, i] of this.sys.pointIndex) out.set(id, [this.pos[2 * i], this.pos[2 * i + 1]]);
    return out;
  }

  energy() {
    return this.sys.energy(this.q, this.qd);
  }

  /** Current rotation of every gear, keyed by gear id. */
  gearAngles(): Map<number, number> {
    const out = new Float64Array(this.sys.gearIds.length);
    this.sys.gearAngles(this.q, out);
    return new Map(this.sys.gearIds.map((id, i) => [id, out[i]]));
  }

  constraintError(): number {
    if (this.nc === 0) return 0;
    this.sys.phi(this.q, this.Phi);
    let e = 0;
    for (let k = 0; k < this.nc; k++) e = Math.max(e, Math.abs(this.Phi[k]));
    return e;
  }
}
