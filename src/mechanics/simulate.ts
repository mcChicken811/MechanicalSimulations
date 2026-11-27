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
  private readonly springParams: { k: number; rest: number; tensionOnly: boolean }[];
  private readonly springBuf: Float64Array;
  private readonly gearAngleFn: CompiledFn;
  /** Per slider stop: [s, ∂s/∂q_0, …, ∂s/∂q_{n−1}]. */
  private readonly stopFn: CompiledFn;
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
    this.springParams = d.springs.map((sp) => ({ k: paramIndex.get(sp.k)!, rest: paramIndex.get(sp.rest)!, tensionOnly: sp.tensionOnly }));
    this.springFn = compile(d.springs.flatMap((sp) => [sp.D, ...sp.grad]), inputs);
    this.springBuf = new Float64Array(d.springs.length * (this.n + 1));
    this.gearIds = d.gearAngles.map((ga) => ga.id);
    this.gearAngleFn = compile(d.gearAngles.map((ga) => ga.angle), inputs);
    this.gearCoord = new Map(this.gearIds.map((id) => [id, d.coords.findIndex((c) => c.sym === `ph${id}`)]));
    this.meshGradFn = compile(d.meshes.flatMap((m) => m.pitchGrad), inputs);
    this.stopFn = compile(d.sliderStops.flatMap((st) => [st.s, ...st.grad]), inputs);
  }
  /** Per slider stop: its travel s and ∂s/∂q (n + 1 entries each, concatenated). */
  stops(q: ArrayLike<number>, out: Float64Array) {
    this.stopFn(q, EMPTY, this.params, out);
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
      if (sp.tensionOnly && len < rest) return; // a slack cable pulls nothing
      // −∂/∂q ½k(ℓ − ℓ₀)² = −k (1 − ℓ₀/ℓ) · ½ ∂(ℓ²)/∂q
      const c = rest === 0 ? params[sp.k] : params[sp.k] * (1 - rest / Math.max(len, 1e-9));
      for (let i = 0; i < n; i++) out[i] -= c * buf[o + 1 + i];
    });
  }
  /** Σ ½ k (ℓ − ℓ₀)² over all springs (tension-only ones only while stretched). */
  springEnergy(q: ArrayLike<number>): number {
    const { n, springParams, springBuf: buf, params } = this;
    if (!springParams.length) return 0;
    this.springFn(q, EMPTY, params, buf);
    let V = 0;
    springParams.forEach((sp, j) => {
      const stretch = Math.sqrt(Math.max(buf[j * (n + 1)], 0)) - params[sp.rest];
      if (!sp.tensionOnly || stretch > 0) V += 0.5 * params[sp.k] * stretch * stretch;
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

/**
 * Ground contact, as compliant (penalty) contact. Every joint and the lowest point of every
 * gear rim is checked against the ground line; a bar cannot reach below the ground before one
 * of its ends does, so its joints cover it. A point that has sunk by δ is pushed out along the
 * ground normal n and held by friction along the ground t, through Q = Jᵀ(F_n n + F_t t):
 *   F_n = max(0, k δ + c δ̇),   k = m ω²,   c = 2 ζ m ω,   ζ set from the restitution e
 *   |F_t| ≤ μ F_n: a stiff spring-damper to where the point first touched (static friction),
 *          sliding along at the Coulomb limit when it is exceeded.
 * m is the effective mass the contact pushes against, 1 / (gᵀ A g) with g = Jᵀn and A the
 * constrained inverse inertia (from the KKT matrix), found once when the contact starts. So
 * every contact responds at the same rate ω, whatever it carries: a 5 g fingertip and a 70 kg
 * leg are equally stable with the 1 ms RK4 step, and anything resting sinks only g/ω² (MuJoCo
 * sets contact softness the same way, as a time constant). The coefficient of restitution e
 * sets the damping ratio ζ, calibrated for the clamped force (see clampedRestitution).
 */
export interface GroundSettings {
  /** Height of the ground line at x = 0 (m). */
  y: number;
  /** Slope (rad); positive rises to the right. */
  slope: number;
  restitution: number;
  friction: number;
}

/** Response rate ω of every contact (rad/s): resting objects sink g/ω² ≈ 0.16 mm. */
export const CONTACT_OMEGA = 250;

/** Largest contact damping ratio: 2ζω h stays within RK4's stable range at the 1 ms step. */
const MAX_ZETA = 4;

/**
 * Restitution of the contact spring-damper with damping ratio ζ. Its force is clamped at zero
 * (the ground cannot pull), so the point leaves while still slightly below the ground and a
 * little faster than the textbook e = exp(−ζπ/√(1−ζ²)): even ζ = 1 bounces back at 0.135.
 * Found by integrating the unit contact δ'' = −max(0, δ + 2ζδ') from δ' = 1 until δ = 0.
 */
function clampedRestitution(zeta: number): number {
  const dt = 2e-3;
  const acc = (d: number, v: number) => -Math.max(0, d + 2 * zeta * v);
  let [d, v] = [0, 1];
  for (let i = 0; i < 20000; i++) {
    const k1v = acc(d, v);
    const k2v = acc(d + (dt / 2) * v, v + (dt / 2) * k1v);
    const k3v = acc(d + (dt / 2) * (v + (dt / 2) * k1v), v + (dt / 2) * k2v);
    const k4v = acc(d + dt * (v + (dt / 2) * k2v), v + dt * k3v);
    d += (dt / 6) * (v + 2 * (v + (dt / 2) * k1v) + 2 * (v + (dt / 2) * k2v) + (v + dt * k3v));
    v += (dt / 6) * (k1v + 2 * k2v + 2 * k3v + k4v);
    if (d <= 0) return Math.max(0, -v);
  }
  return 0;
}

const zetaCache = new Map<number, number>();
/** The damping ratio whose clamped contact has restitution e (e below about 0.014 gives ζ = 4). */
export function dampingRatio(e: number): number {
  const key = Math.round(Math.min(Math.max(e, 0), 1) * 1000) / 1000;
  let z = zetaCache.get(key);
  if (z === undefined) {
    let [lo, hi] = [0, MAX_ZETA];
    if (clampedRestitution(hi) >= key) z = hi;
    else {
      for (let i = 0; i < 40; i++) {
        const mid = (lo + hi) / 2;
        if (clampedRestitution(mid) > key) lo = mid;
        else hi = mid;
      }
      z = (lo + hi) / 2;
    }
    zetaCache.set(key, z);
  }
  return z;
}

/** A touching point: the effective masses along n and t, and where static friction holds it. */
interface ContactState {
  mn: number;
  mt: number;
  anchor: number | null;
}

/** Something that can touch the ground: a joint, or the bottom of a gear's rim. */
interface ContactCandidate {
  point: number;
  gear?: { phi: number; radius: number };
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
  /** Total energy turned into heat by friction and by ground impacts (J). */
  dissipated = 0;
  /** Ground contact, or null for none. */
  ground: GroundSettings | null = null;
  /** Total work done by the motors (J). */
  motorWork = 0;
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
  private readonly candidates: ContactCandidate[];
  private readonly contacts: (ContactState | null)[];
  private readonly Jn: Float64Array;
  private readonly Jt: Float64Array;
  private readonly K2: Float64Array;
  private readonly r2: Float64Array;
  /** Ground-contact power dissipated at each RK4 stage (W). */
  private readonly stageContactPower = new Float64Array(4);
  /** Motor power delivered at each RK4 stage (W). */
  private readonly stageMotorPower = new Float64Array(4);
  private readonly motorParams: { coord: number; tau: number; b: number }[];
  /** Slider end stops: travel and gradient per stop, and the effective mass when touching (per end). */
  private readonly stopBuf: Float64Array;
  private readonly stopMass: (number | null)[];
  private readonly stopGrad: Float64Array;

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
    const paramIndex = new Map(d.params.map((p, i) => [p.sym, i]));
    this.motorParams = d.motors.map((m) => ({ coord: m.coord, tau: paramIndex.get(m.tau)!, b: m.b ? paramIndex.get(m.b)! : -1 }));
    this.candidates = [
      ...[...sys.pointIndex.values()].map((point) => ({ point })),
      ...d.gearDisks.map((g) => ({ point: sys.pointIndex.get(g.center)!, gear: { phi: sys.gearCoord.get(g.id) ?? -1, radius: g.radius } })),
    ];
    this.contacts = this.candidates.map(() => null);
    this.stopBuf = new Float64Array(d.sliderStops.length * (n + 1));
    this.stopMass = d.sliderStops.flatMap(() => [null, null]);
    this.stopGrad = new Float64Array(n);
    this.Jn = new Float64Array(n);
    this.Jt = new Float64Array(n);
    this.K2 = new Float64Array((n + nc) * (n + nc));
    this.r2 = new Float64Array(n + nc);
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
    this.applyContacts(q, qd, f, stage);
    this.applyStops(q, qd, f, stage);
    if (stage >= 0) {
      // the motors' torque is in f already (symbolically); here only its power τ(φ̇) φ̇
      let P = 0;
      for (const m of this.motorParams) {
        const w = qd[m.coord];
        P += (sys.params[m.tau] - (m.b >= 0 ? sys.params[m.b] * w : 0)) * w;
      }
      this.stageMotorPower[stage] = P;
    }

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
      for (const bar of jt.bars) {
        const ia = sys.pointIndex.get(bar.a)!;
        const ib = sys.pointIndex.get(bar.b)!;
        const dx = pos[2 * ib] - pos[2 * ia];
        const dy = pos[2 * ib + 1] - pos[2 * ia + 1];
        const L2 = dx * dx + dy * dy || 1e-12;
        const g = bodyGrad[nb++];
        for (let m = 0; m < n; m++) {
          const ddx = jac[ib * 2 * n + m] - jac[ia * 2 * n + m];
          const ddy = jac[ib * 2 * n + n + m] - jac[ia * 2 * n + n + m];
          g[m] = (dx * ddy - dy * ddx) / L2; // the bar's angle θ = atan2(Δy, Δx)
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
      if (jt.rail !== null && friction.rail > 0) {
        // speed along the ground rail: v = (cos a, sin a) · ṙ
        const g = bodyGrad[0];
        const [c, sn] = [Math.cos(jt.rail), Math.sin(jt.rail)];
        for (let m = 0; m < n; m++) g[m] = c * jac[ij * 2 * n + m] + sn * jac[ij * 2 * n + n + m];
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

  /**
   * Fill Jn, Jt (rows of ∂(contact point)/∂q along the ground normal and tangent) for one
   * candidate, with pos and jac already filled, and return its gap above the ground and its
   * position along the ground.
   */
  private contactGeometry(c: ContactCandidate, g: GroundSettings): { gap: number; along: number } {
    const { n, pos, jac, Jn, Jt } = this;
    const [nx, ny] = [-Math.sin(g.slope), Math.cos(g.slope)];
    const [tx, ty] = [Math.cos(g.slope), Math.sin(g.slope)];
    const i = c.point;
    let px = pos[2 * i];
    let py = pos[2 * i + 1];
    for (let m = 0; m < n; m++) {
      const jx = jac[i * 2 * n + m];
      const jy = jac[i * 2 * n + n + m];
      Jn[m] = nx * jx + ny * jy;
      Jt[m] = tx * jx + ty * jy;
    }
    if (c.gear) {
      // the lowest point of the rim, r = −R n from the centre, moves with the disk:
      // ṙ_p = ṙ_c + φ̇ k × r and k × r = R t, so φ̇ adds R φ̇ along the ground only
      const R = c.gear.radius;
      px -= R * nx;
      py -= R * ny;
      if (c.gear.phi >= 0) Jt[c.gear.phi] += R;
    }
    return { gap: nx * px + ny * (py - g.y), along: tx * px + ty * (py - g.y) };
  }

  /** gᵀ A g: how readily the mechanism moves along g, A being the constrained inverse inertia. */
  private mobility(q: Float64Array, g: Float64Array): number {
    const { n, nc, sys, M, K2: K, r2: r, PhiQ } = this;
    const N = n + nc;
    K.fill(0);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) K[i * N + j] = M[i * n + j];
      r[i] = g[i];
    }
    if (nc > 0) {
      sys.phiQ(q, PhiQ);
      for (let k = 0; k < nc; k++) {
        for (let j = 0; j < n; j++) K[(n + k) * N + j] = K[j * N + n + k] = PhiQ[k * n + j];
        K[(n + k) * N + n + k] = -1e-10;
        r[n + k] = 0;
      }
    }
    if (!solveInPlace(K, r, N)) return 0;
    let w = 0;
    for (let i = 0; i < n; i++) w += g[i] * r[i];
    return w;
  }


  /** Normal and friction force of one touching candidate; also returns the power it dissipates. */
  private contactForce(st: ContactState, g: GroundSettings, delta: number, vn: number, vt: number, along: number, rolling: boolean) {
    const w = CONTACT_OMEGA;
    const zeta = dampingRatio(g.restitution);
    const kn = st.mn * w * w;
    let Fn = kn * delta - 2 * zeta * st.mn * w * vn; // vn = −δ̇
    if (Fn < 0) Fn = 0;
    let Ft = 0;
    if (g.friction > 0 && Fn > 0) {
      const kt = st.mt * w * w;
      const ct = 2 * st.mt * w;
      Ft = !rolling && st.anchor !== null ? -kt * (along - st.anchor) - ct * vt : -ct * vt;
      const lim = g.friction * Fn;
      if (Ft > lim) Ft = lim;
      else if (Ft < -lim) Ft = -lim;
    }
    // power lost: what the normal spring-damper does beyond the stored ½kδ², and all friction work
    const loss = vn * (kn * delta - Fn) - Ft * vt;
    return { Fn, Ft, loss };
  }

  /** Add the ground's contact forces Q = Jᵀ(F_n n + F_t t) to f. */
  private applyContacts(q: Float64Array, qd: Float64Array, f: Float64Array, stage: number) {
    const g = this.ground;
    if (stage >= 0) this.stageContactPower[stage] = 0;
    if (!g) return;
    const { n, sys, pos, jac, Jn, Jt, candidates, contacts } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    let power = 0;
    candidates.forEach((c, ci) => {
      const { gap, along } = this.contactGeometry(c, g);
      if (gap >= 0) return;
      let st = contacts[ci];
      if (!st) {
        // a new contact: the effective masses it pushes against, along n and along t
        const wn = this.mobility(q, Jn);
        if (wn < 1e-12) return; // pinned: it cannot move into the ground anyway
        const wt = this.mobility(q, Jt);
        st = contacts[ci] = { mn: 1 / wn, mt: wt > 1e-12 ? 1 / wt : 1 / wn, anchor: null };
      }
      let vn = 0;
      let vt = 0;
      for (let m = 0; m < n; m++) {
        vn += Jn[m] * qd[m];
        vt += Jt[m] * qd[m];
      }
      const { Fn, Ft, loss } = this.contactForce(st, g, -gap, vn, vt, along, !!c.gear);
      for (let m = 0; m < n; m++) f[m] += Jn[m] * Fn + Jt[m] * Ft;
      power += loss;
    });
    if (stage >= 0) this.stageContactPower[stage] = power;
  }

  /**
   * After a step: drop contacts that have lifted off, start static friction for new ones, and
   * let a contact pushed past μ F_n slide, moving its anchor to where the friction limit holds.
   */
  private updateContacts(h: number) {
    const g = this.ground;
    const P = this.stageContactPower;
    this.dissipated += ((P[0] + 2 * P[1] + 2 * P[2] + P[3]) / 6) * h;
    if (!g) {
      this.contacts.fill(null);
      return;
    }
    const { n, sys, q, qd, pos, jac, Jn, candidates, contacts } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    candidates.forEach((c, ci) => {
      const st = contacts[ci];
      if (!st) return;
      const { gap, along } = this.contactGeometry(c, g);
      if (gap >= 0) {
        contacts[ci] = null;
        return;
      }
      if (c.gear) return; // a rolling rim has no fixed contact point: friction is viscous there
      if (st.anchor === null) {
        st.anchor = along;
        return;
      }
      let vn = 0;
      for (let m = 0; m < n; m++) vn += Jn[m] * qd[m];
      const w = CONTACT_OMEGA;
      const Fn = Math.max(0, st.mn * w * w * -gap - 2 * dampingRatio(g.restitution) * st.mn * w * vn);
      const lim = g.friction * Fn;
      const kt = st.mt * w * w;
      const spring = -kt * (along - st.anchor);
      if (Math.abs(spring) > lim) st.anchor = along + Math.sign(spring) * (lim / kt); // slide
    });
  }

  /** Stored energy of the ground contacts and slider stops, ½ k δ² each. */
  private contactEnergy(): number {
    let V = 0;
    if (this.sys.derivation.sliderStops.length) {
      this.sys.stops(this.q, this.stopBuf);
      this.sys.derivation.sliderStops.forEach((_, k) =>
        this.stopDepths(k).forEach(([delta], end) => {
          const m = this.stopMass[2 * k + end];
          if (delta > 0 && m !== null) V += 0.5 * m * CONTACT_OMEGA * CONTACT_OMEGA * delta * delta;
        }),
      );
    }
    const g = this.ground;
    if (!g) return V;
    const { sys, q, pos, jac, candidates, contacts } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    candidates.forEach((c, ci) => {
      const st = contacts[ci];
      if (!st) return;
      const { gap } = this.contactGeometry(c, g);
      if (gap < 0) V += 0.5 * st.mn * CONTACT_OMEGA * CONTACT_OMEGA * gap * gap;
    });
    return V;
  }

  /**
   * How far each slider joint is past its stops: for stop k, end 0 (min) and end 1 (max), the
   * depth δ ≥ 0 and the sign of the push along ∂s/∂q (+1 pushes s up, off the min stop).
   */
  private stopDepths(k: number): [number, number][] {
    const st = this.sys.derivation.sliderStops[k];
    const s = this.stopBuf[k * (this.n + 1)];
    return [
      [st.min !== null ? st.min - s : -Infinity, 1],
      [st.max !== null ? s - st.max : -Infinity, -1],
    ];
  }

  /**
   * Slider end stops, treated like the ground: past an end by δ, the joint is pushed back along
   * its rail by F = max(0, k δ + c δ̇), k = m ω², c = 2ζmω, m the effective mass along ∂s/∂q, as
   * the generalized force Q = ±F ∂s/∂q (on a body's rail this pushes the body back as well).
   */
  private applyStops(q: Float64Array, qd: Float64Array, f: Float64Array, stage: number) {
    const stops = this.sys.derivation.sliderStops;
    if (!stops.length) return;
    const { n, sys, stopBuf, stopMass, stopGrad: g } = this;
    sys.stops(q, stopBuf);
    let power = 0;
    stops.forEach((st, k) => {
      const o = k * (n + 1);
      for (let m = 0; m < n; m++) g[m] = stopBuf[o + 1 + m];
      let sd = 0;
      for (let m = 0; m < n; m++) sd += g[m] * qd[m];
      this.stopDepths(k).forEach(([delta, push], end) => {
        if (delta <= 0) return;
        const key = 2 * k + end;
        let mass = stopMass[key];
        if (mass === null) {
          const w = this.mobility(q, g);
          if (w < 1e-12) return;
          mass = stopMass[key] = 1 / w;
        }
        const w0 = CONTACT_OMEGA;
        const kk = mass * w0 * w0;
        const ddelta = -push * sd; // δ̇
        let F = kk * delta + 2 * dampingRatio(st.restitution) * mass * w0 * ddelta;
        if (F < 0) F = 0;
        for (let m = 0; m < n; m++) f[m] += push * F * g[m];
        power += ddelta * (F - kk * delta); // what the damper takes
      });
    });
    if (stage >= 0) this.stageContactPower[stage] += power;
  }

  /** Forget stops the joints have left; true if any joint is at, or within reach of, a stop. */
  private updateStops(h: number): boolean {
    const stops = this.sys.derivation.sliderStops;
    if (!stops.length) return false;
    const { n, sys, q, qd, stopBuf, stopMass } = this;
    sys.stops(q, stopBuf);
    let near = false;
    stops.forEach((_, k) => {
      const o = k * (n + 1);
      let sd = 0;
      for (let m = 0; m < n; m++) sd += stopBuf[o + 1 + m] * qd[m];
      this.stopDepths(k).forEach(([delta], end) => {
        if (delta <= 0) stopMass[2 * k + end] = null;
        if (delta > -(2 * h * Math.abs(sd) + 1e-4)) near = true;
      });
    });
    return near;
  }

  /** Number of points touching the ground now. */
  contactCount(): number {
    return this.contacts.reduce((k, c) => k + (c ? 1 : 0), 0);
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
  /**
   * Advance by h. Near the ground (a point touching it, or about to within the step) the step
   * is split into four, as contact forces switch on and off within a few milliseconds; into
   * eight for very dead impacts (ζ > 1), whose strong damping needs it for the energy budget.
   */
  step(h: number): boolean {
    const ground = !!this.ground && this.nearGround(h);
    const stops = this.updateStops(h);
    if (ground || stops) {
      const dead = (ground && dampingRatio(this.ground!.restitution) > 1) || (stops && this.sys.derivation.sliderStops.some((st) => dampingRatio(st.restitution) > 1));
      const parts = dead ? 8 : 4;
      for (let k = 0; k < parts; k++) if (!this.rkStep(h / parts)) return false;
      return true;
    }
    return this.rkStep(h);
  }

  /** Whether any point touches the ground or could reach it within the next step. */
  private nearGround(h: number): boolean {
    const g = this.ground!;
    if (this.contacts.some((c) => c)) return true;
    const { n, sys, q, qd, pos, jac, Jn, candidates } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    return candidates.some((c) => {
      const { gap } = this.contactGeometry(c, g);
      let vn = 0;
      for (let m = 0; m < n; m++) vn += Jn[m] * qd[m];
      return gap < 2 * h * Math.max(0, -vn) + 1e-4;
    });
  }

  /** One classical RK4 step. */
  private rkStep(h: number): boolean {
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
    const mp = this.stageMotorPower;
    this.motorWork += ((mp[0] + 2 * mp[1] + 2 * mp[2] + mp[3]) / 6) * h;
    this.updateHeat(h);
    this.project();
    this.updateContacts(h);
    this.updateStops(h);
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

  /** T and V, V including the energy stored in ground contacts. */
  energy() {
    const e = this.sys.energy(this.q, this.qd);
    return { T: e.T, V: e.V + this.contactEnergy() };
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
