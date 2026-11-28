import { compile, type CompiledFn } from '../cas/codegen';
import type { Poly } from '../cas/poly';
import type { Derivation } from './derive';
import { QUANTITY_IS_RATE, QUANTITY_SCALE, type Quantity } from './model';
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
  private readonly motorAngleFn: CompiledFn;
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
    // the coordinate each gear turns with (its own φ, or the angle of the group it is joined into)
    this.gearCoord = new Map(this.gearIds.map((id) => [id, d.gearCoords.find((g) => g.id === id)?.coord ?? -1]));
    this.meshGradFn = compile(d.meshes.flatMap((m) => m.pitchGrad), inputs);
    this.stopFn = compile(d.sliderStops.flatMap((st) => [st.s, ...st.grad]), inputs);
    this.motorAngleFn = compile(d.motorAngles.map((m) => m.angle), inputs);
  }
  /** Turn ψ of every motor housing, in derivation.motorAngles order. */
  motorAngles(q: ArrayLike<number>, out: Float64Array) {
    this.motorAngleFn(q, EMPTY, this.params, out);
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
 * Ground contact, as compliant (penalty) contact. Every joint is checked against the ground
 * line, and every gear or wheel rim against the nearest points of the terrain (a slope, or a
 * waypoint it rolls over); a bar cannot reach below the ground before one of its ends does,
 * so its joints cover it, except at a terrain peak (see applyPeaks). A point that has sunk by δ is pushed out along the
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
  /** The terrain's waypoints, sorted by x; level beyond the first and the last. */
  points: [number, number][];
  restitution: number;
  friction: number;
}

/** A straight ground through (0, y) at `slope` radians, as simulator settings (for tests and simple cases). */
export function straightGroundSettings(y: number, slope: number, restitution: number, friction: number): GroundSettings {
  const t = Math.tan(slope);
  return { points: [[-1e3, y - 1e3 * t], [1e3, y + 1e3 * t]], restitution, friction };
}

/**
 * The terrain line, ready for queries: for an x, the segment under it (or the level extension
 * beyond an end), its normal n and direction t, and the distance along the terrain from the
 * first waypoint to the segment's start (so friction anchors stay put across segments).
 */
class Terrain {
  private readonly xs: number[];
  private readonly ys: number[];
  private readonly s0: number[];
  constructor(points: [number, number][]) {
    this.xs = points.map((p) => p[0]);
    this.ys = points.map((p) => p[1]);
    this.s0 = [0];
    for (let i = 1; i < points.length; i++) this.s0.push(this.s0[i - 1] + Math.hypot(this.xs[i] - this.xs[i - 1], this.ys[i] - this.ys[i - 1]));
  }
  private segs: TerrainSegment[] | null = null;
  /**
   * The terrain as straight pieces, left to right: the level run in from the left, the segments
   * between waypoints, and the level run out to the right. Waypoint k joins pieces k and k + 1.
   */
  segments(): TerrainSegment[] {
    if (this.segs) return this.segs;
    const { xs, ys, s0 } = this;
    const far = 1e6;
    const last = xs.length - 1;
    const piece = (x0: number, y0: number, x1: number, y1: number, s: number): TerrainSegment => {
      const L = Math.hypot(x1 - x0, y1 - y0) || 1e-12;
      const [tx, ty] = [(x1 - x0) / L, (y1 - y0) / L];
      return { x0, y0, x1, y1, tx, ty, nx: -ty, ny: tx, L, s };
    };
    const out = [piece(xs[0] - far, ys[0], xs[0], ys[0], -far)];
    for (let i = 0; i < last; i++) out.push(piece(xs[i], ys[i], xs[i + 1], ys[i + 1], s0[i]));
    out.push(piece(xs[last], ys[last], xs[last] + far, ys[last], s0[last]));
    return (this.segs = out);
  }

  /** The segment under x: its start point, unit direction t, normal n = k × t, and arc length at its start. */
  at(x: number): { x0: number; y0: number; tx: number; ty: number; nx: number; ny: number; s: number } {
    const { xs, ys, s0 } = this;
    const last = xs.length - 1;
    if (x <= xs[0]) return { x0: xs[0], y0: ys[0], tx: 1, ty: 0, nx: 0, ny: 1, s: 0 };
    if (x >= xs[last]) return { x0: xs[last], y0: ys[last], tx: 1, ty: 0, nx: 0, ny: 1, s: s0[last] };
    let [lo, hi] = [0, last];
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (xs[mid] <= x) lo = mid;
      else hi = mid;
    }
    const [dx, dy] = [xs[hi] - xs[lo], ys[hi] - ys[lo]];
    const L = Math.hypot(dx, dy) || 1;
    return { x0: xs[lo], y0: ys[lo], tx: dx / L, ty: dy / L, nx: -dy / L, ny: dx / L, s: s0[lo] };
  }
}

/** A straight piece of the terrain: its ends, direction t, normal n = k × t, length, and arc length at its start. */
interface TerrainSegment {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
  tx: number;
  ty: number;
  nx: number;
  ny: number;
  L: number;
  s: number;
}

/** Response rate ω of every contact (rad/s): resting objects sink g/ω² ≈ 0.16 mm. */
export const CONTACT_OMEGA = 250;

/** The deepest a terrain peak can be inside a bar and still count as touching it (m). */
const PEAK_REACH = 0.05;

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
  /** Just made: its impact is still to be reported (see Simulator.events). */
  fresh: boolean;
}

/**
 * Where a PID controller's sensor reads, resolved to the simulator's terms: a joint (`point`), a
 * bar's two end joints (`a`, `b`: its centre and angle), or a gear (`gear` with its `center`
 * joint; `angle0` added to its turn, and `spin` for a propeller's blades).
 */
export interface SensorSpec {
  quantity: Quantity;
  point?: number;
  a?: number;
  b?: number;
  gear?: number;
  center?: number;
  angle0?: number;
}

/** A PID controller for the simulator: its motor (gear id), sensor, gains, torque limit and target (display units). */
export interface ControllerSpec {
  motor: number;
  sensor: SensorSpec;
  kp: number;
  ki: number;
  kd: number;
  max: number;
  target: number;
  /**
   * Which way it pushes: 'auto' finds the way the motor moves the quantity when it starts (and
   * does nothing if the motor cannot move it); 'forward' takes a positive torque to raise the
   * quantity, 'reverse' to lower it, whatever the check finds. 'auto' if absent.
   */
  direction?: PidDirection;
}

export type PidDirection = 'auto' | 'forward' | 'reverse';

/** A running controller: its spec and what it is doing now. */
export interface Controller extends ControllerSpec {
  integral: number;
  /** The sign it pushes with: +1 takes a positive torque to raise the measurement, −1 to lower it, 0 (auto only) does nothing. */
  sign: number;
  /** What the check found when it started (or its direction was last set): +1, −1, or 0 if the motor cannot move the quantity. */
  detected: number;
  /** Measurement, error and output (N·m) at the last step. */
  y: number;
  e: number;
  u: number;
  prevY: number | null;
  dFilt: number;
  lastAngle: number | null;
}

/**
 * A force the simulator applies at a point, outside the symbolic equations (ground contact,
 * propeller air, the mouse pull): where, how much, and on what — a joint, a bar or a gear, or a
 * point at fraction s along a joint pair (the mouse pull on a bar, found by its ends).
 */
export interface PointLoad {
  /** What it is: ground contact (or a terrain peak), propeller air, or the mouse pull. */
  type: 'contact' | 'air' | 'pull';
  at: [number, number];
  F: [number, number];
  on: { kind: 'node'; id: number } | { kind: 'link'; id: number } | { kind: 'gear'; id: number } | { kind: 'seg'; a: number; b: number };
}
/** The loads applied at a state: point forces, and slider-stop pushes along their rails (+ raises the travel s). */
export interface Loads {
  points: PointLoad[];
  stops: { joint: number; along: number }[];
}

/** Something that happened in a step, for sound: a landing on the ground or a slider hitting a stop, with its speed and the effective mass it stopped. */
export interface SimEvent {
  kind: 'impact' | 'stop';
  speed: number;
  mass: number;
}

/** A joint, which can touch the ground. */
interface ContactCandidate {
  point: number;
}

/** A gear or wheel rim: its centre's point index, its angle coordinate (−1 if it cannot turn) and radius. */
interface Disk {
  point: number;
  phi: number;
  radius: number;
}

/**
 * Where a rim touches (or nearly touches) the terrain: the inside of a segment, or a waypoint
 * at a crest. n points from the touching point to the rim's centre; gap is the distance between
 * them less the radius.
 */
interface DiskFeature {
  key: string;
  gap: number;
  nx: number;
  ny: number;
  along: number;
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
  /** PID controllers driving motors, run once a step (see setControllers). */
  controllers: Controller[] = [];
  /** Impacts since the UI last emptied this (at most 64 kept), for sound. */
  readonly events: SimEvent[] = [];
  /**
   * After each step, for sound: the fastest a rim rolls along the ground and slips on it, and
   * the fastest a joint scrapes along it (m/s).
   */
  readonly contactMotion = { rolling: 0, slip: 0, scrape: 0 };
  /** Total work done by the motors (J). */
  motorWork = 0;
  /** Total work done by the air on the propellers (J): thrust, rotor drag and air torque. */
  airWork = 0;
  /** Each propeller's thrust along its axis n just now (N), by gear id. */
  readonly thrust = new Map<number, number>();
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
  private readonly disks: Disk[];
  /** The gear id of each disk. */
  private readonly diskGear: number[];
  /** Rim contacts in progress, by "disk:feature" (see diskFeatures). */
  private readonly diskContacts = new Map<string, ContactState>();
  /** Bars, by the point indices of their ends, for the terrain's peaks to push on. */
  private readonly bars: { ia: number; ib: number; link: number }[];
  /** While set, every point force applied is also written here (see snapshot). */
  private capture: Loads | null = null;
  /** Joint ids by point index. */
  private readonly pointIds: number[];
  /** Peak–bar contacts in progress, by "peak:bar": the effective mass along the bar's normal. */
  private readonly peakContacts = new Map<string, number>();
  private readonly Jn: Float64Array;
  private readonly Jt: Float64Array;
  private readonly K2: Float64Array;
  private readonly r2: Float64Array;
  /** Ground-contact power dissipated at each RK4 stage (W). */
  private readonly stageContactPower = new Float64Array(4);
  /** Motor power delivered at each RK4 stage (W). */
  private readonly stageMotorPower = new Float64Array(4);
  /** Air power on the propellers at each RK4 stage (W). */
  private readonly stageAirPower = new Float64Array(4);
  private readonly gearAngleBuf: Float64Array;
  /** Per propeller, the index of its hub's yaw in gearAngles order (−1 if it cannot turn). */
  private readonly propHub: number[];
  private readonly motorParams: { terms: { coord: number; k: number }[]; tau: number; b: number }[];
  /** Slider end stops: travel and gradient per stop, and the effective mass when touching (per end). */
  private readonly stopBuf: Float64Array;
  private readonly stopMass: (number | null)[];
  private readonly stopFresh: boolean[];
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
    this.motorParams = d.motors.map((m) => ({ terms: m.terms, tau: paramIndex.get(m.tau)!, b: m.b ? paramIndex.get(m.b)! : -1 }));
    this.candidates = [...sys.pointIndex.values()].map((point) => ({ point }));
    this.disks = d.gearDisks.map((g) => ({ point: sys.pointIndex.get(g.center)!, phi: sys.gearCoord.get(g.id) ?? -1, radius: g.radius }));
    this.diskGear = d.gearDisks.map((g) => g.id);
    this.contacts = this.candidates.map(() => null);
    const seen = new Set<number>();
    this.bars = d.joints.flatMap((j) =>
      j.bars.flatMap((b) => {
        if (seen.has(b.link)) return [];
        seen.add(b.link);
        return [{ ia: sys.pointIndex.get(b.a)!, ib: sys.pointIndex.get(b.b)!, link: b.link }];
      }),
    );
    this.stopBuf = new Float64Array(d.sliderStops.length * (n + 1));
    this.stopMass = d.sliderStops.flatMap(() => [null, null]);
    this.stopFresh = d.sliderStops.flatMap(() => [false, false]);
    this.stopGrad = new Float64Array(n);
    this.Jn = new Float64Array(n);
    this.Jt = new Float64Array(n);
    this.K2 = new Float64Array((n + nc) * (n + nc));
    this.r2 = new Float64Array(n + nc);
    this.gearAngleBuf = new Float64Array(sys.gearIds.length);
    this.pointIds = [];
    for (const [id, i] of sys.pointIndex) this.pointIds[i] = id;
    this.propHub = d.propellers.map((pr) => sys.gearIds.indexOf(pr.id));
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
    this.applyPropellers(q, qd, f, stage);
    this.applyContacts(q, qd, f, stage);
    this.applyStops(q, qd, f, stage);
    if (stage >= 0) {
      // the motors' torque is in f already (symbolically); here only the power they put in,
      // τ(ρ̇) ρ̇ with ρ̇ the gear's turn rate in its housing
      let P = 0;
      for (const m of this.motorParams) {
        let w = 0;
        for (const t of m.terms) w += t.k * qd[t.coord];
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
  private contactGeometry(c: ContactCandidate, g: GroundSettings): { gap: number; along: number; nx: number; ny: number; tx: number; ty: number } {
    const { n, pos, jac, Jn, Jt } = this;
    const i = c.point;
    const px = pos[2 * i];
    const py = pos[2 * i + 1];
    // the terrain segment under the point
    const seg = this.terrain(g).at(px);
    const { nx, ny, tx, ty } = seg;
    for (let m = 0; m < n; m++) {
      const jx = jac[i * 2 * n + m];
      const jy = jac[i * 2 * n + n + m];
      Jn[m] = nx * jx + ny * jy;
      Jt[m] = tx * jx + ty * jy;
    }
    return { gap: nx * (px - seg.x0) + ny * (py - seg.y0), along: seg.s + tx * (px - seg.x0) + ty * (py - seg.y0), nx, ny, tx, ty };
  }

  /**
   * The places where rim `k` touches the terrain, or is within `margin` of it (pos filled): the
   * inside of each segment its centre is above (at a valley it can touch both sides), and each
   * waypoint at a crest whose corner region holds the centre (where it rolls over the peak).
   */
  private diskFeatures(k: number, g: GroundSettings, margin: number): DiskFeature[] {
    const { point, radius: R } = this.disks[k];
    const cx = this.pos[2 * point];
    const cy = this.pos[2 * point + 1];
    const reach = R + margin;
    const segs = this.terrain(g).segments();
    const out: DiskFeature[] = [];
    segs.forEach((sg, i) => {
      if (cx < Math.min(sg.x0, sg.x1) - reach || cx > Math.max(sg.x0, sg.x1) + reach) return;
      const [rx, ry] = [cx - sg.x0, cy - sg.y0];
      const u = rx * sg.tx + ry * sg.ty;
      const h = rx * sg.nx + ry * sg.ny; // height of the centre above the segment's line
      // inside the segment, and above it (a centre behind the line is on the far side of a cliff)
      if (u > 0 && u < sg.L && h > 0 && h < reach) out.push({ key: `${k}:s${i}`, gap: h - R, nx: sg.nx, ny: sg.ny, along: sg.s + u });
      const next = segs[i + 1];
      if (!next) return;
      // the waypoint joining this segment to the next: touched at its corner when the centre is past
      // this segment's end and short of the next one's start, which happens only at a crest
      const [vx, vy] = [cx - sg.x1, cy - sg.y1];
      if (vx * sg.tx + vy * sg.ty < 0 || vx * next.tx + vy * next.ty > 0) return;
      const dist = Math.hypot(vx, vy);
      if (dist > 1e-9 && dist < reach) out.push({ key: `${k}:v${i}`, gap: dist - R, nx: vx / dist, ny: vy / dist, along: next.s });
    });
    return out;
  }

  /**
   * Fill Jn, Jt for rim k touching at a point with normal n (towards the centre): that point,
   * r = −R n from the centre, moves with the disk, ṙ_p = ṙ_c + φ̇ k × r, and k × r = R t
   * (t = (n_y, −n_x)), so the spin adds R φ̇ along the ground only.
   */
  private diskJacobians(k: number, nx: number, ny: number) {
    const { n, jac, Jn, Jt } = this;
    const { point: i, phi, radius } = this.disks[k];
    const [tx, ty] = [ny, -nx];
    for (let m = 0; m < n; m++) {
      const jx = jac[i * 2 * n + m];
      const jy = jac[i * 2 * n + n + m];
      Jn[m] = nx * jx + ny * jy;
      Jt[m] = tx * jx + ty * jy;
    }
    if (phi >= 0) Jt[phi] += radius;
  }

  private terrainCache: { of: GroundSettings; terrain: Terrain } | null = null;
  /** The ground's terrain, rebuilt when the ground settings change. */
  private terrain(g: GroundSettings): Terrain {
    if (this.terrainCache?.of !== g) this.terrainCache = { of: g, terrain: new Terrain(g.points) };
    return this.terrainCache.terrain;
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
    // one touching point: Jn, Jt filled; its state is created when it first touches
    const push = (gap: number, along: number, rolling: boolean, state: () => ContactState | undefined | null, start: (st: ContactState) => void, where?: () => Omit<PointLoad, 'type'> & { n: [number, number]; t: [number, number] }) => {
      let st = state();
      if (!st) {
        // a new contact: the effective masses it pushes against, along n and along t
        const wn = this.mobility(q, Jn);
        if (wn < 1e-12) return; // pinned: it cannot move into the ground anyway
        const wt = this.mobility(q, Jt);
        st = { mn: 1 / wn, mt: wt > 1e-12 ? 1 / wt : 1 / wn, anchor: null, fresh: true };
        start(st);
      }
      let vn = 0;
      let vt = 0;
      for (let m = 0; m < n; m++) {
        vn += Jn[m] * qd[m];
        vt += Jt[m] * qd[m];
      }
      const { Fn, Ft, loss } = this.contactForce(st, g, -gap, vn, vt, along, rolling);
      for (let m = 0; m < n; m++) f[m] += Jn[m] * Fn + Jt[m] * Ft;
      power += loss;
      if (this.capture && where) {
        const w = where();
        this.capture.points.push({ type: 'contact', at: w.at, on: w.on, F: [Fn * w.n[0] + Ft * w.t[0], Fn * w.n[1] + Ft * w.t[1]] });
      }
    };
    candidates.forEach((c, ci) => {
      const { gap, along, nx, ny, tx, ty } = this.contactGeometry(c, g);
      if (gap < 0)
        push(gap, along, false, () => contacts[ci], (st) => (contacts[ci] = st), () => ({
          at: [pos[2 * c.point], pos[2 * c.point + 1]],
          on: { kind: 'node', id: this.pointIds[c.point] },
          F: [0, 0],
          n: [nx, ny],
          t: [tx, ty],
        }));
    });
    const { diskContacts } = this;
    this.disks.forEach((_, k) => {
      for (const ft of this.diskFeatures(k, g, 0)) {
        if (ft.gap >= 0) continue;
        this.diskJacobians(k, ft.nx, ft.ny);
        // a rolling rim has no fixed contact point: its friction is viscous, on the slip speed
        const dk = this.disks[k];
        push(ft.gap, ft.along, true, () => diskContacts.get(ft.key), (st) => diskContacts.set(ft.key, st), () => ({
          // the rim's touching point, R from the centre against the normal, on the gear
          at: [pos[2 * dk.point] - dk.radius * ft.nx, pos[2 * dk.point + 1] - dk.radius * ft.ny],
          on: { kind: 'gear', id: this.diskGear[k] },
          F: [0, 0],
          n: [ft.nx, ft.ny],
          t: [ft.ny, -ft.nx],
        }));
      }
    });
    power += this.applyPeaks(q, qd, f, g);
    if (stage >= 0) this.stageContactPower[stage] = power;
  }

  /**
   * The terrain's peaks (waypoints where it turns downhill) against the middles of bars: a bar
   * can rest across a peak with both ends above the ground, where the joint checks cannot see
   * it. If a peak is above the bar's line within its span, the bar is pushed up there, along its
   * normal, by the same contact force (F = max(0, kδ + cδ̇), k = mω² with m the effective mass of
   * that point along the normal), with viscous friction along the bar up to μF. Returns the
   * power dissipated. With pos and jac filled.
   */
  private applyPeaks(q: Float64Array, qd: Float64Array, f: Float64Array, g: GroundSettings): number {
    const { n, pos, jac, Jn, Jt, bars, peakContacts } = this;
    const peaks = this.terrainPeaks(g);
    if (!peaks.length || !bars.length) return 0;
    const w0 = CONTACT_OMEGA;
    const zeta = dampingRatio(g.restitution);
    let power = 0;
    const live = new Set<string>();
    for (const [pi, [vx, vy]] of peaks.entries()) {
      bars.forEach(({ ia, ib, link }, bi) => {
        const [ax, ay, bx, by] = [pos[2 * ia], pos[2 * ia + 1], pos[2 * ib], pos[2 * ib + 1]];
        const [dx, dy] = [bx - ax, by - ay];
        const L = Math.hypot(dx, dy);
        if (L < 1e-9) return;
        const s = ((vx - ax) * dx + (vy - ay) * dy) / (L * L);
        if (s <= 0.01 || s >= 0.99) return; // the ends are the joints' own contacts
        // the bar's normal on the upper side (a bar steeper than 60° is left to its ends)
        let [nx, ny] = [-dy / L, dx / L];
        if (ny < 0) [nx, ny] = [-nx, -ny];
        if (ny < 0.5) return;
        const delta = (vx - ax) * nx + (vy - ay) * ny; // how far the peak pokes above the bar
        // a real contact sinks millimetres; a peak far "above" the line is beside the bar, not in it
        if (delta <= 0 || delta > PEAK_REACH) return;
        const key = `${pi}:${bi}`;
        live.add(key);
        // rows of the bar's material point at s: (1 − s) J_a + s J_b, along n and along the bar
        const [tx, ty] = [dx / L, dy / L];
        for (let m = 0; m < n; m++) {
          const jx = (1 - s) * jac[ia * 2 * n + m] + s * jac[ib * 2 * n + m];
          const jy = (1 - s) * jac[ia * 2 * n + n + m] + s * jac[ib * 2 * n + n + m];
          Jn[m] = nx * jx + ny * jy;
          Jt[m] = tx * jx + ty * jy;
        }
        let mass = peakContacts.get(key);
        if (mass === undefined) {
          const wn = this.mobility(q, Jn);
          if (wn < 1e-12) return;
          mass = 1 / wn;
          peakContacts.set(key, mass);
        }
        let vn = 0;
        let vt = 0;
        for (let m = 0; m < n; m++) {
          vn += Jn[m] * qd[m];
          vt += Jt[m] * qd[m];
        }
        const kk = mass * w0 * w0;
        let Fn = kk * delta - 2 * zeta * mass * w0 * vn; // δ̇ = −v_n (the peak stands still)
        if (Fn < 0) Fn = 0;
        let Ft = -2 * mass * w0 * vt;
        const lim = g.friction * Fn;
        Ft = Math.max(-lim, Math.min(lim, Ft));
        for (let m = 0; m < n; m++) f[m] += Jn[m] * Fn + Jt[m] * Ft;
        power += vn * (kk * delta - Fn) - Ft * vt;
        this.capture?.points.push({ type: 'contact', at: [ax + s * dx, ay + s * dy], on: { kind: 'link', id: link }, F: [Fn * nx + Ft * tx, Fn * ny + Ft * ty] });
      });
    }
    for (const key of [...peakContacts.keys()]) if (!live.has(key)) peakContacts.delete(key);
    return power;
  }

  /** For each peak and bar: how far the peak pokes above the bar's middle (−∞ if it cannot touch). With pos filled. */
  private peakDepths(g: GroundSettings, each: (key: string, delta: number) => void) {
    const { pos, bars } = this;
    for (const [pi, [vx, vy]] of this.terrainPeaks(g).entries())
      bars.forEach(({ ia, ib }, bi) => {
        const [ax, ay, bx, by] = [pos[2 * ia], pos[2 * ia + 1], pos[2 * ib], pos[2 * ib + 1]];
        const [dx, dy] = [bx - ax, by - ay];
        const L = Math.hypot(dx, dy);
        if (L < 1e-9) return;
        const s = ((vx - ax) * dx + (vy - ay) * dy) / (L * L);
        if (s <= 0.01 || s >= 0.99) return;
        let [nx, ny] = [-dy / L, dx / L];
        if (ny < 0) [nx, ny] = [-nx, -ny];
        if (ny < 0.5) return;
        const delta = (vx - ax) * nx + (vy - ay) * ny;
        if (delta <= PEAK_REACH) each(`${pi}:${bi}`, delta);
      });
  }

  private peakCache: { of: GroundSettings; peaks: [number, number][] } | null = null;
  /** The terrain's convex corners (where it turns downhill), counting the level ends. */
  private terrainPeaks(g: GroundSettings): [number, number][] {
    if (this.peakCache?.of === g) return this.peakCache.peaks;
    const p = g.points;
    const slope = (i: number) => (i < 0 || i >= p.length - 1 ? 0 : (p[i + 1][1] - p[i][1]) / (p[i + 1][0] - p[i][0] || 1e-9));
    const peaks = p.filter((_, i) => slope(i - 1) - slope(i) > 1e-3); // a real corner, not rounding in a straight run
    this.peakCache = { of: g, peaks };
    return peaks;
  }

  /**
   * After a step: drop contacts that have lifted off, start static friction for new ones, and
   * let a contact pushed past μ F_n slide, moving its anchor to where the friction limit holds.
   */
  private updateContacts(h: number) {
    const g = this.ground;
    const P = this.stageContactPower;
    this.dissipated += ((P[0] + 2 * P[1] + 2 * P[2] + P[3]) / 6) * h;
    const motion = this.contactMotion;
    motion.rolling = motion.slip = motion.scrape = 0;
    if (!g) {
      this.contacts.fill(null);
      this.diskContacts.clear();
      return;
    }
    const { n, sys, q, qd, pos, jac, Jn, Jt, candidates, contacts, diskContacts } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    const rate = (J: Float64Array) => {
      let v = 0;
      for (let m = 0; m < n; m++) v += J[m] * qd[m];
      return v;
    };
    if (diskContacts.size) {
      const live = new Set<string>();
      this.disks.forEach((dk, k) => {
        for (const ft of this.diskFeatures(k, g, 0)) {
          if (ft.gap >= 0) continue;
          live.add(ft.key);
          const st = diskContacts.get(ft.key);
          if (!st) continue;
          this.diskJacobians(k, ft.nx, ft.ny);
          const [vn, vt] = [rate(Jn), rate(Jt)];
          motion.slip = Math.max(motion.slip, Math.abs(vt));
          motion.rolling = Math.max(motion.rolling, Math.abs(vt - (dk.phi >= 0 ? dk.radius * qd[dk.phi] : 0))); // the centre's speed along the ground
          if (st.fresh) {
            st.fresh = false;
            this.event('impact', -vn, st.mn);
          }
        }
      });
      for (const key of [...diskContacts.keys()]) if (!live.has(key)) diskContacts.delete(key);
    }
    candidates.forEach((c, ci) => {
      const st = contacts[ci];
      if (!st) return;
      const { gap, along } = this.contactGeometry(c, g);
      if (gap >= 0) {
        contacts[ci] = null;
        return;
      }
      if (st.fresh) {
        st.fresh = false;
        this.event('impact', -rate(Jn), st.mn);
      }
      if (st.anchor === null) {
        st.anchor = along;
        return;
      }
      motion.scrape = Math.max(motion.scrape, Math.abs(rate(Jt)));
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
    if (this.peakContacts.size) {
      sys.positions(q, pos);
      this.peakDepths(g, (key, delta) => {
        const m = this.peakContacts.get(key);
        if (m !== undefined && delta > 0) V += 0.5 * m * CONTACT_OMEGA * CONTACT_OMEGA * delta * delta;
      });
    }
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    candidates.forEach((c, ci) => {
      const st = contacts[ci];
      if (!st) return;
      const { gap } = this.contactGeometry(c, g);
      if (gap < 0) V += 0.5 * st.mn * CONTACT_OMEGA * CONTACT_OMEGA * gap * gap;
    });
    if (this.diskContacts.size)
      this.disks.forEach((_, k) => {
        for (const ft of this.diskFeatures(k, g, 0)) {
          const st = this.diskContacts.get(ft.key);
          if (st && ft.gap < 0) V += 0.5 * st.mn * CONTACT_OMEGA * CONTACT_OMEGA * ft.gap * ft.gap;
        }
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
          this.stopFresh[key] = true;
        }
        const w0 = CONTACT_OMEGA;
        const kk = mass * w0 * w0;
        const ddelta = -push * sd; // δ̇
        let F = kk * delta + 2 * dampingRatio(st.restitution) * mass * w0 * ddelta;
        if (F < 0) F = 0;
        for (let m = 0; m < n; m++) f[m] += push * F * g[m];
        this.capture?.stops.push({ joint: st.joint, along: push * F });
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
        const key = 2 * k + end;
        if (delta <= 0) stopMass[key] = null;
        else if (this.stopFresh[key] && stopMass[key] !== null) {
          this.stopFresh[key] = false;
          this.event('stop', Math.abs(sd), stopMass[key]!);
        }
        if (delta > -(2 * h * Math.abs(sd) + 1e-4)) near = true;
      });
    });
    return near;
  }

  /**
   * Set the torque at rest of the motor on gear `gearId` while it runs (a lever moving): only
   * the parameter τ changes, so the state carries on. False if the gear has no running motor.
   */
  setMotorTorque(gearId: number, tau: number): boolean {
    const k = this.sys.derivation.motors.findIndex((m) => m.id === gearId);
    if (k < 0) return false;
    this.sys.params[this.motorParams[k].tau] = tau;
    return true;
  }

  /**
   * The air on the propellers, as generalized forces. For a spin of n rev/s (n = φ̇ / 2π),
   * diameter D, air density ρ, hub yaw θ, axis n̂ = (−sin θ, cos θ) and disk line ê = (cos θ, sin θ):
   *   thrust     T = ±ρ D⁴ C_T n|n| − ρ D³ (C_T/J₀) |n| (v · n̂)    (C_T falling linearly with J = V/(nD))
   *   rotor drag H = −ρ D³ C_H |n| (v · ê)                         (the H-force, in the disk's plane)
   *   air torque Q = −ρ D⁵ C_Q n|n| on φ
   * T n̂ + H ê acts at the centre, Q_q = Jᵀ(T n̂ + H ê); the thrust passes through the centre, so
   * it does not turn the hub.
   */
  private applyPropellers(q: Float64Array, qd: Float64Array, f: Float64Array, stage: number) {
    const props = this.sys.derivation.propellers;
    if (stage >= 0) this.stageAirPower[stage] = 0;
    if (!props.length) return;
    const { n, sys, pos, jac, gearAngleBuf } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    sys.gearAngles(q, gearAngleBuf);
    let power = 0;
    props.forEach((pr, k) => {
      const i = sys.pointIndex.get(pr.center);
      if (i === undefined) return;
      const th = pr.angle0 + (this.propHub[k] >= 0 ? gearAngleBuf[this.propHub[k]] : 0);
      const [ex, ey] = [Math.cos(th), Math.sin(th)];
      const [nx, ny] = [-ey, ex];
      let vx = 0;
      let vy = 0;
      for (let m = 0; m < n; m++) {
        vx += jac[i * 2 * n + m] * qd[m];
        vy += jac[i * 2 * n + n + m] * qd[m];
      }
      const w = pr.spin >= 0 ? qd[pr.spin] : 0;
      const rev = w / (2 * Math.PI);
      const { rho, ct, cq, j0, ch, hand } = pr.aero;
      const D = 2 * pr.radius;
      const D3 = D * D * D;
      const T = hand * rho * D3 * D * ct * rev * Math.abs(rev) - rho * D3 * (ct / j0) * Math.abs(rev) * (vx * nx + vy * ny);
      const H = -rho * D3 * ch * Math.abs(rev) * (vx * ex + vy * ey);
      const Q = -rho * D3 * D * D * cq * rev * Math.abs(rev);
      const Fx = T * nx + H * ex;
      const Fy = T * ny + H * ey;
      for (let m = 0; m < n; m++) f[m] += jac[i * 2 * n + m] * Fx + jac[i * 2 * n + n + m] * Fy;
      this.capture?.points.push({ type: 'air', at: [pos[2 * i], pos[2 * i + 1]], on: { kind: 'gear', id: pr.id }, F: [Fx, Fy] });
      if (pr.spin >= 0) f[pr.spin] += Q;
      power += Fx * vx + Fy * vy + Q * w;
      if (stage <= 0) this.thrust.set(pr.id, T);
    });
    if (stage >= 0) this.stageAirPower[stage] = power;
  }

  /** Current spin φ of every propeller's blades, keyed by its gear's id. */
  spinAngles(): Map<number, number> {
    return new Map(this.sys.derivation.propellers.map((pr) => [pr.id, pr.spin >= 0 ? this.q[pr.spin] : 0]));
  }

  private event(kind: SimEvent['kind'], speed: number, mass: number) {
    if (speed > 0 && this.events.length < 64) this.events.push({ kind, speed, mass });
  }

  /** Each motor now, for sound: its gear's turn rate in its housing (rad/s), the torque it gives, and its torque at rest (N·m). */
  motorStates(): { id: number; speed: number; torque: number; stall: number }[] {
    const { sys, qd } = this;
    return this.motorParams.map((m, k) => {
      let w = 0;
      for (const t of m.terms) w += t.k * qd[t.coord];
      const tau = sys.params[m.tau];
      return { id: sys.derivation.motors[k].id, speed: w, torque: tau - (m.b >= 0 ? sys.params[m.b] * w : 0), stall: Math.abs(tau) };
    });
  }

  /** Pitch-line speed of every gear and rack mesh now (m/s), in derivation.meshes order. */
  meshSpeeds(): number[] {
    const { n, sys, q, qd, meshBuf } = this;
    sys.meshGrads(q, meshBuf);
    return sys.derivation.meshes.map((_, k) => {
      let v = 0;
      for (let m = 0; m < n; m++) v += meshBuf[k * n + m] * qd[m];
      return v;
    });
  }

  /** The fastest any joint moves now (m/s). */
  maxJointSpeed(): number {
    const { n, sys, q, qd, jac } = this;
    sys.jacobians(q, jac);
    let best = 0;
    for (let i = 0; i < sys.pointIndex.size; i++) {
      let vx = 0;
      let vy = 0;
      for (let m = 0; m < n; m++) {
        vx += jac[i * 2 * n + m] * qd[m];
        vy += jac[i * 2 * n + n + m] * qd[m];
      }
      best = Math.max(best, Math.hypot(vx, vy));
    }
    return best;
  }

  /**
   * The accelerations q̈ at the current state and every load applied there (point forces and
   * slider-stop pushes), for working out the constraint forces (see forces.ts).
   */
  snapshot(): { qdd: Float64Array; loads: Loads } {
    const qdd = new Float64Array(this.n);
    this.capture = { points: [], stops: [] };
    try {
      if (!this.accelerations(this.q, this.qd, qdd, -1)) qdd.fill(NaN);
    } finally {
      const loads = this.capture;
      this.capture = null;
      // eslint-disable-next-line no-unsafe-finally
      return { qdd, loads: loads! };
    }
  }

  /** Number of points touching the ground now. */
  contactCount(): number {
    return this.contacts.reduce((k, c) => k + (c ? 1 : 0), this.diskContacts.size);
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
    this.capture?.points.push({ type: 'pull', at: [rx, ry], F: [Fx, Fy], on: g.gear ? { kind: 'gear', id: g.gear.id } : g.a === g.b ? { kind: 'node', id: g.a } : { kind: 'seg', a: g.a, b: g.b } });
  }

  /** One classical 4th-order Runge–Kutta step of size h on y = (q, q̇). */
  /**
   * Advance by h. Near the ground (a point touching it, or about to within the step) the step
   * is split into four, as contact forces switch on and off within a few milliseconds; into
   * eight for very dead impacts (ζ > 1), whose strong damping needs it for the energy budget.
   */
  step(h: number): boolean {
    if (this.controllers.length) this.runControllers(h);
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

  /**
   * Set the PID controllers. Each one finds which way its motor moves its quantity here and
   * now (s = sign of rowᵀ A g, A the constrained inverse inertia, row = ∂y/∂q and g = ∂ρ/∂q the
   * motor's generalized-force direction); `carry` keeps the integral of controllers that were
   * running already (a parameter changed mid-run).
   */
  setControllers(specs: ControllerSpec[], carry: Controller[] = []) {
    this.controllers = specs.map((sp) => {
      const old = carry.find((c) => c.motor === sp.motor && c.sensor.quantity === sp.sensor.quantity);
      const c: Controller = { ...sp, integral: old?.integral ?? 0, sign: 0, detected: 0, y: 0, e: 0, u: 0, prevY: null, dFilt: 0, lastAngle: old?.lastAngle ?? null };
      this.applyDirection(c);
      return c;
    });
    this.runControllers(0);
  }

  /** Check which way the motor moves the quantity now, and set the sign the controller pushes with. */
  private applyDirection(c: Controller) {
    c.detected = 0;
    const k = this.sys.derivation.motors.findIndex((m) => m.id === c.motor);
    const row = this.measure(c, this.q, this.qd).row;
    if (k >= 0 && row) {
      const g = new Float64Array(this.n);
      for (const t of this.motorParams[k].terms) g[t.coord] += t.k;
      const r = this.response(this.q, g, row);
      c.detected = Math.abs(r) > 1e-9 ? Math.sign(r) : 0;
    }
    c.sign = c.direction === 'forward' ? 1 : c.direction === 'reverse' ? -1 : c.detected;
  }

  /** Change a controller's target, gains or direction while it runs (keeping its integral). */
  tuneController(motor: number, change: Partial<Pick<ControllerSpec, 'kp' | 'ki' | 'kd' | 'max' | 'target' | 'direction'>>) {
    const c = this.controllers.find((x) => x.motor === motor);
    if (!c) return;
    Object.assign(c, change);
    if (change.direction !== undefined) {
      const was = c.sign;
      this.applyDirection(c); // 'auto' checks again, here and now
      if (c.sign !== was) c.integral = 0; // what it had built up pushed the other way
    }
  }

  /** rowᵀ A g: how the quantity with gradient `row` accelerates under the generalized force g. */
  private response(q: Float64Array, g: Float64Array, row: Float64Array): number {
    const { n, nc, sys, M, K2: K, r2: r, PhiQ } = this;
    const N = n + nc;
    sys.mass(q, M);
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
    for (let i = 0; i < n; i++) w += row[i] * r[i];
    return w;
  }

  /**
   * A sensor reading (SI units): the value, its rate where that is known exactly (positions and
   * angles), and ∂(the underlying position or angle)/∂q.
   */
  private measure(c: Controller, q: Float64Array, qd: Float64Array): { value: number; rate: number | null; row: Float64Array | null } {
    const { n, sys, pos, jac } = this;
    const s = c.sensor;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    const row = new Float64Array(n);
    const dot = (r: Float64Array) => r.reduce((acc, v, m) => acc + v * qd[m], 0);
    const pointRow = (pid: number | undefined, axis: 0 | 1, w = 1) => {
      const i = pid === undefined ? undefined : sys.pointIndex.get(pid);
      if (i === undefined) return 0;
      for (let m = 0; m < n; m++) row[m] += w * jac[i * 2 * n + axis * n + m];
      return w * pos[2 * i + axis];
    };
    const qn = s.quantity;
    const axis: 0 | 1 = qn === 'x' || qn === 'vx' ? 0 : 1;
    let value: number;
    if (qn === 'spin') {
      const pr = sys.derivation.propellers.find((x) => x.id === s.gear);
      if (!pr || pr.spin < 0) return { value: 0, rate: null, row: null };
      row[pr.spin] = 1;
      return { value: qd[pr.spin], rate: null, row };
    }
    if (qn === 'angle' || qn === 'omega') {
      if (s.a !== undefined && s.b !== undefined) {
        // a bar's direction θ = atan2(dy, dx), unwrapped: ∂θ = (dx ∂dy − dy ∂dx) / L²
        const [ia, ib] = [sys.pointIndex.get(s.a), sys.pointIndex.get(s.b)];
        if (ia === undefined || ib === undefined) return { value: 0, rate: null, row: null };
        const [dx, dy] = [pos[2 * ib] - pos[2 * ia], pos[2 * ib + 1] - pos[2 * ia + 1]];
        const L2 = dx * dx + dy * dy || 1e-12;
        for (let m = 0; m < n; m++) {
          const ddx = jac[ib * 2 * n + m] - jac[ia * 2 * n + m];
          const ddy = jac[ib * 2 * n + n + m] - jac[ia * 2 * n + n + m];
          row[m] = (dx * ddy - dy * ddx) / L2;
        }
        const raw = Math.atan2(dy, dx);
        const prev = c.lastAngle;
        value = prev === null ? raw : prev + Math.atan2(Math.sin(raw - prev), Math.cos(raw - prev));
        c.lastAngle = value;
      } else {
        // a gear's turn (a propeller's hub yaw), from its coordinate
        const gi = sys.gearIds.indexOf(s.gear ?? -1);
        const buf = new Float64Array(sys.gearIds.length);
        sys.gearAngles(q, buf);
        const k = sys.gearCoord.get(s.gear ?? -1) ?? -1;
        if (k >= 0) row[k] = 1;
        value = (s.angle0 ?? 0) + (gi >= 0 ? buf[gi] : 0);
      }
    } else if (s.a !== undefined && s.b !== undefined) value = pointRow(s.a, axis, 0.5) + pointRow(s.b, axis, 0.5); // a bar's centre
    else value = pointRow(s.point ?? s.center, axis);
    const rate = dot(row);
    return QUANTITY_IS_RATE[qn] ? { value: rate, rate: null, row } : { value, rate, row };
  }

  /**
   * One control step (h = 0 only measures): e = target − y, u = s (K_p e + K_i ∫e − K_d ẏ),
   * clamped to ±max, the integral held while the output is saturated the way the error pushes.
   * ẏ is exact for positions and angles; for rates it is the measurement's change, smoothed
   * over 20 ms.
   */
  private runControllers(h: number) {
    for (const c of this.controllers) {
      const { value, rate } = this.measure(c, this.q, this.qd);
      const k = QUANTITY_SCALE[c.sensor.quantity];
      const y = value * k;
      let dy: number;
      if (rate !== null) dy = rate * k;
      else {
        const raw = c.prevY === null || h <= 0 ? 0 : (y - c.prevY) / h;
        c.dFilt += h > 0 ? (raw - c.dFilt) * (h / (0.02 + h)) : 0;
        dy = c.dFilt;
      }
      c.prevY = y;
      const e = c.target - y;
      const max = Math.abs(c.max);
      const raw = (u: number) => c.sign * (c.kp * e + c.ki * u - c.kd * dy);
      const free = raw(c.integral + e * h);
      // integrate unless that would push a saturated output further
      if (h > 0 && !(Math.abs(free) > max && Math.sign(free) === Math.sign(c.sign * e))) c.integral += e * h;
      const u = Math.max(-max, Math.min(max, raw(c.integral)));
      [c.y, c.e, c.u] = [y, e, c.sign === 0 ? 0 : u];
      this.setMotorTorque(c.motor, c.u);
    }
  }

  /** Whether any point touches the ground or could reach it within the next step. */
  private nearGround(h: number): boolean {
    const g = this.ground!;
    if (this.contacts.some((c) => c) || this.peakContacts.size || this.diskContacts.size) return true;
    const { n, sys, q, qd, pos, jac, Jn, candidates } = this;
    sys.positions(q, pos);
    sys.jacobians(q, jac);
    let nearPeak = false;
    this.peakDepths(g, (_, delta) => {
      if (delta > -0.01) nearPeak = true; // a bar within a centimetre of a peak
    });
    if (nearPeak) return true;
    const closing = () => {
      let vn = 0;
      for (let m = 0; m < n; m++) vn += Jn[m] * qd[m];
      return 2 * h * Math.max(0, -vn) + 1e-4;
    };
    const near = candidates.some((c) => this.contactGeometry(c, g).gap < closing());
    // a rim: any feature within reach, then whether it closes the gap within the step
    return (
      near ||
      this.disks.some((_, k) =>
        this.diskFeatures(k, g, 0.25).some((ft) => {
          this.diskJacobians(k, ft.nx, ft.ny);
          return ft.gap < closing();
        }),
      )
    );
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
    const ap = this.stageAirPower;
    this.airWork += ((ap[0] + 2 * ap[1] + 2 * ap[2] + ap[3]) / 6) * h;
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

  /** Current turn of every motor housing since the drawing, keyed by its gear's id. */
  motorAngles(): Map<number, number> {
    const d = this.sys.derivation;
    const out = new Float64Array(d.motorAngles.length);
    this.sys.motorAngles(this.q, out);
    return new Map(d.motorAngles.map((m, i) => [m.id, out[i]]));
  }

  constraintError(): number {
    if (this.nc === 0) return 0;
    this.sys.phi(this.q, this.Phi);
    let e = 0;
    for (let k = 0; k < this.nc; k++) e = Math.max(e, Math.abs(this.Phi[k]));
    return e;
  }
}
