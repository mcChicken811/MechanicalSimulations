/** The editable description of a planar linkage (world units: metres, kilograms; y points up). */

/**
 * free  – the joint moves in both x and y (unless `pin` or `slide` says otherwise)
 * fixed – pinned to the ground (both x and y locked)
 * lockX – x is locked: the joint slides along a vertical ground rail (same as a slide at 90°)
 * lockY – y is locked: the joint slides along a horizontal ground rail (same as a slide at 0°)
 */
export type PointKind = 'free' | 'fixed' | 'lockX' | 'lockY';

/**
 * A rail the joint slides along, fixed to a body (a bar, rack or gear) or to the ground
 * (`on` null). It passes through the joint as drawn, at `angle` degrees in the drawing; on a
 * body it then turns and moves with that body.
 */
export interface DSlide {
  on: DBodyRef | null;
  angle: number;
  /**
   * How far the joint may travel along the rail, from where it was drawn, in metres along
   * the rail's direction (min ≤ 0 ≤ max). Past an end it hits a stop, as it would the ground.
   */
  range?: DRange;
}

/** End stops on a rail: null means no stop on that side. */
export interface DRange {
  min: number | null;
  max: number | null;
  /** Coefficient of restitution of the stops. */
  restitution: number;
}

export interface DPoint {
  id: number;
  x: number;
  y: number;
  kind: PointKind;
  /** Optional concentrated mass at the joint (kg). */
  mass: number;
  /**
   * If set, the joint is a pin fixed onto this gear: it moves rigidly with the gear,
   * at the offset from the gear centre it had in the design.
   */
  gear?: number;
  /** If set, the joint is pinned onto this bar or rack, moving rigidly with it where drawn. */
  pin?: DBodyRef;
  /** If set, the joint slides on a rail (on a body or on the ground). */
  slide?: DSlide;
}

/** The rail a joint slides on, if any: its own slide, or a lockX / lockY ground rail. */
export function railOf(p: DPoint): DSlide | null {
  if (p.slide) return p.slide;
  if (p.kind === 'lockX') return { on: null, angle: 90 };
  if (p.kind === 'lockY') return { on: null, angle: 0 };
  return null;
}

/** Remove every constraint from a joint: ground pin, rail, or pin on a body. */
export function clearConstraint(p: DPoint) {
  p.kind = 'free';
  delete p.gear;
  delete p.pin;
  delete p.slide;
}

/** A uniform rigid bar between two joints. Bars sharing a joint are pinned together. */
export interface DLink {
  id: number;
  a: number;
  b: number;
  mass: number;
  /**
   * If set, the bar is a rack: a straight toothed bar whose pitch line runs through its two
   * joints. The teeth are on its left (+1) or right (−1), looking from joint a to joint b.
   */
  rack?: 1 | -1;
}

/**
 * A massless linear spring between two joints. It imposes no kinematic constraint;
 * it only adds V = ½ k (ℓ − ℓ₀)² to the potential energy.
 */
export interface DSpring {
  id: number;
  a: number;
  b: number;
  /** Stiffness (N/m). */
  k: number;
  /** Rest length ℓ₀ (m). */
  rest: number;
  /**
   * A cable, ligament or tendon: it pulls when stretched beyond its rest length but goes
   * slack, with no force and no energy, when shorter: V = ½ k max(0, ℓ − ℓ₀)².
   */
  tensionOnly?: boolean;
}

/**
 * A gear: a uniform disk (pitch radius R, mass m, I = ½ m R²) that turns about the joint
 * `center`. It is a rigid body like a bar: joints with `gear` set to its id are pins on it,
 * and bars attach to those pins (or to the centre, which acts as the axle).
 * A locked gear does not turn.
 */
export interface DGear {
  id: number;
  center: number;
  radius: number;
  teeth: number;
  mass: number;
  locked: boolean;
  /** A motor driving the gear, its stator fixed to the ground. */
  motor?: DMotor;
  /**
   * A wheel: a disk with no teeth, so it meshes with nothing; it rolls on the ground (with
   * friction) like any rim. Joints can still be pinned on it, and it can carry a motor.
   */
  wheel?: boolean;
  /**
   * A propeller: a rotor whose spin axis lies in the plane. Its hub moves like a gear's (x, y,
   * and a yaw θ in the plane, the gear's turn), and its blades spin by φ about the axis, out of
   * the plane, so they are drawn projected. It touches nothing; the air pushes on it (see
   * DPropeller). A motor on it spins the blades, its housing being the hub.
   */
  propeller?: DPropeller;
}

/**
 * A propeller's shape and aerodynamics. Seen in the plane, the rotor disk is edge-on: the blade
 * line runs through the centre at `angle` (to the edge point picked when drawing), and the spin
 * axis is square to it, n = (−sin θ, cos θ) with θ the blade line's direction. With n the spin in
 * rev/s, D = 2R the diameter and ρ the air density (the standard propeller coefficients, made
 * linear in the advance ratio J = V/(nD) for the inflow):
 *   thrust along ±n    T = ρ D⁴ C_T n|n| − ρ D³ (C_T / J₀) |n| V_axial
 *   air torque on φ    Q = −ρ D⁵ C_Q n|n|
 *   rotor drag (H-force, in the disk's plane)   H = −ρ D³ C_H |n| V_inplane
 */
export interface DPropeller {
  /** Direction of the blade line as drawn (rad). */
  angle: number;
  /** Number of blades, 2 to 6. */
  blades: number;
  /** +1: a positive spin rate pushes along +n; −1: along −n (the other hand of pitch). */
  hand?: 1 | -1;
  /** Static thrust coefficient C_T (0.1 if absent). */
  ct?: number;
  /** Torque coefficient C_Q (0.007 if absent; C_P = 2π C_Q). */
  cq?: number;
  /** Advance ratio J₀ at which the thrust falls to zero (0.7 if absent). */
  j0?: number;
  /** Rotor drag coefficient C_H (0.05 if absent). */
  ch?: number;
  /** Air density ρ (kg/m³, 1.225 if absent). */
  rho?: number;
}

/** A propeller's aerodynamic coefficients, defaults filled in. */
export function propellerAero(p: DPropeller) {
  return { hand: p.hand ?? 1, ct: p.ct ?? 0.1, cq: p.cq ?? 0.007, j0: p.j0 ?? 0.7, ch: p.ch ?? 0.05, rho: p.rho ?? 1.225 };
}

/** Fraction of a propeller's mass in its hub (a disk of radius 0.12 R); the rest is in the blades. */
export const PROPELLER_HUB = { mass: 0.2, radius: 0.12, chord: 0.16 };

/**
 * A propeller's principal moments of inertia about its centre: I₁ about the spin axis, I₂ about
 * blade 1 (at φ = 0 lying along the blade line in the plane) and I₃ about the third axis (out of
 * the plane at φ = 0). Blades are uniform rods from the centre to the tip, the hub a thin disk;
 * a motor adds its housing (a disk of radius 0.15 R across the axis) to the hub. Blades have
 * a chord of 0.16 R. With three or more blades I₂ = I₃; with two, I₂ is small (the hub and the
 * blades' chord), so the rotor's yaw inertia I₂ sin²φ + I₃ cos²φ swings widely as it spins.
 */
export function propellerInertia(g: DGear): { I1: number; I2: number; I3: number } {
  const R = g.radius;
  const mb = g.mass * (1 - PROPELLER_HUB.mass);
  const mh = g.mass * PROPELLER_HUB.mass;
  const rh = PROPELLER_HUB.radius * R;
  const I1 = (mb * R * R) / 3 + 0.5 * mh * rh * rh;
  const hubT = 0.25 * mh * rh * rh;
  const motor = g.motor ? 0.25 * (g.motor.mass ?? 0.5) * (0.15 * R) ** 2 : 0;
  const N = g.propeller?.blades ?? 2;
  // each blade a plate of chord 0.16 R about its own span: m c²/12 (along blade 1 for two blades, shared out for more)
  const chord = (mb * (PROPELLER_HUB.chord * R) ** 2) / 12;
  const [b2, b3] = N === 2 ? [chord, (mb * R * R) / 3] : [(mb * R * R) / 6 + chord / 2, (mb * R * R) / 6 + chord / 2];
  return { I1, I2: b2 + hubT + motor, I3: b3 + hubT + motor };
}

/** A new motor for a propeller: fast, as propellers need. */
export const propellerMotor = (mounted: boolean): DMotor => ({ torque: 0.5, noLoadRpm: 3000, mass: 0.1, mounted });

/**
 * A motor's torque on its gear: τ (N·m, positive turns the gear anticlockwise), falling
 * linearly with the gear's speed to zero at the no-load speed, τ − (|τ|/Ω) ω, like a DC motor
 * (so a clockwise motor, τ < 0, runs free at ω = −Ω).
 * A no-load speed of 0 means a constant torque.
 */
export interface DMotor {
  torque: number;
  /** No-load speed Ω (rpm); 0 for a constant torque. */
  noLoadRpm: number;
  /** Mass of the motor's housing (kg); 0.5 if absent. */
  mass?: number;
  /**
   * Whether the housing is mounted on the ground (true if absent), so it cannot turn and the
   * reaction goes into the ground. Only for a housing not joined to another body: joined, it
   * pushes back on that body instead. Unmounted and unjoined, it is free to spin back.
   */
  mounted?: boolean;
  /**
   * A control lever: while it is there, it sets the torque instead of `torque`, as
   * τ = position × scale (see motorTorque), and the speed falloff is b = scale / Ω.
   */
  lever?: DLever;
  /**
   * A PID controller driving the motor from a sensor (instead of a lever: a motor has one or
   * the other). Its output is the motor's torque, within ±|torque|.
   */
  pid?: DPid;
}

/** What a sensor can measure: position, angle, their rates, and a propeller's spin rate. */
export type Quantity = 'x' | 'y' | 'angle' | 'vx' | 'vy' | 'omega' | 'spin';

/** A sensor on a body: a joint, or a bar, rack, gear, wheel or propeller (`link` / `gear`). */
export interface DSensor {
  body: { kind: 'point' | 'link' | 'gear'; id: number };
  quantity: Quantity;
}

/**
 * A PID controller on a motor. It measures a quantity with its sensor and sets the motor's
 * torque to bring it to the target, offset + (its lever's position × scale, with a lever):
 *   u = s (K_p e + K_i ∫e dt − K_d dy/dt),   e = target − y,   |u| ≤ |motor torque|
 * with the derivative taken on the measurement (moving the target gives no kick), the integral
 * held while the output is saturated (anti-windup), and s = ±1 the way the motor moves the
 * quantity, found when the simulation starts. Quantities are in display units (m, m/s, °, °/s,
 * rpm), and so are the gains (N·m per unit).
 */
export interface DPid {
  kp: number;
  ki: number;
  kd: number;
  /** The target with the lever centred (or without a lever). */
  offset: number;
  sensor: DSensor | null;
  /** A lever setting the target: position × scale, in the quantity's units, is added to the offset. */
  lever?: DLever;
  /** Colour of the controller, its sensor and the wire between them. */
  color: string;
  /**
   * Which way it pushes: 'auto' (if absent) finds it when the simulation starts and does nothing
   * if the motor cannot move the quantity; 'forward' takes a positive torque to raise the
   * quantity and 'reverse' to lower it, overriding the check.
   */
  direction?: 'auto' | 'forward' | 'reverse';
}

/** Display unit of a quantity. */
export const QUANTITY_UNIT: Record<Quantity, string> = { x: 'm', y: 'm', angle: '°', vx: 'm/s', vy: 'm/s', omega: '°/s', spin: 'rpm' };
/** Factor from SI (m, rad, rad/s) to the display unit. */
export const QUANTITY_SCALE: Record<Quantity, number> = { x: 1, y: 1, angle: 180 / Math.PI, vx: 1, vy: 1, omega: 180 / Math.PI, spin: 60 / (2 * Math.PI) };
/** Whether a quantity is a rate (its own derivative is not measured directly). */
export const QUANTITY_IS_RATE: Record<Quantity, boolean> = { x: false, y: false, angle: false, vx: true, vy: true, omega: true, spin: true };

/** Starting gains for a quantity, per N·m of the motor's torque. */
export function defaultGains(q: Quantity, maxTorque: number): { kp: number; ki: number; kd: number } {
  const base = { x: [5, 1, 1], y: [5, 1, 1], angle: [0.05, 0.02, 0.004], vx: [1, 2, 0], vy: [1, 2, 0], omega: [0.004, 0.01, 0], spin: [0.002, 0.005, 0] }[q];
  const k = Math.max(Math.abs(maxTorque), 1e-3);
  return { kp: +(base[0] * k).toPrecision(3), ki: +(base[1] * k).toPrecision(3), kd: +(base[2] * k).toPrecision(3) };
}

/** A sensible full throw for a lever setting a target of this quantity. */
export const LEVER_TARGET_SCALE: Record<Quantity, number> = { x: 0.5, y: 0.5, angle: 90, vx: 1, vy: 1, omega: 180, spin: 3000 };

/** The quantities a sensor can measure on a body. */
export function measurable(d: Design, body: DSensor['body']): Quantity[] {
  if (body.kind === 'point') return ['x', 'y', 'vx', 'vy'];
  const g = body.kind === 'gear' ? d.gears.find((x) => x.id === body.id) : undefined;
  return g?.propeller ? ['x', 'y', 'angle', 'vx', 'vy', 'omega', 'spin'] : ['x', 'y', 'angle', 'vx', 'vy', 'omega'];
}

/** A quantity's name on a body, for menus and labels. */
export function quantityName(q: Quantity, body: DSensor['body'], d: Design): string {
  const g = body.kind === 'gear' ? d.gears.find((x) => x.id === body.id) : undefined;
  const turn = body.kind === 'link' ? 'angle θ' : g?.propeller ? 'yaw θ' : 'turn φ';
  const names: Record<Quantity, string> = { x: 'x position', y: 'y position', angle: turn, vx: 'x velocity ẋ', vy: 'y velocity ẏ', omega: `${turn.split(' ')[0]} rate ${turn.split(' ')[1]}̇`, spin: 'spin rate (rpm)' };
  return names[q];
}

/** The lever on a motor: its own, or its PID controller's (which sets the target). */
export function leverOf(m: DMotor | undefined): { lever: DLever; owner: 'motor' | 'pid' } | null {
  if (m?.lever) return { lever: m.lever, owner: 'motor' };
  if (m?.pid?.lever) return { lever: m.pid.lever, owner: 'pid' };
  return null;
}

/** A PID controller's target now. */
export function pidTarget(pid: DPid): number {
  return pid.offset + (pid.lever ? pid.lever.position * pid.lever.scale : 0);
}

/**
 * A lever on a motor, worked from the control dock at the bottom right like a throttle: its
 * position runs from −1 (full reverse) through 0 (centre) to +1 (full forward), and full
 * forward gives `scale` N·m. As in a DC motor under voltage control, b = scale / Ω stays put
 * while the lever moves, so the lever sets both the push and the top speed, ±position × Ω, and
 * at the centre the motor brakes against being turned.
 */
export interface DLever {
  /** CSS colour, shared by the lever on the motor and its control. */
  color: string;
  /** Torque at full forward (N·m). */
  scale: number;
  /** −1 … 1. */
  position: number;
}

/** Colours for new levers, in order. */
export const LEVER_COLORS = ['#e04f5f', '#2f9e8f', '#f2a93b', '#7b5ce0', '#3a8fd9', '#d9579b', '#6fae3a', '#8c6d4f'];

/** A colour for a new lever: the first not used by another lever. */
export function nextLeverColor(d: Design): string {
  const used = new Set(d.gears.flatMap((g) => [g.motor?.lever?.color, g.motor?.pid?.color, g.motor?.pid?.lever?.color].filter((c): c is string => !!c)));
  return LEVER_COLORS.find((c) => !used.has(c)) ?? LEVER_COLORS[used.size % LEVER_COLORS.length];
}

/** The torque a motor applies at rest: its lever's setting if it has one. */
export function motorTorque(m: DMotor): number {
  if (m.pid) return 0; // set by the controller as it runs
  return m.lever ? m.lever.position * m.lever.scale : m.torque;
}

/** A deep copy of a gear, its motor and lever included. */
export function cloneGear(g: DGear): DGear {
  const pid = g.motor?.pid;
  const motor = g.motor && {
    ...g.motor,
    ...(g.motor.lever ? { lever: { ...g.motor.lever } } : {}),
    ...(pid ? { pid: { ...pid, sensor: pid.sensor ? { ...pid.sensor, body: { ...pid.sensor.body } } : null, ...(pid.lever ? { lever: { ...pid.lever } } : {}) } } : {}),
  };
  return { ...g, ...(motor ? { motor } : {}), ...(g.propeller ? { propeller: { ...g.propeller } } : {}) };
}

/** Radius of a motor's housing, as a disk for its inertia (m). */
export const motorHousingRadius = (g: { radius: number }) => 0.6 * g.radius;

/**
 * Two gears in mesh. Their pitch circles roll on each other, which imposes
 * N_a(φ_a − θ_c) + N_b(φ_b − θ_c) = const, θ_c being the direction of the line of centres.
 */
export interface DMesh {
  id: number;
  a: number;
  b: number;
}

/**
 * A gear meshing with a rack. The gear's pitch circle rolls on the rack's pitch line without
 * slipping, which turns rotation into translation: ξ + s R (φ − θ_rack) = const, ξ being
 * where the gear centre sits along the rack and s the side of the rack the gear is on.
 * The gear centre also stays on the pitch line's parallel at distance R.
 */
export interface DRackMesh {
  id: number;
  gear: number;
  rack: number;
}

/** A rigid body of the design: a bar (racks included) or a gear. */
export interface DBodyRef {
  /** A bar or rack, a gear, or the housing of the motor on gear `id`. */
  kind: 'link' | 'gear' | 'motor';
  id: number;
}

/**
 * Two rigid bodies joined into one: neither turns nor moves relative to the other. Body b's
 * reference point (a bar's joint a, a gear's centre) stays where it was drawn in body a's
 * frame, and the two turn together.
 */
export interface DJoin {
  id: number;
  a: DBodyRef;
  b: DBodyRef;
}

export const sameBody = (x: DBodyRef, y: DBodyRef) => x.kind === y.kind && x.id === y.id;

/** "bar 3", "rack 2" or "gear g1". */
export function bodyName(d: Design, ref: DBodyRef): string {
  if (ref.kind === 'gear') return gearName(d, ref.id);
  if (ref.kind === 'motor') return d.gears.find((g) => g.id === ref.id)?.propeller ? gearName(d, ref.id) : `motor M${gearLabel(ref.id)}`; // a propeller's motor housing is its hub
  return `${d.links.find((l) => l.id === ref.id)?.rack ? 'rack' : 'bar'} ${linkLabel(ref.id)}`;
}

/**
 * The ground the mechanism can land on: a terrain line through waypoints (sorted by x), flat
 * beyond the first and the last. Joints, bar ends and gear rims cannot pass below it.
 */
export interface DGround {
  points: [number, number][];
  /** Coefficient of restitution of impacts, 0 (dead stop) to 1 (perfect bounce). */
  restitution: number;
  /** Coulomb friction coefficient μ. */
  friction: number;
}

/** A straight ground through (0, y) rising at `slopeDeg`, as `n` waypoints from x0 to x1. */
export function straightGround(y: number, slopeDeg: number, x0: number, x1: number, n: number, restitution: number, friction: number): DGround {
  const t = Math.tan((slopeDeg * Math.PI) / 180);
  const points = Array.from({ length: n }, (_, i) => {
    const x = x0 + ((x1 - x0) * i) / (n - 1);
    return [+x.toFixed(4), +(y + t * x).toFixed(4)] as [number, number];
  });
  return { points, restitution, friction };
}

/** How much of the ground's height at x comes from waypoint i (linear between waypoints, level beyond). */
export function waypointWeight(pts: [number, number][], i: number, x: number): number {
  const n = pts.length;
  if (i === 0 && x <= pts[0][0]) return 1;
  if (i === n - 1 && x >= pts[n - 1][0]) return 1;
  if (i > 0 && x > pts[i - 1][0] && x <= pts[i][0]) return (x - pts[i - 1][0]) / (pts[i][0] - pts[i - 1][0] || 1);
  if (i < n - 1 && x >= pts[i][0] && x < pts[i + 1][0]) return (pts[i + 1][0] - x) / (pts[i + 1][0] - pts[i][0] || 1);
  return 0;
}

/** The highest waypoint i can be while the ground stays `clearance` below every point in `lows`. */
export function groundCeiling(g: DGround, i: number, lows: [number, number][], clearance: number): number {
  let top = Infinity;
  for (const [x, y] of lows) {
    const w = waypointWeight(g.points, i, x);
    if (w < 1e-6) continue;
    const others = groundHeight(g, x) - w * g.points[i][1]; // the rest of the terrain's height there
    top = Math.min(top, (y - clearance - others) / w);
  }
  return top;
}

/** Lower the ground wherever one of `lows` is less than `clearance` above it. Returns whether it moved. */
export function lowerGroundUnder(g: DGround, lows: [number, number][], clearance: number): boolean {
  let any = false;
  for (let pass = 0; pass < 4; pass++) {
    let moved = false;
    for (const [x, y] of lows) {
      const dip = groundHeight(g, x) - (y - clearance);
      if (dip <= 1e-9) continue;
      // lower the waypoints carrying the ground there, both by the dip, so it drops by the dip
      g.points.forEach((p, k) => {
        if (waypointWeight(g.points, k, x) > 0) p[1] = +(p[1] - dip).toFixed(4);
      });
      moved = any = true;
    }
    if (!moved) break;
  }
  return any;
}

/** Height of the ground at x: on the terrain line, or level with its end waypoint beyond it. */
export function groundHeight(g: DGround, x: number): number {
  const p = g.points;
  if (x <= p[0][0]) return p[0][1];
  for (let i = 1; i < p.length; i++)
    if (x <= p[i][0]) {
      const [x0, y0] = p[i - 1];
      const [x1, y1] = p[i];
      return x1 === x0 ? y1 : y0 + ((y1 - y0) * (x - x0)) / (x1 - x0);
    }
  return p[p.length - 1][1];
}

export interface Design {
  /**
   * Initial rates of the coordinates (by coordinate symbol, e.g. th3, x0; SI units), set with
   * the Initial state tool; any missing are 0. Made consistent with the constraints when the
   * simulation starts. (Initial positions are the drawing itself.)
   */
  initialRates?: Record<string, number>;
  points: DPoint[];
  links: DLink[];
  springs: DSpring[];
  gears: DGear[];
  meshes: DMesh[];
  rackMeshes: DRackMesh[];
  /** Bodies joined into one (absent in designs saved before joins existed). */
  joins?: DJoin[];
  /** Ground contact; absent when there is no ground. */
  ground?: DGround;
}

export function emptyDesign(): Design {
  return { points: [], links: [], springs: [], gears: [], meshes: [], rackMeshes: [], joins: [] };
}

/**
 * Designs saved with the old Fix angle tool: each fixed angle becomes a join of its two bars
 * (the same thing for bars sharing a joint; bars apart become rigidly fixed to each other
 * rather than only kept at their angle). Returns how many were converted.
 */
export function anglesToJoins(d: Design & { angles?: { id: number; a: number; b: number }[] }): number {
  const old = Array.isArray(d.angles) ? d.angles : [];
  delete d.angles;
  let n = 0;
  d.joins ??= [];
  for (const a of old) {
    if (a.a === a.b || !d.links.some((l) => l.id === a.a) || !d.links.some((l) => l.id === a.b)) continue;
    const [ra, rb]: DBodyRef[] = [{ kind: 'link', id: a.a }, { kind: 'link', id: a.b }];
    if (d.joins.some((j) => (sameBody(j.a, ra) && sameBody(j.b, rb)) || (sameBody(j.a, rb) && sameBody(j.b, ra)))) continue;
    d.joins.push({ id: nextId(d.joins), a: ra, b: rb });
    n++;
  }
  return n;
}


export function pointLabel(id: number): string {
  const letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';
  return id < 26 ? letters[id] : `P${id}`;
}

export function linkLabel(id: number): string {
  return `${id + 1}`;
}

export function nextId(items: readonly { id: number }[]): number {
  return items.reduce((m, it) => Math.max(m, it.id + 1), 0);
}

export function gearLabel(id: number): string {
  return `${id + 1}`;
}

/** "gear g3" or "wheel w3". */
export function gearName(d: Design, id: number): string {
  const g = d.gears.find((x) => x.id === id);
  return g?.propeller ? `propeller P${gearLabel(id)}` : g?.wheel ? `wheel w${gearLabel(id)}` : `gear g${gearLabel(id)}`;
}

/** Module (pitch diameter per tooth, m) used to choose a tooth count from a radius. */
export const GEAR_MODULE = 0.03;

export function teethForRadius(radius: number): number {
  return Math.max(6, Math.round((2 * radius) / GEAR_MODULE));
}

/**
 * Tooth pitch (m) of a rack: the circular pitch 2πR/N of the first gear meshing with it,
 * so their teeth interlock, or π × module when nothing meshes with it yet.
 */
export function rackPitch(d: Design, rackId: number): number {
  for (const m of d.rackMeshes) {
    const g = m.rack === rackId && d.gears.find((x) => x.id === m.gear);
    if (g) return (2 * Math.PI * g.radius) / g.teeth;
  }
  return Math.PI * GEAR_MODULE;
}

export function springLabel(id: number): string {
  return `${id + 1}`;
}

export function linkLength(d: Design, l: { a: number; b: number }): number {
  const a = d.points.find((p) => p.id === l.a)!;
  const b = d.points.find((p) => p.id === l.b)!;
  return Math.hypot(b.x - a.x, b.y - a.y);
}

export function cloneDesign(d: Design): Design {
  return { points: d.points.map((p) => ({ ...p })), links: d.links.map((l) => ({ ...l })), springs: d.springs.map((s) => ({ ...s })), gears: d.gears.map(cloneGear), meshes: d.meshes.map((m) => ({ ...m })), rackMeshes: d.rackMeshes.map((m) => ({ ...m })), joins: (d.joins ?? []).map((j) => ({ id: j.id, a: { ...j.a }, b: { ...j.b } })), ...(d.initialRates ? { initialRates: { ...d.initialRates } } : {}), ...(d.ground ? { ground: { ...d.ground, points: d.ground.points.map((p) => [p[0], p[1]] as [number, number]) } } : {}) };
}
