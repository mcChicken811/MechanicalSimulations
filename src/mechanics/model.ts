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
}

/**
 * A motor's torque on its gear: τ (N·m, positive turns the gear anticlockwise), falling
 * linearly with the gear's speed to zero at the no-load speed, τ (1 − ω/Ω), like a DC motor.
 * A no-load speed of 0 means a constant torque.
 */
export interface DMotor {
  torque: number;
  /** No-load speed Ω (rpm); 0 for a constant torque. */
  noLoadRpm: number;
}

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

/**
 * Two bars whose relative angle is held at its drawn value: θ_b − θ_a = α. Bars that share a
 * joint are then welded there into one rigid piece; bars apart from each other stay parallel
 * (or at the drawn angle) wherever they move.
 */
export interface DAngle {
  id: number;
  a: number;
  b: number;
}

/** A rigid body of the design: a bar (racks included) or a gear. */
export interface DBodyRef {
  kind: 'link' | 'gear';
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
  if (ref.kind === 'gear') return `gear g${gearLabel(ref.id)}`;
  return `${d.links.find((l) => l.id === ref.id)?.rack ? 'rack' : 'bar'} ${linkLabel(ref.id)}`;
}

/**
 * A flat ground the mechanism can land on: the line through (0, y), rising at `slope`
 * degrees to the right. Joints, bar ends and gear rims cannot pass below it.
 */
export interface DGround {
  y: number;
  /** Degrees; positive rises to the right. */
  slope: number;
  /** Coefficient of restitution of impacts, 0 (dead stop) to 1 (perfect bounce). */
  restitution: number;
  /** Coulomb friction coefficient μ. */
  friction: number;
}

export interface Design {
  points: DPoint[];
  links: DLink[];
  springs: DSpring[];
  gears: DGear[];
  meshes: DMesh[];
  rackMeshes: DRackMesh[];
  /** Fixed angles between bars (absent in designs saved before they existed). */
  angles?: DAngle[];
  /** Bodies joined into one (absent in designs saved before joins existed). */
  joins?: DJoin[];
  /** Ground contact; absent when there is no ground. */
  ground?: DGround;
}

export function emptyDesign(): Design {
  return { points: [], links: [], springs: [], gears: [], meshes: [], rackMeshes: [], angles: [], joins: [] };
}

/**
 * The angle (degrees, 0–180) between two bars as drawn: the corner angle at the joint they
 * share, or the angle between their lines when they do not touch.
 */
export function drawnAngle(d: Design, la: DLink, lb: DLink): number {
  const p = (id: number) => d.points.find((x) => x.id === id)!;
  const dir = (l: DLink, from: number) => {
    const [o, e] = from === l.a ? [p(l.a), p(l.b)] : [p(l.b), p(l.a)];
    return Math.atan2(e.y - o.y, e.x - o.x);
  };
  const common = sharedJoint(la, lb);
  const period = common !== null ? 2 * Math.PI : Math.PI;
  const diff = Math.abs(dir(lb, common ?? lb.a) - dir(la, common ?? la.a)) % period;
  return (Math.min(diff, period - diff) * 180) / Math.PI;
}

/** The joint two bars share, if any. */
export function sharedJoint(a: { a: number; b: number }, b: { a: number; b: number }): number | null {
  return a.a === b.a || a.a === b.b ? a.a : a.b === b.a || a.b === b.b ? a.b : null;
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
  return { points: d.points.map((p) => ({ ...p })), links: d.links.map((l) => ({ ...l })), springs: d.springs.map((s) => ({ ...s })), gears: d.gears.map((g) => ({ ...g })), meshes: d.meshes.map((m) => ({ ...m })), rackMeshes: d.rackMeshes.map((m) => ({ ...m })), angles: (d.angles ?? []).map((a) => ({ ...a })), joins: (d.joins ?? []).map((j) => ({ id: j.id, a: { ...j.a }, b: { ...j.b } })), ...(d.ground ? { ground: { ...d.ground } } : {}) };
}
