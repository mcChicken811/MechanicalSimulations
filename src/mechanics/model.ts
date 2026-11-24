/** The editable description of a planar linkage (world units: metres, kilograms; y points up). */

/**
 * free  – the joint moves in both x and y
 * fixed – pinned to the ground (both x and y locked)
 * lockX – x is locked: the joint slides along a vertical rail
 * lockY – y is locked: the joint slides along a horizontal rail
 */
export type PointKind = 'free' | 'fixed' | 'lockX' | 'lockY';

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
}

/** A uniform rigid bar between two joints. Bars sharing a joint are pinned together. */
export interface DLink {
  id: number;
  a: number;
  b: number;
  mass: number;
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

export interface Design {
  points: DPoint[];
  links: DLink[];
  springs: DSpring[];
  gears: DGear[];
  meshes: DMesh[];
}

export function emptyDesign(): Design {
  return { points: [], links: [], springs: [], gears: [], meshes: [] };
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

export function springLabel(id: number): string {
  return `${id + 1}`;
}

export function linkLength(d: Design, l: { a: number; b: number }): number {
  const a = d.points.find((p) => p.id === l.a)!;
  const b = d.points.find((p) => p.id === l.b)!;
  return Math.hypot(b.x - a.x, b.y - a.y);
}

export function cloneDesign(d: Design): Design {
  return { points: d.points.map((p) => ({ ...p })), links: d.links.map((l) => ({ ...l })), springs: d.springs.map((s) => ({ ...s })), gears: d.gears.map((g) => ({ ...g })), meshes: d.meshes.map((m) => ({ ...m })) };
}
