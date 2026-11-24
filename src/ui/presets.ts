import { teethForRadius, type Design, type DGear, type DPoint, type DSpring, type PointKind } from '../mechanics/model';

const pt = (id: number, x: number, y: number, kind: PointKind = 'free', mass = 0): DPoint => ({ id, x, y, kind, mass });
const bars = (pairs: [number, number][], mass = 1) => pairs.map(([a, b], id) => ({ id, a, b, mass }));
const gear = (id: number, center: number, radius: number, mass: number, locked = false): DGear => ({ id, center, radius, teeth: teethForRadius(radius), mass, locked });
/** A joint pinned onto a gear. */
const pin = (id: number, x: number, y: number, gearId: number): DPoint => ({ id, x, y, kind: 'free', mass: 0, gear: gearId });
const spring = (id: number, a: number, b: number, k: number, rest: number): DSpring => ({ id, a, b, k, rest });
/** Bars with individual masses: [a, b, mass]. */
const barsM = (list: [number, number, number][]) => list.map(([a, b, mass], id) => ({ id, a, b, mass }));
const weighted = (p: DPoint, mass: number): DPoint => ({ ...p, mass });

type P = [number, number];

/** Intersection of circle (c1, r1) and circle (c2, r2); `pick` chooses between the two solutions. */
function circleIntersect(c1: P, r1: number, c2: P, r2: number, pick: (u: P, v: P) => P): P {
  const dx = c2[0] - c1[0];
  const dy = c2[1] - c1[1];
  const D = Math.hypot(dx, dy);
  const A = (r1 * r1 - r2 * r2 + D * D) / (2 * D);
  const H = Math.sqrt(Math.max(r1 * r1 - A * A, 0));
  const mx = c1[0] + (A * dx) / D;
  const my = c1[1] + (A * dy) / D;
  return pick([mx - (H * dy) / D, my + (H * dx) / D], [mx + (H * dy) / D, my - (H * dx) / D]);
}
const upper = (u: P, v: P) => (u[1] > v[1] ? u : v);
const lower = (u: P, v: P) => (u[1] < v[1] ? u : v);
const outer = (u: P, v: P) => (u[0] < v[0] ? u : v);

/**
 * One leg of Theo Jansen's Strandbeest (Jansen's linkage), built from his "holy numbers"
 * (lengths in units of 1 cm here, so the leg is about 0.9 m tall):
 *   a = 38, l = 7.8  frame: the second ground pivot G sits at (−a, −l) from the crank axle O
 *   m = 15           crank          O – J1
 *   j = 50, b = 41.5 upper bars     J1 – J2, G – J2
 *   e = 55.8, d = 40.1  rocker triangle with b: J2 – J3, G – J3
 *   k = 61.9, c = 39.3  lower bars  J1 – J4, G – J4
 *   f = 39.4         J3 – J5
 *   g = 36.7, h = 65.7, i = 49  foot triangle J4 – J5 – foot
 * Topology from J. Wang, arXiv:2606.22129, §2. Joints are placed by circle intersection
 * at the starting crank angle, so every bar has exactly its holy-number length.
 */
function jansenLeg(crankAngle = Math.PI / 2): Design {
  const s = 0.01;
  const [a, b, c, d, e, f, g, h, i, j, k, l, m] = [38, 41.5, 39.3, 40.1, 55.8, 39.4, 36.7, 65.7, 49, 50, 61.9, 7.8, 15].map((v) => v * s);
  const O: P = [0, 0];
  const G: P = [-a, -l];
  const J1: P = [m * Math.cos(crankAngle), m * Math.sin(crankAngle)];
  const J2 = circleIntersect(J1, j, G, b, upper);
  const J3 = circleIntersect(J2, e, G, d, outer);
  const J4 = circleIntersect(J1, k, G, c, lower);
  const J5 = circleIntersect(J3, f, J4, g, outer);
  const foot = circleIntersect(J4, i, J5, h, lower);
  const lift = 0.5; // raise the drawing so the crank axle is above the origin
  const at = (id: number, [x, y]: P, kind: PointKind = 'free') => pt(id, x, y + lift, kind);
  return {
    points: [at(0, O, 'fixed'), at(1, J1), at(2, J2), at(3, J3), at(4, J4), at(5, J5), at(6, foot), at(7, G, 'fixed')],
    //           m       j       b       e       d       k       c       f       g       i       h
    links: bars([[0, 1], [1, 2], [7, 2], [2, 3], [7, 3], [1, 4], [7, 4], [3, 5], [4, 5], [4, 6], [5, 6]], 0.2),
    springs: [],
    gears: [],
    meshes: [],
  };
}

/**
 * Peaucellier–Lipkin linkage (1864), the first planar linkage to draw an exact straight line.
 * The rhombus A–C–B–P (side s) and the arms O–A, O–B (length l) make P the inverse of C in a
 * circle about O: OC · OP = l² − s². C is held on a circle through O by the crank O₂–C, so P
 * moves on the straight line x = O_x + (l² − s²)/(2r).
 */
function peaucellier(): Design {
  const O: P = [-0.6, 0];
  const r = 0.3;
  const O2: P = [O[0] + r, 0];
  const [l, s] = [0.8, 0.5];
  const crank = Math.PI / 3;
  const C: P = [O2[0] + r * Math.cos(crank), r * Math.sin(crank)];
  const OC = Math.hypot(C[0] - O[0], C[1] - O[1]);
  const u: P = [(C[0] - O[0]) / OC, (C[1] - O[1]) / OC];
  const OP = (l * l - s * s) / OC;
  const Pp: P = [O[0] + OP * u[0], O[1] + OP * u[1]];
  const A = circleIntersect(O, l, C, s, upper);
  const B = circleIntersect(O, l, C, s, lower);
  const lineX = Pp[0];
  return {
    points: [pt(0, O[0], O[1], 'fixed'), pt(1, O2[0], O2[1], 'fixed'), pt(2, C[0], C[1]), pt(3, A[0], A[1]), pt(4, B[0], B[1]), pt(5, Pp[0], Pp[1], 'free', 0.3), pt(6, lineX, 0, 'fixed')],
    //                crank       arms             rhombus
    links: barsM([[1, 2, 0.1], [0, 3, 0.2], [0, 4, 0.2], [3, 2, 0.15], [2, 4, 0.15], [3, 5, 0.15], [4, 5, 0.15]]),
    // a spring along the line pulls P towards its centre, so it oscillates on a perfectly straight path
    springs: [spring(0, 5, 6, 40, 0)],
    gears: [],
    meshes: [],
  };
}

export interface Preset {
  name: string;
  group: 'Famous mechanisms' | 'Basics';
  build: () => Design;
}

export const presets: Preset[] = [
  { name: 'Strandbeest leg (Jansen linkage)', group: 'Famous mechanisms', build: () => jansenLeg() },
  {
    // James Watt, 1781. The planet gear is fixed to the connecting rod (bars E–P and E–Q make
    // them one rigid body) and is kept in mesh with the sun gear by the link O–P. One orbit of
    // the planet turns the sun, and its flywheel, twice.
    name: "Watt's sun-and-planet gear",
    group: 'Famous mechanisms',
    build: () => ({
      points: [pt(0, -0.5, 0.6, 'fixed'), pt(1, 0.75, 0.6), pt(2, 0.6, -0.8, 'fixed'), pt(3, 0.6, -0.52), pin(4, 0.689, -0.532, 1)],
      //              beam          rod            rod strut      link O–P
      links: barsM([[0, 1, 1.5], [1, 3, 0.3], [1, 4, 0.1], [2, 3, 0.2]]),
      springs: [],
      gears: [gear(0, 2, 0.14, 4), gear(1, 3, 0.14, 0.4)], // sun gear on the heavy flywheel shaft, planet on the rod
      meshes: [{ id: 0, a: 0, b: 1 }],
    }),
  },
  {
    // Beam pumping unit ("nodding donkey"). Motor pinion → reduction gear carrying the
    // counterweighted crank → pitman arm → walking beam on the Samson post (a rigid triangle
    // with the saddle bearing below the beam) → horsehead. The elastic sucker-rod string down
    // the well is a spring with the rod weight at its top.
    name: 'Oil-well pump jack',
    group: 'Famous mechanisms',
    build: () => ({
      points: [
        pt(0, 0.5, 0, 'fixed'), // reduction gear / crankshaft
        pt(1, 0.85, 0, 'fixed'), // motor pinion
        weighted(pin(2, 0.641, 0.141, 0), 2), // crank pin with counterweight
        pt(3, -0.3, 0.75, 'fixed'), // Samson post saddle bearing
        pt(4, 0.55, 0.85), // beam tail
        pt(5, -1.2, 0.85, 'free', 0.5), // horsehead, carrying the polished rod
        pt(6, -1.2, -0.6, 'fixed'), // down-hole pump anchor
      ],
      //              pitman         beam: tail, head, top chord
      links: barsM([[2, 4, 0.4], [3, 4, 0.5], [3, 5, 0.6], [4, 5, 0.6]]),
      springs: [spring(0, 5, 6, 40, 1.2)],
      gears: [gear(0, 0, 0.25, 1.5), gear(1, 1, 0.1, 0.3)],
      meshes: [{ id: 0, a: 0, b: 1 }],
    }),
  },
  {
    // Crank, connecting rod and piston (x-locked, so it slides in a vertical cylinder). The
    // crankshaft gear drives the camshaft gear at half speed, as in every four-stroke engine.
    // Compression in the cylinder is modelled by a spring between piston and cylinder head.
    name: 'Four-stroke engine with 2:1 timing gears',
    group: 'Famous mechanisms',
    build: () => ({
      points: [pt(0, 0, 0, 'fixed'), pin(1, 0.06, 0.104, 0), pt(2, 0, 0.55, 'lockX', 0.5), pt(3, 0, 0.95, 'fixed'), pt(4, 0.45, 0, 'fixed')],
      links: barsM([[1, 2, 0.3]]),
      springs: [spring(0, 2, 3, 60, 0.65)],
      gears: [gear(0, 0, 0.15, 2), gear(1, 4, 0.3, 0.6)], // crankshaft gear with flywheel mass, camshaft gear
      meshes: [{ id: 0, a: 0, b: 1 }],
    }),
  },
  { name: 'Peaucellier–Lipkin straight-line linkage', group: 'Famous mechanisms', build: () => peaucellier() },
  {
    // Watt's linkage locates an axle sideways: the midpoint of the centre link moves on a
    // nearly straight vertical line. The body (fixed here) carries the axle on two coil springs;
    // the axle starts above its rest height, so it drops and bounces.
    name: "Watt's linkage axle on coil springs",
    group: 'Famous mechanisms',
    build: () => ({
      points: [pt(0, -0.8, 0.1, 'fixed'), pt(1, -0.1, 0.1), pt(2, 0.1, -0.3), pt(3, 0.8, -0.3, 'fixed'), pt(4, -0.1, 0.8, 'fixed'), pt(5, 0.1, 0.4, 'fixed')],
      links: barsM([[0, 1, 0.3], [1, 2, 3], [2, 3, 0.3]]),
      springs: [spring(0, 1, 4, 120, 0.62), spring(1, 2, 5, 120, 0.62)],
      gears: [],
      meshes: [],
    }),
  },
  {
    // The oscillator of a mechanical watch. The spiral hairspring acts as a torsion spring;
    // here two pre-tensioned springs on a rim pin give the same restoring torque for small
    // swings. The wheel starts turned 30° from rest.
    name: 'Watch balance wheel and hairspring',
    group: 'Famous mechanisms',
    build: () => ({
      points: [pt(0, 0, 0.2, 'fixed'), pin(1, 0.11, 0.39, 0), pt(2, -0.5, 0.42, 'fixed'), pt(3, 0.5, 0.42, 'fixed')],
      links: [],
      springs: [spring(0, 1, 2, 40, 0.3), spring(1, 1, 3, 40, 0.3)],
      gears: [gear(0, 0, 0.3, 1)],
      meshes: [],
    }),
  },
  {
    name: 'Simple pendulum',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.8, 'fixed'), pt(1, 0.8, 0.4, 'free', 1)], links: bars([[0, 1]], 0.2), springs: [], gears: [], meshes: [] }),
  },
  {
    name: 'Double pendulum',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.8, 'fixed'), pt(1, 0.7, 1.0), pt(2, 1.3, 0.6)], links: bars([[0, 1], [1, 2]]), springs: [], gears: [], meshes: [] }),
  },
  {
    name: 'Triple pendulum',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.4, 0.9, 'fixed'), pt(1, 0.2, 0.9), pt(2, 0.7, 0.9), pt(3, 1.1, 0.9)],
      links: bars([[0, 1], [1, 2], [2, 3]]), springs: [],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Cart-pole (y-locked cart)',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.2, 'lockY', 2), pt(1, 0.5, -0.6, 'free', 0.5)], links: bars([[0, 1]], 0.3), springs: [], gears: [], meshes: [] }),
  },
  {
    name: 'Slider-crank',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.9, 0, 'fixed'), pt(1, -0.6, 0.4), pt(2, 0.6, 0, 'lockY', 1)],
      links: bars([[0, 1], [1, 2]]), springs: [],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Four-bar linkage',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.8, -0.3, 'fixed'), pt(1, -0.6, 0.4), pt(2, 0.5, 0.6), pt(3, 0.6, -0.3, 'fixed')],
      links: bars([[0, 1], [1, 2], [2, 3]]), springs: [],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Hanging chain (closed loop)',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -1, 0.8, 'fixed'), pt(1, -0.7, 0.3), pt(2, 0, 0.1), pt(3, 0.6, 0.2), pt(4, 1, 0.8, 'fixed')],
      links: bars([[0, 1], [1, 2], [2, 3], [3, 4]], 0.5), springs: [],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Mass on a spring (horizontal rail)',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.9, 0, 'fixed'), pt(1, 0.4, 0, 'lockY', 1)],
      links: [],
      springs: [spring(0, 0, 1, 40, 0.8)],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Elastic pendulum',
    group: 'Basics',
    build: () => ({
      points: [pt(0, 0, 0.9, 'fixed'), pt(1, 0.5, 0.1, 'free', 1)],
      links: [],
      springs: [spring(0, 0, 1, 60, 0.6)],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Pendulum held by a spring',
    group: 'Basics',
    build: () => ({
      points: [pt(0, 0, 0.8, 'fixed'), pt(1, 0.6, 0, 'free', 0.5), pt(2, 1.3, 0.3, 'fixed')],
      links: bars([[0, 1]]),
      springs: [spring(0, 1, 2, 30, 0.4)],
      gears: [],
      meshes: [],
    }),
  },
  {
    name: 'Spring-loaded four-bar',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.8, -0.3, 'fixed'), pt(1, -0.6, 0.4), pt(2, 0.5, 0.6), pt(3, 0.6, -0.3, 'fixed'), pt(4, -0.2, 1.2, 'fixed')],
      links: bars([[0, 1], [1, 2], [2, 3]]),
      springs: [spring(0, 1, 4, 80, 0.5), spring(1, 2, 4, 80, 0.5)],
      gears: [],
      meshes: [],
    }),
  },
  {
    // Two equal gears on ground axles turn in opposite directions; the coupler bars are
    // pinned onto the gears, so the five-bar loop (2 DOF) is driven with 1 DOF.
    name: 'Geared five-bar linkage',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.36, 0, 'fixed'), pt(1, 0.36, 0, 'fixed'), pin(2, -0.36, 0.26, 0), pin(3, 0.36, 0.26, 1), pt(4, 0.1, 0.9)],
      links: bars([[2, 4], [3, 4]], 0.4),
      springs: [],
      gears: [gear(0, 0, 0.36, 1), gear(1, 1, 0.36, 1)],
      meshes: [{ id: 0, a: 0, b: 1 }],
    }),
  },
  {
    // Locked sun gear at A, carrier bar A–B, planet gear turning on B with a pendulum on it.
    name: 'Planetary gear with pendulum',
    group: 'Basics',
    build: () => ({
      points: [pt(0, 0, 0.3, 'fixed'), pt(1, 0.45, 0.3), pin(2, 0.55, 0.3, 1), pt(3, 0.55, -0.2, 'free', 0.3)],
      links: bars([[0, 1], [2, 3]], 0.3),
      springs: [],
      gears: [gear(0, 0, 0.3, 1, true), gear(1, 1, 0.15, 0.5)],
      meshes: [{ id: 0, a: 0, b: 1 }],
    }),
  },
  {
    // Three gears on fixed axles; a weighted arm pinned on the big gear falls and drives the train.
    name: 'Gear train with crank weight',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.6, 0.2, 'fixed'), pt(1, -0.21, 0.2, 'fixed'), pt(2, 0.39, 0.2, 'fixed'), pin(3, 0.6, 0.45, 2), pt(4, 0.85, 0.8, 'free', 0.4)],
      links: bars([[3, 4]], 0.2),
      springs: [],
      gears: [gear(0, 0, 0.15, 0.3), gear(1, 1, 0.24, 0.6), gear(2, 2, 0.36, 0.9)],
      meshes: [{ id: 0, a: 0, b: 1 }, { id: 1, a: 1, b: 2 }],
    }),
  },
  {
    name: 'Pendulum on a vertical rail',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.6, 'lockX', 1), pt(1, 0.6, 0.3)], links: bars([[0, 1]]), springs: [], gears: [], meshes: [] }),
  },
];
