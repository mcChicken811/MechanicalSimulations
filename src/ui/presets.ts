import { straightGround, teethForRadius, type Design, type DGear, type DJoin, type DLink, type DPoint, type DSpring, type PointKind } from '../mechanics/model';

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
const nearest = (ref: P) => (u: P, v: P) => (Math.hypot(u[0] - ref[0], u[1] - ref[1]) < Math.hypot(v[0] - ref[0], v[1] - ref[1]) ? u : v);
const rotateAbout = (c: P, p: P, a: number): P => [
  c[0] + (p[0] - c[0]) * Math.cos(a) - (p[1] - c[1]) * Math.sin(a),
  c[1] + (p[0] - c[0]) * Math.sin(a) + (p[1] - c[1]) * Math.cos(a),
];
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
    meshes: [], rackMeshes: [],
  };
}

/**
 * The Strandbeest leg with its crank driven: a motor on a gear fixed to the crank (the gear is
 * joined to the crank bar) turns it against gravity, and the foot traces Jansen's flat-bottomed
 * walking path. Select the foot (joint G) before Play to draw it. The motor gives 3 N·m at rest
 * and none at 60 rpm, so the crank settles near the speed where the two balance.
 */
function drivenJansenLeg(): Design {
  const d = jansenLeg();
  return {
    ...d,
    gears: [{ ...gear(0, 0, 0.12, 0.5), motor: { torque: 3, noLoadRpm: 60 } }],
    joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'gear', id: 0 } }],
  };
}

/**
 * The crank-and-slotted-lever quick-return mechanism, as in a shaping machine. A motor turns
 * the crank; its pin slides in the slot of a lever pivoted below (a rail fixed to the lever,
 * turning with it), and the lever's top drives the ram along its guide (a ground rail). The
 * crank pin spends longer on the far side of its circle, so the ram's cutting stroke is slower
 * than its return.
 */
function quickReturn(): Design {
  const crank = 0.12;
  const O: P = [0, 0];
  const Q: P = [0, -0.35];
  const a0 = (30 * Math.PI) / 180;
  const Pp: P = [O[0] + crank * Math.cos(a0), O[1] + crank * Math.sin(a0)];
  const lever = Math.atan2(Pp[1] - Q[1], Pp[0] - Q[0]);
  const R: P = [Q[0] + 0.7 * Math.cos(lever), Q[1] + 0.7 * Math.sin(lever)];
  const ramY = 0.4;
  const S: P = [R[0] + Math.sqrt(0.3 ** 2 - (ramY - R[1]) ** 2), ramY];
  return {
    points: [
      pt(0, ...O, 'fixed'), // crank axle
      { ...pt(1, ...Pp), slide: { on: { kind: 'link', id: 1 }, angle: (lever * 180) / Math.PI } }, // crank pin, in the lever's slot
      pt(2, ...Q, 'fixed'), // lever pivot
      pt(3, ...R), // lever top
      { ...pt(4, ...S, 'free', 1), slide: { on: null, angle: 0 } }, // ram, on its guide
    ],
    //              crank          lever          link to the ram
    links: barsM([[0, 1, 0.2], [2, 3, 0.6], [3, 4, 0.2]]),
    springs: [],
    gears: [{ ...gear(0, 0, 0.08, 0.4), motor: { torque: 2, noLoadRpm: 60 } }],
    meshes: [],
    rackMeshes: [],
    joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'gear', id: 0 } }],
  };
}

/**
 * A simple Strandbeest: four Jansen legs one after another, each crank a quarter turn behind
 * the one before, so the four legs step in turn through the cycle. Every crank carries a gear (joined to it), and the gears are linked by coupling rods
 * between pins at the same place on each gear, so each pair of gears forms a parallelogram and
 * all four cranks turn together, like the side rods of a locomotive. A motor on the last
 * gear drives the whole beest. The frame is held still, so the legs step in place.
 */
function strandbeest(legs = 4, spacing = 1.1): Design {
  const points: DPoint[] = [];
  const links: DLink[] = [];
  const gears: DGear[] = [];
  const joins: DJoin[] = [];
  const pinR = 0.08; // coupling pins, at the same angle on every gear
  const pinAngle = Math.PI / 4;
  for (let i = 0; i < legs; i++) {
    const leg = jansenLeg(Math.PI / 2 + (i * Math.PI) / 2); // each crank a quarter turn from the last
    const dp = points.length;
    const dl = links.length;
    const dx = i * spacing;
    for (const p of leg.points) points.push({ ...p, id: p.id + dp, x: p.x + dx });
    for (const l of leg.links) links.push({ ...l, id: l.id + dl, a: l.a + dp, b: l.b + dp });
    const axle = dp; // joint 0 of each leg is its crank axle
    gears.push({ ...gear(i, axle, 0.12, 0.4), ...(i === legs - 1 ? { motor: { torque: 8, noLoadRpm: 40 } } : {}) });
    joins.push({ id: i, a: { kind: 'link', id: dl }, b: { kind: 'gear', id: i } }); // the crank turns with its gear
  }
  // coupling pins and rods
  const pinIds = gears.map((g) => {
    const c = points.find((p) => p.id === g.center)!;
    const id = points.length;
    points.push(pin(id, c.x + pinR * Math.cos(pinAngle), c.y + pinR * Math.sin(pinAngle), g.id));
    return id;
  });
  for (let i = 0; i + 1 < pinIds.length; i++) links.push({ id: links.length, a: pinIds[i], b: pinIds[i + 1], mass: 0.2 });
  return { points, links, springs: [], gears, meshes: [], rackMeshes: [], joins };
}

/**
 * Rack-and-pinion steering, seen from above. The steering-column pinion moves the rack
 * sideways on its rails; tie rods push the steering arms, turning each wheel's knuckle about
 * its kingpin. The arms point inwards (Ackermann geometry), so the inner wheel turns more.
 * Two pre-tensioned springs stand in for the self-centring of the tyres. The rack starts
 * shifted 6 cm to the right, so the wheels start turned and swing back through centre.
 */
function steering(shift = 0.06): Design {
  const R = 0.25;
  const A0: P = [-0.45, 0];
  const B0: P = [0.45, 0];
  const side = (s: 1 | -1) => {
    const K: P = [0.85 * s, 0.3]; // kingpin
    const C0: P = [0.72 * s, 0.05]; // steering arm end, centred
    const W0: P = [0.85 * s, 0.65]; // front of the wheel
    const end: P = s < 0 ? A0 : B0;
    const arm = Math.hypot(C0[0] - K[0], C0[1] - K[1]);
    const tie = Math.hypot(C0[0] - end[0], C0[1] - end[1]);
    const C = circleIntersect(K, arm, [end[0] + shift, end[1]], tie, nearest(C0));
    const turn = Math.atan2(C[1] - K[1], C[0] - K[0]) - Math.atan2(C0[1] - K[1], C0[0] - K[0]);
    return { K, C, W: rotateAbout(K, W0, turn) };
  };
  const L = side(-1);
  const Rt = side(1);
  return {
    points: [
      pt(0, 0, R, 'fixed'), // steering column
      pt(1, A0[0] + shift, 0, 'lockY'),
      pt(2, B0[0] + shift, 0, 'lockY'),
      pt(3, ...L.K, 'fixed'),
      pt(4, ...L.C),
      pt(5, ...L.W),
      pt(6, ...Rt.K, 'fixed'),
      pt(7, ...Rt.C),
      pt(8, ...Rt.W),
      pt(9, -0.45, -0.4, 'fixed'),
      pt(10, 0.45, -0.4, 'fixed'),
    ],
    //               rack          tie rods                  left knuckle: arm, wheel, brace      right knuckle
    links: [{ id: 0, a: 1, b: 2, mass: 1, rack: 1 }, ...barsM([[1, 4, 0.2], [2, 7, 0.2], [3, 4, 0.3], [3, 5, 2], [4, 5, 0.1], [6, 7, 0.3], [6, 8, 2], [7, 8, 0.1]]).map((l) => ({ ...l, id: l.id + 1 }))],
    springs: [spring(0, 1, 9, 60, 0.25), spring(1, 2, 10, 60, 0.25)],
    gears: [gear(0, 0, R, 1)],
    meshes: [],
    rackMeshes: [{ id: 0, gear: 0, rack: 0 }],
  };
}

/**
 * Trapezoid (Ackermann) steering of a beam axle, seen from above. The two steering arms
 * point back and inwards, so with the tie rod they form a trapezoid. When the wheels turn,
 * the inner wheel turns more than the outer one, and the lines of their axles meet close to
 * the line of the rear axle (marked by its two hubs), so no tyre has to slip sideways. The steering box turns the pitman arm, and the drag link pushes the left arm.
 * Two pre-tensioned springs on the pitman arm stand in for the self-centring of the tyres;
 * the pitman arm starts turned by `turn`, so the wheels swing back and forth through centre.
 */
function trapezoidSteering(turn = (20 * Math.PI) / 180): Design {
  const track = 0.8; // half the distance between the kingpins
  const axleY = 0.3;
  const rearY = axleY - 1.6; // wheelbase 1.6 m
  const armLen = 0.18;
  // Inward angle of the steering arms. The textbook rule (aim at the middle of the rear axle)
  // gives atan(0.8 / 1.6) = 26.6° here, which corrects only about 70% of the way; 35° keeps
  // the outer wheel within half a degree of true Ackermann from 10° to 30° of steering.
  const beta = (35 * Math.PI) / 180;
  const knuckle = (s: 1 | -1) => {
    const K: P = [track * s, axleY];
    const C0: P = [K[0] - s * armLen * Math.sin(beta), K[1] - armLen * Math.cos(beta)];
    return { K, C0, W0: [K[0], axleY + 0.3] as P, B0: [K[0], axleY - 0.3] as P }; // tyre front and back
  };
  const L = knuckle(-1);
  const R = knuckle(1);
  // The drag link meets the left steering arm at a right angle (best leverage, far from a
  // dead point); the pitman arm hangs straight down from the steering box, so it moves the
  // drag link end mostly sideways.
  const drag = 0.4;
  const armDir: P = [(L.C0[0] - L.K[0]) / armLen, (L.C0[1] - L.K[1]) / armLen];
  const Q0: P = [L.C0[0] - drag * armDir[1], L.C0[1] + drag * armDir[0]]; // pitman arm end, centred
  const S: P = [Q0[0], Q0[1] + 0.2]; // steering box output shaft
  const tie = Math.hypot(R.C0[0] - L.C0[0], R.C0[1] - L.C0[1]);
  const angle = (K: P, C: P) => Math.atan2(C[1] - K[1], C[0] - K[0]);
  // start turned: rotate the pitman arm, then close the drag link and the trapezoid
  const Q = rotateAbout(S, Q0, turn);
  const CL = circleIntersect(L.K, armLen, Q, drag, nearest(L.C0));
  const CR = circleIntersect(R.K, armLen, CL, tie, nearest(R.C0));
  const turnL = angle(L.K, CL) - angle(L.K, L.C0);
  const turnR = angle(R.K, CR) - angle(R.K, R.C0);
  const [WL, BL] = [rotateAbout(L.K, L.W0, turnL), rotateAbout(L.K, L.B0, turnL)];
  const [WR, BR] = [rotateAbout(R.K, R.W0, turnR), rotateAbout(R.K, R.B0, turnR)];
  return {
    points: [
      pt(0, ...L.K, 'fixed'), // left kingpin
      pt(1, ...CL),
      pt(2, ...WL),
      pt(3, ...R.K, 'fixed'), // right kingpin
      pt(4, ...CR),
      pt(5, ...WR),
      pt(6, ...S, 'fixed'), // steering box
      pt(7, ...Q),
      pt(8, Q0[0] - 0.4, Q0[1], 'fixed'), // spring anchors
      pt(9, Q0[0] + 0.4, Q0[1], 'fixed'),
      pt(10, -track, rearY, 'fixed'), // rear wheel hubs, for reference
      pt(11, track, rearY, 'fixed'),
      pt(12, ...BL), // backs of the front tyres
      pt(13, ...BR),
    ],
    // each knuckle is rigid: steering arm, the two halves of the tyre, and two braces to the arm
    links: barsM([
      [0, 1, 0.3], [0, 2, 1], [0, 12, 1], [1, 2, 0.05], [1, 12, 0.05], // left knuckle
      [3, 4, 0.3], [3, 5, 1], [3, 13, 1], [4, 5, 0.05], [4, 13, 0.05], // right knuckle
      [1, 4, 0.4], // tie rod
      [6, 7, 0.3], // pitman arm
      [7, 1, 0.3], // drag link
    ]),
    springs: [spring(0, 7, 8, 60, 0.25), spring(1, 7, 9, 60, 0.25)],
    gears: [],
    meshes: [],
    rackMeshes: [],
  };
}

/**
 * A human leg in its swing phase, seen from the side (walking to the right), hanging from a
 * hip held still. Literature it is built from:
 *
 * - Knee: the crossed four-bar linkage of O'Connor, Shercliff, Biden & Goodfellow, "The geometry
 *   of the knee in the sagittal plane", Proc. IMechE H 203 (1989) 223–233. The isometric fibres
 *   of the cruciate ligaments and the two bones are the four links. Sagittal lengths: ACL
 *   29.9 mm, PCL 32.2 mm, femoral link 12.8 mm, tibial link 30.5 mm, as tabulated by Hernaiz
 *   Alzamora et al., "Anthropometry of the cruciate ligaments of the knee: MRI study of the
 *   human four-bar linkage device", Eur. J. Anat. 21 (2017) 1–11. The flexion axis passes
 *   where the two ligaments cross, so it rolls back as the knee bends.
 * - At full extension the ACL rises at 45° from the tibial link, within the 40–55° range of
 *   sagittal ACL angles measured on MRI. The tibial link slopes down 10° towards the back, as
 *   the PCL inserts low on the back of the tibia.
 * - Segments: Winter, "Biomechanics and Motor Control of Human Movement" (after Drillis &
 *   Contini, 1966) for a 1.75 m, 70 kg adult. Thigh 0.245 H, shank 0.246 H, foot 0.152 H long,
 *   ankle 0.039 H above the sole; masses thigh 0.100 M, shank 0.0465 M, foot 0.0145 M, centres
 *   of mass 0.433 of the length from the hip and from the knee.
 * - Swing: Mochon & McMahon, "Ballistic walking", J. Biomech. 13 (1980) 49–57, showed that the
 *   swing leg moves much like a passive double pendulum under gravity, with the foot held at
 *   the ankle. Here the foot is joined to the shank, and the leg starts at rest at toe-off,
 *   hip extended by `hipExt` and knee bent by `kneeFlex`, then swings forward.
 * - Extension stop: the posterior capsule is a tension-only spring, taut exactly at full
 *   extension and slack in flexion, from above the back of the femoral condyles to the back of
 *   the tibia. Its stiffness, 200 N/mm, is of the order of a knee ligament's (the ACL's is
 *   about 240 N/mm, Woo et al., Am. J. Sports Med. 19 (1991) 217–225). Without it the
 *   cruciate linkage alone would let the knee swing on into hyperextension.
 *
 * Each bone is a uniform bar plus a point mass at its upper end, sized so the bone's mass and
 * centre of mass match Winter: a bar of 0.866 m puts the centre at 0.433 L, and its moment of
 * inertia about the upper joint, m L²/3 · 0.866, is within 1% (thigh) and 4% (shank) of his.
 * The femur and the tibia are each a long bar joined to the short link between the ligament
 * attachments, so they show as one colour per bone.
 */
function humanLeg(hipExt = (15 * Math.PI) / 180, kneeFlex = (40 * Math.PI) / 180): Design {
  const H = 1.75;
  const M = 70;
  const [thighL, shankL, footL, ankleH] = [0.245 * H, 0.246 * H, 0.152 * H, 0.039 * H];
  const [thighM, shankM, footM] = [0.1 * M, 0.0465 * M, 0.0145 * M];
  const [ACL, PCL, FEM, TIB] = [0.0299, 0.0322, 0.0128, 0.0305];
  const deg = Math.PI / 180;

  // Knee at full extension, tibia frame: tibial plateau through the origin, anterior +x.
  const TA: P = [0.015, 0]; // ACL insertion on the tibia (anterior)
  const TP: P = [TA[0] - TIB * Math.cos(10 * deg), TA[1] - TIB * Math.sin(10 * deg)]; // PCL insertion (posterior, lower)
  const FA0: P = [TA[0] + ACL * Math.cos(Math.PI - 45 * deg), TA[1] + ACL * Math.sin(Math.PI - 45 * deg)]; // ACL on the femur
  const front = (u: P, v: P) => (u[0] > v[0] ? u : v);
  const FP0 = circleIntersect(FA0, FEM, TP, PCL, front); // PCL on the femur: anterior, so the ligaments cross
  const hip0: P = [0, thighL];
  const ankle: P = [0, -shankL];
  const heel: P = [-0.05, -shankL - ankleH];
  const toe: P = [heel[0] + footL, heel[1]];
  const capF: P = [-0.035, 0.045]; // posterior capsule on the femur, above the condyles
  const capT: P = [-0.03, -0.02]; // and on the back of the tibia
  const capRest = Math.hypot(capF[0] - capT[0], capF[1] - capT[1]); // taut at full extension

  // Bend the knee: swing the ACL back about its tibial insertion (it turns towards the tibia in
  // flexion), close the linkage, and carry the femur along as a rigid body.
  const femurAt = (delta: number) => {
    const FA = rotateAbout(TA, FA0, delta);
    const FP = circleIntersect(FA, FEM, TP, PCL, nearest(FP0));
    const turn = Math.atan2(FP[1] - FA[1], FP[0] - FA[0]) - Math.atan2(FP0[1] - FA0[1], FP0[0] - FA0[0]);
    const move = (p: P): P => {
      const r = rotateAbout(FA0, p, turn);
      return [r[0] + FA[0] - FA0[0], r[1] + FA[1] - FA0[1]];
    };
    return { FA, FP, turn, hip: move(hip0), capF: move(capF) };
  };
  let [lo, hi] = [0, Math.PI / 2];
  for (let i = 0; i < 60; i++) {
    const mid = (lo + hi) / 2;
    if (femurAt(mid).turn < kneeFlex) lo = mid;
    else hi = mid;
  }
  const knee = femurAt(lo);

  // Pose the whole leg: hip at (0, 1), thigh tilted back by the hip extension.
  const thighDir = Math.atan2(TA[1] - knee.hip[1], TA[0] - knee.hip[0]);
  const spin = -Math.PI / 2 - hipExt - thighDir;
  const place = (p: P): P => {
    const r = rotateAbout(knee.hip, p, spin);
    return [r[0] - knee.hip[0], r[1] - knee.hip[1] + 1];
  };
  const at = (id: number, p: P, mass = 0, kind: PointKind = 'free') => {
    const [x, y] = place(p);
    return pt(id, x, y, kind, mass);
  };
  return {
    points: [
      at(0, knee.hip, 0, 'fixed'), // hip, held still
      at(1, knee.FA), // femoral attachment of the ACL
      at(2, knee.FP), // femoral attachment of the PCL
      at(3, TA, 0.134 * shankM), // tibial attachment of the ACL, carrying part of the shank's mass
      at(4, TP), // tibial attachment of the PCL
      at(5, ankle),
      at(6, heel),
      at(7, toe),
      at(8, knee.capF), // posterior capsule attachments
      at(9, capT),
    ],
    links: barsM([
      [0, 1, 0.866 * thighM], // femur
      [1, 2, 0.01], // femoral link (intercondylar roof)
      [3, 1, 0.002], // ACL
      [4, 2, 0.002], // PCL
      [3, 4, 0.01], // tibial link (plateau)
      [3, 5, 0.866 * shankM], // tibia
      [5, 6, 0.25 * footM], // foot: ankle to heel
      [6, 7, 0.75 * footM], // foot: sole
      [1, 8, 0.002], // femur, to the capsule attachment
      [4, 9, 0.002], // tibia, to the capsule attachment
    ]),
    springs: [{ id: 0, a: 8, b: 9, k: 2e5, rest: capRest, tensionOnly: true }], // posterior capsule
    gears: [],
    meshes: [],
    rackMeshes: [],
    joins: [
      { id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 1 } }, // femur is one bone
      { id: 1, a: { kind: 'link', id: 5 }, b: { kind: 'link', id: 4 } }, // tibia is one bone
      { id: 2, a: { kind: 'link', id: 5 }, b: { kind: 'link', id: 6 } }, // ankle held, as in ballistic walking
      { id: 3, a: { kind: 'link', id: 6 }, b: { kind: 'link', id: 7 } }, // foot is one piece
      { id: 4, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 8 } },
      { id: 5, a: { kind: 'link', id: 5 }, b: { kind: 'link', id: 9 } },
    ],
  };
}

/**
 * A whole human hand, flattened into the plane of the palm (palm towards you, hanging from
 * the wrist): the "planar hand" of grasping and robotics papers. Every digit bends at each of
 * its joints, but within the drawing plane: the fingers curl towards the thumb and the thumb
 * towards the fingers, as they would to pinch. Literature it is built from:
 *
 * - Bone lengths and widths: Table II of Cerruti, Chablat, Gouaillier & Sakka, "Design method
 *   for an anthropomorphic hand able to gesture and grasp", IEEE ICRA 2015 (arXiv:1504.01151),
 *   derived there from human hand proportions (Davidoff & Freivalds 1993; Chao et al. 1989).
 * - Knuckles: their Table III positions and angles of the finger bases (MCP joints) in the
 *   palm, on Isobe's arc about the middle finger's MCP joint.
 * - Size: their hand (wrist to middle fingertip 127.5 mm, for a child-sized robot) is scaled
 *   to a 19 cm adult hand. They assume bone proportions are the same at any hand size.
 * - Thumb: the metacarpal leaves the palm at their 45°, 12 mm from the wrist. Its base sits on
 *   the radial edge of the wrist, 20 mm from the midline, because in this flattened view their
 *   base (under the palm) would lie on top of the palm.
 * - Masses: as in Tsakonas et al., Healthc. Technol. Lett. 13 (2026) e70078. Each phalanx is a
 *   uniform cylinder of the width above, and the palm a block 30 mm thick, all at Dempster's hand
 *   density of 1.16 g/cm³. That gives 0.55 kg (palm 0.30 kg): a little heavy next to Winter's
 *   0.6% of body mass (0.42 kg at 70 kg), as full-width cylinders overestimate the fingers.
 * - Joint stops: each joint's volar plate is a tension-only spring on the flexion side, which
 *   goes taut when the joint straightens. It runs from the bone before the joint (for the
 *   knuckles, a point on the metacarpal 35 mm back, in line with the finger). Stiffness
 *   10 N/mm, and 40 N/mm at the finger knuckles, which carry a whole finger's momentum. Passive hyperextension allowed: MCP 20°, DIP and thumb
 *   IP 5–10°, PIP and thumb MCP 0°. The wrist's collateral ligaments limit its side tilt to 20°
 *   radial and 30° ulnar. Flexion is not limited.
 * - Start: the fingers in a relaxed cascade, each more bent than the one before, from index to
 *   little. Tissue damping comes from the preset's air drag.
 */
function humanHand(): Design {
  const s = 0.19 / 0.1275; // scale from their hand to a 19 cm hand
  const mm = (v: number) => (v / 1000) * s;
  const rho = 1160; // kg/m³
  const deg = Math.PI / 180;
  // their hand frame: x along the hand from the wrist, y towards the index; drawing: X = −y, Y = −x
  const fromTheirs = (x: number, y: number): P => [-mm(y), -mm(x)];
  const dirOf = (alpha: number): number => Math.atan2(-Math.cos(alpha * deg), -Math.sin(alpha * deg));
  const cylinder = (len: number, width: number) => Math.PI * rho * (mm(width) / 2) ** 2 * mm(len);

  const points: DPoint[] = [];
  const links: { id: number; a: number; b: number; mass: number }[] = [];
  const springs: DSpring[] = [];
  const joins: { id: number; a: { kind: 'link'; id: number }; b: { kind: 'link'; id: number } }[] = [];
  const add = (p: P, kind: PointKind = 'free') => {
    points.push(pt(points.length, p[0], p[1], kind));
    return points.length - 1;
  };
  const bar = (a: number, b: number, mass: number) => {
    links.push({ id: links.length, a, b, mass });
    return links.length - 1;
  };
  const join = (a: number, b: number) => joins.push({ id: joins.length, a: { kind: 'link', id: a }, b: { kind: 'link', id: b } });
  const at = (id: number): P => [points[id].x, points[id].y];
  const unit = (a: number): P => [Math.cos(a), Math.sin(a)];

  // palm: the wrist (held) and the bases of the five digits, one rigid body
  const wrist = add([0, 0], 'fixed');
  const fingers = [
    { base: [59.3, 18.1], alpha: 7.03, len: [26.4, 17.1, 16.8], width: [15, 14, 13], flex: [20, 30, 15] }, // index
    { base: [60.5, 0], alpha: 0, len: [29.7, 19.1, 18.2], width: [15, 14, 13], flex: [25, 35, 15] }, // middle
    { base: [56.1, -17.2], alpha: -5.1, len: [26.9, 18.5, 18.1], width: [15, 14, 13], flex: [30, 40, 20] }, // ring
    { base: [46.1, -31.1], alpha: -7.27, len: [21.4, 13.1, 15.7], width: [13, 12, 11], flex: [35, 45, 20] }, // little
  ];
  const thumb = { base: [12, 20], alpha: 45, len: [30.8, 22.6, 20.7], width: [25, 16, 16], flex: [0, 10, 10] };
  const palmMass = mm(60.5) * mm(18.1 + 31.1 + 15) * 0.03 * rho;
  const palmBars: number[] = [];
  const digitBase = (d: typeof thumb) => add(fromTheirs(d.base[0], d.base[1]));
  const bases = [thumb, ...fingers].map((d) => digitBase(d));
  for (const b of bases) palmBars.push(bar(wrist, b, palmMass / bases.length));
  for (const b of palmBars.slice(1)) join(palmBars[0], b);

  // A digit: a chain of three bones from its base, each joint bent by its flexion angle
  // (fingers curl clockwise, towards the thumb; the thumb anticlockwise, towards the fingers),
  // with a volar plate across each joint.
  const digit = (d: typeof thumb, base: number, behind: number, sign: 1 | -1, hyper: number[]) => {
    let dir = dirOf(d.alpha); // direction of the straight digit
    let zero = dir; // direction of the next bone when its joint is straight
    let joint = base;
    let prevStart = behind; // a point of the bone before the joint, on the digit's line
    d.len.forEach((len, k) => {
      dir += sign * d.flex[k] * deg;
      const end = add([at(joint)[0] + mm(len) * Math.cos(dir), at(joint)[1] + mm(len) * Math.sin(dir)]);
      const bone = bar(joint, end, cylinder(len, d.width[k]));
      // volar plate: from the bone before the joint to a point on this bone, 30% along it and
      // half its width to the flexion side, taut when the joint is hyperextended by hyper[k]
      const off = (u: number): P => {
        const [ux, uy] = unit(u);
        const [nx, ny] = unit(u + (sign * Math.PI) / 2);
        const [jx, jy] = at(joint);
        return [jx + 0.3 * mm(len) * ux + (mm(d.width[k]) / 2) * nx, jy + 0.3 * mm(len) * uy + (mm(d.width[k]) / 2) * ny];
      };
      const q = add(off(dir));
      join(bone, bar(joint, q, 0.001));
      const straight = off(zero - sign * hyper[k] * deg);
      const p0 = at(prevStart);
      springs.push({ id: springs.length, a: prevStart, b: q, k: k === 0 && behind !== wrist ? 4e4 : 1e4, rest: Math.hypot(straight[0] - p0[0], straight[1] - p0[1]), tensionOnly: true });
      prevStart = joint;
      joint = end;
      zero = dir;
    });
  };
  digit(thumb, bases[0], wrist, 1, [0, 0, 10]);
  fingers.forEach((f, i) => {
    // a point on the finger's metacarpal, 35 mm back from the knuckle along the finger's line,
    // fixed in the palm: the MCP volar plate pulls from here
    const [bx, by] = at(bases[i + 1]);
    const u = dirOf(f.alpha);
    const mc = add([bx - mm(35) * Math.cos(u), by - mm(35) * Math.sin(u)]);
    join(palmBars[0], bar(mc, bases[i + 1], 0.001));
    digit(f, bases[i + 1], mc, -1, [20, 0, 5]);
  });

  // wrist collateral ligaments, from the forearm to the radial and ulnar edges of the palm
  const radial = add([-mm(25), mm(10)], 'fixed');
  const ulnar = add([mm(25), mm(10)], 'fixed');
  const taut = (anchor: number, palmPoint: number, tilt: number) => {
    const p = rotateAbout([0, 0], at(palmPoint), tilt);
    const a = at(anchor);
    return Math.hypot(p[0] - a[0], p[1] - a[1]);
  };
  // ulnar tilt turns the hand anticlockwise (fingers towards +X), radial tilt clockwise
  springs.push({ id: springs.length, a: radial, b: bases[0], k: 5e4, rest: taut(radial, bases[0], 30 * deg), tensionOnly: true });
  springs.push({ id: springs.length, a: ulnar, b: bases[4], k: 5e4, rest: taut(ulnar, bases[4], -20 * deg), tensionOnly: true });

  return { points, links, springs, gears: [], meshes: [], rackMeshes: [], joins };
}

/**
 * The rimless wheel, the simplest passive walker (McGeer, "Passive dynamic walking", Int. J.
 * Robotics Research 9 (1990) 62–82). Spokes around a heavy hub roll down a slope with no
 * motor: each step the wheel falls forward onto the next spoke, and that impact (restitution 0,
 * no slip) takes away the energy the step down the slope gave, so it settles into a steady
 * gait. The spokes are joined into one rigid body.
 */
function rimlessWheel(spokes = 8, L = 0.3, slopeDeg = -12): Design {
  const alpha = Math.PI / spokes; // half the angle between spokes
  const lean = (4 * Math.PI) / 180; // standing on one spoke, leaning a little downhill
  const hub: P = [L * Math.sin(lean), L * Math.cos(lean) + 1e-4]; // the spoke just touching
  const points: DPoint[] = [pt(0, hub[0], hub[1], 'free', 4)];
  for (let k = 0; k < spokes; k++) {
    const a = -Math.PI / 2 - lean + 2 * alpha * k; // the first spoke reaches the ground at the origin
    points.push(pt(k + 1, hub[0] + L * Math.cos(a), hub[1] + L * Math.sin(a)));
  }
  const links = barsM(points.slice(1).map((p) => [0, p.id, 0.1] as [number, number, number]));
  return {
    points,
    links,
    springs: [],
    gears: [],
    meshes: [],
    rackMeshes: [],
    joins: links.slice(1).map((l, i) => ({ id: i, a: { kind: 'link' as const, id: 0 }, b: { kind: 'link' as const, id: l.id } })),
    ground: straightGround(0, slopeDeg, -3, 45, 13, 0, 1), // a long slope to walk down
  };
}

/**
 * A little motor car on two wheels over hilly ground. The rear wheel carries the motor, whose
 * housing is joined to the chassis: the chassis takes the motor's reaction, so it noses up as
 * the car pulls away and up the slopes. The wheels grip the ground through friction.
 */
export function motorCar(): Design {
  const R = 0.15;
  const y = R + 1e-4; // resting just on the ground
  return {
    points: [pt(0, 0, y), pt(1, 0.7, y), pt(2, 0.3, 0.45, 'free', 0.5)], // rear axle, front axle, cab with its load
    links: barsM([[0, 1, 1], [0, 2, 0.2], [1, 2, 0.2]]), // chassis and cab frame
    springs: [],
    gears: [
      { ...gear(0, 0, R, 0.5), wheel: true, motor: { torque: -2.5, noLoadRpm: 150, mass: 0.3, mounted: false } }, // clockwise: drives right
      { ...gear(1, 1, R, 0.5), wheel: true },
    ],
    meshes: [], rackMeshes: [],
    joins: [{ id: 0, a: { kind: 'motor', id: 0 }, b: { kind: 'link', id: 0 } }],
    ground: {
      points: [[-2, 0], [1.5, 0], [2.5, 0.25], [3.5, 0.3], [4.5, 0.05], [5.5, -0.15], [6.5, 0.2], [7.5, 0.55], [8.5, 0.6], [9.5, 0.3], [10.5, 0], [12, 0.1], [14, 0]],
      restitution: 0,
      friction: 0.9,
    },
  };
}

/**
 * A twin-rotor drone seen from the side: a frame with a motor and propeller at each end, the
 * motors joined to the frame (a propeller's motor is its hub). Each motor has a lever, set a
 * little above hover, so it lifts off the ground and climbs; pull one lever back and the
 * frame tilts and flies sideways.
 */
export function twinRotor(): Design {
  const y = 0.0001;
  const rotor = (id: number, center: number, color: string): DGear => ({
    ...gear(id, center, 0.15, 0.045),
    teeth: 2,
    propeller: { angle: 0, blades: 2 },
    motor: { torque: 0.3, noLoadRpm: 8000, mass: 0.1, mounted: false, lever: { color, scale: 0.3, position: 0.71 } },
  });
  return {
    points: [pt(0, -0.4, y), pt(1, 0.4, y), pt(2, 0, y, 'free', 0.1)], // frame ends and its middle, carrying a battery
    links: barsM([[0, 2, 0.15], [2, 1, 0.15]]),
    springs: [],
    gears: [rotor(0, 0, '#e04f5f'), rotor(1, 1, '#2f9e8f')],
    meshes: [], rackMeshes: [],
    joins: [
      { id: 0, a: { kind: 'gear', id: 0 }, b: { kind: 'link', id: 0 } },
      { id: 1, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 1 } }, // one rigid frame
      { id: 2, a: { kind: 'gear', id: 1 }, b: { kind: 'link', id: 1 } },
    ],
    ground: straightGround(0, 0, -3, 3, 7, 0.2, 0.8),
  };
}

/**
 * A robot-arm joint: a motor on a fixed axle turns a two-part arm (joined to its gear), and a PID
 * controller measuring the upper arm's angle holds it at the target its lever sets, against
 * gravity and the swinging forearm.
 */
export function pidArm(): Design {
  return {
    points: [pt(0, 0, 0, 'fixed'), pt(1, 0.8, 0), pt(2, 1.4, 0, 'free', 0.3)],
    links: barsM([[0, 1, 0.6], [1, 2, 0.2]]), // upper arm, and a forearm hanging free at the elbow with a load
    springs: [],
    gears: [{ ...gear(0, 0, 0.12, 0.3), motor: { torque: 8, noLoadRpm: 60, mass: 0.5, mounted: true, pid: { kp: 0.4, ki: 0.3, kd: 0.06, offset: 0, sensor: { body: { kind: 'link', id: 0 }, quantity: 'angle' }, color: '#7b5ce0', lever: { color: '#e04f5f', scale: 90, position: 0.5 } } } }],
    meshes: [], rackMeshes: [],
    joins: [{ id: 0, a: { kind: 'gear', id: 0 }, b: { kind: 'link', id: 0 } }],
  };
}

/** The motor car with cruise control: a PID controller on its motor holds the chassis's speed over the hills. */
export function cruiseCar(): Design {
  const d = motorCar();
  d.gears[0].motor = { torque: 3, noLoadRpm: 150, mass: 0.3, mounted: false, pid: { kp: 3, ki: 6, kd: 0, offset: 0, sensor: { body: { kind: 'link', id: 0 }, quantity: 'vx' }, color: '#2f9e8f', lever: { color: '#f2a93b', scale: 1.5, position: 0.6 } } };
  return d;
}

/** A rigid square frame (braced by a diagonal) dropped corner-first onto the floor. */
function tumblingBox(): Design {
  const s = 0.4;
  const c: P = [0, 0.8];
  const tilt = (25 * Math.PI) / 180;
  const corners = [
    [-s / 2, -s / 2],
    [s / 2, -s / 2],
    [s / 2, s / 2],
    [-s / 2, s / 2],
  ].map(([x, y], i) => pt(i, c[0] + x * Math.cos(tilt) - y * Math.sin(tilt), c[1] + x * Math.sin(tilt) + y * Math.cos(tilt)));
  return {
    points: corners,
    links: barsM([[0, 1, 0.5], [1, 2, 0.5], [2, 3, 0.5], [3, 0, 0.5], [0, 2, 0.2]]),
    springs: [],
    gears: [],
    meshes: [],
    rackMeshes: [],
    ground: straightGround(0, 0, -2, 2, 9, 0.4, 0.6),
  };
}

/** A loose chain of 12 links dropped onto the floor, many joints touching at once. */
function droppedChain(n = 12, len = 0.1): Design {
  const points: DPoint[] = [pt(0, -0.55, 0.9)];
  for (let k = 1; k <= n; k++) {
    // links heading right and a little down, along a gentle wave
    const a = -0.35 + 0.5 * Math.sin((k / n) * Math.PI * 1.5);
    const p = points[k - 1];
    points.push(pt(k, p.x + len * Math.cos(a), p.y + len * Math.sin(a)));
  }
  return {
    points,
    links: bars(points.slice(1).map((p) => [p.id - 1, p.id] as [number, number]), 0.05),
    springs: [],
    gears: [],
    meshes: [],
    rackMeshes: [],
    ground: straightGround(0, 0, -2, 2, 9, 0.1, 0.7),
  };
}

/**
 * A rack railway (Riggenbach, Marsh, Abt…): on a slope too steep for wheel friction, the
 * car's cog wheels engage a toothed rack laid between the rails. Here two cog wheels joined
 * by the chassis roll on a 20° rack; the haulage cable is a spring to the top station.
 */
function cogRailway(): Design {
  const al = (20 * Math.PI) / 180;
  const R = 0.15;
  const A: P = [-1.2, -0.7];
  const e: P = [Math.cos(al), Math.sin(al)];
  const n: P = [-e[1], e[0]];
  const along = (xi: number, h = 0): P => [A[0] + e[0] * xi + n[0] * h, A[1] + e[1] * xi + n[1] * h];
  return {
    points: [pt(0, ...A, 'fixed'), pt(1, ...along(2.6), 'fixed'), pt(2, ...along(1.3, R)), pt(3, ...along(0.8, R)), pt(4, ...along(2.6, R), 'fixed')],
    links: [{ id: 0, a: 0, b: 1, mass: 1, rack: 1 }, { id: 1, a: 3, b: 2, mass: 2 }],
    springs: [spring(0, 2, 4, 40, 0.75)],
    gears: [gear(0, 2, R, 0.5), gear(1, 3, R, 0.5)],
    meshes: [],
    rackMeshes: [{ id: 0, gear: 0, rack: 0 }, { id: 1, gear: 1, rack: 0 }],
  };
}

/**
 * A closed chain of n equal bars whose joints start on a circle of radius r (a regular n-gon),
 * pinned to the ground at its top joint. Each link is L = 2 r sin(π/n) long.
 */
function ring(n: number, r: number): Design {
  const centre: P = [0, 0.2];
  const points = Array.from({ length: n }, (_, k) => {
    const a = Math.PI / 2 + (2 * Math.PI * k) / n;
    return pt(k, centre[0] + r * Math.cos(a), centre[1] + r * Math.sin(a), k === 0 ? 'fixed' : 'free');
  });
  return {
    points,
    links: bars(Array.from({ length: n }, (_, k) => [k, (k + 1) % n] as [number, number]), 0.1),
    springs: [],
    gears: [],
    meshes: [],
    rackMeshes: [],
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
    meshes: [], rackMeshes: [],
  };
}

export interface Preset {
  name: string;
  group: 'Famous mechanisms' | 'Basics';
  build: () => Design;
  /** Seen from above, so loading it turns gravity off (other presets turn it back on). */
  topView?: boolean;
  /** Air drag (1/s) to set when loading it, standing in for tissue damping (others set 0). */
  drag?: number;
  /** Turn on "Follow the mechanism" when loading it, for things that travel (others turn it off). */
  follow?: boolean;
  /** The area to fit in view when loading it (world metres), when its motion reaches beyond its drawing. */
  view?: { x: [number, number]; y: [number, number] };
}

export const presets: Preset[] = [
  { name: 'Strandbeest leg (Jansen linkage)', group: 'Famous mechanisms', build: () => jansenLeg() },
  { name: 'Strandbeest leg driven by a crank motor', group: 'Famous mechanisms', build: () => drivenJansenLeg() },
  { name: 'Strandbeest with four legs, gear-coupled and motor-driven', group: 'Famous mechanisms', build: () => strandbeest() },
  { name: 'Quick-return mechanism (crank and slotted lever)', group: 'Famous mechanisms', build: () => quickReturn() },
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
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
    }),
  },
  { name: 'Peaucellier–Lipkin straight-line linkage', group: 'Famous mechanisms', build: () => peaucellier() },
  { name: 'Rack-and-pinion steering (top view)', group: 'Famous mechanisms', build: () => steering(), topView: true },
  { name: 'Trapezoid (Ackermann) steering linkage (top view)', group: 'Famous mechanisms', build: () => trapezoidSteering(), topView: true },
  { name: 'Cog railway car on a rack incline', group: 'Famous mechanisms', build: () => cogRailway() },
  { name: 'Human leg swing with a four-bar knee (O\'Connor 1989)', group: 'Famous mechanisms', build: () => humanLeg() },
  { name: 'Human hand, flattened (Cerruti et al. 2015 proportions)', group: 'Famous mechanisms', build: () => humanHand(), drag: 2 },
  { name: 'Rimless wheel walking down a slope (McGeer 1990)', group: 'Famous mechanisms', build: () => rimlessWheel(), follow: true },
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
      meshes: [], rackMeshes: [],
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
      meshes: [], rackMeshes: [],
    }),
  },
  {
    name: 'Simple pendulum',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.8, 'fixed'), pt(1, 0.8, 0.4, 'free', 1)], links: bars([[0, 1]], 0.2), springs: [], gears: [], meshes: [], rackMeshes: [] }),
  },
  {
    name: 'Double pendulum',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.8, 'fixed'), pt(1, 0.7, 1.0), pt(2, 1.3, 0.6)], links: bars([[0, 1], [1, 2]]), springs: [], gears: [], meshes: [], rackMeshes: [] }),
  },
  {
    name: 'Triple pendulum',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.4, 0.9, 'fixed'), pt(1, 0.2, 0.9), pt(2, 0.7, 0.9), pt(3, 1.1, 0.9)],
      links: bars([[0, 1], [1, 2], [2, 3]]), springs: [],
      gears: [],
      meshes: [], rackMeshes: [],
    }),
  },
  {
    name: 'Cart-pole (y-locked cart)',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.2, 'lockY', 2), pt(1, 0.5, -0.6, 'free', 0.5)], links: bars([[0, 1]], 0.3), springs: [], gears: [], meshes: [], rackMeshes: [] }),
  },
  {
    name: 'Slider-crank',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.9, 0, 'fixed'), pt(1, -0.6, 0.4), pt(2, 0.6, 0, 'lockY', 1)],
      links: bars([[0, 1], [1, 2]]), springs: [],
      gears: [],
      meshes: [], rackMeshes: [],
    }),
  },
  {
    name: 'Four-bar linkage',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.8, -0.3, 'fixed'), pt(1, -0.6, 0.4), pt(2, 0.5, 0.6), pt(3, 0.6, -0.3, 'fixed')],
      links: bars([[0, 1], [1, 2], [2, 3]]), springs: [],
      gears: [],
      meshes: [], rackMeshes: [],
    }),
  },
  { name: 'Box dropped onto the floor', group: 'Basics', build: () => tumblingBox() },
  { name: 'Chain dropped onto the floor', group: 'Basics', build: () => droppedChain() },
  {
    // A rod pivoted at one end, swinging, with a bead free to slide along it between two end
    // stops: the bead is flung out to the far stop and knocks against it as the rod swings.
    name: 'Bead on a swinging rod, with end stops',
    group: 'Basics',
    view: { x: [-1.1, 1.1], y: [-0.5, 1.2] },
    build: () => ({
      points: [
        pt(0, 0, 0.6, 'fixed'),
        pt(1, 0.9, 0.6),
        { ...pt(2, 0.4, 0.6, 'free', 0.3), slide: { on: { kind: 'link', id: 0 }, angle: 0, range: { min: -0.25, max: 0.4, restitution: 0.3 } } },
      ],
      links: bars([[0, 1]], 1),
      springs: [],
      gears: [],
      meshes: [],
      rackMeshes: [],
    }),
  },
  {
    name: 'Hanging chain (closed loop)',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -1, 0.8, 'fixed'), pt(1, -0.7, 0.3), pt(2, 0, 0.1), pt(3, 0.6, 0.2), pt(4, 1, 0.8, 'fixed')],
      links: bars([[0, 1], [1, 2], [2, 3], [3, 4]], 0.5), springs: [],
      gears: [],
      meshes: [], rackMeshes: [],
    }),
  },
  {
    // A closed chain of 30 equal links, started as a regular 30-gon inscribed in a circle and
    // hung from its top joint. Gravity collapses the ring into a hanging double strand.
    name: 'Ring chain of 30 links',
    group: 'Basics',
    build: () => ring(30, 0.6),
  },
  {
    name: 'Mass on a spring (horizontal rail)',
    group: 'Basics',
    build: () => ({
      points: [pt(0, -0.9, 0, 'fixed'), pt(1, 0.4, 0, 'lockY', 1)],
      links: [],
      springs: [spring(0, 0, 1, 40, 0.8)],
      gears: [],
      meshes: [], rackMeshes: [],
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
      meshes: [], rackMeshes: [],
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
      meshes: [], rackMeshes: [],
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
      meshes: [], rackMeshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }, { id: 1, a: 1, b: 2 }], rackMeshes: [],
    }),
  },
  {
    // A vertical rack on two x-locked guides, standing on a spring, meshes with a heavy pinion
    // on a fixed axle: the rack bobs up and down and the pinion turns back and forth.
    name: 'Rack and pinion on a spring',
    group: 'Basics',
    build: () => ({
      points: [pt(0, 0, 0.2, 'fixed'), pt(1, 0.25, -0.4, 'lockX'), pt(2, 0.25, 0.8, 'lockX'), pt(3, 0.25, -0.9, 'fixed')],
      links: [{ id: 0, a: 1, b: 2, mass: 1, rack: 1 }],
      springs: [spring(0, 1, 3, 60, 0.5)],
      gears: [gear(0, 0, 0.25, 1.5)],
      meshes: [],
      rackMeshes: [{ id: 0, gear: 0, rack: 0 }],
    }),
  },
  { name: 'Motor car on wheels over hills', group: 'Basics', build: () => motorCar(), follow: true },
  { name: 'Twin-rotor drone with a lever per motor', group: 'Basics', build: () => twinRotor(), follow: true },
  { name: 'Robot-arm joint held by a PID controller', group: 'Basics', build: () => pidArm() },
  { name: 'Motor car with cruise control (PID on its speed)', group: 'Basics', build: () => cruiseCar(), follow: true },
  {
    name: 'Pendulum on a vertical rail',
    group: 'Basics',
    build: () => ({ points: [pt(0, 0, 0.6, 'lockX', 1), pt(1, 0.6, 0.3)], links: bars([[0, 1]]), springs: [], gears: [], meshes: [], rackMeshes: [] }),
  },
];
