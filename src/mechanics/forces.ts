/**
 * The forces in the joints: what every pin, rail, gear mesh, rack, weld and ground support
 * carries at a state, worked out from the motion (inverse dynamics, Newton–Euler per body).
 *
 * The simulator's own multipliers λ exist only for the constraints that close loops (most joints
 * are built into the coordinates), so the joint forces are found afterwards: with q, q̇ and q̈
 * known, every body obeys
 *     m a_G = Σ F,        I_G α (+ İ ω for a two-bladed propeller) = Σ M_G,
 * and every joint (a massless pin, or a point mass) Σ F = m a. The known loads are gravity,
 * springs, motor torques (on the gear, and back on its housing), pin and rail friction, ground
 * contact, slider stops, propeller air and the mouse pull; the unknowns are the joint forces:
 *
 *   pin (a body at a joint)      2   at the joint
 *   rail (frictionless)          1   at the joint, square to the rail
 *   gear mesh                    1   at the pitch point, along the pitch circles' tangent
 *   rack mesh                    1–2 at the pitch point, along the rack (and square to it, when
 *                                    the rack holds the gear's centre distance)
 *   weld (joined bodies)         1–3 a couple, and a force where they share no joint
 *   ground (a fixed joint)       2   at the joint
 *   locked gear / mounted hub    1   a couple
 *
 * Solved together by least squares: x = (AᵀA + εI)⁻¹ Aᵀ b. The residual |Ax − b| checks that the
 * model accounts for the motion (it is not zero with mesh friction on, which is left out); a
 * rank-deficient A means redundant supports, whose split of the load a rigid model cannot fix
 * (the minimum-norm split is returned, flagged `indeterminate`).
 */
import { solveInPlace } from './linalg';
import { motorHousingRadius, propellerInertia, railOf, type DBodyRef, type Design } from './model';
import type { Loads, NumericSystem } from './simulate';

type Vec = [number, number];

/** A force on a body (or, for supports, on a joint), where it acts, with a couple for welds and locks. */
export interface ForceItem {
  /** Stable across states, for graphs: e.g. 'pin:link:3@5'. */
  key: string;
  /** What it is, e.g. 'joint B', 'tooth contact with g2', 'weld to bar 2'. */
  label: string;
  /** The body it acts on, or null for a support acting on a joint. */
  body: DBodyRef | null;
  /** The joint it acts at (or on), if any. */
  node?: number;
  at: Vec;
  F: Vec;
  M?: number;
}

export interface ForceResult {
  items: ForceItem[];
  /** RMS of the Newton–Euler equations' residual (N, N·m). */
  residual: number;
  /** Redundant supports: the split between them is the minimum-norm one, not the only one. */
  indeterminate: boolean;
}

export interface ForceSettings {
  gravity: number;
  friction: { enabled: boolean; pin: number; rail: number } | null;
}

const cross = (a: Vec, b: Vec) => a[0] * b[1] - a[1] * b[0];
const sub = (a: Vec, b: Vec): Vec => [a[0] - b[0], a[1] - b[1]];

/**
 * The joint forces of design `d` (derived as sys) at state q, q̇ with accelerations q̈ and the
 * simulator's point loads.
 */
export function jointForces(d: Design, sys: NumericSystem, q: ArrayLike<number>, qd: ArrayLike<number>, qdd: ArrayLike<number>, loads: Loads, settings: ForceSettings): ForceResult {
  const n = sys.n;
  const der = sys.derivation;
  const np = sys.pointIndex.size;
  // ---- kinematics: joint positions, velocities and accelerations (a = J q̈ + J̇ q̇, J̇ q̇ by central differences)
  const pos = new Float64Array(2 * np);
  const jac = new Float64Array(2 * n * np);
  sys.positions(q, pos);
  sys.jacobians(q, jac);
  const vel = new Float64Array(2 * np);
  const acc = new Float64Array(2 * np);
  {
    const speed = Math.sqrt(Array.from(qd).reduce((a, v) => a + v * v, 0));
    const h = speed > 0 ? 1e-5 / Math.max(1, speed) : 0;
    const jp = new Float64Array(2 * n * np);
    const jm = new Float64Array(2 * n * np);
    if (h > 0) {
      sys.jacobians(Array.from(q, (v, m) => v + h * qd[m]), jp);
      sys.jacobians(Array.from(q, (v, m) => v - h * qd[m]), jm);
    }
    for (let i = 0; i < np; i++)
      for (let ax = 0; ax < 2; ax++) {
        let [v, a] = [0, 0];
        for (let m = 0; m < n; m++) {
          const k = i * 2 * n + ax * n + m;
          v += jac[k] * qd[m];
          a += jac[k] * qdd[m] + (h > 0 ? ((jp[k] - jm[k]) / (2 * h)) * qd[m] : 0);
        }
        vel[2 * i + ax] = v;
        acc[2 * i + ax] = a;
      }
  }
  const P = (id: number): Vec | null => {
    const i = sys.pointIndex.get(id);
    return i === undefined ? null : [pos[2 * i], pos[2 * i + 1]];
  };
  const V = (id: number): Vec => {
    const i = sys.pointIndex.get(id)!;
    return [vel[2 * i], vel[2 * i + 1]];
  };
  const A = (id: number): Vec => {
    const i = sys.pointIndex.get(id)!;
    return [acc[2 * i], acc[2 * i + 1]];
  };
  const gearAngles = new Float64Array(sys.gearIds.length);
  sys.gearAngles(q, gearAngles);
  const gearTurn = (id: number) => {
    const k = sys.gearIds.indexOf(id);
    return k >= 0 ? gearAngles[k] : 0;
  };
  const gearRate = (id: number, x: ArrayLike<number>) => {
    const k = sys.gearCoord.get(id) ?? -1;
    return k >= 0 ? x[k] : 0;
  };
  const g = settings.gravity;

  // ---- bodies: mass, centre of mass and its acceleration, inertia, angular acceleration
  interface Body {
    ref: DBodyRef;
    key: string;
    m: number;
    G: Vec;
    aG: Vec;
    /** I α (+ İ ω): the moment needed about G. */
    Ia: number;
    w: number;
    nodes: Set<number>;
    /** The body's turn since the drawing (for rails on it). */
    turn: number;
  }
  const bodies: Body[] = [];
  const bodyIndex = new Map<string, number>();
  const addBody = (b: Body) => {
    bodyIndex.set(b.key, bodies.length);
    bodies.push(b);
  };
  const bodyKey = (r: DBodyRef) => `${r.kind}:${r.id}`;
  for (const l of d.links) {
    const [pa, pb] = [P(l.a), P(l.b)];
    if (!pa || !pb) continue;
    const [va, vb, aa, ab] = [V(l.a), V(l.b), A(l.a), A(l.b)];
    const dd = sub(pb, pa);
    const L2 = dd[0] ** 2 + dd[1] ** 2 || 1e-12;
    const w = cross(dd, sub(vb, va)) / L2;
    const alpha = cross(dd, sub(ab, aa)) / L2; // a_b − a_a = α k × d − ω² d
    const a0 = d.points.find((p) => p.id === l.a)!;
    const b0 = d.points.find((p) => p.id === l.b)!;
    addBody({
      ref: { kind: 'link', id: l.id },
      key: `link:${l.id}`,
      m: l.mass,
      G: [(pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2],
      aG: [(aa[0] + ab[0]) / 2, (aa[1] + ab[1]) / 2],
      Ia: ((l.mass * L2) / 12) * alpha,
      w,
      nodes: new Set([l.a, l.b]),
      turn: Math.atan2(dd[1], dd[0]) - Math.atan2(b0.y - a0.y, b0.x - a0.x),
    });
  }
  const housings = new Set<number>(); // gears whose motor housing is a body of its own
  const joinsOf = (d.joins ?? []).map((j) => {
    // a propeller's motor is its hub
    const asBody = (r: DBodyRef): DBodyRef => (r.kind === 'motor' && d.gears.find((x) => x.id === r.id)?.propeller ? { kind: 'gear', id: r.id } : r);
    return { id: j.id, a: asBody(j.a), b: asBody(j.b) };
  });
  const joinedHousing = (id: number) => joinsOf.some((j) => [j.a, j.b].some((r) => r.kind === 'motor' && r.id === id));
  const motorAngleIdx = new Map(der.motorAngles.map((m, i) => [m.id, i]));
  const motorAngles = new Float64Array(der.motorAngles.length);
  sys.motorAngles(q, motorAngles);
  // a housing's turn is linear in the coordinates: its gradient by differences, for ψ̇ and ψ̈
  const housingRates = (id: number): [number, number] => {
    const k = motorAngleIdx.get(id);
    if (k === undefined) return [0, 0];
    const buf = new Float64Array(der.motorAngles.length);
    let [w, a] = [0, 0];
    for (let m = 0; m < n; m++) {
      const qq = Array.from(q);
      qq[m] += 1e-6;
      sys.motorAngles(qq, buf);
      const grad = (buf[k] - motorAngles[k]) / 1e-6;
      w += grad * qd[m];
      a += grad * qdd[m];
    }
    return [w, a];
  };
  for (const gr of d.gears) {
    const c = P(gr.center);
    if (!c || !sys.gearIds.includes(gr.id)) continue;
    const nodes = new Set([gr.center, ...d.points.filter((p) => p.gear === gr.id || (p.pin?.kind === 'gear' && p.pin.id === gr.id)).map((p) => p.id)]);
    const alpha = gearRate(gr.id, qdd);
    const w = gearRate(gr.id, qd);
    if (gr.propeller) {
      // the 3D rotor in the plane: yaw inertia I₂ sin²φ + I₃ cos²φ, changing as the blades spin
      const { I2, I3 } = propellerInertia(gr);
      const pr = der.propellers.find((x) => x.id === gr.id);
      const [phi, phid] = pr && pr.spin >= 0 ? [q[pr.spin], qd[pr.spin]] : [0, 0];
      const I = I2 * Math.sin(phi) ** 2 + I3 * Math.cos(phi) ** 2;
      const Idot = (I2 - I3) * Math.sin(2 * phi) * phid;
      addBody({ ref: { kind: 'gear', id: gr.id }, key: `gear:${gr.id}`, m: gr.mass + (gr.motor ? (gr.motor.mass ?? 0.5) : 0), G: c, aG: A(gr.center), Ia: I * alpha + Idot * w, w, nodes, turn: gearTurn(gr.id) });
      continue;
    }
    addBody({ ref: { kind: 'gear', id: gr.id }, key: `gear:${gr.id}`, m: gr.mass, G: c, aG: A(gr.center), Ia: 0.5 * gr.mass * gr.radius ** 2 * alpha, w, nodes, turn: gearTurn(gr.id) });
    // its motor's housing: a body unless mounted on the ground (and not joined to anything)
    if (gr.motor && (gr.motor.mounted === false || joinedHousing(gr.id))) {
      const mM = gr.motor.mass ?? 0.5;
      const [wM, aM] = housingRates(gr.id);
      housings.add(gr.id);
      addBody({ ref: { kind: 'motor', id: gr.id }, key: `motor:${gr.id}`, m: mM, G: c, aG: A(gr.center), Ia: 0.5 * mM * motorHousingRadius(gr) ** 2 * aM, w: wM, nodes: new Set([gr.center]), turn: 0 });
    }
  }
  const nodeIds = [...sys.pointIndex.keys()];
  const nodeRow = new Map(nodeIds.map((id, k) => [id, 3 * bodies.length + 2 * k]));
  const rows = 3 * bodies.length + 2 * nodeIds.length;

  // ---- the right-hand side: m a − known forces (rows: Fx, Fy, M_G per body; Fx, Fy per joint)
  const b = new Float64Array(rows);
  bodies.forEach((B, k) => {
    b[3 * k] = B.m * B.aG[0];
    b[3 * k + 1] = B.m * B.aG[1] + B.m * g; // gravity, −m g, known
    b[3 * k + 2] = B.Ia;
  });
  for (const id of nodeIds) {
    const p = d.points.find((x) => x.id === id);
    const r = nodeRow.get(id)!;
    const m = p?.mass ?? 0;
    const a = A(id);
    b[r] = m * a[0];
    b[r + 1] = m * a[1] + m * g;
  }
  /** A known force on body k at point `at` (subtracted from the right-hand side). */
  const knownOnBody = (k: number, at: Vec, F: Vec, M = 0) => {
    b[3 * k] -= F[0];
    b[3 * k + 1] -= F[1];
    b[3 * k + 2] -= cross(sub(at, bodies[k].G), F) + M;
  };
  const knownOnNode = (id: number, F: Vec) => {
    const r = nodeRow.get(id);
    if (r === undefined) return;
    b[r] -= F[0];
    b[r + 1] -= F[1];
  };
  // springs, pulling their ends together (or apart)
  for (const sp of d.springs) {
    const [pa, pb] = [P(sp.a), P(sp.b)];
    if (!pa || !pb) continue;
    const dd = sub(pb, pa);
    const L = Math.hypot(...dd) || 1e-12;
    let F = sp.k * (L - sp.rest);
    if (sp.tensionOnly && F < 0) F = 0;
    const u: Vec = [dd[0] / L, dd[1] / L];
    knownOnNode(sp.a, [F * u[0], F * u[1]]);
    knownOnNode(sp.b, [-F * u[0], -F * u[1]]);
  }
  // motors: +τ on the gear, −τ on its housing (a mounted one passes it to the ground)
  const paramIndex = new Map(der.params.map((p, i) => [p.sym, i]));
  for (const mo of der.motors) {
    const gr = d.gears.find((x) => x.id === mo.id);
    if (!gr || gr.propeller) continue; // a propeller's motor spins its blades about an axis in the plane
    let rho = 0;
    for (const t of mo.terms) rho += t.k * qd[t.coord];
    const tau = sys.params[paramIndex.get(mo.tau)!] - (mo.b ? sys.params[paramIndex.get(mo.b)!] * rho : 0);
    const kg = bodyIndex.get(`gear:${gr.id}`);
    if (kg !== undefined) knownOnBody(kg, bodies[kg].G, [0, 0], tau);
    const kh = bodyIndex.get(`motor:${gr.id}`);
    if (kh !== undefined) knownOnBody(kh, bodies[kh].G, [0, 0], -tau);
  }
  // friction: pin torques between the bodies meeting at a joint; drag along ground rails
  if (settings.friction?.enabled) {
    const fr = settings.friction;
    for (const jt of der.joints) {
      const parts: { k: number | null; w: number }[] = [
        ...jt.bars.map((x) => ({ k: bodyIndex.get(`link:${x.link}`) ?? null, w: bodyIndex.has(`link:${x.link}`) ? bodies[bodyIndex.get(`link:${x.link}`)!].w : 0 })),
        ...jt.gears.map((gid) => ({ k: bodyIndex.get(`gear:${gid}`) ?? null, w: gearRate(gid, qd) })),
        ...(jt.ground ? [{ k: null, w: 0 }] : []),
      ];
      if (fr.pin > 0)
        for (let i = 0; i < parts.length; i++)
          for (let j = i + 1; j < parts.length; j++) {
            const T = fr.pin * (parts[i].w - parts[j].w);
            if (parts[i].k !== null) knownOnBody(parts[i].k!, bodies[parts[i].k!].G, [0, 0], -T);
            if (parts[j].k !== null) knownOnBody(parts[j].k!, bodies[parts[j].k!].G, [0, 0], T);
          }
      if (jt.rail !== null && fr.rail > 0) {
        const t: Vec = [Math.cos(jt.rail), Math.sin(jt.rail)];
        const v = V(jt.id);
        const f = -fr.rail * (v[0] * t[0] + v[1] * t[1]);
        knownOnNode(jt.id, [f * t[0], f * t[1]]);
      }
    }
  }
  // the simulator's point loads: contact, air, the mouse pull
  const linkByEnds = (a: number, c: number) => d.links.find((l) => (l.a === a && l.b === c) || (l.a === c && l.b === a));
  for (const pl of loads.points) {
    if (pl.on.kind === 'node') knownOnNode(pl.on.id, pl.F);
    else {
      const ref: DBodyRef | undefined = pl.on.kind === 'seg' ? (() => {
        const l = linkByEnds((pl.on as { a: number }).a, (pl.on as { b: number }).b);
        return l ? ({ kind: 'link', id: l.id } as DBodyRef) : undefined;
      })() : { kind: pl.on.kind, id: pl.on.id };
      const k = ref ? bodyIndex.get(bodyKey(ref)) : undefined;
      if (k !== undefined) knownOnBody(k, pl.at, pl.F);
    }
  }

  // ---- the unknowns, column by column: how each one enters the equations
  interface Col {
    entries: [number, number][];
  }
  const cols: Col[] = [];
  const col = (entries: [number, number][]) => {
    cols.push({ entries });
    return cols.length - 1;
  };
  /** A force component along unit `u` acting on body k at `at` (and, optionally, the opposite on joint `node`). */
  const forceOnBody = (k: number, at: Vec, u: Vec, sign = 1): [number, number][] => [
    [3 * k, sign * u[0]],
    [3 * k + 1, sign * u[1]],
    [3 * k + 2, sign * cross(sub(at, bodies[k].G), u)],
  ];
  const forceOnNode = (id: number, u: Vec, sign = 1): [number, number][] => {
    const r = nodeRow.get(id)!;
    return [
      [r, sign * u[0]],
      [r + 1, sign * u[1]],
    ];
  };
  const X: Vec = [1, 0];
  const Y: Vec = [0, 1];
  /** Readers that turn the solution into force items. */
  const readers: ((x: Float64Array) => ForceItem)[] = [];
  const pointName = (id: number) => d.points.find((p) => p.id === id);
  const name = (r: DBodyRef) => (r.kind === 'link' ? `bar ${r.id}` : r.kind === 'motor' ? `motor ${r.id}` : `gear ${r.id}`);

  // pins: each body at each of its joints
  bodies.forEach((B, k) => {
    // joints pinned onto this body (p.pin), besides its own
    for (const p of d.points) if (p.pin && bodyKey(p.pin) === B.key) B.nodes.add(p.id);
    for (const id of B.nodes) {
      const at = P(id);
      if (!at || !nodeRow.has(id)) continue;
      const cx = col([...forceOnBody(k, at, X), ...forceOnNode(id, X, -1)]);
      const cy = col([...forceOnBody(k, at, Y), ...forceOnNode(id, Y, -1)]);
      readers.push((x) => ({ key: `pin:${B.key}@${id}`, label: `joint`, body: B.ref, node: id, at, F: [x[cx], x[cy]] }));
    }
  });
  // supports: fixed joints, rails
  for (const id of nodeIds) {
    const p = pointName(id)!;
    const at = P(id)!;
    if (p.kind === 'fixed') {
      const cx = col(forceOnNode(id, X));
      const cy = col(forceOnNode(id, Y));
      readers.push((x) => ({ key: `ground@${id}`, label: 'ground support', body: null, node: id, at, F: [x[cx], x[cy]] }));
      continue;
    }
    const rail = railOf(p);
    if (!rail) continue;
    const onBody = rail.on ? bodyIndex.get(bodyKey(rail.on)) : undefined;
    const a = (rail.angle * Math.PI) / 180 + (onBody !== undefined ? bodies[onBody].turn : 0);
    const nrm: Vec = [-Math.sin(a), Math.cos(a)];
    const t: Vec = [Math.cos(a), Math.sin(a)];
    // slider stops push along the rail (and back on the body it is on)
    for (const st of loads.stops)
      if (st.joint === id) {
        knownOnNode(id, [st.along * t[0], st.along * t[1]]);
        if (onBody !== undefined) knownOnBody(onBody, at, [-st.along * t[0], -st.along * t[1]]);
      }
    const c = col([...forceOnNode(id, nrm), ...(onBody !== undefined ? forceOnBody(onBody, at, nrm, -1) : [])]);
    readers.push((x) => ({ key: `rail@${id}`, label: onBody !== undefined ? `rail on ${name(bodies[onBody].ref)}` : 'ground rail', body: null, node: id, at, F: [x[c] * nrm[0], x[c] * nrm[1]] }));
    if (onBody !== undefined)
      readers.push((x) => ({ key: `brail:${bodies[onBody].key}@${id}`, label: 'slider on its rail', body: bodies[onBody].ref, node: id, at, F: [-x[c] * nrm[0], -x[c] * nrm[1]] }));
  }
  // a locked gear, or a propeller whose hub is mounted: a couple from the ground
  for (const gr of d.gears) {
    const k = bodyIndex.get(`gear:${gr.id}`);
    if (k === undefined) continue;
    const held = gr.propeller ? !!gr.motor && gr.motor.mounted !== false && !joinsOf.some((j) => [j.a, j.b].some((r) => r.kind === 'gear' && r.id === gr.id)) : gr.locked;
    if (!held) continue;
    const c = col([[3 * k + 2, 1]]);
    readers.push((x) => ({ key: `lock:${gr.id}`, label: 'held by the ground', body: bodies[k].ref, at: bodies[k].G, F: [0, 0], M: x[c] }));
  }
  // gear meshes: a tangential tooth force at the pitch point
  for (const mesh of d.meshes) {
    const [ka, kb] = [bodyIndex.get(`gear:${mesh.a}`), bodyIndex.get(`gear:${mesh.b}`)];
    const [ga, gb] = [d.gears.find((x) => x.id === mesh.a), d.gears.find((x) => x.id === mesh.b)];
    if (ka === undefined || kb === undefined || !ga || !gb || ga.wheel || gb.wheel) continue;
    if (!der.meshes.some((m) => m.id === mesh.id && m.kind === 'gear')) continue; // ignored by the derivation
    const [ca, cb] = [bodies[ka].G, bodies[kb].G];
    const dd = sub(cb, ca);
    const L = Math.hypot(...dd) || 1e-12;
    const u: Vec = [dd[0] / L, dd[1] / L];
    const at: Vec = [ca[0] + u[0] * ga.radius, ca[1] + u[1] * ga.radius];
    const t: Vec = [-u[1], u[0]];
    const c = col([...forceOnBody(ka, at, t), ...forceOnBody(kb, at, t, -1)]);
    readers.push((x) => ({ key: `mesh:${mesh.id}:a`, label: `teeth on g${gb.id}`, body: bodies[ka].ref, at, F: [x[c] * t[0], x[c] * t[1]] }));
    readers.push((x) => ({ key: `mesh:${mesh.id}:b`, label: `teeth on g${ga.id}`, body: bodies[kb].ref, at, F: [-x[c] * t[0], -x[c] * t[1]] }));
  }
  // rack meshes: along the rack at the pitch point, and square to it when the rack holds the centre
  for (const rm of d.rackMeshes) {
    const kg = bodyIndex.get(`gear:${rm.gear}`);
    const kr = bodyIndex.get(`link:${rm.rack}`);
    const rack = d.links.find((l) => l.id === rm.rack);
    const gr = d.gears.find((x) => x.id === rm.gear);
    if (kg === undefined || kr === undefined || !rack || !gr || gr.wheel) continue;
    if (!der.meshes.some((m) => m.id === rm.id && m.kind === 'rack')) continue;
    const [pa, pb] = [P(rack.a)!, P(rack.b)!];
    const dd = sub(pb, pa);
    const L = Math.hypot(...dd) || 1e-12;
    const e: Vec = [dd[0] / L, dd[1] / L];
    const c0 = bodies[kg].G;
    const s = (c0[0] - pa[0]) * e[0] + (c0[1] - pa[1]) * e[1];
    const at: Vec = [pa[0] + s * e[0], pa[1] + s * e[1]];
    const nr: Vec = [-e[1], e[0]];
    const ct = col([...forceOnBody(kg, at, e), ...forceOnBody(kr, at, e, -1)]);
    const holds = der.constraints.some((c) => c.rackDistance && c.rackMesh === rm.id);
    const cn = holds ? col([...forceOnBody(kg, at, nr), ...forceOnBody(kr, at, nr, -1)]) : -1;
    const F = (x: Float64Array): Vec => [x[ct] * e[0] + (cn >= 0 ? x[cn] * nr[0] : 0), x[ct] * e[1] + (cn >= 0 ? x[cn] * nr[1] : 0)];
    readers.push((x) => ({ key: `rack:${rm.id}:g`, label: `teeth on rack ${rack.id}`, body: bodies[kg].ref, at, F: F(x) }));
    readers.push((x) => {
      const f = F(x);
      return { key: `rack:${rm.id}:r`, label: `teeth on g${gr.id}`, body: bodies[kr].ref, at, F: [-f[0], -f[1]] };
    });
  }
  // welds: joined bodies hold each other with a couple, and a force where they share no joint
  for (const j of joinsOf) {
    const [ka, kb] = [bodyIndex.get(bodyKey(j.a)), bodyIndex.get(bodyKey(j.b))];
    if (ka === undefined || kb === undefined || ka === kb) continue;
    const [A0, B0] = [bodies[ka], bodies[kb]];
    const shared = [...A0.nodes].some((id) => B0.nodes.has(id));
    const at: Vec = [(A0.G[0] + B0.G[0]) / 2, (A0.G[1] + B0.G[1]) / 2];
    const cm = col([
      [3 * ka + 2, 1],
      [3 * kb + 2, -1],
    ]);
    const [cx, cy] = shared ? [-1, -1] : [col([...forceOnBody(ka, at, X), ...forceOnBody(kb, at, X, -1)]), col([...forceOnBody(ka, at, Y), ...forceOnBody(kb, at, Y, -1)])];
    const Fv = (x: Float64Array): Vec => (shared ? [0, 0] : [x[cx], x[cy]]);
    readers.push((x) => ({ key: `weld:${j.id}:a`, label: `weld to ${name(B0.ref)}`, body: A0.ref, at, F: Fv(x), M: x[cm] }));
    readers.push((x) => {
      const f = Fv(x);
      return { key: `weld:${j.id}:b`, label: `weld to ${name(A0.ref)}`, body: B0.ref, at, F: [-f[0], -f[1]], M: -x[cm] };
    });
  }

  // ---- solve A x = b by least squares (minimum norm where the supports are redundant)
  const U = cols.length;
  const AtA = new Float64Array(U * U);
  const Atb = new Float64Array(U);
  const dense = cols.map((c) => {
    const v = new Float64Array(rows);
    for (const [r, val] of c.entries) v[r] += val;
    return v;
  });
  for (let i = 0; i < U; i++) {
    let s = 0;
    for (let r = 0; r < rows; r++) s += dense[i][r] * b[r];
    Atb[i] = s;
    for (let j = i; j < U; j++) {
      let t = 0;
      for (let r = 0; r < rows; r++) t += dense[i][r] * dense[j][r];
      AtA[i * U + j] = AtA[j * U + i] = t;
    }
  }
  let trace = 0;
  for (let i = 0; i < U; i++) trace += AtA[i * U + i];
  const eps = Math.max(1e-12 * (trace / Math.max(U, 1)), 1e-15);
  // redundant supports: AᵀA is singular (a pivot vanishes)
  const indeterminate = rankDeficient(AtA, U, 1e-9 * (trace / Math.max(U, 1)));
  const K = Float64Array.from(AtA);
  for (let i = 0; i < U; i++) K[i * U + i] += eps;
  const x = Float64Array.from(Atb);
  if (U && !solveInPlace(K, x, U)) x.fill(NaN);
  let res = 0;
  for (let r = 0; r < rows; r++) {
    let s = -b[r];
    for (let i = 0; i < U; i++) s += dense[i][r] * x[i];
    res += s * s;
  }
  return { items: readers.map((rd) => rd(x)), residual: Math.sqrt(res / Math.max(rows, 1)), indeterminate };
}

/** Whether a symmetric matrix has a pivot below tol (Gaussian elimination with partial pivoting). */
function rankDeficient(M0: Float64Array, n: number, tol: number): boolean {
  const M = Float64Array.from(M0);
  for (let k = 0; k < n; k++) {
    let piv = k;
    for (let i = k + 1; i < n; i++) if (Math.abs(M[i * n + k]) > Math.abs(M[piv * n + k])) piv = i;
    if (Math.abs(M[piv * n + k]) < tol) return true;
    if (piv !== k)
      for (let j = 0; j < n; j++) {
        const t = M[k * n + j];
        M[k * n + j] = M[piv * n + j];
        M[piv * n + j] = t;
      }
    for (let i = k + 1; i < n; i++) {
      const f = M[i * n + k] / M[k * n + k];
      if (f) for (let j = k; j < n; j++) M[i * n + j] -= f * M[k * n + j];
    }
  }
  return false;
}
