/**
 * Automatic derivation of the equations of motion of a planar linkage.
 *
 * 1. Choose generalized coordinates from a spanning forest of the joint graph:
 *    each tree bar gets its absolute angle θ, and each component that is not
 *    pinned to the ground gets the free coordinate(s) of one root joint.
 * 2. Express every joint position r_P(q) symbolically.
 * 3. Kinetic energy of a uniform bar with end velocities v_A, v_B is
 *    T = m/6 (v_A·v_A + v_A·v_B + v_B·v_B); joint masses add ½ m v².
 *    Potential energy is gravitational.
 * 4. Lagrange's equations in mass-matrix form, with q and q̇ independent:
 *    M = ∂²L/∂q̇²,  f = ∂L/∂q − (∂²L/∂q̇∂q) q̇,   M q̈ = f + Q + Φ_qᵀ λ.
 * 5. Bars that close a loop, and locked axes on joints reached through the
 *    tree, become holonomic constraints Φ(q) = 0 (Lagrange multipliers).
 * 6. Springs add V = ½ k (ℓ − ℓ₀)² with ℓ = √D, D = Δx² + Δy². The square root is
 *    outside the trig-polynomial algebra, so D and ½ ∂D/∂q are kept symbolic and
 *    the exact generalized force −∂V/∂q = −k (1 − ℓ₀/ℓ) · ½ ∂D/∂q is evaluated
 *    numerically from them.
 * 7. A gear is a rigid disk with its own rotation φ (a coordinate unless the gear is
 *    locked). Its centre and pins sit at r_c + Rot(φ)(u, v), so bars attach to it like
 *    to any joint; it adds ½ m v_c² + ½ (½ m R²) φ̇² to T and m g y_c to V. A joint that is
 *    reached twice through gears closes a loop: r_P(q) = r_P'(q), two constraints.
 * 8. Two gears in mesh add N_a(φ_a − θ_c) + N_b(φ_b − θ_c) = C, θ_c being the direction of
 *    the line of centres (constant, a carrier bar, or a common gear). A carrier bar whose
 *    angle is not a coordinate gets an explicit angle θ and r_b − r_a = L(cos θ, sin θ).
 * 9. A rack is a bar with teeth; its pitch line runs through its two joints. A gear meshing
 *    with it rolls on that line: with e the rack direction, n = k × e, ξ = (r_c − r_a)·e and
 *    s = ±1 the side the gear is on, rolling without slip gives ξ + s R (φ − θ_rack) = C, and
 *    the centre stays at (r_c − r_a)·n = s R. The second constraint is dropped when the rest
 *    of the mechanism already enforces it (a rack on rails meshing with a pinion on an axle).
 */
import { defineSymbol, Poly, timeDerivative, trigSimplify } from '../cas/poly';
import { gearLabel, linkLabel, pointLabel, springLabel, type Design, type DLink, type DPoint } from './model';

export interface Coordinate {
  sym: string;
  vel: string;
  latex: string;
  /** LaTeX base symbol and subscript, e.g. '\\theta' and '1'. */
  base: string;
  sub: string;
  /** What the coordinate measures. */
  kind: 'angle' | 'x' | 'y';
  initial: number;
}

export interface Parameter {
  sym: string;
  latex: string;
  value: number;
  unit: string;
}

export interface Constraint {
  phi: Poly;
  /** Short human-readable origin, e.g. "bar 3 closes a loop". */
  reason: string;
  /** For loop closures Φ = Δx² + Δy² − L², kept unexpanded for display. */
  loop?: { dx: Poly; dy: Poly; L: Poly; bar: number };
  /** For gear meshes Φ = N_a(θ_a − θ_c) + N_b(θ_b − θ_c) − C, kept for display. */
  gear?: { Na: Poly; Nb: Poly; ta: Poly; tb: Poly; tc: Poly; C: Poly };
  /** For a rack mesh Φ = ξ + sR (φ − θ_rack) − C, kept for display. */
  rack?: { xi: Poly; sR: Poly; rel: Poly; C: Poly };
  /** A rack's centre-distance constraint, dropped when the mechanism already enforces it. */
  rackDistance?: boolean;
}

/** A spring's contribution V = ½ k (ℓ − ℓ₀)², ℓ = √D. */
export interface SpringTerm {
  id: number;
  /** Parameter symbols of the stiffness and rest length. */
  k: string;
  rest: string;
  /** LaTeX for k, ℓ, ℓ₀ of this spring. */
  kLatex: string;
  lLatex: string;
  restLatex: string;
  dx: Poly;
  dy: Poly;
  /** D = ℓ² = Δx² + Δy². */
  D: Poly;
  /** ½ ∂D/∂q_i = ℓ ∂ℓ/∂q_i for each coordinate. */
  grad: Poly[];
}

/** The bodies that meet at a joint, for joint friction. */
export interface JointBodies {
  id: number;
  /** Bars ending here, each with the joint at its other end (its angle is measured towards it). */
  bars: { link: number; other: number }[];
  /** Gears this joint belongs to (as centre or pin) that can turn. */
  gears: number[];
  /** The ground or a slider block: a body that does not rotate. */
  ground: boolean;
  /** For a joint sliding on a rail, the free direction. */
  rail: 'x' | 'y' | null;
}

/**
 * A gear mesh, for tooth friction: ∂v/∂q̇ of the pitch-line speed v = R_a(φ̇_a − θ̇_c).
 * For a rack mesh (kind 'rack'), a is the gear, b the rack, and v = R(φ̇ − θ̇_rack).
 */
export interface MeshInfo {
  id: number;
  kind: 'gear' | 'rack';
  a: number;
  b: number;
  pitchGrad: Poly[];
}

export interface Derivation {
  coords: Coordinate[];
  params: Parameter[];
  /** Rotation angle φ of every gear (zero for a locked gear), for drawing. */
  gearAngles: { id: number; angle: Poly }[];
  /** Symbolic position of every active joint. */
  positions: Map<number, [Poly, Poly]>;
  T: Poly;
  V: Poly;
  M: Poly[][];
  f: Poly[];
  constraints: Constraint[];
  springs: SpringTerm[];
  /** Φ_q (nc × n). */
  phiQ: Poly[][];
  /** γ = −(Φ_q q̇)_q q̇, so that Φ_q q̈ = γ. */
  gamma: Poly[];
  activePoints: number[];
  activeLinks: number[];
  /** Topology for friction (Rayleigh dissipation). */
  joints: JointBodies[];
  meshes: MeshInfo[];
  warnings: string[];
}

export interface DeriveOptions {
  gravity: number; // m/s², 0 disables gravity entirely
}

export function deriveEquations(design: Design, opts: DeriveOptions): Derivation {
  const steps = deriveSteps(design, opts);
  for (;;) {
    const r = steps.next();
    if (r.done) return r.value;
  }
}

/**
 * The derivation as a generator: it yields its progress (0 → 1) between pieces of work, so the
 * app can run it a slice at a time and show a progress bar, and returns the Derivation.
 */
export function* deriveSteps(design: Design, opts: DeriveOptions): Generator<number, Derivation, void> {
  const warnings: string[] = [];
  const pointById = new Map(design.points.map((p) => [p.id, p]));
  const links = design.links.filter((l) => l.a !== l.b && pointById.has(l.a) && pointById.has(l.b));
  const springs = (design.springs ?? []).filter((s) => s.a !== s.b && pointById.has(s.a) && pointById.has(s.b));
  // Gears: rigid bodies made of a centre joint plus the joints pinned onto them.
  const gears = (design.gears ?? []).filter((gr) => pointById.has(gr.center));
  const gearById = new Map(gears.map((gr) => [gr.id, gr]));
  const gearJoints = new Map<number, number[]>(gears.map((gr) => [gr.id, [gr.center]]));
  for (const p of design.points) if (p.gear !== undefined && gearById.has(p.gear) && gearById.get(p.gear)!.center !== p.id) gearJoints.get(p.gear)!.push(p.id);
  const gearsAt = new Map<number, number[]>();
  for (const [gid, ids] of gearJoints) for (const id of ids) gearsAt.set(id, [...(gearsAt.get(id) ?? []), gid]);

  const bodyIds = new Set([...links.flatMap((l) => [l.a, l.b]), ...gearsAt.keys()]);
  // A joint carrying a mass is a particle even when nothing else is attached to it.
  const activeIds = new Set([...bodyIds, ...springs.flatMap((s) => [s.a, s.b]), ...design.points.filter((p) => p.mass > 0).map((p) => p.id)]);
  const active = design.points.filter((p) => activeIds.has(p.id)).sort((a, b) => a.id - b.id);

  const adjacency = new Map<number, DLink[]>();
  for (const l of links) {
    for (const end of [l.a, l.b]) {
      if (!adjacency.has(end)) adjacency.set(end, []);
      adjacency.get(end)!.push(l);
    }
  }

  // ---- parameters --------------------------------------------------------
  const params: Parameter[] = [];
  const param = (sym: string, latex: string, value: number, unit: string): Poly => {
    defineSymbol(sym, latex, 'param');
    if (!params.some((q) => q.sym === sym)) params.push({ sym, latex, value, unit });
    return Poly.sym(sym);
  };
  const linkMass = new Map<number, Poly>();
  const linkLen = new Map<number, Poly>();
  for (const l of links) {
    const a = pointById.get(l.a)!;
    const b = pointById.get(l.b)!;
    const lab = linkLabel(l.id);
    linkMass.set(l.id, param(`m${l.id}`, `m_{${lab}}`, Math.max(l.mass, 1e-6), 'kg'));
    linkLen.set(l.id, param(`L${l.id}`, `L_{${lab}}`, Math.hypot(b.x - a.x, b.y - a.y), 'm'));
  }
  const pointMass = new Map<number, Poly>();
  for (const p of active) if (p.mass > 0) pointMass.set(p.id, param(`mp${p.id}`, `m_{${pointLabel(p.id)}}`, p.mass, 'kg'));
  const g = opts.gravity !== 0 ? param('g', 'g', opts.gravity, 'm/s²') : null;
  const anchorX = (p: DPoint) => param(`X${p.id}`, `X_{${pointLabel(p.id)}}`, p.x, 'm');
  const anchorY = (p: DPoint) => param(`Y${p.id}`, `Y_{${pointLabel(p.id)}}`, p.y, 'm');
  const gearMass = new Map<number, Poly>();
  const gearRadius = new Map<number, Poly>();
  const gearTeeth = new Map<number, Poly>();
  for (const gr of gears) {
    const lab = gearLabel(gr.id);
    gearMass.set(gr.id, param(`mg${gr.id}`, `m_{\\mathrm{g}${lab}}`, Math.max(gr.mass, 1e-6), 'kg'));
    gearRadius.set(gr.id, param(`R${gr.id}`, `R_{${lab}}`, gr.radius, 'm'));
    gearTeeth.set(gr.id, param(`N${gr.id}`, `N_{${lab}}`, gr.teeth, 'teeth'));
  }
  /** Offset (u, v) of a joint from its gear's centre, in the gear's frame at φ = 0. */
  const pinOffset = (gid: number, pid: number): [Poly, Poly] => {
    const c = pointById.get(gearById.get(gid)!.center)!;
    if (pid === c.id) return [Poly.zero, Poly.zero];
    const p = pointById.get(pid)!;
    const lab = pointLabel(pid);
    return [param(`u${pid}`, `u_{${lab}}`, p.x - c.x, 'm'), param(`v${pid}`, `v_{${lab}}`, p.y - c.y, 'm')];
  };

  // ---- coordinates and joint positions -------------------------------------
  const coords: Coordinate[] = [];
  const coord = (sym: string, base: string, sub: string, kind: Coordinate['kind'], initial: number): Poly => {
    const vel = `${sym}_d`;
    const latex = `${base}_{${sub}}`;
    defineSymbol(sym, latex, 'coord');
    defineSymbol(vel, `\\dot{${base}}_{${sub}}`, 'vel');
    coords.push({ sym, vel, latex, base, sub, kind, initial });
    return Poly.sym(sym);
  };

  const positions = new Map<number, [Poly, Poly]>();
  const constraints: Constraint[] = [];
  const queue: number[] = [];
  const visitedLink = new Set<number>();
  /** Bars whose absolute angle is a coordinate, and that coordinate's symbol. */
  const barAngle = new Map<number, string>();
  /** Rotation of every placed gear (Poly.zero when locked). */
  const gearAngle = new Map<number, Poly>();

  for (const p of active) {
    if (p.kind === 'fixed') {
      positions.set(p.id, [anchorX(p), anchorY(p)]);
      queue.push(p.id);
    } else if (!bodyIds.has(p.id) && p.mass <= 0) {
      // Only springs touch it and it has no mass: it cannot have dynamics of its own.
      positions.set(p.id, [anchorX(p), anchorY(p)]);
      warnings.push(`Joint ${pointLabel(p.id)} has no mass and no bars, so it is held in place as a spring anchor.`);
    }
  }

  const lockConstraints = (p: DPoint, [x, y]: [Poly, Poly], via: string) => {
    if (p.kind === 'fixed' || p.kind === 'lockX')
      constraints.push({ phi: x.sub(anchorX(p)), reason: `joint ${pointLabel(p.id)}: x locked (${via})` });
    if (p.kind === 'fixed' || p.kind === 'lockY')
      constraints.push({ phi: y.sub(anchorY(p)), reason: `joint ${pointLabel(p.id)}: y locked (${via})` });
  };

  /** Place a whole gear from one of its joints that is already placed. */
  const placeGear = (gid: number, fromPid: number) => {
    const gr = gearById.get(gid)!;
    const lab = gearLabel(gid);
    let cos = Poly.const(1);
    let sin = Poly.zero;
    let phi = Poly.zero;
    if (!gr.locked) {
      const sym = `ph${gid}`;
      phi = coord(sym, '\\varphi', lab, 'angle', 0);
      cos = Poly.cos([[sym, 1]]);
      sin = Poly.sin([[sym, 1]]);
    }
    gearAngle.set(gid, phi);
    const rot = ([u, v]: [Poly, Poly]): [Poly, Poly] => [u.mul(cos).sub(v.mul(sin)), u.mul(sin).add(v.mul(cos))];
    const [fx, fy] = positions.get(fromPid)!;
    const [fu, fv] = rot(pinOffset(gid, fromPid));
    const cx = fx.sub(fu);
    const cy = fy.sub(fv);
    for (const pid of gearJoints.get(gid)!) {
      if (pid === fromPid) continue;
      const [ou, ov] = rot(pinOffset(gid, pid));
      const pos: [Poly, Poly] = [cx.add(ou), cy.add(ov)];
      const p = pointById.get(pid)!;
      if (!positions.has(pid)) {
        positions.set(pid, pos);
        if (p.kind !== 'free') lockConstraints(p, pos, `on gear ${lab}`);
        queue.push(pid);
      } else {
        // Reached a second way: the gear closes a loop, so the two positions must agree.
        const [ex, ey] = positions.get(pid)!;
        const why = `joint ${pointLabel(pid)} on gear ${lab} closes a loop`;
        constraints.push({ phi: pos[0].sub(ex), reason: `${why} (x)` }, { phi: pos[1].sub(ey), reason: `${why} (y)` });
      }
    }
  };

  const propagate = () => {
    while (queue.length) {
      const pid = queue.shift()!;
      for (const gid of gearsAt.get(pid) ?? []) if (!gearAngle.has(gid)) placeGear(gid, pid);
      const [px, py] = positions.get(pid)!;
      for (const l of adjacency.get(pid) ?? []) {
        if (visitedLink.has(l.id)) continue;
        visitedLink.add(l.id);
        const otherId = l.a === pid ? l.b : l.a;
        const other = pointById.get(otherId)!;
        if (!positions.has(otherId)) {
          // Tree bar: introduce its absolute angle, measured from end a towards end b.
          const a = pointById.get(l.a)!;
          const b = pointById.get(l.b)!;
          const th = `th${l.id}`;
          coord(th, '\\theta', linkLabel(l.id), 'angle', Math.atan2(b.y - a.y, b.x - a.x));
          barAngle.set(l.id, th);
          const L = linkLen.get(l.id)!;
          const s = l.a === pid ? 1 : -1;
          const pos: [Poly, Poly] = [
            px.add(L.mul(Poly.cos([[th, 1]])).scale(s)),
            py.add(L.mul(Poly.sin([[th, 1]])).scale(s)),
          ];
          positions.set(otherId, pos);
          if (other.kind !== 'free') lockConstraints(other, pos, `reached through bar ${linkLabel(l.id)}`);
          queue.push(otherId);
        } else {
          // Both ends already placed: the bar closes a loop, so its length is a constraint.
          const [ox, oy] = positions.get(otherId)!;
          const dx = ox.sub(px);
          const dy = oy.sub(py);
          const L = linkLen.get(l.id)!;
          constraints.push({
            phi: dx.mul(dx).add(dy.mul(dy)).sub(L.mul(L)),
            reason: `bar ${linkLabel(l.id)} closes a loop: |r_${pointLabel(otherId)} − r_${pointLabel(pid)}| = L_${linkLabel(l.id)}`,
            loop: { dx, dy, L, bar: l.id },
          });
        }
      }
    }
  };

  propagate();
  for (;;) {
    const unplaced = active.filter((p) => !positions.has(p.id));
    if (unplaced.length === 0) break;
    // Seed a floating component, preferring a joint with a locked axis (fewer coordinates).
    const seed = unplaced.find((p) => p.kind === 'lockX' || p.kind === 'lockY') ?? unplaced[0];
    const lab = pointLabel(seed.id);
    const x = seed.kind === 'lockX' ? anchorX(seed) : coord(`x${seed.id}`, 'x', lab, 'x', seed.x);
    const y = seed.kind === 'lockY' ? anchorY(seed) : coord(`y${seed.id}`, 'y', lab, 'y', seed.y);
    positions.set(seed.id, [x, y]);
    queue.push(seed.id);
    propagate();
  }

  // ---- gears ----------------------------------------------------------------
  const movable = (e: Poly) => coords.some((c) => e.dependsOn(c.sym));
  const relative = (l: DLink): [Poly, Poly] => {
    const [ax, ay] = positions.get(l.a)!;
    const [bx, by] = positions.get(l.b)!;
    return [bx.sub(ax), by.sub(ay)];
  };
  /** The absolute angle of a bar (direction a → b) as an expression in the coordinates. */
  const angleOf = (l: DLink): Poly => {
    const known = barAngle.get(l.id);
    if (known) return Poly.sym(known);
    const [dx, dy] = relative(l);
    if (!movable(dx) && !movable(dy)) return Poly.zero; // both ends fixed: constant angle
    // A loop-closing bar: make its angle explicit, replacing |Δr| = L by Δr = L(cos θ, sin θ).
    const a = pointById.get(l.a)!;
    const b = pointById.get(l.b)!;
    const th = `th${l.id}`;
    const thP = coord(th, '\\theta', linkLabel(l.id), 'angle', Math.atan2(b.y - a.y, b.x - a.x));
    barAngle.set(l.id, th);
    const L = linkLen.get(l.id)!;
    const idx = constraints.findIndex((c) => c.loop?.bar === l.id);
    const why = `bar ${linkLabel(l.id)} closes a loop; its angle θ_${linkLabel(l.id)} is explicit because a gear uses it`;
    constraints.splice(
      idx < 0 ? constraints.length : idx,
      idx < 0 ? 0 : 1,
      { phi: dx.sub(L.mul(Poly.cos([[th, 1]]))), reason: `${why} (x)` },
      { phi: dy.sub(L.mul(Poly.sin([[th, 1]]))), reason: `${why} (y)` },
    );
    return thP;
  };

  const initialEnv = () => {
    const env: Record<string, number> = {};
    for (const c of coords) env[c.sym] = c.initial;
    for (const p of params) env[p.sym] = p.value;
    return env;
  };
  const meshParts: { id: number; kind: MeshInfo['kind']; a: number; b: number; v: Poly }[] = [];
  for (const mesh of design.meshes ?? []) {
    const ga = gearById.get(mesh.a);
    const gb = gearById.get(mesh.b);
    const lab = `${gearLabel(mesh.a)}–${gearLabel(mesh.b)}`;
    if (!ga || !gb || ga === gb || !gearAngle.has(ga.id) || !gearAngle.has(gb.id)) continue;
    // θ_c: direction of the line of centres, which must keep its length for the gears to stay in mesh.
    const [rax, ray] = positions.get(ga.center)!;
    const [rbx, rby] = positions.get(gb.center)!;
    let tc: Poly | null = null;
    if (!movable(rbx.sub(rax)) && !movable(rby.sub(ray))) tc = Poly.zero;
    else {
      const carrier = links.find((l) => (l.a === ga.center && l.b === gb.center) || (l.a === gb.center && l.b === ga.center));
      if (carrier) tc = angleOf(carrier);
      else {
        const common = [...gearJoints].find(([, ids]) => ids.includes(ga.center) && ids.includes(gb.center));
        if (common) tc = gearAngle.get(common[0])!;
      }
    }
    if (!tc) {
      warnings.push(`Gears ${lab}: their centres can move apart, so they would not stay in mesh. Join the centres with a bar; the mesh was ignored.`);
      continue;
    }
    const Na = gearTeeth.get(ga.id)!;
    const Nb = gearTeeth.get(gb.id)!;
    const ta = gearAngle.get(ga.id)!;
    const tb = gearAngle.get(gb.id)!;
    const expr = Na.mul(ta.sub(tc)).add(Nb.mul(tb.sub(tc)));
    const C = param(`Cm${mesh.id}`, `C_{${gearLabel(mesh.a)}${gearLabel(mesh.b)}}`, expr.evaluate(initialEnv()), 'rad');
    constraints.push({
      phi: expr.sub(C),
      reason: `gears ${lab} mesh (${ga.teeth} : ${gb.teeth} teeth)`,
      gear: { Na, Nb, ta, tb, tc, C },
    });
    meshParts.push({ id: mesh.id, kind: 'gear', a: ga.id, b: gb.id, v: gearRadius.get(ga.id)!.mul(ta.sub(tc)) });
  }

  // ---- racks ----------------------------------------------------------------
  for (const rm of design.rackMeshes ?? []) {
    const gr = gearById.get(rm.gear);
    const rack = links.find((l) => l.id === rm.rack && l.rack);
    if (!gr || !rack || !gearAngle.has(gr.id)) continue;
    const gl = gearLabel(gr.id);
    const rl = linkLabel(rack.id);
    const lab = `gear g${gl} and rack ${rl}`;
    // Rack direction e and angle θ_rack (zero when the rack cannot turn: it is then absorbed in C).
    const [dx, dy] = relative(rack);
    let ex: Poly;
    let ey: Poly;
    let th = Poly.zero;
    if (!movable(dx) && !movable(dy)) {
      const a = pointById.get(rack.a)!;
      const b = pointById.get(rack.b)!;
      const L0 = Math.hypot(b.x - a.x, b.y - a.y);
      ex = Poly.const((b.x - a.x) / L0);
      ey = Poly.const((b.y - a.y) / L0);
    } else {
      th = angleOf(rack);
      const sym = barAngle.get(rack.id)!;
      ex = Poly.cos([[sym, 1]]);
      ey = Poly.sin([[sym, 1]]);
    }
    const [ax, ay] = positions.get(rack.a)!;
    const [cx, cy] = positions.get(gr.center)!;
    const rx = cx.sub(ax);
    const ry = cy.sub(ay);
    const xi = rx.mul(ex).add(ry.mul(ey)); // position of the gear centre along the rack
    const h = ry.mul(ex).sub(rx.mul(ey)); // its distance from the pitch line, (r_c − r_a)·n
    const h0 = h.evaluate(initialEnv());
    if (Math.abs(h0) < 1e-9) {
      warnings.push(`${lab[0].toUpperCase()}${lab.slice(1)}: the gear centre lies on the rack's pitch line, so they cannot mesh. The mesh was ignored.`);
      continue;
    }
    const side = h0 > 0 ? 1 : -1;
    const R = gearRadius.get(gr.id)!;
    if (movable(h)) {
      // Drawn tangent: the centre stays R from the pitch line. Otherwise keep the drawn distance.
      const target = Math.abs(Math.abs(h0) - gr.radius) < 1e-6 ? R.scale(side) : param(`hr${rm.id}`, `h_{${gl},${rl}}`, h0, 'm');
      constraints.push({ phi: h.sub(target), reason: `${lab}: the gear centre keeps its distance from the rack's pitch line`, rackDistance: true });
    }
    const rel = gearAngle.get(gr.id)!.sub(th);
    const sR = R.scale(side);
    const expr = xi.add(sR.mul(rel));
    const C = param(`Cr${rm.id}`, `C_{${gl},${rl}}`, expr.evaluate(initialEnv()), 'm');
    constraints.push({ phi: expr.sub(C), reason: `${lab} mesh: the pitch circle rolls along the rack without slipping`, rack: { xi, sR, rel, C } });
    meshParts.push({ id: rm.id, kind: 'rack', a: gr.id, b: rack.id, v: R.mul(rel) });
  }

  const qs = coords.map((c) => c.sym);
  const qsDepend = (e: Poly) => qs.some((q) => e.dependsOn(q));
  const qds = coords.map((c) => c.vel);
  const velocity = ([x, y]: [Poly, Poly]): [Poly, Poly] => [timeDerivative(x, qs, qds), timeDerivative(y, qs, qds)];
  const dot = (u: [Poly, Poly], v: [Poly, Poly]) => u[0].mul(v[0]).add(u[1].mul(v[1]));

  // ---- energies ------------------------------------------------------------
  yield 0.05;
  const bodies = links.length + gears.length + pointMass.size;
  let bodiesDone = 0;
  const bodyDone = () => 0.05 + (0.2 * ++bodiesDone) / bodies;
  let T = Poly.zero;
  let V = Poly.zero;
  for (const l of links) {
    yield bodyDone();
    const ra = positions.get(l.a)!;
    const rb = positions.get(l.b)!;
    const va = velocity(ra);
    const vb = velocity(rb);
    const m = linkMass.get(l.id)!;
    T = T.add(m.scale(1 / 6).mul(dot(va, va).add(dot(va, vb)).add(dot(vb, vb))));
    if (g) V = V.add(m.mul(g).scale(0.5).mul(ra[1].add(rb[1])));
  }
  for (const gr of gears) {
    yield bodyDone();
    if (!gearAngle.has(gr.id)) continue;
    const rc = positions.get(gr.center)!;
    const vc = velocity(rc);
    const m = gearMass.get(gr.id)!;
    const R = gearRadius.get(gr.id)!;
    T = T.add(m.scale(0.5).mul(dot(vc, vc)));
    if (!gr.locked) {
      const w = Poly.sym(`ph${gr.id}_d`);
      T = T.add(m.mul(R).mul(R).mul(w).mul(w).scale(0.25)); // ½ I φ̇², I = ½ m R²
    }
    if (g) V = V.add(m.mul(g).mul(rc[1]));
  }
  for (const [pid, m] of pointMass) {
    yield bodyDone();
    const r = positions.get(pid)!;
    const v = velocity(r);
    T = T.add(m.scale(0.5).mul(dot(v, v)));
    if (g) V = V.add(m.mul(g).mul(r[1]));
  }
  T = trigSimplify(T);
  yield 0.3;
  // Terms of V that do not depend on q are an irrelevant additive constant.
  V = Poly.fromTerms([...trigSimplify(V).terms.values()].filter((t) => qsDepend(Poly.fromTerms([t]))));

  // ---- springs ---------------------------------------------------------------
  const springTerms: SpringTerm[] = [];
  for (const [si, s] of springs.entries()) {
    yield 0.3 + (0.05 * si) / springs.length;
    const lab = springLabel(s.id);
    const k = `k${s.id}`;
    const rest = `l0${s.id}`;
    param(k, `k_{${lab}}`, Math.max(s.k, 0), 'N/m');
    param(rest, `\\ell_{0,${lab}}`, Math.max(s.rest, 0), 'm');
    const [ax, ay] = positions.get(s.a)!;
    const [bx, by] = positions.get(s.b)!;
    const dx = bx.sub(ax);
    const dy = by.sub(ay);
    const D = trigSimplify(dx.mul(dx).add(dy.mul(dy)));
    if (!qsDepend(D)) {
      warnings.push(`Spring k${lab} joins two joints that cannot move relative to each other, so it has no effect.`);
      continue;
    }
    const grad = qs.map((q) => trigSimplify(D.diff(q).scale(0.5)));
    springTerms.push({ id: s.id, k, rest, kLatex: `k_{${lab}}`, lLatex: `\\ell_{${lab}}`, restLatex: `\\ell_{0,${lab}}`, dx, dy, D, grad });
  }

  // ---- Lagrange's equations ---------------------------------------------------
  const L = T.sub(V);
  const n = coords.length;
  const p = qds.map((qd) => L.diff(qd)); // generalized momenta ∂L/∂q̇
  const M: Poly[][] = [];
  const f: Poly[] = [];
  for (let i = 0; i < n; i++) {
    yield 0.35 + (0.5 * i) / n;
    M.push(qds.map((qd) => trigSimplify(p[i].diff(qd))));
    let fi = L.diff(qs[i]);
    for (let j = 0; j < n; j++) if (p[i].dependsOn(qs[j])) fi = fi.sub(p[i].diff(qs[j]).mul(Poly.sym(qds[j])));
    f.push(trigSimplify(fi));
  }

  // ---- constraints -----------------------------------------------------------
  const movableKept: Constraint[] = [];
  for (const c of constraints) {
    const phi = trigSimplify(c.phi);
    if (qs.some((q) => phi.dependsOn(q))) movableKept.push({ ...c, phi });
    // (a rack pinned at both ends is the usual stationary rack, so that needs no warning)
    else if (!links.find((l) => l.id === c.loop?.bar)?.rack) warnings.push(`${c.reason}: it cannot move, so the constraint was dropped`);
  }
  // A rack's distance constraint is often enforced already, e.g. by rails holding the rack and a
  // fixed axle holding the pinion. Keep it only if its gradient at the start is independent of
  // the other constraints', since a redundant constraint would make the equations singular.
  const env0 = initialEnv();
  const basis: number[][] = [];
  const independent = (phi: Poly): boolean => {
    const r = qs.map((q) => phi.diff(q).evaluate(env0));
    const norm0 = Math.hypot(...r);
    if (norm0 < 1e-12) return false;
    for (const b of basis) {
      const d = r.reduce((s, x, i) => s + x * b[i], 0);
      for (let i = 0; i < r.length; i++) r[i] -= d * b[i];
    }
    const norm = Math.hypot(...r);
    if (norm < 1e-8 * norm0) return false;
    basis.push(r.map((x) => x / norm));
    return true;
  };
  for (const c of movableKept) if (!c.rackDistance) independent(c.phi);
  const kept = movableKept.filter((c) => !c.rackDistance || independent(c.phi));
  const phiQ: Poly[][] = [];
  const gamma: Poly[] = [];
  for (const [k, c] of kept.entries()) {
    yield 0.85 + (0.15 * k) / kept.length;
    const row = qs.map((q) => trigSimplify(c.phi.diff(q)));
    let phiDot = Poly.zero;
    row.forEach((d, j) => (phiDot = phiDot.add(d.mul(Poly.sym(qds[j])))));
    phiQ.push(row);
    gamma.push(trigSimplify(timeDerivative(phiDot, qs, qds).scale(-1)));
  }
  yield 1;

  if (n > 0 && kept.length >= n) warnings.push('The mechanism has as many constraints as coordinates, so it may be locked.');

  return {
    coords,
    params,
    positions,
    T,
    V,
    M,
    f,
    constraints: kept,
    springs: springTerms,
    gearAngles: [...gearAngle].map(([id, angle]) => ({ id, angle })),
    phiQ,
    gamma,
    activePoints: active.map((pt) => pt.id),
    joints: active.map((p) => ({
      id: p.id,
      bars: links.filter((l) => l.a === p.id || l.b === p.id).map((l) => ({ link: l.id, other: l.a === p.id ? l.b : l.a })),
      gears: (gearsAt.get(p.id) ?? []).filter((gid) => gearAngle.has(gid) && !gearById.get(gid)!.locked),
      ground: p.kind !== 'free' || (gearsAt.get(p.id) ?? []).some((gid) => gearById.get(gid)!.locked),
      rail: p.kind === 'lockX' ? 'y' : p.kind === 'lockY' ? 'x' : null,
    })),
    meshes: meshParts.map(({ id, kind, a, b, v }) => ({ id, kind, a, b, pitchGrad: qs.map((q) => v.diff(q)) })),
    activeLinks: links.map((l) => l.id),
    warnings,
  };
}
