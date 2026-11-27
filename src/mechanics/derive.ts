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
 * 10. A fixed angle between two bars adds θ_b − θ_a = α, α being the drawn angle. A bar whose
 *    angle is not a coordinate gets an explicit one, as for gears. The constraint is dropped,
 *    with a warning, when the mechanism already holds that angle (say, two sides of a rigid
 *    triangle), since a redundant constraint would make the equations singular.
 * 11. Bodies joined into one (bars, racks, gears, joined directly or through others) are
 *    placed as a single rigid body: one rotation coordinate for the whole group (the first
 *    bar's angle θ, or the first gear's φ), and every joint of the group at
 *    r = r_origin + Rot(θ)(u, v), the offsets taken from the drawing. So a group needs at most
 *    three coordinates and no constraints of its own; its other bars turn at θ plus a constant
 *    and its gears with θ.
 * 13. A joint pinned onto a bar keeps its drawn place in the bar's frame, r_P − r_A = Rot(θ)(u, v)
 *    (two constraints); a joint on a rail fixed to a body stays on that line as the body moves,
 *    (r_P − r_A) · n(θ) = d with n the rail's normal turning with the body (one constraint). A
 *    ground rail at any angle gives n · (r_P − r_0) = 0, or, for a joint that seeds a floating
 *    part, a single coordinate s along the rail.
 * 12. A motor on a gear adds the generalized force Q_φ = τ − b φ̇ to the gear's equation:
 *    a constant torque τ, less b = τ/Ω as the gear nears the motor's no-load speed Ω.
 */
import { defineSymbol, Poly, timeDerivative, trigSimplify } from '../cas/poly';
import { bodyName, drawnAngle, gearLabel, linkLabel, railOf, sameBody, sharedJoint, type DBodyRef, pointLabel, springLabel, type Design, type DLink, type DPoint } from './model';

export interface Coordinate {
  sym: string;
  vel: string;
  latex: string;
  /** LaTeX base symbol and subscript, e.g. '\\theta' and '1'. */
  base: string;
  sub: string;
  /** What the coordinate measures ('s': distance along a ground rail). */
  kind: 'angle' | 'x' | 'y' | 's';
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
  /** A fixed angle between bars a and b, dropped (with a warning) when already enforced. */
  fixedAngle?: { a: number; b: number };
  /** One of the (up to three) equations of a join, by join id; dropped when already enforced. */
  joined?: number;
  /** An equation of a joint pinned to, or sliding on, a body, by joint id; dropped when already enforced. */
  attached?: number;
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
  /** Slack (no force) while shorter than its rest length. */
  tensionOnly: boolean;
}

/** The bodies that meet at a joint, for joint friction. */
export interface JointBodies {
  id: number;
  /** Bars meeting here (ending here, or with the joint pinned on them), by their two ends. */
  bars: { link: number; a: number; b: number }[];
  /** Gears this joint belongs to (as centre or pin) that can turn. */
  gears: number[];
  /** The ground or a slider block: a body that does not rotate. */
  ground: boolean;
  /** For a joint sliding on a ground rail, the rail's direction (rad); null otherwise. */
  rail: number | null;
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
  /**
   * End stops of sliders with a range: the joint's travel s(q) along its rail from where it was
   * drawn (relative to the body the rail is on), its gradient, and the limits.
   */
  sliderStops: { joint: number; s: Poly; grad: Poly[]; min: number | null; max: number | null; restitution: number }[];
  /** Motors: the index of the driven gear's φ in q, and the parameter indices of τ and b. */
  motors: { id: number; coord: number; tau: string; b: string | null }[];
  /** For every gear that turns, the coordinate it turns with one for one (its φ, or its group's angle). */
  gearCoords: { id: number; coord: number }[];
  /** Centre joint and pitch radius of every gear in the mechanism, for ground contact. */
  gearDisks: { id: number; center: number; radius: number }[];
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
  /** The coordinate symbol a gear's frame turns with (its own φ, or its group's), null if it cannot turn. */
  const gearSym = new Map<number, string | null>();

  // ---- joined bodies: groups placed as one rigid body ---------------------------------
  const linkById = new Map(links.map((l) => [l.id, l]));
  const bodyKey = (r: DBodyRef) => `${r.kind}:${r.id}`;
  const parent = new Map<string, string>();
  const find = (k: string): string => {
    const p = parent.get(k) ?? k;
    if (p === k) return k;
    const root = find(p);
    parent.set(k, root);
    return root;
  };
  const exists = (r: DBodyRef) => (r.kind === 'link' ? linkById.has(r.id) : gearById.has(r.id));
  for (const j of design.joins ?? []) if (!sameBody(j.a, j.b) && exists(j.a) && exists(j.b)) parent.set(find(bodyKey(j.a)), find(bodyKey(j.b)));
  const members = new Map<string, DBodyRef[]>();
  for (const j of design.joins ?? [])
    for (const r of [j.a, j.b])
      if (exists(r)) {
        const root = find(bodyKey(r));
        const list = members.get(root) ?? [];
        if (!list.some((x) => sameBody(x, r))) list.push(r);
        members.set(root, list);
      }
  /** A group: its bars and gears, and every joint on them. */
  interface Group {
    bars: DLink[];
    gears: typeof gears;
    joints: number[];
  }
  const groups: Group[] = [];
  /** Group index of a bar or gear in a group. */
  const groupOfBody = new Map<string, number>();
  for (const list of members.values()) {
    if (list.length < 2) continue;
    const bars = list.filter((r) => r.kind === 'link').map((r) => linkById.get(r.id)!);
    const gs = list.filter((r) => r.kind === 'gear').map((r) => gearById.get(r.id)!);
    const joints = [...new Set([...bars.flatMap((l) => [l.a, l.b]), ...gs.flatMap((g) => gearJoints.get(g.id)!)])];
    for (const r of list) groupOfBody.set(bodyKey(r), groups.length);
    groups.push({ bars, gears: gs, joints });
  }
  const groupsAt = new Map<number, number[]>();
  groups.forEach((g, gi) => g.joints.forEach((pid) => groupsAt.set(pid, [...(groupsAt.get(pid) ?? []), gi])));
  const groupPlaced = new Set<number>();
  /** Bars of a group other than its reference bar: their angle is the group's coordinate plus `off`. */
  const groupBar = new Map<number, { sym: string | null; off: number }>();

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

  /** A ground rail's direction in radians, folded to [0, π), or null if the joint has none. */
  const groundRail = (p: DPoint): number | null => {
    const r = railOf(p);
    if (!r || r.on) return null;
    const a = (((r.angle % 180) + 180) % 180) * (Math.PI / 180);
    return a;
  };
  const isAxis = (a: number, target: number) => Math.abs(a - target) < 1e-9;
  const lockConstraints = (p: DPoint, [x, y]: [Poly, Poly], via: string) => {
    const lab = pointLabel(p.id);
    if (p.kind === 'fixed') {
      constraints.push({ phi: x.sub(anchorX(p)), reason: `joint ${lab}: x locked (${via})` });
      constraints.push({ phi: y.sub(anchorY(p)), reason: `joint ${lab}: y locked (${via})` });
      return;
    }
    const a = groundRail(p);
    if (a === null) return;
    if (isAxis(a, Math.PI / 2)) constraints.push({ phi: x.sub(anchorX(p)), reason: `joint ${lab}: x locked, on a vertical rail (${via})` });
    else if (isAxis(a, 0)) constraints.push({ phi: y.sub(anchorY(p)), reason: `joint ${lab}: y locked, on a horizontal rail (${via})` });
    else {
      // stays on the line through its drawn place: n · (r − r₀) = 0, n = (−sin a, cos a)
      const phi = x.sub(anchorX(p)).scale(-Math.sin(a)).add(y.sub(anchorY(p)).scale(Math.cos(a)));
      constraints.push({ phi, reason: `joint ${lab}: slides on a ground rail at ${((a * 180) / Math.PI).toFixed(1)}° (${via})` });
    }
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
    gearSym.set(gid, gr.locked ? null : `ph${gid}`);
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
        lockConstraints(p, pos, `on gear ${lab}`);
        queue.push(pid);
      } else {
        // Reached a second way: the gear closes a loop, so the two positions must agree.
        const [ex, ey] = positions.get(pid)!;
        const why = `joint ${pointLabel(pid)} on gear ${lab} closes a loop`;
        constraints.push({ phi: pos[0].sub(ex), reason: `${why} (x)` }, { phi: pos[1].sub(ey), reason: `${why} (y)` });
      }
    }
  };

  /**
   * Place a whole group of joined bodies from one of its joints that is already placed: one
   * rotation coordinate (its first bar's angle, or its first gear's φ; none if it holds a locked
   * gear), and every joint at r_origin + Rot(θ)(u, v) with (u, v) from the drawing.
   */
  const placeGroup = (gi: number, fromPid: number) => {
    groupPlaced.add(gi);
    const G = groups[gi];
    const locked = G.gears.some((g) => g.locked);
    const ref = G.bars[0];
    const origin = ref ? ref.a : G.gears[0].center;
    const o = pointById.get(origin)!;
    // the frame: the reference bar's direction (drawn at th0), or the first gear's
    let sym: string | null = null;
    let th0 = 0;
    if (ref) {
      const b = pointById.get(ref.b)!;
      th0 = Math.atan2(b.y - o.y, b.x - o.x);
    }
    if (!locked) {
      if (ref) {
        sym = `th${ref.id}`;
        coord(sym, '\\theta', linkLabel(ref.id), 'angle', th0);
        barAngle.set(ref.id, sym);
      } else {
        sym = `ph${G.gears[0].id}`;
        coord(sym, '\\varphi', gearLabel(G.gears[0].id), 'angle', 0);
      }
    }
    // Rot(θ) of the frame (θ absolute for a bar frame, θ = φ for a gear frame); a locked group keeps its drawn frame
    const [cos, sin] = sym ? [Poly.cos([[sym, 1]]), Poly.sin([[sym, 1]])] : [Poly.const(Math.cos(th0)), Poly.const(Math.sin(th0))];
    const offset = (pid: number): [Poly, Poly] => {
      if (pid === origin) return [Poly.zero, Poly.zero];
      if (ref && pid === ref.b) return [linkLen.get(ref.id)!, Poly.zero];
      const p = pointById.get(pid)!;
      const [dx, dy] = [p.x - o.x, p.y - o.y];
      const [u, v] = [Math.cos(th0) * dx + Math.sin(th0) * dy, -Math.sin(th0) * dx + Math.cos(th0) * dy];
      const lab = pointLabel(pid);
      return [Math.abs(u) < 1e-12 ? Poly.zero : param(`gu${pid}`, `u_{${lab}}`, u, 'm'), Math.abs(v) < 1e-12 ? Poly.zero : param(`gv${pid}`, `v_{${lab}}`, v, 'm')];
    };
    const rot = ([u, v]: [Poly, Poly]): [Poly, Poly] => [u.mul(cos).sub(v.mul(sin)), u.mul(sin).add(v.mul(cos))];
    const [fx, fy] = positions.get(fromPid)!;
    const [fu, fv] = rot(offset(fromPid));
    const [ox, oy] = [fx.sub(fu), fy.sub(fv)];
    const name = `the joined ${[...G.bars.map((l) => `bar ${linkLabel(l.id)}`), ...G.gears.map((g) => `g${gearLabel(g.id)}`)].join(', ')}`;
    for (const pid of G.joints) {
      if (pid === fromPid) continue;
      const [du, dv] = rot(offset(pid));
      const pos: [Poly, Poly] = [ox.add(du), oy.add(dv)];
      if (!positions.has(pid)) {
        positions.set(pid, pos);
        lockConstraints(pointById.get(pid)!, pos, `on ${name}`);
        queue.push(pid);
      } else {
        // reached a second way: the group closes a loop
        const [ex, ey] = positions.get(pid)!;
        const why = `joint ${pointLabel(pid)} on ${name} closes a loop`;
        constraints.push({ phi: pos[0].sub(ex), reason: `${why} (x)` }, { phi: pos[1].sub(ey), reason: `${why} (y)` });
      }
    }
    for (const l of G.bars) {
      visitedLink.add(l.id); // rigid inside the group: no loop of its own
      if (l === ref) continue;
      const [a, b] = [pointById.get(l.a)!, pointById.get(l.b)!];
      groupBar.set(l.id, { sym, off: Math.atan2(b.y - a.y, b.x - a.x) - (sym ? th0 : 0) });
    }
    for (const g of G.gears) {
      // a gear's φ is its turn since the drawing: the group's turn
      gearAngle.set(g.id, sym ? (ref ? Poly.sym(sym).sub(Poly.const(th0)) : Poly.sym(sym)) : Poly.zero);
      gearSym.set(g.id, sym);
    }
  };

  const propagate = () => {
    while (queue.length) {
      const pid = queue.shift()!;
      for (const gi of groupsAt.get(pid) ?? []) if (!groupPlaced.has(gi)) placeGroup(gi, pid);
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
          lockConstraints(other, pos, `reached through bar ${linkLabel(l.id)}`);
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
    // Seed a floating component, preferring a joint on a ground rail (one coordinate, not two).
    const seed = unplaced.find((p) => groundRail(p) !== null) ?? unplaced[0];
    const lab = pointLabel(seed.id);
    const a = groundRail(seed);
    let x: Poly;
    let y: Poly;
    if (a === null) [x, y] = [coord(`x${seed.id}`, 'x', lab, 'x', seed.x), coord(`y${seed.id}`, 'y', lab, 'y', seed.y)];
    else if (isAxis(a, Math.PI / 2)) [x, y] = [anchorX(seed), coord(`y${seed.id}`, 'y', lab, 'y', seed.y)];
    else if (isAxis(a, 0)) [x, y] = [coord(`x${seed.id}`, 'x', lab, 'x', seed.x), anchorY(seed)];
    else {
      // r = r₀ + s (cos a, sin a): s is the distance along the rail from where it was drawn
      const s = coord(`s${seed.id}`, 's', lab, 's', 0);
      [x, y] = [anchorX(seed).add(s.scale(Math.cos(a))), anchorY(seed).add(s.scale(Math.sin(a)))];
    }
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
    const inGroup = groupBar.get(l.id);
    if (inGroup) return inGroup.sym ? Poly.sym(inGroup.sym).add(Poly.const(inGroup.off)) : Poly.const(inGroup.off);
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
      const inGroup = groupBar.get(rack.id);
      const sym = barAngle.get(rack.id) ?? inGroup?.sym ?? null;
      const off = inGroup?.off ?? 0;
      // its direction: cos and sin of θ (+ the group offset)
      const [c, sn] = sym ? [Poly.cos([[sym, 1]]), Poly.sin([[sym, 1]])] : [Poly.const(1), Poly.zero];
      ex = c.scale(Math.cos(off)).sub(sn.scale(Math.sin(off)));
      ey = sn.scale(Math.cos(off)).add(c.scale(Math.sin(off)));
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

  // ---- fixed angles -------------------------------------------------------------
  for (const fa of design.angles ?? []) {
    const la = links.find((l) => l.id === fa.a);
    const lb = links.find((l) => l.id === fa.b);
    if (!la || !lb || la === lb) continue;
    const lab = `bars ${linkLabel(la.id)} and ${linkLabel(lb.id)}`;
    const rel = angleOf(lb).sub(angleOf(la));
    if (!movable(rel)) {
      warnings.push(`The angle between ${lab} cannot change anyway, so fixing it has no effect.`);
      continue;
    }
    const a0 = rel.evaluate(initialEnv());
    const alpha = param(`al${fa.id}`, `\\alpha_{${linkLabel(la.id)},${linkLabel(lb.id)}}`, a0, 'rad');
    const at = sharedJoint(la, lb);
    const where = at !== null ? `at joint ${pointLabel(at)} ` : '';
    const deg = drawnAngle(design, la, lb).toFixed(1);
    constraints.push({ phi: rel.sub(alpha), reason: `the angle ${where}between ${lab} is fixed (${deg}°)`, fixedAngle: { a: la.id, b: lb.id } });
  }

  // ---- joined bodies -------------------------------------------------------------
  /** A body's angle (and its coordinate symbol, null if it cannot turn) and reference point. */
  const bodyFrame = (ref: DBodyRef): { angle: Poly; sym: string | null; at: [Poly, Poly] } | null => {
    if (ref.kind === 'link') {
      const l = links.find((x) => x.id === ref.id);
      if (!l) return null;
      const inGroup = groupBar.get(l.id);
      // a group's bar uses the group's frame (a constant turn from its own direction)
      if (inGroup) return { angle: inGroup.sym ? Poly.sym(inGroup.sym) : Poly.zero, sym: inGroup.sym, at: positions.get(l.a)! };
      const angle = angleOf(l);
      return { angle, sym: barAngle.get(l.id) ?? null, at: positions.get(l.a)! };
    }
    const gr = gearById.get(ref.id);
    if (!gr || !gearAngle.has(gr.id)) return null;
    const sym = gearSym.get(gr.id) ?? null;
    return { angle: sym ? Poly.sym(sym) : Poly.zero, sym, at: positions.get(gr.center)! };
  };
  const joinNames = new Map<number, string>();
  for (const j of design.joins ?? []) {
    if (sameBody(j.a, j.b)) continue;
    // bodies placed as one group are rigid already
    const [ga, gb] = [groupOfBody.get(bodyKey(j.a)), groupOfBody.get(bodyKey(j.b))];
    if (ga !== undefined && ga === gb && groupPlaced.has(ga)) continue;
    const A = bodyFrame(j.a);
    const B = bodyFrame(j.b);
    if (!A || !B) continue;
    const lab = `${bodyName(design, j.a)} and ${bodyName(design, j.b)}`;
    joinNames.set(j.id, lab);
    const env = initialEnv();
    const cos = A.sym ? Poly.cos([[A.sym, 1]]) : Poly.const(1);
    const sin = A.sym ? Poly.sin([[A.sym, 1]]) : Poly.zero;
    const dx = B.at[0].sub(A.at[0]);
    const dy = B.at[1].sub(A.at[1]);
    // where B's reference point sits in A's frame, from the drawing
    const [c0, s0, dx0, dy0] = [cos, sin, dx, dy].map((e) => e.evaluate(env));
    const constant = (sym: string, latex: string, value: number, unit: string) => (Math.abs(value) < 1e-12 ? Poly.zero : param(sym, latex, value, unit));
    const u = constant(`ju${j.id}`, `u_{\\mathrm{J}${j.id + 1}}`, c0 * dx0 + s0 * dy0, 'm');
    const v = constant(`jv${j.id}`, `v_{\\mathrm{J}${j.id + 1}}`, -s0 * dx0 + c0 * dy0, 'm');
    const rel = B.angle.sub(A.angle);
    const alpha = constant(`ja${j.id}`, `\\alpha_{\\mathrm{J}${j.id + 1}}`, rel.evaluate(env), 'rad');
    const parts: [Poly, string][] = [
      [rel.sub(alpha), 'they turn together'],
      [dx.sub(u.mul(cos)).add(v.mul(sin)), 'x'],
      [dy.sub(u.mul(sin)).sub(v.mul(cos)), 'y'],
    ];
    const moving = parts.filter(([phi]) => movable(phi));
    if (!moving.length) {
      warnings.push(`${lab[0].toUpperCase()}${lab.slice(1)} cannot move, so joining them has no effect.`);
      continue;
    }
    for (const [phi, what] of moving) constraints.push({ phi, reason: `${lab} are one body (${what})`, joined: j.id });
  }

  // ---- joints pinned to, or sliding on, a body ------------------------------------
  const attachNames = new Map<number, string>();
  for (const p of active) {
    const rail = railOf(p);
    const ref = p.pin ?? (rail?.on ? rail.on : null);
    if (!ref || !positions.has(p.id)) continue;
    const B = bodyFrame(ref);
    if (!B) continue;
    const lab = `joint ${pointLabel(p.id)}`;
    const body = bodyName(design, ref);
    const env = initialEnv();
    const cos = B.sym ? Poly.cos([[B.sym, 1]]) : Poly.const(1);
    const sin = B.sym ? Poly.sin([[B.sym, 1]]) : Poly.zero;
    const [px, py] = positions.get(p.id)!;
    const dx = px.sub(B.at[0]);
    const dy = py.sub(B.at[1]);
    const constant = (sym: string, latex: string, value: number, unit: string) => (Math.abs(value) < 1e-12 ? Poly.zero : param(sym, latex, value, unit));
    const parts: [Poly, string][] = [];
    if (p.pin) {
      attachNames.set(p.id, `${lab} is pinned to ${body}`);
      const [c0, s0, dx0, dy0] = [cos, sin, dx, dy].map((e) => e.evaluate(env));
      const u = constant(`pu${p.id}`, `u_{${pointLabel(p.id)}}`, c0 * dx0 + s0 * dy0, 'm');
      const v = constant(`pv${p.id}`, `v_{${pointLabel(p.id)}}`, -s0 * dx0 + c0 * dy0, 'm');
      parts.push([dx.sub(u.mul(cos)).add(v.mul(sin)), `${lab} is pinned to ${body} (x)`]);
      parts.push([dy.sub(u.mul(sin)).sub(v.mul(cos)), `${lab} is pinned to ${body} (y)`]);
    } else if (rail) {
      attachNames.set(p.id, `${lab} slides on ${body}`);
      // the rail's normal, drawn at angle a when the body was at θ₀, turns with the body:
      // n(θ) = Rot(θ − θ₀)(−sin a, cos a)
      const a = (rail.angle * Math.PI) / 180;
      const th0 = B.angle.evaluate(env);
      const [n0x, n0y] = [-Math.sin(a), Math.cos(a)];
      // Rot(θ − θ₀) = cos θ Rot(−θ₀) + sin θ Rot(π/2 − θ₀): expand with numeric θ₀
      const [rc, rs] = [Math.cos(th0), Math.sin(th0)];
      const nx = cos.scale(rc * n0x + rs * n0y).sub(sin.scale(-rs * n0x + rc * n0y));
      const ny = sin.scale(rc * n0x + rs * n0y).add(cos.scale(-rs * n0x + rc * n0y));
      const along = dx.mul(nx).add(dy.mul(ny));
      const d = constant(`pd${p.id}`, `d_{${pointLabel(p.id)}}`, along.evaluate(env), 'm');
      parts.push([along.sub(d), `${lab} slides on a rail on ${body}`]);
    }
    const moving = parts.filter(([phi]) => movable(phi));
    if (!moving.length) {
      warnings.push(`${lab[0].toUpperCase()}${lab.slice(1)} and ${body} cannot move, so the constraint has no effect.`);
      continue;
    }
    for (const [phi, reason] of moving) constraints.push({ phi, reason, attached: p.id });
  }

  const qs = coords.map((c) => c.sym);
  /** The coordinate a gear turns with, one for one (its own φ, or its group's angle); −1 if none. */
  const gearCoordOf = (gid: number): number => {
    const a = gearAngle.get(gid);
    if (!a) return -1;
    const deps = qs.filter((q) => a.dependsOn(q));
    return deps.length === 1 && a.diff(deps[0]).constantValue() === 1 ? qs.indexOf(deps[0]) : -1;
  };

  // ---- slider end stops --------------------------------------------------------------
  // s = (r_P − r_A) · t(θ) − s₀: travel along the rail, t its direction turning with the body
  // (fixed for a ground rail, r_A then being the drawn position), zero where it was drawn.
  const sliderStops: Derivation['sliderStops'] = [];
  for (const p of active) {
    const rail = railOf(p);
    const range = p.slide?.range;
    if (!rail || !range || (range.min === null && range.max === null) || !positions.has(p.id)) continue;
    const a = (rail.angle * Math.PI) / 180;
    const [px, py] = positions.get(p.id)!;
    let s: Poly;
    if (!rail.on) s = px.sub(anchorX(p)).scale(Math.cos(a)).add(py.sub(anchorY(p)).scale(Math.sin(a)));
    else {
      const B = bodyFrame(rail.on);
      if (!B) continue;
      const env0 = initialEnv();
      const cos = B.sym ? Poly.cos([[B.sym, 1]]) : Poly.const(1);
      const sin = B.sym ? Poly.sin([[B.sym, 1]]) : Poly.zero;
      const th0 = B.angle.evaluate(env0);
      // t(θ) = Rot(θ − θ₀)(cos a, sin a) = Rot(θ) m, m = Rot(−θ₀)(cos a, sin a)
      const [mx, my] = [Math.cos(th0) * Math.cos(a) + Math.sin(th0) * Math.sin(a), -Math.sin(th0) * Math.cos(a) + Math.cos(th0) * Math.sin(a)];
      const tx = cos.scale(mx).sub(sin.scale(my));
      const ty = sin.scale(mx).add(cos.scale(my));
      s = px.sub(B.at[0]).mul(tx).add(py.sub(B.at[1]).mul(ty));
      s = s.sub(Poly.const(s.evaluate(env0)));
    }
    if (!qs.some((q) => s.dependsOn(q))) continue;
    s = trigSimplify(s);
    sliderStops.push({ joint: p.id, s, grad: qs.map((q) => trigSimplify(s.diff(q))), min: range.min, max: range.max, restitution: range.restitution });
  }

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
      const w = timeDerivative(gearAngle.get(gr.id)!, qs, qds); // φ̇: its own coordinate's rate, or its group's
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
    springTerms.push({ id: s.id, k, rest, kLatex: `k_{${lab}}`, lLatex: `\\ell_{${lab}}`, restLatex: `\\ell_{0,${lab}}`, dx, dy, D, grad, tensionOnly: !!s.tensionOnly });
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

  // ---- motors ----------------------------------------------------------------
  const motors: Derivation['motors'] = [];
  for (const gr of gears) {
    if (!gr.motor) continue;
    const lab = gearLabel(gr.id);
    const k = gearCoordOf(gr.id);
    if (k < 0) {
      warnings.push(`Gear g${lab} is locked, so its motor cannot turn it.`);
      continue;
    }
    const tau = param(`tau${gr.id}`, `\\tau_{\\mathrm{g}${lab}}`, gr.motor.torque, 'N·m');
    let q = tau;
    let b: string | null = null;
    if (gr.motor.noLoadRpm > 0) {
      // the torque falls linearly to zero at the no-load speed: b = τ / Ω
      b = `bm${gr.id}`;
      const bp = param(b, `b_{\\mathrm{g}${lab}}`, gr.motor.torque / ((gr.motor.noLoadRpm * 2 * Math.PI) / 60), 'N·m·s');
      q = q.sub(bp.mul(Poly.sym(qds[k])));
    }
    f[k] = f[k].add(q);
    motors.push({ id: gr.id, coord: k, tau: `tau${gr.id}`, b });
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
  // Fixed angles get the same test: welding two sides of a rigid triangle adds nothing.
  const optional = (c: Constraint) => c.rackDistance || c.fixedAngle || c.joined !== undefined || c.attached !== undefined;
  for (const c of movableKept) if (!optional(c)) independent(c.phi);
  const kept = movableKept.filter((c) => {
    if (!optional(c)) return true;
    if (independent(c.phi)) return true;
    if (c.fixedAngle) warnings.push(`${c.reason[0].toUpperCase()}${c.reason.slice(1)}, but the mechanism already holds that angle, so the constraint was dropped.`);
    return false;
  });
  for (const [id, lab] of attachNames)
    if (movableKept.some((c) => c.attached === id) && !kept.some((c) => c.attached === id))
      warnings.push(`${lab[0].toUpperCase()}${lab.slice(1)}, but it already moves that way, so the constraint was dropped.`);
  for (const [id, lab] of joinNames)
    if (movableKept.some((c) => c.joined === id) && !kept.some((c) => c.joined === id))
      warnings.push(`${lab[0].toUpperCase()}${lab.slice(1)} already move as one body, so joining them adds nothing.`);
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
    motors,
    sliderStops,
    gearCoords: [...gearAngle.keys()].map((id) => ({ id, coord: gearCoordOf(id) })).filter((g) => g.coord >= 0),
    gearDisks: gears.filter((gr) => gearAngle.has(gr.id)).map((gr) => ({ id: gr.id, center: gr.center, radius: gr.radius })),
    phiQ,
    gamma,
    activePoints: active.map((pt) => pt.id),
    joints: active.map((p) => ({
      id: p.id,
      bars: links.filter((l) => l.a === p.id || l.b === p.id || (p.pin?.kind === 'link' && p.pin.id === l.id)).map((l) => ({ link: l.id, a: l.a, b: l.b })),
      gears: (gearsAt.get(p.id) ?? []).filter((gid) => gearAngle.has(gid) && !gearById.get(gid)!.locked),
      ground: p.kind === 'fixed' || groundRail(p) !== null || (gearsAt.get(p.id) ?? []).some((gid) => gearById.get(gid)!.locked),
      rail: groundRail(p),
    })),
    meshes: meshParts.map(({ id, kind, a, b, v }) => ({ id, kind, a, b, pitchGrad: qs.map((q) => v.diff(q)) })),
    activeLinks: links.map((l) => l.id),
    warnings,
  };
}
