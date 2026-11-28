import { gearLabel, GEAR_MODULE, leverOf, type DPid, linkLabel, motorHousingRadius, pointLabel, rackPitch, railOf, sharedJoint, springLabel, type DBodyRef, type Design, type DLink, type DPoint, type DSpring } from '../mechanics/model';
import type { PartSet } from '../mechanics/fragment';
import type { Heat } from '../mechanics/simulate';

export interface View {
  scale: number; // pixels per metre
  ox: number; // screen position of the world origin
  oy: number;
}

export type Vec = [number, number];

export const toScreen = (v: View, [x, y]: Vec): Vec => [v.ox + x * v.scale, v.oy - y * v.scale];
export const toWorld = (v: View, [sx, sy]: Vec): Vec => [(sx - v.ox) / v.scale, (v.oy - sy) / v.scale];

export type Selection = { kind: 'point'; id: number } | { kind: 'link'; id: number } | { kind: 'spring'; id: number } | { kind: 'gear'; id: number } | { kind: 'lever'; id: number } | { kind: 'pid'; id: number }; // a lever's and a PID controller's id is its motor's gear

export interface Scene {
  design: Design;
  pos: (id: number) => Vec;
  selection: Selection | null;
  hover: Selection | null;
  /** A part being drawn; `meshAt` marks where it will mesh, `side` is a rack's tooth side. */
  preview: { from: Vec; to: Vec; kind: 'bar' | 'spring' | 'gear' | 'wheel' | 'propeller' | 'rack'; meshAt?: Vec | null; side?: 1 | -1 } | null;
  /** Current rotation of a gear (0 while editing). */
  gearAngle: (id: number) => number;
  /** Current turn of the housing of the motor on a gear, since the drawing (0 while editing). */
  motorAngle?: (id: number) => number;
  /** Spin φ of a propeller's blades (0 while editing). */
  spinAngle?: (id: number) => number;
  /** Spin rate φ̇ of a propeller's blades (rad/s), for its blur. */
  spinRate?: (id: number) => number;
  /** A propeller's thrust along its axis (N) while running, for its arrow. */
  thrust?: (id: number) => number | undefined;
  /** Temperature rise (K) of joints and gear meshes from friction, while simulating. */
  heat?: Heat | null;
  grab: { at: Vec; to: Vec; force: Vec } | null;
  simulating: boolean;
  /** Draw the names of joints, bars, racks, springs and gears, and mass tags (default true). */
  labels?: boolean;
  /** Parts chosen together (box selection, a paste), highlighted like the selection. */
  multi?: PartSet | null;
  /** The selection box being dragged, in world coordinates. */
  box?: { from: Vec; to: Vec } | null;
  /** Parts being duplicated, drawn as an outline shifted by (dx, dy) to where a click will place them. */
  ghost?: { design: Design; dx: number; dy: number } | null;
  /** A rail being placed with the Slider tool: through `at`, at `angle` degrees, with the ends chosen so far (m along it). */
  railPreview?: { at: Vec; angle: number; ends?: number[] } | null;
  /** The ground's terrain waypoints, if there is a ground. */
  ground?: [number, number][] | null;
  /** The ground is selected: its waypoints show as handles, `waypoint` the selected one. */
  groundSelected?: { waypoint: number | null } | null;
  /** Recent path of the selected joint, a ring buffer of world positions (oldest at `start`). */
  trail?: { xs: Float64Array; ys: Float64Array; start: number; n: number } | null;
}

const partsKey = { point: 'points', link: 'links', spring: 'springs', gear: 'gears', lever: 'gears', pid: 'gears' } as const;
/** Whether a part is the selection or one of the parts chosen together. */
function isSelected(s: Scene, kind: Selection['kind'], id: number): boolean {
  return (s.selection?.kind === kind && s.selection.id === id) || !!s.multi?.[partsKey[kind]].has(id);
}

/** Set from the scene at the start of every frame. */
let showLabels = true;

const C = {
  bg: '#f6f6f3',
  gridMinor: '#ebebe6',
  gridMajor: '#dcdcd5',
  axis: '#c9c9c0',
  ink: '#2b2d31',
  muted: '#7b7e86',
  ground: '#6b6e76',
  accent: '#e07b39',
  force: '#d64545',
  spring: '#5b6b7d',
  angle: '#6a4fb3',
  gear: ['#c99a3b', '#8f9aa6', '#7fa36b', '#b07aa1', '#5f97b8'],
  bars: ['#3b6ea8', '#4c8f6a', '#8a5aa8', '#b0763a', '#3a8f99', '#a8475a'],
};

export function barColor(id: number): string {
  return C.bars[id % C.bars.length];
}

/** Colours of joined bodies, by `link:id` / `gear:id`; set from the scene every frame. */
let groupColors = new Map<string, string>();
const linkColor = (id: number) => groupColors.get(`link:${id}`) ?? barColor(id);
const gearColor = (id: number) => groupColors.get(`gear:${id}`) ?? C.gear[id % C.gear.length];

/**
 * Bodies joined into one, directly or through others, share one colour: that of the group's
 * lowest-numbered bar, or of its lowest-numbered gear if it has no bars.
 */
function joinColors(d: Design): Map<string, string> {
  const parent = new Map<string, string>();
  const find = (k: string): string => {
    const p = parent.get(k) ?? k;
    if (p === k) return k;
    const root = find(p);
    parent.set(k, root);
    return root;
  };
  // a propeller's motor is its hub, so it shares the propeller's colour
  const key = (r: DBodyRef) => (r.kind === 'motor' && d.gears.some((g) => g.id === r.id && g.propeller) ? `gear:${r.id}` : `${r.kind}:${r.id}`);
  for (const j of d.joins ?? []) parent.set(find(key(j.a)), find(key(j.b)));
  const groups = new Map<string, DBodyRef[]>();
  for (const j of d.joins ?? [])
    for (const r of [j.a, j.b]) {
      const g = groups.get(find(key(r))) ?? [];
      if (!g.some((x) => key(x) === key(r))) g.push(r);
      groups.set(find(key(r)), g);
    }
  const out = new Map<string, string>();
  for (const members of groups.values()) {
    const bars = members.filter((m) => m.kind === 'link').map((m) => m.id);
    const color = bars.length ? barColor(Math.min(...bars)) : C.gear[Math.min(...members.map((m) => m.id)) % C.gear.length];
    for (const m of members) out.set(key(m), color);
  }
  return out;
}

export function drawScene(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, s: Scene) {
  showLabels = s.labels !== false;
  groupColors = joinColors(s.design);
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, w, h);
  drawGrid(ctx, w, h, v);
  if (s.ground) drawGroundLine(ctx, w, h, v, s.ground, s.groundSelected ?? null);

  for (const p of s.design.points) if (railOf(p)) drawRail(ctx, w, h, v, s, p);

  const phases = toothPhases(s.design);
  for (const g of s.design.gears) if (g.motor && !g.propeller) drawHousing(ctx, v, s, g); // under its gear (a propeller draws its own)
  drawGears(ctx, v, s, phases.gears);
  for (const sp of s.design.springs) drawSpring(ctx, v, s, sp);
  for (const l of s.design.links) {
    if (l.rack) drawRack(ctx, v, s, l, phases.racks.get(l.id) ?? 0);
    else drawBar(ctx, v, s, l);
  }
  if (s.preview) {
    const a = toScreen(v, s.preview.from);
    const b = toScreen(v, s.preview.to);
    ctx.save();
    ctx.strokeStyle = C.accent;
    if (s.preview.kind === 'spring') {
      ctx.lineWidth = 2;
      zigzag(ctx, a, b);
    } else if (s.preview.kind === 'propeller') {
      // the blade line through the centre, and the axis it will push along
      const [dx, dy] = [b[0] - a[0], b[1] - a[1]];
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 5]);
      line(ctx, [a[0] - dx, a[1] - dy], b);
      ctx.setLineDash([]);
      const len = Math.hypot(dx, dy) || 1;
      arrowLine(ctx, a, [a[0] + (dy / len) * len * 0.5, a[1] - (dx / len) * len * 0.5], C.accent, 2);
    } else if (s.preview.kind === 'gear' || s.preview.kind === 'wheel') {
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.arc(a[0], a[1], Math.hypot(b[0] - a[0], b[1] - a[1]), 0, Math.PI * 2);
      ctx.stroke();
    } else if (s.preview.kind === 'rack') {
      ctx.globalAlpha = 0.8;
      rackShape(ctx, v, s.preview.from, s.preview.to, s.preview.side ?? 1, Math.PI * GEAR_MODULE, 0);
      ctx.lineWidth = 1.5;
      ctx.fillStyle = 'rgba(224,123,57,0.15)';
      ctx.fill();
      ctx.stroke();
    } else {
      ctx.lineWidth = 3;
      ctx.setLineDash([8, 6]);
      line(ctx, a, b);
    }
    if (s.preview.meshAt) dot(ctx, toScreen(v, s.preview.meshAt), 4, C.accent, C.accent, 0); // where it meshes
    ctx.restore();
    dot(ctx, b, 5, '#fff', C.accent);
  }
  for (const fa of s.design.angles ?? []) drawFixedAngle(ctx, v, s, fa.a, fa.b);
  if (s.trail) drawTrail(ctx, v, s.trail);
  if (s.heat) drawHeat(ctx, v, s, s.heat);
  for (const p of s.design.points) drawJoint(ctx, v, s, p);
  for (const g of s.design.gears) if (g.motor?.pid?.sensor) drawSensor(ctx, v, s, g); // on top: its light shows
  if (s.grab) drawGrab(ctx, v, s.grab);
  if (s.railPreview) drawRailPreview(ctx, w, h, v, s.railPreview);
  if (s.box) drawBox(ctx, v, s.box);
  if (s.ghost) drawGhost(ctx, v, s.ghost);
}

/** The outline of parts about to be placed: dashed bars, springs, gears and joints in the accent colour. */
function drawGhost(ctx: CanvasRenderingContext2D, v: View, gh: { design: Design; dx: number; dy: number }) {
  const at = new Map(gh.design.points.map((p) => [p.id, toScreen(v, [p.x + gh.dx, p.y + gh.dy])]));
  ctx.save();
  ctx.strokeStyle = C.accent;
  ctx.fillStyle = 'rgba(224,123,57,0.08)';
  ctx.lineCap = 'round';
  ctx.setLineDash([6, 5]);
  ctx.lineWidth = 1.6;
  for (const g of gh.design.gears) {
    const c = at.get(g.center);
    if (!c) continue;
    const R = g.radius * v.scale;
    if (g.propeller) {
      const [ex, ey] = [Math.cos(g.propeller.angle), -Math.sin(g.propeller.angle)];
      line(ctx, [c[0] - R * ex, c[1] - R * ey], [c[0] + R * ex, c[1] + R * ey]);
      continue;
    }
    ctx.beginPath();
    ctx.arc(c[0], c[1], R, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.lineWidth = 3;
  for (const l of gh.design.links) {
    const [a, b] = [at.get(l.a), at.get(l.b)];
    if (a && b) line(ctx, a, b);
  }
  ctx.lineWidth = 1.5;
  for (const sp of gh.design.springs) {
    const [a, b] = [at.get(sp.a), at.get(sp.b)];
    if (a && b) zigzag(ctx, a, b);
  }
  ctx.setLineDash([]);
  ctx.globalAlpha = 0.9;
  for (const c of at.values()) {
    ctx.beginPath();
    ctx.arc(c[0], c[1], 4, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
  ctx.restore();
}

/**
 * A fixed angle between two bars: a double arc at the joint they share, with the angle in
 * degrees. Bars that do not touch are joined by a dashed line between their midpoints.
 */
function drawFixedAngle(ctx: CanvasRenderingContext2D, v: View, s: Scene, aId: number, bId: number) {
  const la = s.design.links.find((l) => l.id === aId);
  const lb = s.design.links.find((l) => l.id === bId);
  if (!la || !lb) return;
  const sp = (id: number) => toScreen(v, s.pos(id));
  ctx.save();
  ctx.strokeStyle = C.angle;
  ctx.fillStyle = C.angle;
  ctx.lineWidth = 1.6;
  ctx.font = '600 11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  const common = sharedJoint(la, lb);
  if (common !== null) {
    const c = sp(common);
    const end = (l: { a: number; b: number }) => sp(l.a === common ? l.b : l.a);
    const [ea, eb] = [end(la), end(lb)];
    const t0 = Math.atan2(ea[1] - c[1], ea[0] - c[0]);
    let d = Math.atan2(eb[1] - c[1], eb[0] - c[0]) - t0;
    d = Math.atan2(Math.sin(d), Math.cos(d)); // the smaller side, in (−π, π]
    for (const r of [13, 17]) {
      ctx.beginPath();
      ctx.arc(c[0], c[1], r, t0, t0 + d, d < 0);
      ctx.stroke();
    }
    if (showLabels) {
      const mid = t0 + d / 2;
      ctx.fillText(`${Math.round((Math.abs(d) * 180) / Math.PI)}°`, c[0] + 30 * Math.cos(mid), c[1] + 30 * Math.sin(mid));
    }
  } else {
    const mid = (l: { a: number; b: number }): Vec => {
      const [p, q] = [sp(l.a), sp(l.b)];
      return [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
    };
    const [ma, mb] = [mid(la), mid(lb)];
    ctx.setLineDash([4, 4]);
    line(ctx, ma, mb);
    ctx.setLineDash([]);
    for (const m of [ma, mb]) dot(ctx, m, 3, C.angle, C.angle, 0);
    if (showLabels) {
      const dir = (l: { a: number; b: number }) => Math.atan2(s.pos(l.b)[1] - s.pos(l.a)[1], s.pos(l.b)[0] - s.pos(l.a)[0]);
      let d = Math.abs(dir(lb) - dir(la)) % Math.PI;
      d = Math.min(d, Math.PI - d); // the angle between the two lines
      ctx.fillText(`∠ ${Math.round((d * 180) / Math.PI)}°`, (ma[0] + mb[0]) / 2, (ma[1] + mb[1]) / 2 - 10);
    }
  }
  ctx.restore();
}

/** The ground: a line across the view, rising at its slope, shaded and hatched below. */
/**
 * The ground: its terrain line through the waypoints, level beyond the ends, shaded and hatched
 * below. When it is selected, the waypoints show as handles to drag up and down.
 */
function drawGroundLine(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, pts: [number, number][], sel: { waypoint: number | null } | null) {
  const [x0] = toWorld(v, [0, 0]);
  const [x1] = toWorld(v, [w, 0]);
  const path: Vec[] = [
    toScreen(v, [Math.min(x0, pts[0][0]) - 1, pts[0][1]]),
    ...pts.map((p) => toScreen(v, p)),
    toScreen(v, [Math.max(x1, pts[pts.length - 1][0]) + 1, pts[pts.length - 1][1]]),
  ];
  const bottom = Math.max(...path.map((p) => p[1])) + h;
  ctx.save();
  ctx.fillStyle = sel ? 'rgba(224,123,57,0.10)' : 'rgba(107,110,118,0.10)';
  ctx.beginPath();
  ctx.moveTo(path[0][0], path[0][1]);
  for (const p of path.slice(1)) ctx.lineTo(p[0], p[1]);
  ctx.lineTo(path[path.length - 1][0], bottom);
  ctx.lineTo(path[0][0], bottom);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = sel ? C.accent : C.ground;
  ctx.lineWidth = sel ? 2.5 : 2;
  ctx.beginPath();
  ctx.moveTo(path[0][0], path[0][1]);
  for (const p of path.slice(1)) ctx.lineTo(p[0], p[1]);
  ctx.stroke();
  // hatching under each segment
  ctx.lineWidth = 1;
  ctx.globalAlpha = 0.5;
  ctx.strokeStyle = C.ground;
  for (let i = 1; i < path.length; i++) {
    const [a, b] = [path[i - 1], path[i]];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    if (len < 1 || Math.max(a[0], b[0]) < -20 || Math.min(a[0], b[0]) > w + 20) continue;
    const [ux, uy] = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
    for (let d = 0; d < len; d += 14) {
      const p: Vec = [a[0] + ux * d, a[1] + uy * d];
      if (p[0] < -20 || p[0] > w + 20) continue;
      line(ctx, p, [p[0] - 8, p[1] + 10]);
    }
  }
  ctx.globalAlpha = 1;
  if (sel)
    pts.forEach((p, i) => {
      const [sx, sy] = toScreen(v, p);
      ctx.fillStyle = i === sel.waypoint ? C.accent : '#fff';
      ctx.strokeStyle = C.accent;
      ctx.lineWidth = 2;
      ctx.fillRect(sx - 5, sy - 5, 10, 10);
      ctx.strokeRect(sx - 5, sy - 5, 10, 10);
    });
  ctx.restore();
}

/** The selection box: a dashed rectangle with a light fill. */
function drawBox(ctx: CanvasRenderingContext2D, v: View, b: { from: Vec; to: Vec }) {
  const [a, c] = [toScreen(v, b.from), toScreen(v, b.to)];
  ctx.save();
  ctx.fillStyle = 'rgba(224,123,57,0.08)';
  ctx.strokeStyle = C.accent;
  ctx.lineWidth = 1.2;
  ctx.setLineDash([5, 4]);
  ctx.fillRect(Math.min(a[0], c[0]), Math.min(a[1], c[1]), Math.abs(c[0] - a[0]), Math.abs(c[1] - a[1]));
  ctx.strokeRect(Math.min(a[0], c[0]) + 0.5, Math.min(a[1], c[1]) + 0.5, Math.abs(c[0] - a[0]), Math.abs(c[1] - a[1]));
  ctx.restore();
}

/** The selected joint's path; once it is long, its oldest part fades out. */
function drawTrail(ctx: CanvasRenderingContext2D, v: View, t: NonNullable<Scene['trail']>) {
  const max = t.xs.length;
  const last = t.n - 1;
  const BANDS = 24;
  const per = Math.ceil(last / BANDS);
  const fadeLen = max / 4; // the oldest quarter of a full buffer fades out
  ctx.save();
  ctx.strokeStyle = C.accent;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  for (let i0 = 0; i0 < last; i0 += per) {
    const i1 = Math.min(last, i0 + per);
    // fade by how soon a point is dropped from the buffer: short paths stay solid
    ctx.globalAlpha = 0.1 + 0.75 * Math.min(1, (i1 + max - t.n) / fadeLen);
    ctx.beginPath();
    for (let i = i0; i <= i1; i++) {
      const k = (t.start + i) % max;
      const x = v.ox + t.xs[k] * v.scale;
      const y = v.oy - t.ys[k] * v.scale;
      if (i === i0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.restore();
}

function line(ctx: CanvasRenderingContext2D, a: Vec, b: Vec) {
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
}

function dot(ctx: CanvasRenderingContext2D, [x, y]: Vec, r: number, fill: string, stroke: string, width = 2) {
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = fill;
  ctx.fill();
  ctx.lineWidth = width;
  ctx.strokeStyle = stroke;
  ctx.stroke();
}

function drawGrid(ctx: CanvasRenderingContext2D, w: number, h: number, v: View) {
  // finer grids as you zoom in (down to millimetres, for small parts such as a knee's ligaments)
  const step = v.scale >= 12000 ? 0.001 : v.scale >= 1200 ? 0.01 : v.scale >= 120 ? 0.1 : v.scale >= 40 ? 0.5 : 1;
  const major = (u: number) => Math.abs(u / (step * 10) - Math.round(u / (step * 10))) < 1e-6;
  const [x0, y1] = toWorld(v, [0, 0]);
  const [x1, y0] = toWorld(v, [w, h]);
  ctx.lineWidth = 1;
  for (let i = Math.floor(x0 / step); i <= Math.ceil(x1 / step); i++) {
    const x = i * step;
    const sx = Math.round(v.ox + x * v.scale) + 0.5;
    ctx.strokeStyle = i === 0 ? C.axis : (step >= 0.5 ? Math.abs(x - Math.round(x)) < 1e-9 : major(x)) ? C.gridMajor : C.gridMinor;
    line(ctx, [sx, 0], [sx, h]);
  }
  for (let j = Math.floor(y0 / step); j <= Math.ceil(y1 / step); j++) {
    const y = j * step;
    const sy = Math.round(v.oy - y * v.scale) + 0.5;
    ctx.strokeStyle = j === 0 ? C.axis : (step >= 0.5 ? Math.abs(y - Math.round(y)) < 1e-9 : major(y)) ? C.gridMajor : C.gridMinor;
    line(ctx, [0, sy], [w, sy]);
  }
  // scale bar
  ctx.fillStyle = C.muted;
  ctx.strokeStyle = C.muted;
  ctx.lineWidth = 2;
  // the longest round length that fits in about 200 px
  const bar = [1, 0.5, 0.1, 0.05, 0.01, 0.005, 0.001].find((m) => m * v.scale <= 200) ?? 0.001;
  const len = v.scale * bar;
  line(ctx, [w - 30 - len, h - 24], [w - 30, h - 24]);
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(bar >= 0.1 ? `${bar} m` : `${Math.round(bar * 1000)} mm`, w - 30 - len / 2, h - 30);
}

/**
 * How a body has moved since it was drawn: its rotation and the map from drawn positions to
 * where they are now (a bar by its two ends, a gear by its centre and turn).
 */
function bodyMotion(s: Scene, ref: DBodyRef): { turn: number; map: (p: Vec) => Vec } | null {
  const at = (id: number) => s.design.points.find((q) => q.id === id);
  let o0: Vec;
  let o1: Vec;
  let turn: number;
  if (ref.kind === 'link') {
    const l = s.design.links.find((x) => x.id === ref.id);
    const [a, b] = l ? [at(l.a), at(l.b)] : [undefined, undefined];
    if (!l || !a || !b) return null;
    [o0, o1] = [[a.x, a.y], s.pos(l.a)];
    const [pa, pb] = [s.pos(l.a), s.pos(l.b)];
    turn = Math.atan2(pb[1] - pa[1], pb[0] - pa[0]) - Math.atan2(b.y - a.y, b.x - a.x);
  } else {
    const g = s.design.gears.find((x) => x.id === ref.id);
    const c = g && at(g.center);
    if (!g || !c) return null;
    [o0, o1] = [[c.x, c.y], s.pos(g.center)];
    turn = ref.kind === 'motor' ? (s.motorAngle?.(g.id) ?? 0) : s.gearAngle(g.id);
  }
  const [cs, sn] = [Math.cos(turn), Math.sin(turn)];
  return { turn, map: ([x, y]) => [o1[0] + (x - o0[0]) * cs - (y - o0[1]) * sn, o1[1] + (x - o0[0]) * sn + (y - o0[1]) * cs] };
}

/** A joint's rail: across the view for a ground rail, a dashed track on the body otherwise. */
function drawRail(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, s: Scene, p: DPoint) {
  const r = railOf(p)!;
  const a = (r.angle * Math.PI) / 180;
  ctx.save();
  ctx.lineWidth = 2;
  ctx.setLineDash([10, 6]);
  const range = p.slide?.range;
  if (!r.on && range) {
    // a ground rail with stops: the track between them, and a stop bar at each end
    const far = (w + h) / v.scale;
    const [lo, hi] = [range.min ?? -far, range.max ?? far];
    const at = (t: number): Vec => toScreen(v, [p.x + Math.cos(a) * t, p.y + Math.sin(a) * t]);
    ctx.strokeStyle = C.ground;
    ctx.globalAlpha = 0.55;
    line(ctx, at(lo), at(hi));
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);
    for (const t of [range.min, range.max]) if (t !== null) drawStop(ctx, at(t), a, C.ground);
  } else if (!r.on) {
    // through the drawn position, long enough to cross the view
    const [sx, sy] = toScreen(v, [p.x, p.y]);
    const L = w + h;
    const [dx, dy] = [Math.cos(a) * L, -Math.sin(a) * L];
    ctx.strokeStyle = C.ground;
    ctx.globalAlpha = 0.55;
    line(ctx, [sx - dx, sy - dy], [sx + dx, sy + dy]);
  } else {
    const m = bodyMotion(s, r.on);
    if (m) {
      const half = r.on.kind === 'gear' ? (s.design.gears.find((g) => g.id === r.on!.id)?.radius ?? 0.1) * 1.4 : 0.25;
      const [lo, hi] = [range?.min ?? -half, range?.max ?? half];
      const along = (t: number) => toScreen(v, m.map([p.x + Math.cos(a) * t, p.y + Math.sin(a) * t]));
      const color = r.on.kind === 'link' ? linkColor(r.on.id) : r.on.kind === 'motor' ? (groupColors.get(`motor:${r.on.id}`) ?? C.ink) : gearColor(r.on.id);
      ctx.strokeStyle = color;
      ctx.lineCap = 'round';
      line(ctx, along(lo), along(hi));
      ctx.setLineDash([]);
      for (const t of [range?.min ?? null, range?.max ?? null]) if (t !== null) drawStop(ctx, along(t), a + m.turn, color);
    }
  }
  ctx.restore();
}

/** An end stop on a rail at angle `a` (world radians): a short thick bar across the rail. */
function drawStop(ctx: CanvasRenderingContext2D, [x, y]: Vec, a: number, color: string) {
  const [nx, ny] = [Math.sin(a) * 9, Math.cos(a) * 9]; // across the rail, in screen space (y down)
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 4;
  ctx.lineCap = 'round';
  line(ctx, [x - nx, y - ny], [x + nx, y + ny]);
  ctx.restore();
}

/** The rail being placed: a dashed accent line through the joint, with its angle and any ends chosen. */
function drawRailPreview(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, r: { at: Vec; angle: number; ends?: number[] }) {
  const [sx, sy] = toScreen(v, r.at);
  const a = (r.angle * Math.PI) / 180;
  const L = w + h;
  ctx.save();
  ctx.strokeStyle = C.accent;
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 6]);
  line(ctx, [sx - Math.cos(a) * L, sy + Math.sin(a) * L], [sx + Math.cos(a) * L, sy - Math.sin(a) * L]);
  ctx.setLineDash([]);
  ctx.fillStyle = C.accent;
  ctx.font = '600 12px system-ui, sans-serif';
  ctx.fillText(`${(((r.angle % 180) + 180) % 180).toFixed(0)}°`, sx + 14, sy - 14);
  for (const t of r.ends ?? []) {
    const p = toScreen(v, [r.at[0] + Math.cos(a) * t, r.at[1] + Math.sin(a) * t]);
    drawStop(ctx, p, a, C.accent);
    ctx.fillText(`${t >= 0 ? '+' : ''}${t.toFixed(2)} m`, p[0] + 10, p[1] + 16);
  }
  ctx.restore();
}

function drawBar(ctx: CanvasRenderingContext2D, v: View, s: Scene, l: DLink) {
  const a = toScreen(v, s.pos(l.a));
  const b = toScreen(v, s.pos(l.b));
  const selected = isSelected(s, 'link', l.id);
  const hovered = s.hover?.kind === 'link' && s.hover.id === l.id;
  ctx.save();
  ctx.lineCap = 'round';
  if (selected || hovered) {
    ctx.strokeStyle = C.accent;
    ctx.globalAlpha = selected ? 0.45 : 0.25;
    ctx.lineWidth = 22;
    line(ctx, a, b);
    ctx.globalAlpha = 1;
  }
  // very light bars (under 5 g: ligaments, tendon anchors) are drawn thin
  const thin = l.mass < 0.005;
  ctx.strokeStyle = '#1f2125';
  ctx.lineWidth = thin ? 5.5 : 13;
  line(ctx, a, b);
  ctx.strokeStyle = linkColor(l.id);
  ctx.lineWidth = thin ? 3.5 : 10;
  line(ctx, a, b);
  // label
  const mx = (a[0] + b[0]) / 2;
  const my = (a[1] + b[1]) / 2;
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
  const nx = -(b[1] - a[1]) / len;
  const ny = (b[0] - a[0]) / len;
  ctx.font = '600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = C.ink;
  if (showLabels) ctx.fillText(linkLabel(l.id), mx + nx * 18, my + ny * 18);
  ctx.restore();
}

/** A coil spring drawn as a zigzag with straight leads at both ends. */
function zigzag(ctx: CanvasRenderingContext2D, a: Vec, b: Vec, coils = 7, width = 9) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const len = Math.hypot(dx, dy);
  if (len < 1) return;
  const ux = dx / len;
  const uy = dy / len;
  const lead = Math.min(14, len * 0.15);
  const body = len - 2 * lead;
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(a[0] + ux * lead, a[1] + uy * lead);
  const n = coils * 2;
  for (let i = 1; i < n; i++) {
    const t = lead + (body * i) / n;
    const side = i % 2 ? width : -width;
    ctx.lineTo(a[0] + ux * t - uy * side, a[1] + uy * t + ux * side);
  }
  ctx.lineTo(b[0] - ux * lead, b[1] - uy * lead);
  ctx.lineTo(b[0], b[1]);
  ctx.lineJoin = 'round';
  ctx.stroke();
}

/** Where a rack lies: its start, direction e, left normal n, length and angle. */
function rackFrame(a: Vec, b: Vec) {
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1e-9;
  const e: Vec = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
  return { a, e, n: [-e[1], e[0]] as Vec, L, angle: Math.atan2(e[1], e[0]) };
}

/** A gear centre c relative to a rack: ξ along it, and the side (±1) it is on. */
function onRack(f: ReturnType<typeof rackFrame>, c: Vec) {
  const r: Vec = [c[0] - f.a[0], c[1] - f.a[1]];
  const h = r[0] * f.n[0] + r[1] * f.n[1];
  return { xi: r[0] * f.e[0] + r[1] * f.e[1], side: h >= 0 ? 1 : -1 };
}

/**
 * Tooth phase of every gear, and tooth offset along every rack, in the design configuration,
 * chosen so that each meshing pair interlocks at its pitch point (a tooth of one in a gap of
 * the other). Meshes are visited breadth-first; the mesh constraints then keep them interlocked.
 *
 * A gear tooth at angle α from the direction of the contact point reaches the rack, as the gear
 * rolls, at ξ + s R α along it. A rack's teeth sit at offset + k p from its joint a.
 */
function toothPhases(d: Design): { gears: Map<number, number>; racks: Map<number, number> } {
  const phase = new Map<number, number>();
  const offset = new Map<number, number>();
  const at = (id: number): Vec => {
    const p = d.points.find((q) => q.id === id)!;
    return [p.x, p.y];
  };
  const gearById = new Map(d.gears.map((g) => [g.id, g]));
  const rackById = new Map(d.links.filter((l) => l.rack).map((l) => [l.id, l]));
  type Node = { kind: 'gear' | 'rack'; id: number };
  const visit = (root: Node) => {
    const queue = [root];
    while (queue.length) {
      const node = queue.shift()!;
      if (node.kind === 'gear') {
        const ga = gearById.get(node.id)!;
        for (const m of d.meshes) {
          const gb = gearById.get(m.a === ga.id ? m.b : m.b === ga.id ? m.a : -1);
          if (!gb || phase.has(gb.id)) continue;
          const [ca, cb] = [at(ga.center), at(gb.center)];
          const psi = Math.atan2(cb[1] - ca[1], cb[0] - ca[0]);
          const pa = (2 * Math.PI) / ga.teeth;
          const pb = (2 * Math.PI) / gb.teeth;
          const t = (psi - phase.get(ga.id)!) / pa;
          const f = t - Math.round(t); // where gear a's nearest tooth sits, in pitches from the contact line
          phase.set(gb.id, psi + Math.PI + pb / 2 - f * pb);
          queue.push({ kind: 'gear', id: gb.id });
        }
        for (const m of d.rackMeshes) {
          const r = m.gear === ga.id ? rackById.get(m.rack) : undefined;
          if (!r || offset.has(r.id)) continue;
          const f = rackFrame(at(r.a), at(r.b));
          const { xi, side } = onRack(f, at(ga.center));
          const alpha = phase.get(ga.id)! - f.angle + (side * Math.PI) / 2;
          offset.set(r.id, xi + side * ga.radius * alpha - rackPitch(d, r.id) / 2); // a rack gap meets the tooth
          queue.push({ kind: 'rack', id: r.id });
        }
      } else {
        const r = rackById.get(node.id)!;
        const f = rackFrame(at(r.a), at(r.b));
        for (const m of d.rackMeshes) {
          const g = m.rack === r.id ? gearById.get(m.gear) : undefined;
          if (!g || phase.has(g.id)) continue;
          const { xi, side } = onRack(f, at(g.center));
          const tooth = offset.get(r.id)! + rackPitch(d, r.id) / 2; // a gap of the rack
          phase.set(g.id, (tooth - xi) / (side * g.radius) + f.angle - (side * Math.PI) / 2);
          queue.push({ kind: 'gear', id: g.id });
        }
      }
    }
  };
  for (const g of d.gears) if (!phase.has(g.id) && d.points.some((p) => p.id === g.center)) (phase.set(g.id, 0), visit({ kind: 'gear', id: g.id }));
  for (const r of rackById.values()) if (!offset.has(r.id)) (offset.set(r.id, 0), visit({ kind: 'rack', id: r.id }));
  return { gears: phase, racks: offset };
}

/**
 * Outline of a rack from a to b (world), its pitch line on ab: teeth of pitch p on side `side`
 * centred at offset + k p from a, and the body on the other side. Leaves the path open for fill/stroke.
 */
function rackShape(ctx: CanvasRenderingContext2D, v: View, a: Vec, b: Vec, side: number, p: number, offset: number) {
  const f = rackFrame(a, b);
  const m = p / Math.PI; // module
  const tip = side * m;
  const root = -side * 1.25 * m;
  const back = -side * (1.25 * m + Math.max(0.6 * m, 0.025)); // far edge of the body
  const world = (u: number, h: number) => toScreen(v, [a[0] + f.e[0] * u + f.n[0] * h, a[1] + f.e[1] * u + f.n[1] * h]);
  const profile: [number, number][] = [[-0.5, root], [-0.3, root], [-0.16, tip], [0.16, tip], [0.3, root]];
  const pts: Vec[] = [world(0, back), world(0, root)];
  const k0 = Math.floor(-offset / p) - 1;
  const k1 = Math.ceil((f.L - offset) / p) + 1;
  for (let k = k0; k <= k1; k++)
    for (const [frac, h] of profile) {
      const u = offset + (k + frac) * p;
      if (u > 0 && u < f.L) pts.push(world(u, h));
    }
  pts.push(world(f.L, root), world(f.L, back));
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.closePath();
}

/** A rack: a toothed bar. Its joints sit on the pitch line (dashed). */
function drawRack(ctx: CanvasRenderingContext2D, v: View, s: Scene, l: DLink, offset: number) {
  const A = s.pos(l.a);
  const B = s.pos(l.b);
  const side = l.rack ?? 1;
  const selected = isSelected(s, 'link', l.id);
  const hovered = s.hover?.kind === 'link' && s.hover.id === l.id;
  const color = linkColor(l.id);
  ctx.save();
  ctx.lineJoin = 'round';
  rackShape(ctx, v, A, B, side, rackPitch(s.design, l.id), offset);
  if (selected || hovered) {
    ctx.strokeStyle = C.accent;
    ctx.globalAlpha = selected ? 0.45 : 0.25;
    ctx.lineWidth = 10;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = color;
  ctx.globalAlpha = 0.35;
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = '#1f2125';
  ctx.stroke();
  const a = toScreen(v, A);
  const b = toScreen(v, B);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 4]);
  line(ctx, a, b);
  ctx.setLineDash([]);
  // label on the body side
  const f = rackFrame(A, B);
  const m = rackPitch(s.design, l.id) / Math.PI;
  const off = -side * (1.25 * m + Math.max(0.6 * m, 0.025)) * v.scale - side * 12;
  const [mx, my] = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  ctx.font = '600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = C.ink;
  if (showLabels) ctx.fillText(linkLabel(l.id), mx + f.n[0] * off, my - f.n[1] * off);
  ctx.restore();
}

function drawGears(ctx: CanvasRenderingContext2D, v: View, s: Scene, phases: Map<number, number>) {
  for (const g of s.design.gears) {
    if (!s.design.points.some((p) => p.id === g.center)) continue;
    const c = s.pos(g.center);
    const sc = toScreen(v, c);
    const color = gearColor(g.id);
    const selected = isSelected(s, 'gear', g.id);
    const hovered = s.hover?.kind === 'gear' && s.hover.id === g.id;
    const angle = s.gearAngle(g.id);
    if (g.propeller) {
      drawPropeller(ctx, v, s, g, color, selected || hovered);
      continue;
    }
    if (g.wheel) drawWheel(ctx, v, c, g.radius, angle, color, selected || hovered, g.locked);
    else drawGear(ctx, v, c, g.radius, g.teeth, phases.get(g.id)! + angle, color, selected || hovered, g.locked);
    // spokes to the joints pinned on this gear show that they turn with it
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    for (const p of s.design.points) if (p.gear === g.id && p.id !== g.center) line(ctx, sc, toScreen(v, s.pos(p.id)));
    ctx.font = 'italic 600 12px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    if (showLabels) ctx.fillText(`${g.wheel ? 'w' : 'g'}${gearLabel(g.id)}`, sc[0] - 14, sc[1] + 20);
    ctx.restore();
    if (g.locked) drawPadlock(ctx, [sc[0] - 20, sc[1] - 20]);
    if (g.motor) drawMotor(ctx, sc, Math.min(g.radius * v.scale * 0.6, 34), g.motor.torque);
  }
}

/**
 * The torque arrow of a motor on its gear: a curved red arrow around the axle in the direction
 * of the torque (drawn in screen space, where y points down, so anticlockwise in the world is
 * anticlockwise here too). The motor's housing is drawn separately, under the gear.
 */
function drawMotor(ctx: CanvasRenderingContext2D, [x, y]: Vec, r: number, torque: number) {
  if (torque === 0) return;
  r = Math.max(r, 14);
  const ccw = torque > 0; // world anticlockwise = screen anticlockwise (both flip together)
  const [a0, a1] = ccw ? [0.35 * Math.PI, -1.15 * Math.PI] : [0.65 * Math.PI, 2.15 * Math.PI];
  ctx.save();
  ctx.strokeStyle = C.force;
  ctx.fillStyle = C.force;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(x, y, r, a0, a1, ccw);
  ctx.stroke();
  // arrowhead at the end of the arc, pointing along it
  const [ex, ey] = [x + r * Math.cos(a1), y + r * Math.sin(a1)];
  const dir = ccw ? -1 : 1; // direction of travel along the arc in screen angle
  const [tx, ty] = [-Math.sin(a1) * dir, Math.cos(a1) * dir];
  const [nx, ny] = [-ty, tx];
  ctx.beginPath();
  ctx.moveTo(ex + tx * 7, ey + ty * 7);
  ctx.lineTo(ex - tx * 3 + nx * 5, ey - ty * 3 + ny * 5);
  ctx.lineTo(ex - tx * 3 - nx * 5, ey - ty * 3 - ny * 5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/**
 * A motor's housing: a rounded box from the axle out to one side (to the right as drawn),
 * turning with the housing, with an "M" on it. In its group's colour when joined to other bodies,
 * dark grey otherwise, and hatched underneath when mounted on the ground.
 */
function drawHousing(ctx: CanvasRenderingContext2D, v: View, s: Scene, g: Design['gears'][number]) {
  const c = toScreen(v, s.pos(g.center));
  const turn = s.motorAngle?.(g.id) ?? 0;
  const r = Math.max(motorHousingRadius(g) * v.scale, 9);
  const joined = groupColors.get(`motor:${g.id}`);
  const selected = isSelected(s, 'gear', g.id);
  const hovered = s.hover?.kind === 'gear' && s.hover.id === g.id;
  ctx.save();
  ctx.translate(c[0], c[1]);
  ctx.rotate(-turn); // the screen's y points down
  const [x0, x1, hh] = [-r * 0.9, r * 2.6, r * 0.9];
  if (selected || hovered) {
    ctx.fillStyle = C.accent;
    ctx.globalAlpha = selected ? 0.35 : 0.2;
    ctx.beginPath();
    ctx.roundRect(x0 - 6, -hh - 6, x1 - x0 + 12, 2 * hh + 12, 8);
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = joined ?? '#5d6068';
  ctx.strokeStyle = C.ink;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(x0, -hh, x1 - x0, 2 * hh, Math.min(6, hh));
  ctx.fill();
  ctx.stroke();
  // cooling fins and the end cap
  ctx.strokeStyle = 'rgba(255,255,255,0.35)';
  ctx.lineWidth = 1;
  for (let k = 1; k <= 3; k++) line(ctx, [r * (1.0 + 0.25 * k), -hh + 3], [r * (1.0 + 0.25 * k), hh - 3]);
  ctx.fillStyle = '#fff';
  ctx.font = `700 ${Math.round(Math.min(14, hh))}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('M', r * 2.15, 0.5); // out beyond the gear's rim
  ctx.restore();
  if (g.motor?.pid) drawPid(ctx, v, s, g);
  if (leverOf(g.motor)) drawLever(ctx, v, s, g);
  if (g.motor?.mounted !== false && !joined) {
    // mounted on the ground: a short hatched base under the far end
    const end = toScreen(v, [s.pos(g.center)[0] + 2 * motorHousingRadius(g), s.pos(g.center)[1] - motorHousingRadius(g)]);
    ctx.save();
    ctx.strokeStyle = C.ground;
    ctx.lineWidth = 1.5;
    line(ctx, [end[0] - 10, end[1] + 2], [end[0] + 10, end[1] + 2]);
    for (let k = -8; k <= 8; k += 4) line(ctx, [end[0] + k, end[1] + 2], [end[0] + k - 4, end[1] + 7]);
    ctx.restore();
  }
}

/**
 * Where the lever on a motor's housing is drawn (screen): its pivot on top of the housing's far
 * end, the knob, and the knob's radius. It tilts with its position, up to 34° either way
 * (forward leans away from the axle), and turns with the housing.
 */
export function leverGeometry(v: View, centre: Vec, turn: number, g: Design['gears'][number]): { base: Vec; knob: Vec; radius: number } {
  if (g.motor?.pid?.lever) {
    // on the PID controller's outer side, sticking out sideways and leaning up for a higher target
    const P = pidGeometry(v, centre, turn, g);
    const t = g.motor.pid.lever.position * 0.6;
    const local: [Vec, Vec] = [[P.side * P.w * 0.5, 0], [P.side * (P.w * 0.5 + 20 * Math.cos(t)), -20 * Math.sin(t)]];
    const [cs, sn] = [Math.cos(P.rot), Math.sin(P.rot)];
    const [base, knob] = local.map(([x, y]): Vec => [P.c[0] + x * cs - y * sn, P.c[1] + x * sn + y * cs]);
    return { base, knob, radius: 6 };
  }
  if (g.propeller) {
    // on the side of the propeller's motor, leaning towards the blades for forward
    const th = g.propeller.angle + turn;
    const [e, n] = [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]];
    const R = g.radius;
    const base = toScreen(v, [centre[0] + 0.13 * R * e[0] - 0.32 * R * n[0], centre[1] + 0.13 * R * e[1] - 0.32 * R * n[1]]);
    const t = (g.motor?.lever?.position ?? 0) * 0.6;
    const [dx, dy] = [Math.cos(t) * e[0] + Math.sin(t) * n[0], Math.cos(t) * e[1] + Math.sin(t) * n[1]];
    const len = Math.max(0.4 * R * v.scale, 18);
    return { base, knob: [base[0] + len * dx, base[1] - len * dy], radius: Math.min(Math.max(0.1 * R * v.scale, 6), 12) };
  }
  const c = toScreen(v, centre);
  const r = Math.max(motorHousingRadius(g) * v.scale, 9);
  const len = Math.max(r * 1.1, 18);
  const tilt = (g.motor?.lever?.position ?? 0) * 0.6;
  // in the housing's frame (screen axes: y down), then turned by −turn like the housing
  const local: [Vec, Vec] = [[r * 2.15, -r * 0.9], [r * 2.15 + len * Math.sin(tilt), -r * 0.9 - len * Math.cos(tilt)]];
  const [cs, sn] = [Math.cos(-turn), Math.sin(-turn)];
  const [base, knob] = local.map(([x, y]): Vec => [c[0] + x * cs - y * sn, c[1] + x * sn + y * cs]);
  return { base, knob, radius: Math.min(Math.max(r * 0.28, 6), 14) };
}

/**
 * Where a motor's PID controller is drawn (screen): a small box under the far end of a gear's
 * motor housing (beside a propeller's motor), its centre, size, turn (screen radians) and the
 * side its lever sticks out of.
 */
export function pidGeometry(v: View, centre: Vec, turn: number, g: Design['gears'][number]): { c: Vec; w: number; h: number; rot: number; side: 1 | -1 } {
  const [w, h] = [30, 15];
  if (g.propeller) {
    const th = g.propeller.angle + turn;
    const [e, n] = [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]];
    const R = g.radius;
    const at = toScreen(v, [centre[0] - 0.13 * R * e[0] - 0.32 * R * n[0], centre[1] - 0.13 * R * e[1] - 0.32 * R * n[1]]);
    const [ex, ey] = [e[0], -e[1]]; // e on screen
    return { c: [at[0] - (w / 2 + 3) * ex, at[1] - (w / 2 + 3) * ey], w, h, rot: Math.atan2(ey, ex), side: -1 };
  }
  const c = toScreen(v, centre);
  const r = Math.max(motorHousingRadius(g) * v.scale, 9);
  const [x, y] = [r * 2.15, r * 0.9 + h / 2 + 3];
  const [cs, sn] = [Math.cos(-turn), Math.sin(-turn)];
  return { c: [c[0] + x * cs - y * sn, c[1] + x * sn + y * cs], w, h, rot: -turn, side: 1 };
}

/** The motor's turn the PID box and its lever ride on: the housing's, or a propeller's hub. */
const motorTurn = (s: Scene, g: Design['gears'][number]) => (g.propeller ? s.gearAngle(g.id) : (s.motorAngle?.(g.id) ?? 0));

/** A PID controller: a dark box with its name and a light in its colour. */
function drawPid(ctx: CanvasRenderingContext2D, v: View, s: Scene, g: Design['gears'][number]) {
  const pid = g.motor!.pid!;
  const P = pidGeometry(v, s.pos(g.center), motorTurn(s, g), g);
  const selected = isSelected(s, 'pid', g.id);
  const hovered = s.hover?.kind === 'pid' && s.hover.id === g.id;
  ctx.save();
  ctx.translate(P.c[0], P.c[1]);
  ctx.rotate(P.rot);
  if (selected || hovered) {
    ctx.fillStyle = C.accent;
    ctx.globalAlpha = selected ? 0.4 : 0.25;
    ctx.beginPath();
    ctx.roundRect(-P.w / 2 - 5, -P.h / 2 - 5, P.w + 10, P.h + 10, 7);
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = '#26282d';
  ctx.strokeStyle = pid.color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-P.w / 2, -P.h / 2, P.w, P.h, 4);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = '700 8.5px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('PID', 3, 0.5);
  ctx.fillStyle = pid.sensor ? pid.color : '#777';
  ctx.beginPath();
  ctx.arc(-P.w / 2 + 5, 0, 2.2, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

/** Where a sensor sits (world): on its joint, at its bar's middle, or at its gear's centre. */
function sensorAt(s: Scene, sensor: NonNullable<DPid['sensor']>): Vec | null {
  const { kind, id } = sensor.body;
  const has = (pid: number) => s.design.points.some((p) => p.id === pid);
  if (kind === 'point') return has(id) ? s.pos(id) : null;
  if (kind === 'link') {
    const l = s.design.links.find((x) => x.id === id);
    if (!l || !has(l.a) || !has(l.b)) return null;
    const [a, b] = [s.pos(l.a), s.pos(l.b)];
    return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  }
  const g = s.design.gears.find((x) => x.id === id);
  return g && has(g.center) ? s.pos(g.center) : null;
}

const QUANTITY_MARK: Record<string, string> = { x: 'x', y: 'y', angle: 'θ', vx: 'ẋ', vy: 'ẏ', omega: 'θ̇', spin: 'rpm' };

/** A sensor: a little box with a flashing light in its controller's colour, wired to the controller. */
function drawSensor(ctx: CanvasRenderingContext2D, v: View, s: Scene, g: Design['gears'][number]) {
  const pid = g.motor!.pid!;
  const at = sensorAt(s, pid.sensor!);
  if (!at) return;
  const sc = toScreen(v, at);
  const p: Vec = [sc[0] + 11, sc[1] - 11]; // beside the joint or centre, not on it
  const P = pidGeometry(v, s.pos(g.center), motorTurn(s, g), g);
  ctx.save();
  ctx.strokeStyle = pid.color;
  ctx.globalAlpha = 0.45;
  ctx.lineWidth = 1.2;
  ctx.setLineDash([4, 4]);
  line(ctx, p, P.c);
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
  ctx.fillStyle = '#26282d';
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(p[0] - 7, p[1] - 7, 14, 14, 3);
  ctx.fill();
  ctx.stroke();
  // the light flashes twice a second
  const on = (performance.now() / 1000) % 0.5 < 0.18;
  ctx.fillStyle = pid.color;
  ctx.globalAlpha = on ? 1 : 0.3;
  if (on) {
    ctx.shadowColor = pid.color;
    ctx.shadowBlur = 10;
  }
  ctx.beginPath();
  ctx.arc(p[0], p[1], 3.6, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.globalAlpha = 1;
  ctx.font = 'italic 600 11px system-ui, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = pid.color;
  ctx.fillText(QUANTITY_MARK[pid.sensor!.quantity], p[0] + 10, p[1]);
  ctx.restore();
}

function drawLever(ctx: CanvasRenderingContext2D, v: View, s: Scene, g: Design['gears'][number]) {
  const lever = leverOf(g.motor)!.lever;
  const { base, knob, radius } = leverGeometry(v, s.pos(g.center), motorTurn(s, g), g);
  const selected = isSelected(s, 'lever', g.id);
  const hovered = s.hover?.kind === 'lever' && s.hover.id === g.id;
  ctx.save();
  ctx.lineCap = 'round';
  ctx.strokeStyle = C.ink;
  ctx.lineWidth = Math.max(2.5, radius * 0.45);
  line(ctx, base, knob);
  ctx.fillStyle = C.ink; // the pivot
  ctx.beginPath();
  ctx.arc(base[0], base[1], Math.max(3, radius * 0.5), 0, Math.PI * 2);
  ctx.fill();
  if (selected || hovered) {
    ctx.fillStyle = C.accent;
    ctx.globalAlpha = selected ? 0.4 : 0.25;
    ctx.beginPath();
    ctx.arc(knob[0], knob[1], radius + 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = lever.color;
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(knob[0], knob[1], radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

/** Small padlock marking a gear that is locked against turning. */
function drawPadlock(ctx: CanvasRenderingContext2D, [x, y]: Vec) {
  ctx.save();
  ctx.strokeStyle = C.ink;
  ctx.fillStyle = C.ink;
  ctx.lineWidth = 1.8;
  ctx.beginPath();
  ctx.arc(x, y - 3, 3.6, Math.PI, 0);
  ctx.lineTo(x + 3.6, y);
  ctx.moveTo(x - 3.6, y);
  ctx.lineTo(x - 3.6, y - 3);
  ctx.stroke();
  ctx.beginPath();
  ctx.roundRect(x - 5.5, y, 11, 8, 1.5);
  ctx.fill();
  ctx.restore();
}

/** A line with an arrowhead at its end (screen coordinates). */
function arrowLine(ctx: CanvasRenderingContext2D, a: Vec, b: Vec, color: string, width: number) {
  const [dx, dy] = [b[0] - a[0], b[1] - a[1]];
  const len = Math.hypot(dx, dy);
  if (len < 2) return;
  const [ux, uy] = [dx / len, dy / len];
  const head = Math.min(9, len * 0.5);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = 'round';
  line(ctx, a, [b[0] - ux * head * 0.6, b[1] - uy * head * 0.6]);
  ctx.beginPath();
  ctx.moveTo(b[0], b[1]);
  ctx.lineTo(b[0] - ux * head - uy * head * 0.5, b[1] - uy * head + ux * head * 0.5);
  ctx.lineTo(b[0] - ux * head + uy * head * 0.5, b[1] - uy * head - ux * head * 0.5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/**
 * A propeller, projected onto the plane. Its spin axis n lies in the plane, square to the blade
 * line e; blade k, at spin angle a = φ + 2πk/N, points along cos a e + sin a z, so it shows as a
 * blade foreshortened to cos a of its length, and its chord (pitched 20° to the disk) as
 * −cos β sin a e + sin β n. Blades behind the plane (sin a < 0) are drawn first, paler. Below
 * the hub sits its motor, if it has one; an arrow along the axis shows the thrust while running
 * (which way it will push while editing).
 */
function drawPropeller(ctx: CanvasRenderingContext2D, v: View, s: Scene, g: Design['gears'][number], color: string, highlight: boolean) {
  const P = g.propeller!;
  const c = s.pos(g.center);
  const R = g.radius;
  const th = P.angle + s.gearAngle(g.id);
  const phi = s.spinAngle?.(g.id) ?? 0;
  const e: Vec = [Math.cos(th), Math.sin(th)];
  const n: Vec = [-Math.sin(th), Math.cos(th)];
  const at = (u: number, w: number): Vec => toScreen(v, [c[0] + u * e[0] + w * n[0], c[1] + u * e[1] + w * n[1]]);
  const N = Math.max(2, Math.min(6, Math.round(P.blades)));
  const beta = (20 * Math.PI) / 180;
  const hand = P.hand ?? 1;
  ctx.save();
  if (highlight) {
    ctx.strokeStyle = C.accent;
    ctx.globalAlpha = 0.3;
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    line(ctx, at(-R, 0), at(R, 0));
    ctx.globalAlpha = 1;
  }
  // the rotor disk, edge-on
  ctx.strokeStyle = color;
  ctx.globalAlpha = 0.5;
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 4]);
  line(ctx, at(-R, 0), at(R, 0));
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
  // motor under the hub
  if (g.motor) {
    const joined = groupColors.get(`gear:${g.id}`);
    const box = [at(-0.13 * R, -0.1 * R), at(0.13 * R, -0.1 * R), at(0.13 * R, -0.5 * R), at(-0.13 * R, -0.5 * R)];
    ctx.beginPath();
    box.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.closePath();
    ctx.fillStyle = joined ?? '#5d6068';
    ctx.fill();
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1.2;
    ctx.stroke();
    if (g.motor.mounted !== false && !joined) {
      const [a1, a2] = [at(-0.2 * R, -0.5 * R), at(0.2 * R, -0.5 * R)];
      ctx.strokeStyle = C.ground;
      line(ctx, a1, a2);
    }
  }
  const blades = Array.from({ length: N }, (_, k) => phi + (2 * Math.PI * k) / N).sort((a, b) => Math.sin(a) - Math.sin(b));
  const blade = (a: number) => {
    const [ca, sa] = [Math.cos(a), Math.sin(a)];
    const chord: Vec = [-Math.cos(beta) * sa * hand, Math.sin(beta)]; // in (e, n)
    const pts: Vec[] = [];
    const back: Vec[] = [];
    for (let k = 0; k <= 8; k++) {
      const r = R * (0.1 + (0.9 * k) / 8);
      const w = R * (0.2 - (0.1 * k) / 8) * (k === 8 ? 0.6 : 1);
      pts.push(at(r * ca + 0.5 * w * chord[0], 0.5 * w * chord[1]));
      back.push(at(r * ca - 0.5 * w * chord[0], -0.5 * w * chord[1]));
    }
    ctx.beginPath();
    [...pts, ...back.reverse()].forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.globalAlpha = sa < 0 ? 0.35 : 0.85;
    ctx.fill();
    ctx.globalAlpha = sa < 0 ? 0.5 : 1;
    ctx.strokeStyle = C.ink;
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.globalAlpha = 1;
  };
  // spinning fast, the blades blur into a band along the disk
  const rate = Math.abs(s.spinRate?.(g.id) ?? 0);
  if (rate > 15) {
    const band = [at(-R, 0.09 * R), at(R, 0.09 * R), at(R, -0.09 * R), at(-R, -0.09 * R)];
    ctx.beginPath();
    band.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.25 * Math.min(1, rate / 120);
    ctx.fill();
    ctx.globalAlpha = 1;
  }
  for (const a of blades) if (Math.sin(a) < 0) blade(a);
  // the hub: a spinner along the axis
  const hub = [at(-0.09 * R, -0.08 * R), at(0.09 * R, -0.08 * R), at(0.06 * R, 0.1 * R), at(0, 0.15 * R), at(-0.06 * R, 0.1 * R)];
  ctx.beginPath();
  hub.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
  ctx.closePath();
  ctx.fillStyle = C.ink;
  ctx.fill();
  for (const a of blades) if (Math.sin(a) >= 0) blade(a);
  // thrust arrow: its size while running, the way it pushes for forward spin while editing
  const T = s.thrust?.(g.id);
  const sc = toScreen(v, c);
  const tip = (len: number): Vec => [sc[0] + len * n[0], sc[1] - len * n[1]];
  if (T === undefined) {
    ctx.globalAlpha = 0.55;
    arrowLine(ctx, toScreen(v, [c[0] + 0.18 * R * hand * n[0], c[1] + 0.18 * R * hand * n[1]]), tip(hand * Math.max(0.5 * R * v.scale, 22)), C.muted, 1.5);
    ctx.globalAlpha = 1;
  } else if (Math.abs(T) > 1e-3) {
    const len = Math.sign(T) * Math.min(14 * Math.sqrt(Math.abs(T)) + 6, 160);
    arrowLine(ctx, toScreen(v, [c[0] + 0.18 * R * Math.sign(T) * n[0], c[1] + 0.18 * R * Math.sign(T) * n[1]]), tip(len + Math.sign(T) * 0.18 * R * v.scale), C.force, 2.5);
  }
  ctx.font = 'italic 600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = color;
  if (showLabels) {
    const lp = at(R + 0.08, 0);
    ctx.fillText(`P${gearLabel(g.id)}`, lp[0] + 8, lp[1] + 10);
  }
  ctx.restore();
  if (g.motor?.pid) drawPid(ctx, v, s, g);
  if (leverOf(g.motor)) drawLever(ctx, v, s, g);
  if (g.locked) drawPadlock(ctx, [sc[0] - 16, sc[1] - 16]);
}

/** A wheel: a dark tyre round a tinted disk, with spokes that turn with it. */
export function drawWheel(ctx: CanvasRenderingContext2D, v: View, c: Vec, r: number, phase: number, color: string, highlight: boolean, locked = false) {
  const [x, y] = toScreen(v, c);
  const R = r * v.scale;
  if (R < 2) return;
  const tyre = Math.max(2, Math.min(R * 0.16, 14));
  ctx.save();
  ctx.beginPath();
  ctx.arc(x, y, R - tyre, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.globalAlpha = locked ? 0.45 : 0.22;
  ctx.fill();
  ctx.globalAlpha = 1;
  // the tyre, its outer edge on the rolling radius
  ctx.beginPath();
  ctx.arc(x, y, R - tyre / 2, 0, Math.PI * 2);
  ctx.lineWidth = tyre;
  ctx.strokeStyle = C.ink;
  ctx.stroke();
  if (highlight) {
    ctx.beginPath();
    ctx.arc(x, y, R + 1.5, 0, Math.PI * 2);
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = C.accent;
    ctx.stroke();
  }
  // spokes (screen y points down, so world angle φ is screen angle −φ) and the hub
  ctx.strokeStyle = color;
  ctx.lineWidth = Math.max(1.5, Math.min(R * 0.05, 3));
  ctx.lineCap = 'round';
  const hub = Math.max(3, R * 0.14);
  for (let k = 0; k < 5; k++) {
    const ang = -(phase + (k * 2 * Math.PI) / 5);
    line(ctx, [x + hub * Math.cos(ang), y + hub * Math.sin(ang)], [x + (R - tyre) * Math.cos(ang), y + (R - tyre) * Math.sin(ang)]);
  }
  ctx.beginPath();
  ctx.arc(x, y, hub, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  // a white mark on the tyre makes the rolling visible
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = Math.max(1.5, tyre * 0.35);
  line(ctx, [x + (R - tyre * 0.85) * Math.cos(-phase), y + (R - tyre * 0.85) * Math.sin(-phase)], [x + (R - tyre * 0.15) * Math.cos(-phase), y + (R - tyre * 0.15) * Math.sin(-phase)]);
  ctx.restore();
}

function drawGear(ctx: CanvasRenderingContext2D, v: View, c: Vec, r: number, teeth: number, phase: number, color: string, highlight: boolean, locked = false) {
  const [x, y] = toScreen(v, c);
  const R = r * v.scale;
  if (R < 2) return;
  const module = (2 * R) / teeth; // pitch diameter / teeth
  const tip = R + module;
  const root = Math.max(R - 1.25 * module, R * 0.3);
  const pitch = (2 * Math.PI) / teeth;
  ctx.save();
  ctx.beginPath();
  // Outline in increasing world angle; screen y points down, so world angle φ is screen angle −φ.
  const profile: [number, number][] = [[root, -0.5], [root, -0.3], [tip, -0.16], [tip, 0.16], [root, 0.3]];
  for (let k = 0; k < teeth; k++) {
    for (const [rr, frac] of profile) {
      const ang = -(phase + (k + frac) * pitch);
      const px = x + rr * Math.cos(ang);
      const py = y + rr * Math.sin(ang);
      if (k === 0 && frac === -0.5) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
  }
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.globalAlpha = locked ? 0.45 : 0.28;
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.lineWidth = highlight ? 2.5 : 1.3;
  ctx.strokeStyle = highlight ? C.accent : color;
  ctx.stroke();
  // pitch circle and a spoke showing the gear's rotation
  ctx.setLineDash([3, 4]);
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(x, y, R, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.lineWidth = 2;
  // a short tick near the rim makes the rotation visible
  line(ctx, [x + root * 0.55 * Math.cos(-phase), y + root * 0.55 * Math.sin(-phase)], [x + root * 0.85 * Math.cos(-phase), y + root * 0.85 * Math.sin(-phase)]);
  ctx.restore();
}

function drawSpring(ctx: CanvasRenderingContext2D, v: View, s: Scene, sp: DSpring) {
  const a = toScreen(v, s.pos(sp.a));
  const b = toScreen(v, s.pos(sp.b));
  const selected = isSelected(s, 'spring', sp.id);
  const hovered = s.hover?.kind === 'spring' && s.hover.id === sp.id;
  ctx.save();
  if (selected || hovered) {
    ctx.strokeStyle = C.accent;
    ctx.globalAlpha = selected ? 0.45 : 0.25;
    ctx.lineWidth = 22;
    ctx.lineCap = 'round';
    line(ctx, a, b);
    ctx.globalAlpha = 1;
  }
  ctx.strokeStyle = C.spring;
  ctx.lineWidth = 2.2;
  const worldLen = Math.hypot(s.pos(sp.b)[0] - s.pos(sp.a)[0], s.pos(sp.b)[1] - s.pos(sp.a)[1]);
  if (sp.tensionOnly && worldLen < sp.rest) {
    // a slack cable: dashed and straight, as it pulls nothing
    ctx.lineWidth = 1.6;
    ctx.setLineDash([5, 4]);
    line(ctx, a, b);
    ctx.setLineDash([]);
  } else zigzag(ctx, a, b);
  const mx = (a[0] + b[0]) / 2;
  const my = (a[1] + b[1]) / 2;
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
  const nx = -(b[1] - a[1]) / len;
  const ny = (b[0] - a[0]) / len;
  ctx.font = 'italic 600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = C.spring;
  if (showLabels) ctx.fillText(`k${springLabel(sp.id)}`, mx + nx * 20, my + ny * 20);
  ctx.restore();
}

/** Temperature rise (K) at which a joint glows white-hot. */
export const WHITE_HOT = 60;

/** Incandescence colour for t = ΔT / WHITE_HOT: dull red → red → orange → yellow → white. */
export function heatColor(t: number): [number, number, number] {
  const stops: [number, number, number, number][] = [
    [0, 110, 10, 0],
    [0.25, 200, 30, 0],
    [0.5, 255, 100, 0],
    [0.75, 255, 200, 50],
    [1, 255, 255, 235],
  ];
  const x = Math.min(Math.max(t, 0), 1);
  for (let i = 1; i < stops.length; i++) {
    if (x <= stops[i][0]) {
      const [a, b] = [stops[i - 1], stops[i]];
      const u = (x - a[0]) / (b[0] - a[0]);
      return [0, 1, 2].map((k) => Math.round(a[k + 1] + (b[k + 1] - a[k + 1]) * u)) as [number, number, number];
    }
  }
  return [255, 255, 235];
}

function glow(ctx: CanvasRenderingContext2D, [x, y]: Vec, dT: number) {
  if (dT < 0.05) return;
  const t = dT / WHITE_HOT;
  const alpha = Math.min(1, 0.3 + 0.7 * Math.sqrt(Math.min(t, 1)));
  const radius = 12 + 36 * Math.min(Math.sqrt(t), 1.4);
  // like hot metal: the core has the full colour, the halo is cooler (more orange), which also
  // keeps a white-hot joint visible on the light background
  const rgba = (k: number, a: number) => `rgba(${heatColor(t * k).join(',')},${a})`;
  const grad = ctx.createRadialGradient(x, y, 0, x, y, radius);
  grad.addColorStop(0, rgba(1, alpha));
  grad.addColorStop(0.3, rgba(0.7, alpha * 0.8));
  grad.addColorStop(0.65, rgba(0.45, alpha * 0.35));
  grad.addColorStop(1, rgba(0.3, 0));
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fill();
}

/** Joints and gear meshes glow with the heat friction has put into them. */
function drawHeat(ctx: CanvasRenderingContext2D, v: View, s: Scene, heat: NonNullable<Scene['heat']>) {
  ctx.save();
  ctx.globalCompositeOperation = 'source-over';
  for (const [id, dT] of heat.joints) if (s.design.points.some((p) => p.id === id)) glow(ctx, toScreen(v, s.pos(id)), dT);
  for (const [id, dT] of heat.meshes) {
    const m = s.design.meshes.find((x) => x.id === id);
    const ga = m && s.design.gears.find((g) => g.id === m.a);
    const gb = m && s.design.gears.find((g) => g.id === m.b);
    if (!ga || !gb) continue;
    const ca = s.pos(ga.center);
    const cb = s.pos(gb.center);
    const d = Math.hypot(cb[0] - ca[0], cb[1] - ca[1]) || 1;
    const t = ga.radius / d; // the pitch point, where the teeth rub
    glow(ctx, toScreen(v, [ca[0] + (cb[0] - ca[0]) * t, ca[1] + (cb[1] - ca[1]) * t]), dT);
  }
  for (const [id, dT] of heat.racks) {
    const m = s.design.rackMeshes.find((x) => x.id === id);
    const g = m && s.design.gears.find((x) => x.id === m.gear);
    const r = m && s.design.links.find((x) => x.id === m.rack);
    if (!g || !r) continue;
    const f = rackFrame(s.pos(r.a), s.pos(r.b));
    const { xi } = onRack(f, s.pos(g.center)); // the pitch point is the foot of the perpendicular
    glow(ctx, toScreen(v, [f.a[0] + f.e[0] * xi, f.a[1] + f.e[1] * xi]), dT);
  }
  ctx.restore();
}

function drawJoint(ctx: CanvasRenderingContext2D, v: View, s: Scene, p: DPoint) {
  const used =
    [...s.design.links, ...s.design.springs].some((l) => l.a === p.id || l.b === p.id) ||
    p.gear !== undefined ||
    s.design.gears.some((g) => g.center === p.id);
  const c = toScreen(v, s.pos(p.id));
  const selected = isSelected(s, 'point', p.id);
  const hovered = s.hover?.kind === 'point' && s.hover.id === p.id;

  if (p.kind === 'fixed') drawGround(ctx, c);
  const rail = railOf(p);
  if (rail) {
    // the slider block lies along the rail, turned with the body the rail is on
    const turn = rail.on ? (bodyMotion(s, rail.on)?.turn ?? 0) : 0;
    drawSlider(ctx, c, (rail.angle * Math.PI) / 180 + turn);
  }
  if (p.pin) {
    // a short spoke from the joint to the bar it is pinned on, in the bar's colour
    const l = s.design.links.find((x) => x.id === p.pin!.id);
    if (l) {
      const [a, b] = [toScreen(v, s.pos(l.a)), toScreen(v, s.pos(l.b))];
      const { s: t } = segmentDistance(c, a, b);
      const foot: Vec = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
      ctx.save();
      ctx.strokeStyle = linkColor(l.id);
      ctx.lineWidth = 4;
      ctx.lineCap = 'round';
      line(ctx, c, foot);
      ctx.restore();
      dot(ctx, c, 8, linkColor(l.id), C.ink, 2);
    }
  }

  if (selected || hovered) {
    ctx.save();
    ctx.globalAlpha = selected ? 0.45 : 0.25;
    dot(ctx, c, 14, C.accent, C.accent, 0);
    ctx.restore();
  }
  const r = p.mass > 0 ? 6 + 4 * Math.sqrt(p.mass) : 5;
  const dT = s.heat?.joints.get(p.id) ?? 0;
  const fill = dT >= 0.05 ? `rgb(${heatColor(dT / WHITE_HOT).join(',')})` : p.mass > 0 ? '#2b2d31' : '#fff';
  dot(ctx, c, r, fill, used ? '#1f2125' : C.muted);

  ctx.font = '600 12px system-ui, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'bottom';
  ctx.fillStyle = C.ink;
  if (!showLabels) return;
  ctx.fillText(pointLabel(p.id), c[0] + r + 4, c[1] - r - 2);
  if (p.mass > 0) {
    ctx.font = '11px system-ui, sans-serif';
    ctx.textBaseline = 'top';
    ctx.fillStyle = C.muted;
    ctx.fillText(`${+p.mass.toFixed(2)} kg`, c[0] + r + 4, c[1] + r - 2);
  }
}

function drawGround(ctx: CanvasRenderingContext2D, [x, y]: Vec) {
  ctx.save();
  ctx.fillStyle = '#d9d9d2';
  ctx.strokeStyle = C.ground;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - 13, y + 20);
  ctx.lineTo(x + 13, y + 20);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  line(ctx, [x - 20, y + 20], [x + 20, y + 20]);
  ctx.lineWidth = 1.5;
  for (let i = -18; i <= 14; i += 8) line(ctx, [x + i, y + 28], [x + i + 7, y + 21]);
  ctx.restore();
}

/** A slider block along a rail at `angle` (world radians). */
function drawSlider(ctx: CanvasRenderingContext2D, [x, y]: Vec, angle: number) {
  ctx.save();
  ctx.fillStyle = '#e4e4dd';
  ctx.strokeStyle = C.ground;
  ctx.lineWidth = 2;
  ctx.translate(x, y);
  ctx.rotate(-angle); // the screen's y points down
  const [w, h] = [34, 18];
  ctx.beginPath();
  ctx.roundRect(-w / 2, -h / 2, w, h, 4);
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawGrab(ctx: CanvasRenderingContext2D, v: View, g: NonNullable<Scene['grab']>) {
  const a = toScreen(v, g.at);
  const b = toScreen(v, g.to);
  ctx.save();
  ctx.strokeStyle = C.force;
  ctx.globalAlpha = 0.5;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  line(ctx, a, b);
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
  dot(ctx, b, 4, C.force, C.force, 0);

  // force arrow at the material point
  const mag = Math.hypot(g.force[0], g.force[1]);
  if (mag > 1e-6) {
    const len = Math.min(140, 18 + 10 * Math.sqrt(mag));
    const ux = g.force[0] / mag;
    const uy = -g.force[1] / mag;
    const tip: Vec = [a[0] + ux * len, a[1] + uy * len];
    ctx.strokeStyle = C.force;
    ctx.fillStyle = C.force;
    ctx.lineWidth = 3;
    line(ctx, a, tip);
    ctx.beginPath();
    ctx.moveTo(tip[0] + ux * 4, tip[1] + uy * 4);
    ctx.lineTo(tip[0] - ux * 10 - uy * 6, tip[1] - uy * 10 + ux * 6);
    ctx.lineTo(tip[0] - ux * 10 + uy * 6, tip[1] - uy * 10 - ux * 6);
    ctx.closePath();
    ctx.fill();
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${mag.toFixed(1)} N`, tip[0] + 10, tip[1]);
  }
  dot(ctx, a, 4, '#fff', C.force);
  ctx.restore();
}

/** Distance from point p to segment ab, and the segment parameter of the closest point. */
export function segmentDistance(p: Vec, a: Vec, b: Vec): { d: number; s: number } {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const L2 = dx * dx + dy * dy;
  const s = L2 === 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2));
  return { d: Math.hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy), s };
}
