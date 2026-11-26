import { gearLabel, GEAR_MODULE, linkLabel, pointLabel, rackPitch, sharedJoint, springLabel, type Design, type DLink, type DPoint, type DSpring } from '../mechanics/model';
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

export type Selection = { kind: 'point'; id: number } | { kind: 'link'; id: number } | { kind: 'spring'; id: number } | { kind: 'gear'; id: number };

export interface Scene {
  design: Design;
  pos: (id: number) => Vec;
  selection: Selection | null;
  hover: Selection | null;
  /** A part being drawn; `meshAt` marks where it will mesh, `side` is a rack's tooth side. */
  preview: { from: Vec; to: Vec; kind: 'bar' | 'spring' | 'gear' | 'rack'; meshAt?: Vec | null; side?: 1 | -1 } | null;
  /** Current rotation of a gear (0 while editing). */
  gearAngle: (id: number) => number;
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
  /** The ground line, if there is one (slope in degrees). */
  ground?: { y: number; slope: number } | null;
  /** Recent path of the selected joint, a ring buffer of world positions (oldest at `start`). */
  trail?: { xs: Float64Array; ys: Float64Array; start: number; n: number } | null;
}

const partsKey = { point: 'points', link: 'links', spring: 'springs', gear: 'gears' } as const;
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
  const key = (r: { kind: 'link' | 'gear'; id: number }) => `${r.kind}:${r.id}`;
  for (const j of d.joins ?? []) parent.set(find(key(j.a)), find(key(j.b)));
  const groups = new Map<string, { kind: 'link' | 'gear'; id: number }[]>();
  for (const j of d.joins ?? [])
    for (const r of [j.a, j.b]) {
      const g = groups.get(find(key(r))) ?? [];
      if (!g.some((x) => x.kind === r.kind && x.id === r.id)) g.push(r);
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
  if (s.ground) drawGroundLine(ctx, w, h, v, s.ground);

  for (const p of s.design.points) if (p.kind === 'lockX' || p.kind === 'lockY') drawRail(ctx, w, h, v, p);

  const phases = toothPhases(s.design);
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
    } else if (s.preview.kind === 'gear') {
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
  if (s.grab) drawGrab(ctx, v, s.grab);
  if (s.box) drawBox(ctx, v, s.box);
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
function drawGroundLine(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, g: { y: number; slope: number }) {
  const t = Math.tan((g.slope * Math.PI) / 180);
  const [x0] = toWorld(v, [0, 0]);
  const [x1] = toWorld(v, [w, 0]);
  const a = toScreen(v, [x0, g.y + t * x0]);
  const b = toScreen(v, [x1, g.y + t * x1]);
  ctx.save();
  ctx.fillStyle = 'rgba(107,110,118,0.10)';
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.lineTo(w, Math.max(a[1], b[1]) + h);
  ctx.lineTo(0, Math.max(a[1], b[1]) + h);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = C.ground;
  ctx.lineWidth = 2;
  line(ctx, a, b);
  // hatching under the line
  ctx.lineWidth = 1;
  ctx.globalAlpha = 0.5;
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
  const [ux, uy] = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
  for (let d = 0; d < len; d += 14) {
    const p: Vec = [a[0] + ux * d, a[1] + uy * d];
    line(ctx, p, [p[0] - 8, p[1] + 10]);
  }
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

function drawRail(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, p: DPoint) {
  const [sx, sy] = toScreen(v, [p.x, p.y]);
  ctx.save();
  ctx.strokeStyle = C.ground;
  ctx.globalAlpha = 0.55;
  ctx.lineWidth = 2;
  ctx.setLineDash([10, 6]);
  if (p.kind === 'lockY') line(ctx, [0, sy], [w, sy]);
  else line(ctx, [sx, 0], [sx, h]);
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
    drawGear(ctx, v, c, g.radius, g.teeth, phases.get(g.id)! + angle, color, selected || hovered, g.locked);
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
    if (showLabels) ctx.fillText(`g${gearLabel(g.id)}`, sc[0] - 14, sc[1] + 20);
    ctx.restore();
    if (g.locked) drawPadlock(ctx, [sc[0] - 20, sc[1] - 20]);
  }
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
  if (p.kind === 'lockX' || p.kind === 'lockY') drawSlider(ctx, c, p.kind);

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

function drawSlider(ctx: CanvasRenderingContext2D, [x, y]: Vec, kind: 'lockX' | 'lockY') {
  ctx.save();
  ctx.fillStyle = '#e4e4dd';
  ctx.strokeStyle = C.ground;
  ctx.lineWidth = 2;
  const [w, h] = kind === 'lockY' ? [34, 18] : [18, 34];
  ctx.beginPath();
  ctx.roundRect(x - w / 2, y - h / 2, w, h, 4);
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
