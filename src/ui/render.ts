import { gearLabel, linkLabel, pointLabel, springLabel, type Design, type DGear, type DLink, type DPoint, type DSpring } from '../mechanics/model';

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
  preview: { from: Vec; to: Vec; kind: 'bar' | 'spring' | 'gear'; meshWith?: number | null } | null;
  /** Current rotation of a gear (0 while editing). */
  gearAngle: (id: number) => number;
  /** Temperature rise (K) of joints and gear meshes from friction, while simulating. */
  heat?: { joints: Map<number, number>; meshes: Map<number, number> } | null;
  grab: { at: Vec; to: Vec; force: Vec } | null;
  simulating: boolean;
}

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
  gear: ['#c99a3b', '#8f9aa6', '#7fa36b', '#b07aa1', '#5f97b8'],
  bars: ['#3b6ea8', '#4c8f6a', '#8a5aa8', '#b0763a', '#3a8f99', '#a8475a'],
};

export function barColor(id: number): string {
  return C.bars[id % C.bars.length];
}

export function drawScene(ctx: CanvasRenderingContext2D, w: number, h: number, v: View, s: Scene) {
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, w, h);
  drawGrid(ctx, w, h, v);

  for (const p of s.design.points) if (p.kind === 'lockX' || p.kind === 'lockY') drawRail(ctx, w, h, v, p);

  drawGears(ctx, v, s);
  for (const sp of s.design.springs) drawSpring(ctx, v, s, sp);
  for (const l of s.design.links) drawBar(ctx, v, s, l);
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
      if (s.preview.meshWith !== null && s.preview.meshWith !== undefined) {
        ctx.setLineDash([]);
        dot(ctx, b, 4, C.accent, C.accent, 0); // the mesh point
      }
    } else {
      ctx.lineWidth = 3;
      ctx.setLineDash([8, 6]);
      line(ctx, a, b);
    }
    ctx.restore();
    dot(ctx, b, 5, '#fff', C.accent);
  }
  if (s.heat) drawHeat(ctx, v, s, s.heat);
  for (const p of s.design.points) drawJoint(ctx, v, s, p);
  if (s.grab) drawGrab(ctx, v, s.grab);
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
  const step = v.scale >= 120 ? 0.1 : v.scale >= 40 ? 0.5 : 1;
  const [x0, y1] = toWorld(v, [0, 0]);
  const [x1, y0] = toWorld(v, [w, h]);
  ctx.lineWidth = 1;
  for (let i = Math.floor(x0 / step); i <= Math.ceil(x1 / step); i++) {
    const x = i * step;
    const sx = Math.round(v.ox + x * v.scale) + 0.5;
    ctx.strokeStyle = i === 0 ? C.axis : Math.abs(x - Math.round(x)) < 1e-9 ? C.gridMajor : C.gridMinor;
    line(ctx, [sx, 0], [sx, h]);
  }
  for (let j = Math.floor(y0 / step); j <= Math.ceil(y1 / step); j++) {
    const y = j * step;
    const sy = Math.round(v.oy - y * v.scale) + 0.5;
    ctx.strokeStyle = j === 0 ? C.axis : Math.abs(y - Math.round(y)) < 1e-9 ? C.gridMajor : C.gridMinor;
    line(ctx, [0, sy], [w, sy]);
  }
  // scale bar
  ctx.fillStyle = C.muted;
  ctx.strokeStyle = C.muted;
  ctx.lineWidth = 2;
  const len = v.scale * (v.scale >= 60 ? 0.5 : 1);
  line(ctx, [w - 30 - len, h - 24], [w - 30, h - 24]);
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(v.scale >= 60 ? '0.5 m' : '1 m', w - 30 - len / 2, h - 30);
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
  const selected = s.selection?.kind === 'link' && s.selection.id === l.id;
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
  ctx.strokeStyle = '#1f2125';
  ctx.lineWidth = 13;
  line(ctx, a, b);
  ctx.strokeStyle = barColor(l.id);
  ctx.lineWidth = 10;
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
  ctx.fillText(linkLabel(l.id), mx + nx * 18, my + ny * 18);
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

/**
 * Tooth phase of every gear in the design configuration, chosen so that each meshing pair
 * interlocks on its line of centres (a tooth of one gear in a gap of the other). Meshes
 * are visited breadth-first; the tooth-ratio constraint then keeps them interlocked.
 */
function toothPhases(d: Design): Map<number, number> {
  const phase = new Map<number, number>();
  const at = (g: DGear): Vec => {
    const p = d.points.find((q) => q.id === g.center)!;
    return [p.x, p.y];
  };
  for (const root of d.gears) {
    if (phase.has(root.id)) continue;
    phase.set(root.id, 0);
    const queue = [root];
    while (queue.length) {
      const ga = queue.shift()!;
      for (const m of d.meshes) {
        const otherId = m.a === ga.id ? m.b : m.b === ga.id ? m.a : null;
        const gb = d.gears.find((x) => x.id === otherId);
        if (!gb || phase.has(gb.id)) continue;
        const [ca, cb] = [at(ga), at(gb)];
        const psi = Math.atan2(cb[1] - ca[1], cb[0] - ca[0]);
        const pa = (2 * Math.PI) / ga.teeth;
        const pb = (2 * Math.PI) / gb.teeth;
        const t = (psi - phase.get(ga.id)!) / pa;
        const f = t - Math.round(t); // where gear a's nearest tooth sits, in pitches from the contact line
        phase.set(gb.id, psi + Math.PI + pb / 2 - f * pb);
        queue.push(gb);
      }
    }
  }
  return phase;
}

function drawGears(ctx: CanvasRenderingContext2D, v: View, s: Scene) {
  if (!s.design.gears.length) return;
  const phases = toothPhases(s.design);
  for (const g of s.design.gears) {
    if (!s.design.points.some((p) => p.id === g.center)) continue;
    const c = s.pos(g.center);
    const sc = toScreen(v, c);
    const color = C.gear[g.id % C.gear.length];
    const selected = s.selection?.kind === 'gear' && s.selection.id === g.id;
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
    ctx.fillText(`g${gearLabel(g.id)}`, sc[0] - 14, sc[1] + 20);
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
  const selected = s.selection?.kind === 'spring' && s.selection.id === sp.id;
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
  zigzag(ctx, a, b);
  const mx = (a[0] + b[0]) / 2;
  const my = (a[1] + b[1]) / 2;
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
  const nx = -(b[1] - a[1]) / len;
  const ny = (b[0] - a[0]) / len;
  ctx.font = 'italic 600 12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = C.spring;
  ctx.fillText(`k${springLabel(sp.id)}`, mx + nx * 20, my + ny * 20);
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
  ctx.restore();
}

function drawJoint(ctx: CanvasRenderingContext2D, v: View, s: Scene, p: DPoint) {
  const used =
    [...s.design.links, ...s.design.springs].some((l) => l.a === p.id || l.b === p.id) ||
    p.gear !== undefined ||
    s.design.gears.some((g) => g.center === p.id);
  const c = toScreen(v, s.pos(p.id));
  const selected = s.selection?.kind === 'point' && s.selection.id === p.id;
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
