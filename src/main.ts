import 'katex/dist/katex.min.css';
import './style.css';
import { deriveEquations, type Derivation } from './mechanics/derive';
import { cloneDesign, emptyDesign, gearLabel, linkLabel, linkLength, nextId, pointLabel, springLabel, teethForRadius, type Design, type DGear, type PointKind } from './mechanics/model';
import { defaultFriction, NumericSystem, Simulator } from './mechanics/simulate';
import { renderEomPanel } from './ui/eomPanel';
import { presets } from './ui/presets';
import { drawScene, segmentDistance, toScreen, toWorld, type Selection, type Vec, type View } from './ui/render';

type Tool = 'select' | 'bar' | 'spring' | 'gear' | 'mass' | 'lockGear' | PointKind | 'delete';

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;
const canvas = $<HTMLCanvasElement>('#canvas');
const ctx = canvas.getContext('2d')!;
const eomEl = $<HTMLDivElement>('#eom');
const inspectorEl = $<HTMLDivElement>('#inspector');
const statusEl = $<HTMLDivElement>('#status');
const hintEl = $<HTMLDivElement>('#hint');

const STORAGE_KEY = 'linkage-lab-design';
const H_STEP = 1e-3; // RK4 step (s)
const GRAVITY = 9.81;
const DEFAULT_SPRING_K = 50; // N/m
const DEFAULT_MASS = 1; // kg, for the mass tool

// ---- state -----------------------------------------------------------------
let W = 0;
let H = 0;
const view: View = { scale: 170, ox: 0, oy: 0 };
let design: Design = loadDesign() ?? presets[0].build(); // first visit: the Strandbeest leg
let tool: Tool = 'bar';
let selection: Selection | null = null;
let hover: Selection | null = null;
let drawing: { kind: 'bar' | 'spring' | 'gear'; meshWith?: number | null; fromId: number | null; from: Vec; to: Vec; toId: number | null; downAt: Vec; clickMode: boolean } | null = null;
let dragging: { id: number; pushed: boolean } | null = null;
let panning: { last: Vec } | null = null;
let derivation: Derivation | null = null;
let deriveError: string | null = null;
let sim: Simulator | null = null;
let simPositions = new Map<number, Vec>();
let simGearAngles = new Map<number, number>();
let running = false;
let simError: string | null = null;
let grab: { a: number; b: number; s: number; gear?: { id: number; u: number; v: number } } | null = null;
let mouseWorld: Vec = [0, 0];
const undoStack: Design[] = [];
const settings = { gravity: true, snap: true, damping: 0, stiffness: 80, speed: 1 };
const friction = defaultFriction();
let simHeat: { joints: Map<number, number>; meshes: Map<number, number> } = { joints: new Map(), meshes: new Map() };

// ---- persistence & undo ------------------------------------------------------
function loadDesign(): Design | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return null;
    const d = JSON.parse(raw) as Design;
    if (!Array.isArray(d.points) || !Array.isArray(d.links)) return null;
    if (!Array.isArray(d.springs)) d.springs = []; // saved before springs existed
    // Saved before gears existed, or with the earlier gear-pair format: drop those gears.
    if (!Array.isArray(d.gears) || d.gears.some((g) => typeof (g as Partial<DGear>).center !== 'number')) d.gears = [];
    if (!Array.isArray(d.meshes)) d.meshes = [];
    d.meshes = d.meshes.filter((m) => d.gears.some((g) => g.id === m.a) && d.gears.some((g) => g.id === m.b));
    return d;
  } catch {
    return null;
  }
}

function pushUndo() {
  undoStack.push(cloneDesign(design));
  if (undoStack.length > 200) undoStack.shift();
}

function undo() {
  if (sim || !undoStack.length) return;
  design = undoStack.pop()!;
  selection = null;
  designChanged();
}

/** Call after every edit of the design. */
function designChanged(opts: { keepInspector?: boolean } = {}) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(design));
  scheduleDerive();
  if (!opts.keepInspector) renderInspector();
}

// ---- derivation --------------------------------------------------------------
let deriveTimer = 0;
function scheduleDerive() {
  clearTimeout(deriveTimer);
  deriveTimer = window.setTimeout(derive, 40);
}

function derive() {
  clearTimeout(deriveTimer);
  try {
    derivation = deriveEquations(design, { gravity: settings.gravity ? GRAVITY : 0 });
    deriveError = null;
  } catch (e) {
    derivation = null;
    deriveError = e instanceof Error ? e.message : String(e);
  }
  renderEomPanel(eomEl, derivation, deriveError, friction.enabled);
}

// ---- simulation ----------------------------------------------------------------
function startSim() {
  derive();
  if (!derivation || derivation.coords.length === 0) {
    simError = 'Nothing can move yet: draw a bar attached to something that is not fully pinned.';
    return;
  }
  sim = new Simulator(new NumericSystem(derivation));
  sim.damping = settings.damping;
  sim.friction = friction;
  simPositions = sim.jointPositions();
  simHeat = sim.heat();
  simGearAngles = sim.gearAngles();
  simError = null;
  running = true;
  drawing = null;
  dragging = null;
  if (selection?.kind !== 'spring') selection = null; // springs stay editable while running
  refreshControls();
}

function stopSim() {
  sim = null;
  running = false;
  grab = null;
  simError = null;
  refreshControls();
}

function togglePlay() {
  if (!sim) startSim();
  else running = !running;
  refreshControls();
}

/** Re-derive with new settings while keeping the current state (coordinates are unchanged). */
function rebuildSim() {
  derive();
  if (!sim || !derivation) return;
  const next = new Simulator(new NumericSystem(derivation));
  next.q.set(sim.q);
  next.qd.set(sim.qd);
  next.t = sim.t;
  next.damping = settings.damping;
  next.friction = friction;
  // same topology (only parameters changed), so the heat carries over
  if (next.jointHeat.length === sim.jointHeat.length && next.meshHeat.length === sim.meshHeat.length) {
    next.jointHeat.set(sim.jointHeat);
    next.meshHeat.set(sim.meshHeat);
    next.dissipated = sim.dissipated;
  }
  sim = next;
}

function advance(seconds: number) {
  if (!sim) return;
  const steps = Math.min(400, Math.round(seconds / H_STEP));
  sim.damping = settings.damping;
  sim.grab = grab ? { ...grab, target: mouseWorld, stiffness: settings.stiffness } : null;
  for (let i = 0; i < steps; i++) {
    if (!sim.step(H_STEP)) {
      running = false;
      simError = 'The solver hit a singular configuration (for example a fully stretched loop). Press Reset to edit.';
      refreshControls();
      break;
    }
  }
  if (!grab) sim.appliedForce = [0, 0];
  simPositions = sim.jointPositions();
  simGearAngles = sim.gearAngles();
  simHeat = sim.heat();
}

// ---- geometry helpers ----------------------------------------------------------
function pos(id: number): Vec {
  const s = simPositions.get(id);
  if (sim && s) return s;
  const p = design.points.find((pt) => pt.id === id)!;
  return [p.x, p.y];
}

const snapValue = (v: number) => Math.round(v / 0.1) * 0.1;
function snap(w: Vec): Vec {
  return settings.snap ? [snapValue(w[0]), snapValue(w[1])] : w;
}

function pointAt(w: Vec, exclude: number | null = null, onlyActive = false): number | null {
  let best: number | null = null;
  let bestD = 11 / view.scale;
  for (const p of design.points) {
    if (p.id === exclude) continue;
    if (onlyActive && !sim?.sys.pointIndex.has(p.id)) continue;
    const [x, y] = pos(p.id);
    const d = Math.hypot(x - w[0], y - w[1]);
    if (d < bestD) {
      bestD = d;
      best = p.id;
    }
  }
  return best;
}

function segmentAt(w: Vec, segments: readonly { id: number; a: number; b: number }[]): { id: number; s: number } | null {
  let best: { id: number; s: number } | null = null;
  let bestD = 9 / view.scale;
  for (const l of segments) {
    const { d, s } = segmentDistance(w, pos(l.a), pos(l.b));
    if (d < bestD) {
      bestD = d;
      best = { id: l.id, s };
    }
  }
  return best;
}

const linkAt = (w: Vec) => segmentAt(w, design.links);
const springAt = (w: Vec) => segmentAt(w, design.springs);

function hitTest(w: Vec): Selection | null {
  const p = pointAt(w);
  if (p !== null) return { kind: 'point', id: p };
  const l = linkAt(w);
  if (l) return { kind: 'link', id: l.id };
  const sp = springAt(w);
  if (sp) return { kind: 'spring', id: sp.id };
  const g = gearAt(w);
  return g ? { kind: 'gear', id: g.id } : null;
}

/** The gear whose pitch disk contains w (the smallest one when several do). */
function gearAt(w: Vec): DGear | null {
  let best: DGear | null = null;
  for (const g of design.gears) {
    if (!design.points.some((p) => p.id === g.center)) continue;
    const c = pos(g.center);
    if (Math.hypot(w[0] - c[0], w[1] - c[1]) < g.radius && (!best || g.radius < best.radius)) best = g;
  }
  return best;
}

const gearAngleNow = (id: number) => (sim ? (simGearAngles.get(id) ?? 0) : 0);

/** World position of the point at offset (u, v) in a gear's frame. */
function gearPoint(g: DGear, u: number, v: number): Vec {
  const c = pos(g.center);
  const a = gearAngleNow(g.id);
  return [c[0] + u * Math.cos(a) - v * Math.sin(a), c[1] + u * Math.sin(a) + v * Math.cos(a)];
}

/**
 * Radius for a gear centred at `c` with the pointer at `w`: snaps so that the pitch circle
 * touches a nearby gear's pitch circle, which also makes the two gears mesh.
 */
function gearRadiusAt(c: Vec, w: Vec, centerId: number | null): { r: number; meshWith: number | null } {
  const r = Math.hypot(w[0] - c[0], w[1] - c[1]);
  for (const g of design.gears) {
    if (g.center === centerId) continue;
    const gc = pos(g.center);
    const d = Math.hypot(gc[0] - c[0], gc[1] - c[1]);
    const touching = d - g.radius;
    if (touching > 0.03 && Math.abs(r - touching) < 14 / view.scale) return { r: touching, meshWith: g.id };
  }
  return { r: settings.snap ? Math.max(0.05, Math.round(r / 0.05) * 0.05) : r, meshWith: null };
}

// ---- editing operations --------------------------------------------------------
function addPoint([x, y]: Vec, kind: PointKind = 'free', mass = 0): number {
  const id = nextId(design.points);
  design.points.push({ id, x, y, kind, mass });
  return id;
}

/** A new joint at w: pinned onto the gear under it if there is one, otherwise of the given kind. */
function newJoint(w: Vec, kind: PointKind = 'free', mass = 0): number {
  const g = gearAt(w);
  if (!g) return addPoint(w, kind, mass);
  const id = addPoint(w, 'free', mass);
  design.points.find((p) => p.id === id)!.gear = g.id;
  return id;
}

/** Move a joint, carrying along everything pinned on gears centred there. */
function moveJoint(id: number, dx: number, dy: number, seen = new Set<number>()) {
  if (seen.has(id)) return;
  seen.add(id);
  const p = design.points.find((q) => q.id === id);
  if (!p) return;
  p.x += dx;
  p.y += dy;
  for (const g of design.gears) if (g.center === id) for (const q of design.points) if (q.gear === g.id) moveJoint(q.id, dx, dy, seen);
}

/** Delete a gear: its meshes go too, and the joints pinned on it become ordinary joints. */
function removeGear(id: number) {
  design.gears = design.gears.filter((g) => g.id !== id);
  design.meshes = design.meshes.filter((m) => m.a !== id && m.b !== id);
  for (const p of design.points) if (p.gear === id) delete p.gear;
}

/** Finish drawing a bar, a spring or a gear. */
function finishSegment() {
  const d = drawing;
  drawing = null;
  if (!d) return;
  if (d.toId !== null && d.toId === d.fromId) return;
  const length = Math.hypot(d.to[0] - d.from[0], d.to[1] - d.from[1]);
  if (length < 1e-6) return;
  if (d.kind === 'gear') {
    const r = length;
    if (r < 0.03) return;
    pushUndo();
    // A gear centred in empty space turns on a fixed axle; inside another gear it rides on that gear.
    const center = d.fromId ?? (gearAt(d.from) ? newJoint(d.from) : addPoint(d.from, 'fixed'));
    const other = design.gears.find((g) => g.id === d.meshWith);
    const teeth = other ? Math.max(6, Math.round((other.teeth * r) / other.radius)) : teethForRadius(r);
    const id = nextId(design.gears);
    design.gears.push({ id, center, radius: +r.toFixed(4), teeth, mass: +Math.max(0.1, 25 * r * r).toFixed(2), locked: false });
    if (other) design.meshes.push({ id: nextId(design.meshes), a: other.id, b: id });
    selection = { kind: 'gear', id };
    designChanged();
    return;
  }
  const spring = d.kind === 'spring';
  const list = spring ? design.springs : design.links;
  const same = (l: { a: number; b: number }, a: number | null, b: number | null) => (l.a === a && l.b === b) || (l.a === b && l.b === a);
  if (d.fromId !== null && d.toId !== null && list.some((l) => same(l, d.fromId, d.toId))) return;
  pushUndo();
  if (!spring) {
    const a = d.fromId ?? newJoint(d.from);
    const b = d.toId ?? newJoint(d.to);
    const id = nextId(design.links);
    design.links.push({ id, a, b, mass: 1 });
    selection = { kind: 'link', id };
  } else {
    // A spring end drawn in empty space is anchored to the ground. If both ends are new,
    // the far end gets a 1 kg mass so the spring has something to move.
    // An end on a gear is pinned onto it.
    const a = d.fromId ?? newJoint(d.from, 'fixed');
    const b = d.toId ?? (d.fromId === null ? newJoint(d.to, 'free', 1) : newJoint(d.to, 'fixed'));
    const id = nextId(design.springs);
    design.springs.push({ id, a, b, k: DEFAULT_SPRING_K, rest: +length.toFixed(4) });
    selection = { kind: 'spring', id };
  }
  designChanged();
}

function removeOrphans() {
  design.points = design.points.filter(
    (p) =>
      [...design.links, ...design.springs].some((l) => l.a === p.id || l.b === p.id) ||
      design.gears.some((g) => g.center === p.id) ||
      p.mass > 0, // a particle, or a weight pinned on a gear
  );
}

function deleteSelection(sel: Selection | null) {
  if (!sel) return;
  pushUndo();
  if (sel.kind === 'point') {
    design.links = design.links.filter((l) => l.a !== sel.id && l.b !== sel.id);
    design.springs = design.springs.filter((l) => l.a !== sel.id && l.b !== sel.id);
    for (const g of design.gears.filter((x) => x.center === sel.id)) removeGear(g.id);
    design.points = design.points.filter((p) => p.id !== sel.id);
  } else if (sel.kind === 'link') {
    design.links = design.links.filter((l) => l.id !== sel.id);
  } else if (sel.kind === 'gear') {
    removeGear(sel.id);
  } else {
    design.springs = design.springs.filter((l) => l.id !== sel.id);
  }
  removeOrphans();
  selection = null;
  designChanged();
}

/**
 * Mass tool: on a joint, hang a mass there (or take it off again). In empty space, place a
 * free mass; on a gear, the new mass is a weight pinned to that gear.
 */
function toggleMass(w: Vec) {
  const pid = pointAt(w);
  pushUndo();
  if (pid !== null) {
    const p = design.points.find((q) => q.id === pid)!;
    p.mass = p.mass > 0 ? 0 : DEFAULT_MASS;
    removeOrphans();
    selection = design.points.some((q) => q.id === pid) ? { kind: 'point', id: pid } : null;
  } else {
    selection = { kind: 'point', id: newJoint(snap(w), 'free', DEFAULT_MASS) };
  }
  designChanged();
}

function setPointKind(id: number, kind: PointKind) {
  const p = design.points.find((pt) => pt.id === id);
  if (!p) return;
  pushUndo();
  p.kind = p.kind === kind && kind !== 'free' ? 'free' : kind;
  selection = { kind: 'point', id };
  designChanged();
}

// ---- pointer input ---------------------------------------------------------------
const screenOf = (e: PointerEvent | WheelEvent): Vec => [e.offsetX, e.offsetY];

canvas.addEventListener('contextmenu', (e) => e.preventDefault());

canvas.addEventListener('pointerdown', (e) => {
  canvas.setPointerCapture(e.pointerId);
  const sp = screenOf(e);
  const w = toWorld(view, sp);
  mouseWorld = w;
  if (e.button === 1 || e.button === 2) {
    panning = { last: sp };
    return;
  }
  if (e.button !== 0) return;

  if (sim) {
    const pid = pointAt(w, null, true);
    if (pid !== null) grab = { a: pid, b: pid, s: 0 };
    else {
      const l = linkAt(w);
      const link = l && design.links.find((x) => x.id === l.id);
      const spr = springAt(w);
      const gr = gearAt(w);
      if (l && link && sim.sys.pointIndex.has(link.a)) grab = { a: link.a, b: link.b, s: l.s };
      else if (spr) {
        selection = { kind: 'spring', id: spr.id };
        renderInspector();
      } else if (gr && sim.sys.pointIndex.has(gr.center)) {
        // grab the gear disk at this material point, stored in the gear's own frame
        const c = pos(gr.center);
        const a = gearAngleNow(gr.id);
        const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
        grab = { a: gr.center, b: gr.center, s: 0, gear: { id: gr.id, u: dx * Math.cos(a) + dy * Math.sin(a), v: -dx * Math.sin(a) + dy * Math.cos(a) } };
      } else {
        selection = null;
        renderInspector();
      }
    }
    return;
  }

  switch (tool) {
    case 'bar':
    case 'spring':
    case 'gear': {
      if (drawing?.clickMode) {
        finishSegment();
        return;
      }
      const hit = pointAt(w);
      const from = hit !== null ? pos(hit) : snap(w);
      drawing = { kind: tool, meshWith: null, fromId: hit, from, to: from, toId: null, downAt: sp, clickMode: false };
      break;
    }
    case 'select': {
      const hit = hitTest(w);
      selection = hit;
      if (hit?.kind === 'point') dragging = { id: hit.id, pushed: false };
      renderInspector();
      break;
    }
    case 'delete':
      deleteSelection(hitTest(w));
      break;
    case 'mass':
      toggleMass(w);
      break;
    case 'lockGear': {
      const pid = pointAt(w);
      const g = (pid !== null && design.gears.find((x) => x.center === pid)) || gearAt(w);
      if (!g) break;
      pushUndo();
      g.locked = !g.locked;
      selection = { kind: 'gear', id: g.id };
      designChanged();
      break;
    }
    default: {
      const pid = pointAt(w);
      if (pid !== null) setPointKind(pid, tool);
    }
  }
});

canvas.addEventListener('pointermove', (e) => {
  const sp = screenOf(e);
  const w = toWorld(view, sp);
  mouseWorld = w;
  if (panning) {
    view.ox += sp[0] - panning.last[0];
    view.oy += sp[1] - panning.last[1];
    panning.last = sp;
    return;
  }
  if (drawing?.kind === 'gear') {
    const { r, meshWith } = gearRadiusAt(drawing.from, w, drawing.fromId);
    const len = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]) || 1;
    drawing.to = [drawing.from[0] + ((w[0] - drawing.from[0]) / len) * r, drawing.from[1] + ((w[1] - drawing.from[1]) / len) * r];
    drawing.meshWith = meshWith;
    drawing.toId = null;
  } else if (drawing) {
    const hit = pointAt(w, drawing.fromId);
    drawing.toId = hit;
    drawing.to = hit !== null ? pos(hit) : snap(w);
  }
  if (dragging) {
    const p = design.points.find((pt) => pt.id === dragging!.id);
    if (p) {
      const [x, y] = snap(w);
      if (x !== p.x || y !== p.y) {
        if (!dragging.pushed) {
          pushUndo();
          dragging.pushed = true;
        }
        moveJoint(p.id, x - p.x, y - p.y); // joints pinned on a gear centred here move with it
        designChanged({ keepInspector: true });
      }
    }
  }
  hover = dragging || grab ? hover : hitTest(w);
  canvas.style.cursor = sim ? (grab ? 'grabbing' : hover?.kind === 'spring' ? 'pointer' : hover ? 'grab' : 'default') : tool === 'bar' || tool === 'spring' || tool === 'gear' ? 'crosshair' : hover ? 'pointer' : 'default';
});

canvas.addEventListener('pointerup', (e) => {
  const sp = screenOf(e);
  if (panning) {
    panning = null;
    return;
  }
  if (drawing && !drawing.clickMode) {
    if (Math.hypot(sp[0] - drawing.downAt[0], sp[1] - drawing.downAt[1]) > 6) finishSegment();
    else drawing.clickMode = true; // click-click mode: the next click ends the bar
  }
  if (dragging) {
    dragging = null;
    renderInspector();
  }
  grab = null;
  if (sim) sim.grab = null;
});

canvas.addEventListener(
  'wheel',
  (e) => {
    e.preventDefault();
    const sp = screenOf(e);
    const [wx, wy] = toWorld(view, sp);
    view.scale = Math.min(900, Math.max(25, view.scale * Math.exp(-e.deltaY * 0.0015)));
    view.ox = sp[0] - wx * view.scale;
    view.oy = sp[1] + wy * view.scale;
  },
  { passive: false },
);

// ---- keyboard -------------------------------------------------------------------
const toolKeys: Record<string, Tool> = { v: 'select', b: 'bar', s: 'spring', g: 'gear', m: 'mass', l: 'lockGear', f: 'fixed', x: 'lockX', y: 'lockY', n: 'free', d: 'delete' };

window.addEventListener('keydown', (e) => {
  const target = e.target as HTMLElement;
  if (target.closest('input, select, textarea')) return;
  const k = e.key.toLowerCase();
  if ((e.metaKey || e.ctrlKey) && k === 'z') {
    e.preventDefault();
    undo();
  } else if (k === ' ') {
    e.preventDefault();
    togglePlay();
  } else if (k === 'r') {
    stopSim();
  } else if (k === 'escape') {
    drawing = null;
    selection = null;
    renderInspector();
  } else if ((k === 'delete' || k === 'backspace') && !sim) {
    deleteSelection(selection);
  } else if (toolKeys[k] && !sim && !e.metaKey && !e.ctrlKey) {
    setTool(toolKeys[k]);
  }
});

// ---- toolbar ----------------------------------------------------------------------
const toolButtons = [...document.querySelectorAll<HTMLButtonElement>('#tools button')];
toolButtons.forEach((b) => b.addEventListener('click', () => setTool(b.dataset.tool as Tool)));

function setTool(t: Tool) {
  tool = t;
  drawing = null;
  refreshControls();
}

const playBtn = $<HTMLButtonElement>('#play');
const stepBtn = $<HTMLButtonElement>('#step');
const resetBtn = $<HTMLButtonElement>('#reset');
const undoBtn = $<HTMLButtonElement>('#undo');
const clearBtn = $<HTMLButtonElement>('#clear');
playBtn.addEventListener('click', togglePlay);
resetBtn.addEventListener('click', stopSim);
stepBtn.addEventListener('click', () => {
  if (!sim) {
    startSim();
    running = false;
  }
  advance(1 / 60);
  refreshControls();
});
undoBtn.addEventListener('click', undo);
clearBtn.addEventListener('click', () => {
  stopSim();
  pushUndo();
  design = emptyDesign();
  selection = null;
  designChanged();
});
$<HTMLButtonElement>('#fit').addEventListener('click', fitView);

const presetSel = $<HTMLSelectElement>('#preset');
for (const group of [...new Set(presets.map((p) => p.group))]) {
  const og = document.createElement('optgroup');
  og.label = group;
  presets.forEach((p, i) => p.group === group && og.append(new Option(p.name, String(i))));
  presetSel.append(og);
}
presetSel.addEventListener('change', () => {
  const p = presets[Number(presetSel.value)];
  presetSel.value = '';
  if (!p) return;
  stopSim();
  pushUndo();
  design = p.build();
  selection = null;
  designChanged();
  fitView();
});

function bindSlider(id: 'damping' | 'stiffness' | 'speed', fmt: (v: number) => string) {
  const input = $<HTMLInputElement>(`#${id}`);
  const out = $<HTMLOutputElement>(`#${id}-out`);
  const update = () => {
    settings[id] = Number(input.value);
    out.textContent = fmt(settings[id]);
  };
  input.addEventListener('input', update);
  update();
}
bindSlider('damping', (v) => `${v.toFixed(2)} s⁻¹`);
bindSlider('stiffness', (v) => `${v} N/m`);
bindSlider('speed', (v) => `${v.toFixed(1)}×`);

// friction: the simulator reads this object directly, so changes apply while running
const frictionBox = $<HTMLInputElement>('#friction');
const frictionPanel = $<HTMLDivElement>('#friction-settings');
frictionBox.addEventListener('change', () => {
  friction.enabled = frictionBox.checked;
  frictionPanel.hidden = !friction.enabled;
  renderEomPanel(eomEl, derivation, deriveError, friction.enabled);
});
function bindFriction(id: string, key: 'pin' | 'mesh' | 'rail' | 'coolingTime', fmt: (v: number) => string) {
  const input = $<HTMLInputElement>(`#${id}`);
  const out = $<HTMLOutputElement>(`#${id}-out`);
  const update = () => {
    friction[key] = Number(input.value);
    out.textContent = fmt(friction[key]);
  };
  input.addEventListener('input', update);
  update();
}
bindFriction('fpin', 'pin', (v) => `${v.toFixed(3)} N·m·s`);
bindFriction('fmesh', 'mesh', (v) => `${v.toFixed(1)} N·s/m`);
bindFriction('frail', 'rail', (v) => `${v.toFixed(1)} N·s/m`);
bindFriction('fcool', 'coolingTime', (v) => `${v.toFixed(1)} s`);

$<HTMLInputElement>('#gravity').addEventListener('change', (e) => {
  settings.gravity = (e.target as HTMLInputElement).checked;
  if (sim) rebuildSim();
  else derive();
});
$<HTMLInputElement>('#snap').addEventListener('change', (e) => (settings.snap = (e.target as HTMLInputElement).checked));

const eomWrap = $<HTMLElement>('#eom-wrap');
$<HTMLButtonElement>('#eom-toggle').addEventListener('click', (e) => {
  eomWrap.classList.toggle('collapsed');
  (e.target as HTMLButtonElement).textContent = eomWrap.classList.contains('collapsed') ? '+' : '–';
});

const hints: Record<Tool, string> = {
  select: 'Click a joint or bar to inspect it. Drag joints to reshape the mechanism.',
  bar: 'Drag, or click twice, to draw a bar. Start or end on an existing joint to pin bars together.',
  spring: 'Drag, or click twice, to draw a spring between joints. An end in empty space is anchored to the ground.',
  gear: 'Click a centre and drag out the radius. Touch another gear to mesh with it. Bars drawn on a gear are pinned to it.',
  fixed: 'Click a joint to pin it to the ground. Click again to free it.',
  lockX: 'Click a joint to lock its x position. It then slides on a vertical rail.',
  lockY: 'Click a joint to lock its y position. It then slides on a horizontal rail.',
  free: 'Click a joint to remove its constraint.',
  delete: 'Click a joint, bar, spring or gear to delete it.',
  mass: 'Click a joint to hang a 1 kg mass on it (click again to remove). Click on a gear to pin a weight to it, or in empty space for a free mass.',
  lockGear: 'Click a gear to lock it so it cannot turn. Click again to unlock it.',
};

function refreshControls() {
  toolButtons.forEach((b) => {
    b.classList.toggle('active', b.dataset.tool === tool && !sim);
    b.disabled = !!sim;
  });
  playBtn.textContent = running ? 'Pause' : sim ? 'Resume' : 'Play';
  resetBtn.disabled = !sim;
  undoBtn.disabled = !!sim;
  clearBtn.disabled = !!sim;
  hintEl.textContent = sim
    ? 'Drag a bar, joint or gear to pull it (red arrow = force). Click a spring to edit it live.'
    : hints[tool];
  renderInspector();
}

// ---- inspector ------------------------------------------------------------------
function renderInspector() {
  const disabled = sim ? 'disabled' : '';
  const sel = selection;
  if (sel?.kind === 'point') {
    const p = design.points.find((pt) => pt.id === sel.id);
    if (p) {
      const kinds: [PointKind, string][] = [
        ['free', 'Free'],
        ['fixed', 'Pinned to ground'],
        ['lockX', 'x locked (vertical rail)'],
        ['lockY', 'y locked (horizontal rail)'],
      ];
      inspectorEl.innerHTML = `
        <h4>Joint ${pointLabel(p.id)}</h4>
        <label class="field">Constraint<select id="i-kind" ${disabled}>${kinds
          .map(([k, label]) => `<option value="${k}" ${p.kind === k ? 'selected' : ''}>${label}</option>`)
          .join('')}</select></label>
        <label class="field">Point mass (kg)<input id="i-mass" type="number" min="0" step="0.1" value="${p.mass}" ${disabled}></label>
        <label class="field">x (m)<input id="i-x" type="number" step="0.1" value="${+p.x.toFixed(4)}" ${disabled}></label>
        <label class="field">y (m)<input id="i-y" type="number" step="0.1" value="${+p.y.toFixed(4)}" ${disabled}></label>
        ${p.gear !== undefined ? `<div class="field"><span>Pinned on</span><span>gear g${gearLabel(p.gear)} <button id="i-detach" ${disabled}>Detach</button></span></div>` : ''}`;
      inspectorEl.querySelector('#i-detach')?.addEventListener('click', () => {
        pushUndo();
        delete p.gear;
        designChanged();
      });
      bindField('#i-kind', (v) => (p.kind = v as PointKind));
      bindField('#i-mass', (v) => (p.mass = Math.max(0, Number(v) || 0)));
      bindField('#i-x', (v) => (p.x = Number(v) || 0));
      bindField('#i-y', (v) => (p.y = Number(v) || 0));
      return;
    }
  }
  if (sel?.kind === 'link') {
    const l = design.links.find((x) => x.id === sel.id);
    if (l) {
      inspectorEl.innerHTML = `
        <h4>Bar ${linkLabel(l.id)} <span class="muted">(${pointLabel(l.a)}–${pointLabel(l.b)})</span></h4>
        <label class="field">Mass (kg)<input id="i-mass" type="number" min="0.01" step="0.1" value="${l.mass}" ${disabled}></label>
        <div class="field"><span>Length</span><span>${linkLength(design, l).toFixed(3)} m</span></div>
        <p class="muted">Uniform rigid bar, I = mL²/12 about its centre.</p>`;
      bindField('#i-mass', (v) => (l.mass = Math.max(0.01, Number(v) || 0.01)));
      return;
    }
  }
  if (sel?.kind === 'spring') {
    const sp = design.springs.find((x) => x.id === sel.id);
    if (sp) {
      const now = sim ? Math.hypot(pos(sp.b)[0] - pos(sp.a)[0], pos(sp.b)[1] - pos(sp.a)[1]) : linkLength(design, sp);
      inspectorEl.innerHTML = `
        <h4>Spring k${springLabel(sp.id)} <span class="muted">(${pointLabel(sp.a)}–${pointLabel(sp.b)})</span></h4>
        <label class="field">Stiffness (N/m)<input id="i-k" type="number" min="0" step="5" value="${sp.k}"></label>
        <label class="field">Rest length (m)<input id="i-rest" type="number" min="0" step="0.05" value="${sp.rest}"></label>
        <div class="field"><span>Length now</span><span>${now.toFixed(3)} m <button id="i-rest-now" title="Set the rest length to the current length" ${disabled}>Use</button></span></div>
        <p class="muted">Massless, no constraint. Adds V = ½k(ℓ − ℓ₀)².</p>`;
      const apply = (fn: () => void) => {
        pushUndo();
        fn();
        designChanged({ keepInspector: true });
        if (sim) rebuildSim(); // parameters only: the coordinates and the current state carry over
      };
      inspectorEl.querySelector<HTMLInputElement>('#i-k')!.addEventListener('change', (e) =>
        apply(() => (sp.k = Math.max(0, Number((e.target as HTMLInputElement).value) || 0))),
      );
      inspectorEl.querySelector<HTMLInputElement>('#i-rest')!.addEventListener('change', (e) =>
        apply(() => (sp.rest = Math.max(0, Number((e.target as HTMLInputElement).value) || 0))),
      );
      inspectorEl.querySelector<HTMLButtonElement>('#i-rest-now')!.addEventListener('click', () => {
        apply(() => (sp.rest = +linkLength(design, sp).toFixed(4)));
        renderInspector();
      });
      return;
    }
  }
  if (sel?.kind === 'gear') {
    const g = design.gears.find((x) => x.id === sel.id);
    if (g) {
      const label = (id: number) => `g${gearLabel(id)}`;
      const meshed = design.meshes.filter((m) => m.a === g.id || m.b === g.id);
      const partner = (m: { a: number; b: number }) => (m.a === g.id ? m.b : m.a);
      const candidates = design.gears.filter((x) => x.id !== g.id && !meshed.some((m) => partner(m) === x.id));
      const I = 0.5 * g.mass * g.radius * g.radius;
      inspectorEl.innerHTML = `
        <h4>Gear ${label(g.id)} <span class="muted">(centre ${pointLabel(g.center)})</span></h4>
        <label class="field">Radius (m)<input id="i-r" type="number" min="0.03" step="0.01" value="${g.radius}" ${disabled}></label>
        <label class="field">Teeth<input id="i-n" type="number" min="6" max="300" step="1" value="${g.teeth}" ${disabled}></label>
        <label class="field">Mass (kg)<input id="i-m" type="number" min="0.01" step="0.1" value="${g.mass}" ${disabled}></label>
        <label class="field">Locked<span><input id="i-lock" type="checkbox" ${g.locked ? 'checked' : ''} ${disabled}> does not turn</span></label>
        <div class="field"><span>Meshes with</span><span>${
          meshed.map((m) => `${label(partner(m))} <button data-unmesh="${m.id}" title="Remove this mesh" ${disabled}>×</button>`).join(' ') || '<span class="muted">none</span>'
        }</span></div>
        ${candidates.length ? `<label class="field">Add mesh<select id="i-mesh" ${disabled}><option value="">choose a gear…</option>${candidates.map((x) => `<option value="${x.id}">${label(x.id)}</option>`).join('')}</select></label>` : ''}
        <p class="muted">Uniform disk, I = ½mR² = ${I.toFixed(4)} kg·m². Bars drawn on it are pinned to it.</p>`;
      bindField('#i-r', (v) => (g.radius = Math.max(0.03, Number(v) || 0.03)));
      bindField('#i-n', (v) => (g.teeth = Math.min(300, Math.max(6, Math.round(Number(v) || 6)))));
      bindField('#i-m', (v) => (g.mass = Math.max(0.01, Number(v) || 0.01)));
      inspectorEl.querySelector<HTMLInputElement>('#i-lock')!.addEventListener('change', (e) => {
        pushUndo();
        g.locked = (e.target as HTMLInputElement).checked;
        designChanged({ keepInspector: true });
      });
      inspectorEl.querySelectorAll<HTMLButtonElement>('[data-unmesh]').forEach((b) =>
        b.addEventListener('click', () => {
          pushUndo();
          design.meshes = design.meshes.filter((m) => m.id !== Number(b.dataset.unmesh));
          designChanged();
        }),
      );
      inspectorEl.querySelector<HTMLSelectElement>('#i-mesh')?.addEventListener('change', (e) => {
        const other = Number((e.target as HTMLSelectElement).value);
        if (!design.gears.some((x) => x.id === other)) return;
        pushUndo();
        design.meshes.push({ id: nextId(design.meshes), a: g.id, b: other });
        designChanged();
      });
      return;
    }
  }
  inspectorEl.innerHTML = `
    <h4>Inspector</h4>
    <p class="muted">Select a joint, bar, spring or gear to edit it. Press <b>Play</b> to derive Lagrange's equations and integrate them with RK4. While it runs, drag any part to pull it. Right-drag pans, the wheel zooms.</p>`;
}

function bindField(sel: string, apply: (v: string) => void) {
  const el = inspectorEl.querySelector<HTMLInputElement | HTMLSelectElement>(sel);
  el?.addEventListener('change', () => {
    pushUndo();
    apply(el.value);
    designChanged({ keepInspector: sel !== '#i-kind' });
  });
}

// ---- status ---------------------------------------------------------------------
function heatStatus(s: Simulator): string {
  let hottest = 0;
  let where = '';
  for (const [id, T] of simHeat.joints) if (T > hottest) [hottest, where] = [T, `joint ${pointLabel(id)}`];
  for (const [id, T] of simHeat.meshes) {
    const m = design.meshes.find((x) => x.id === id);
    if (T > hottest && m) [hottest, where] = [T, `gears g${gearLabel(m.a)}–g${gearLabel(m.b)}`];
  }
  const { T, V } = s.energy();
  return (
    ` · heat W = <b>${s.dissipated.toFixed(3)} J</b> · E + W = ${(T + V + s.dissipated).toFixed(4)} J` +
    (hottest > 0.01 ? ` · hottest: ${where} +${hottest.toFixed(1)} K` : '')
  );
}

function renderStatus() {
  const nBars = design.links.length;
  const nSprings = design.springs.length;
  const nGears = design.gears.length;
  if (simError) {
    statusEl.innerHTML = `<span class="err">${simError}</span>`;
    return;
  }
  if (!sim) {
    const dof = derivation ? derivation.coords.length - derivation.constraints.length : 0;
    statusEl.innerHTML = `Editing · <b>${nBars}</b> bar${nBars === 1 ? '' : 's'}${nSprings ? ` · <b>${nSprings}</b> spring${nSprings === 1 ? '' : 's'}` : ''}${nGears ? ` · <b>${nGears}</b> gear${nGears === 1 ? '' : 's'}` : ''} · <b>${dof}</b> degree${dof === 1 ? '' : 's'} of freedom`;
    return;
  }
  const { T, V } = sim.energy();
  const F = Math.hypot(...sim.appliedForce);
  const phi = sim.constraintError();
  statusEl.innerHTML =
    `${running ? 'Running' : 'Paused'} · t = <b>${sim.t.toFixed(2)} s</b> · E = T + V = <b>${(T + V).toFixed(4)} J</b>` +
    (sim.sys.nc ? ` · max|Φ| = ${phi.toExponential(1)}` : '') +
    (grab ? ` · |F| = <b>${F.toFixed(1)} N</b>` : '') +
    (friction.enabled || sim.dissipated > 0 ? heatStatus(sim) : '') +
    ` · RK4, h = 1 ms`;
}

// ---- view -----------------------------------------------------------------------
function resize() {
  const dpr = window.devicePixelRatio || 1;
  W = window.innerWidth;
  H = window.innerHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

/** Fit the mechanism into the area between the toolbar and the equations panel. */
function fitView() {
  const left = 240;
  const right = W > 900 ? W - 530 : W - 20;
  const top = 50;
  const bottom = H - 70;
  const pts = design.points.map((p) => [p.x, p.y] as Vec);
  if (!pts.length) pts.push([0, 0]);
  const xs = pts.map((p) => p[0]);
  const ys = pts.map((p) => p[1]);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  const scale = Math.min((right - left) / Math.max(x1 - x0 + 1.2, 1), (bottom - top) / Math.max(y1 - y0 + 1.2, 1));
  view.scale = Math.min(300, Math.max(60, scale));
  view.ox = (left + right) / 2 - ((x0 + x1) / 2) * view.scale;
  view.oy = (top + bottom) / 2 + ((y0 + y1) / 2) * view.scale;
}

// ---- main loop ------------------------------------------------------------------
let last = performance.now();
function frame(now: number) {
  const dt = Math.min((now - last) / 1000, 1 / 20);
  last = now;
  if (sim && running) advance(dt * settings.speed);

  let grabScene = null;
  if (sim && grab) {
    const a = pos(grab.a);
    const b = pos(grab.b);
    const gr = grab.gear && design.gears.find((x) => x.id === grab!.gear!.id);
    const at: Vec = gr ? gearPoint(gr, grab.gear!.u, grab.gear!.v) : [a[0] + (b[0] - a[0]) * grab.s, a[1] + (b[1] - a[1]) * grab.s];
    grabScene = { at, to: mouseWorld, force: running ? sim.appliedForce : ([0, 0] as Vec) };
  }
  drawScene(ctx, W, H, view, {
    design,
    pos,
    selection,
    hover,
    preview: drawing ? { from: drawing.from, to: drawing.to, kind: drawing.kind, meshWith: drawing.meshWith } : null,
    gearAngle: gearAngleNow,
    heat: sim ? simHeat : null,
    grab: grabScene,
    simulating: !!sim,
  });
  renderStatus();
  requestAnimationFrame(frame);
}

window.addEventListener('resize', resize);
resize();
fitView();
derive();
refreshControls();
requestAnimationFrame(frame);

// Exposed for debugging in the browser console.
Object.assign(window, { linkageLab: { get design() { return design; }, get derivation() { return derivation; }, get sim() { return sim; }, toScreen: (v: Vec) => toScreen(view, v) } });
