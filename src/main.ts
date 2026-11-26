import 'katex/dist/katex.min.css';
import './style.css';
import { deriveEquations, deriveSteps, type Derivation } from './mechanics/derive';
import { bodyName, cloneDesign, drawnAngle, emptyDesign, sameBody, type DBodyRef, gearLabel, linkLabel, linkLength, nextId, pointLabel, rackPitch, sharedJoint, springLabel, teethForRadius, type Design, type DGear, type DLink, type PointKind } from './mechanics/model';
import { CONTACT_OMEGA, dampingRatio, defaultFriction, NumericSystem, Simulator, type GroundSettings, type Heat } from './mechanics/simulate';
import { allParts, emptyParts, extractParts, fragmentCentre, partCount, partsInBox, pasteParts, type PartSet } from './mechanics/fragment';
import { enableLongEquations, eomPanelSteps } from './ui/eomPanel';
import { presets } from './ui/presets';
import { drawScene, segmentDistance, toScreen, toWorld, type Selection, type Vec, type View } from './ui/render';

type Tool = 'select' | 'bar' | 'rack' | 'spring' | 'gear' | 'mass' | 'lockGear' | 'angle' | 'join' | PointKind | 'delete';
/** What a part being drawn will mesh with, and where. */
type MeshTarget = { gear?: number; rack?: number; at: Vec; side?: 1 | -1 };

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
let drawing: { kind: 'bar' | 'rack' | 'spring' | 'gear'; mesh: MeshTarget | null; fromId: number | null; from: Vec; to: Vec; toId: number | null; downAt: Vec; clickMode: boolean } | null = null;
/** A dragged joint, or (`group`) the parts chosen together, moved by the snapped mouse offset. */
let dragging: { id: number; pushed: boolean; group?: { start: Vec; applied: Vec } } | null = null;
/** Parts chosen together by a selection box, Select all or a paste. */
let multi: PartSet | null = null;
/** The selection box being dragged in Select mode. */
let boxSel: { from: Vec; to: Vec; downAt: Vec } | null = null;
/** Parts copied with Ctrl/Cmd+C, as a design of their own. */
let clipboard: Design | null = null;
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
const LABELS_KEY = 'linkage-lab-labels';
const settings = { gravity: true, snap: true, labels: localStorage.getItem(LABELS_KEY) !== 'off', follow: false, damping: 0, stiffness: 80, speed: 1 };
const friction = defaultFriction();
let simHeat: Heat = { joints: new Map(), meshes: new Map(), racks: new Map() };

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
    if (d.ground && (typeof d.ground.y !== 'number' || typeof d.ground.slope !== 'number')) delete d.ground;
    if (!Array.isArray(d.angles)) d.angles = []; // saved before fixed angles existed
    d.angles = d.angles.filter((a) => a.a !== a.b && d.links.some((l) => l.id === a.a) && d.links.some((l) => l.id === a.b));
    if (!Array.isArray(d.joins)) d.joins = []; // saved before joins existed
    d.joins = d.joins.filter((j) => bodyExists(d, j.a) && bodyExists(d, j.b));
    if (!Array.isArray(d.rackMeshes)) d.rackMeshes = [];
    d.rackMeshes = d.rackMeshes.filter((m) => d.gears.some((g) => g.id === m.gear) && d.links.some((l) => l.id === m.rack && l.rack));
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
  multi = null;
  syncGroundUI();
  designChanged();
}

/** Call after every edit of the design. */
function designChanged(opts: { keepInspector?: boolean } = {}) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(design));
  scheduleDerive();
  if (!opts.keepInspector) renderInspector();
}

// ---- derivation --------------------------------------------------------------
// Deriving and typesetting a big mechanism takes a while (a 30-link chain: about a second), so
// both run as generators, a slice at a time, and the equations panel shows a green fill
// meanwhile. The new equations appear only once both are done.
const SLICE_MS = 12; // work per slice, so the page keeps repainting
const PROGRESS_DELAY_MS = 150; // quick jobs swap the panel without showing progress
const DERIVE_SHARE = 0.7; // part of the bar for deriving; the rest is typesetting
const eomProgress = $<HTMLDivElement>('#eom-progress');
const eomFill = $<HTMLDivElement>('#eom-progress .fill');
const eomProgressText = $<HTMLSpanElement>('#eom-progress .label');
enableLongEquations(eomEl);
let deriveTimer = 0;
let jobToken = 0; // starting a job bumps it, which cancels the job before
let jobStart = 0;
let deriving = false;

function scheduleDerive() {
  clearTimeout(deriveTimer);
  deriveTimer = window.setTimeout(deriveInBackground, 40);
}

const deriveOptions = () => ({ gravity: settings.gravity ? GRAVITY : 0 });
const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

/** Run a generator a slice at a time. It stops quietly if another job has started since. */
function runSliced<T>(steps: Generator<number, T, void>, token: number, progress: (f: number) => void, done: (v: T) => void, fail: (e: unknown) => void) {
  const tick = () => {
    if (token !== jobToken) return;
    const end = performance.now() + SLICE_MS;
    try {
      for (;;) {
        const r = steps.next();
        if (r.done) return done(r.value);
        if (performance.now() > end) {
          progress(r.value);
          setTimeout(tick, 0);
          return;
        }
      }
    } catch (e) {
      fail(e);
    }
  };
  tick();
}

function newJob(): number {
  clearTimeout(deriveTimer);
  jobStart = performance.now();
  return ++jobToken;
}

/** Fill the panel with green up to fraction f, once the job has run for a moment. */
function showProgress(f: number, what: string) {
  if (performance.now() - jobStart < PROGRESS_DELAY_MS) return;
  eomEl.hidden = true;
  eomProgress.hidden = false;
  eomFill.style.height = `${(100 * Math.min(Math.max(f, 0), 1)).toFixed(1)}%`;
  eomProgressText.textContent = `${what}… ${Math.round(100 * f)}%`;
}

function hideProgress() {
  eomProgress.hidden = true;
  eomEl.hidden = false;
}

function deriveInBackground() {
  const token = newJob();
  deriving = true;
  const snapshot = cloneDesign(design); // the design may change while this runs
  const finish = (d: Derivation | null, err: string | null) => {
    deriving = false;
    derivation = d;
    deriveError = err;
    typeset(token, DERIVE_SHARE);
  };
  runSliced(
    deriveSteps(snapshot, deriveOptions()),
    token,
    (f) => showProgress(DERIVE_SHARE * f, "Deriving Lagrange's equations"),
    (d) => finish(d, null),
    (e) => finish(null, errorText(e)),
  );
}

/** Derive right away (Play needs the equations now), then typeset the panel in the background. */
function derive() {
  const token = newJob();
  deriving = false;
  try {
    derivation = deriveEquations(design, deriveOptions());
    deriveError = null;
  } catch (e) {
    derivation = null;
    deriveError = errorText(e);
  }
  typeset(token, 0);
}

/** Typeset the current derivation into the panel; the bar runs from `from` to 100 %. */
function typeset(token: number, from: number) {
  runSliced(
    eomPanelSteps(derivation, deriveError, friction.enabled, groundNote()),
    token,
    (f) => showProgress(from + (1 - from) * f, 'Typesetting the equations'),
    (html) => {
      eomEl.innerHTML = html;
      hideProgress();
    },
    (e) => {
      eomEl.textContent = `Could not typeset the equations: ${errorText(e)}`;
      hideProgress();
    },
  );
}

/** Re-typeset after a display option changed; a derivation still running will typeset when done. */
function retypeset() {
  if (!deriving) typeset(newJob(), 0);
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
  sim.ground = groundSettings();
  if (sim.ground && belowGround()) flash('Some joints start below the ground, so it pushes them up.');
  simPositions = sim.jointPositions();
  simHeat = sim.heat();
  simGearAngles = sim.gearAngles();
  simError = null;
  running = true;
  drawing = null;
  dragging = null;
  // springs stay editable while running; a selected joint draws its path
  if (selection?.kind !== 'spring' && selection?.kind !== 'point') selection = null;
  multi = null;
  boxSel = null;
  resetTrail();
  renderInspector();
  refreshControls();
}

function stopSim() {
  sim = null;
  resetTrail();
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
  next.ground = groundSettings();
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
    if (trail.id >= 0) recordTrail(sim);
    if (!sim.step(H_STEP)) {
      running = false;
      simError = 'The solver hit a singular configuration (for example a fully stretched loop). Press Reset to edit.';
      refreshControls();
      break;
    }
  }
  if (trail.id >= 0) recordTrail(sim);
  if (!grab) sim.appliedForce = [0, 0];
  simPositions = sim.jointPositions();
  simGearAngles = sim.gearAngles();
  simHeat = sim.heat();
}

// ---- path of the selected joint ---------------------------------------------------
/** Points kept in the path; the oldest are dropped first. */
const TRAIL_MAX = 24000;
const trail = { id: -1, xs: new Float64Array(TRAIL_MAX), ys: new Float64Array(TRAIL_MAX), start: 0, n: 0 };

/** The joint whose path is drawn: the selected joint, while a simulation exists. */
function trailTarget(): number {
  return sim && selection?.kind === 'point' && sim.sys.pointIndex.has(selection.id) ? selection.id : -1;
}

function resetTrail(id = -1) {
  trail.id = id;
  trail.start = 0;
  trail.n = 0;
  if (id >= 0 && sim) recordTrail(sim);
}

/** Append the joint's current position, skipping moves smaller than about a pixel. */
function recordTrail(s: Simulator) {
  const p = s.jointPosition(trail.id);
  if (!p) return;
  if (trail.n) {
    const k = (trail.start + trail.n - 1) % TRAIL_MAX;
    if (Math.hypot(p[0] - trail.xs[k], p[1] - trail.ys[k]) < 1.5 / view.scale) return;
  }
  const k = (trail.start + trail.n) % TRAIL_MAX;
  trail.xs[k] = p[0];
  trail.ys[k] = p[1];
  if (trail.n < TRAIL_MAX) trail.n++;
  else trail.start = (trail.start + 1) % TRAIL_MAX;
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
function gearRadiusAt(c: Vec, w: Vec, centerId: number | null): { r: number; mesh: MeshTarget | null } {
  const r = Math.hypot(w[0] - c[0], w[1] - c[1]);
  const tol = 14 / view.scale;
  for (const g of design.gears) {
    if (g.center === centerId) continue;
    const gc = pos(g.center);
    const d = Math.hypot(gc[0] - c[0], gc[1] - c[1]);
    const touching = d - g.radius;
    if (touching > 0.03 && Math.abs(r - touching) < tol)
      return { r: touching, mesh: { gear: g.id, at: [c[0] + ((gc[0] - c[0]) * touching) / d, c[1] + ((gc[1] - c[1]) * touching) / d] } };
  }
  // touching a rack's pitch line: the gear rolls on the rack
  for (const l of design.links) {
    if (!l.rack || l.a === centerId || l.b === centerId) continue;
    const f = rackLine(l);
    const rx = c[0] - f.a[0];
    const ry = c[1] - f.a[1];
    const xi = rx * f.e[0] + ry * f.e[1];
    const h = ry * f.e[0] - rx * f.e[1];
    if (xi > 0 && xi < f.L && Math.abs(h) > 0.03 && Math.abs(r - Math.abs(h)) < tol)
      return { r: Math.abs(h), mesh: { rack: l.id, at: [f.a[0] + f.e[0] * xi, f.a[1] + f.e[1] * xi], side: h > 0 ? 1 : -1 } };
  }
  return { r: settings.snap ? Math.max(0.05, Math.round(r / 0.05) * 0.05) : r, mesh: null };
}

/** A rack's pitch line: start, unit direction and length. */
function rackLine(l: DLink) {
  const a = pos(l.a);
  const b = pos(l.b);
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1e-9;
  return { a, e: [(b[0] - a[0]) / L, (b[1] - a[1]) / L] as Vec, L };
}

/**
 * While drawing a rack from `from` towards `to`: if its pitch line passes close to a gear's
 * pitch circle, turn it about `from` so that it is exactly tangent, which makes them mesh.
 */
function rackTangentAt(from: Vec, to: Vec): { to: Vec; mesh: MeshTarget } | null {
  const len = Math.hypot(to[0] - from[0], to[1] - from[1]);
  if (len < 1e-6) return null;
  const u: Vec = [(to[0] - from[0]) / len, (to[1] - from[1]) / len];
  for (const g of design.gears) {
    const c = pos(g.center);
    const rc: Vec = [c[0] - from[0], c[1] - from[1]];
    const d = Math.hypot(rc[0], rc[1]);
    if (d <= g.radius + 1e-6) continue;
    const h = u[0] * rc[1] - u[1] * rc[0]; // > 0: the gear is on the rack's left
    if (Math.abs(Math.abs(h) - g.radius) > 14 / view.scale) continue;
    const ang = Math.atan2(rc[1], rc[0]) - Math.sign(h) * Math.asin(g.radius / d);
    const e: Vec = [Math.cos(ang), Math.sin(ang)];
    const xi = rc[0] * e[0] + rc[1] * e[1];
    if (xi <= 0 || xi >= len) continue;
    return { to: [from[0] + e[0] * len, from[1] + e[1] * len], mesh: { gear: g.id, at: [from[0] + e[0] * xi, from[1] + e[1] * xi], side: h > 0 ? 1 : -1 } };
  }
  return null;
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
  design.rackMeshes = design.rackMeshes.filter((m) => m.gear !== id);
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
    const other = design.gears.find((g) => g.id === d.mesh?.gear);
    const rack = design.links.find((l) => l.id === d.mesh?.rack && l.rack);
    // match the tooth pitch of what it meshes with
    const teeth = other
      ? Math.max(6, Math.round((other.teeth * r) / other.radius))
      : rack
        ? Math.max(6, Math.round((2 * Math.PI * r) / rackPitch(design, rack.id)))
        : teethForRadius(r);
    const id = nextId(design.gears);
    design.gears.push({ id, center, radius: +r.toFixed(4), teeth, mass: +Math.max(0.1, 25 * r * r).toFixed(2), locked: false });
    if (other) design.meshes.push({ id: nextId(design.meshes), a: other.id, b: id });
    if (rack) addRackMesh(id, rack, d.mesh?.side);
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
    const link: DLink = { id, a, b, mass: 1 };
    design.links.push(link);
    if (d.kind === 'rack') {
      link.rack = d.mesh?.side ?? 1;
      if (d.mesh?.gear !== undefined) addRackMesh(d.mesh.gear, link, d.mesh.side);
    }
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

/** Mesh a gear with a rack. A rack's first gear also decides which side its teeth are drawn on. */
function addRackMesh(gearId: number, rack: DLink, side?: 1 | -1) {
  if (side && !design.rackMeshes.some((m) => m.rack === rack.id)) rack.rack = side;
  design.rackMeshes.push({ id: nextId(design.rackMeshes), gear: gearId, rack: rack.id });
}

/** Which side of a rack a gear's centre is on. */
function sideOfRack(rack: DLink, g: DGear): 1 | -1 {
  const f = rackLine(rack);
  const c = pos(g.center);
  return (c[1] - f.a[1]) * f.e[0] - (c[0] - f.a[0]) * f.e[1] >= 0 ? 1 : -1;
}

function removeOrphans() {
  design.joins = joinsOf(design).filter((j) => bodyExists(design, j.a) && bodyExists(design, j.b));
  design.angles = anglesOf(design).filter((a) => design.links.some((l) => l.id === a.a) && design.links.some((l) => l.id === a.b));
  design.rackMeshes = design.rackMeshes.filter((m) => design.gears.some((g) => g.id === m.gear) && design.links.some((l) => l.id === m.rack && l.rack));
  design.points = design.points.filter(
    (p) =>
      [...design.links, ...design.springs].some((l) => l.a === p.id || l.b === p.id) ||
      design.gears.some((g) => g.center === p.id) ||
      p.mass > 0, // a particle, or a weight pinned on a gear
  );
}

// ---- fixed angles ---------------------------------------------------------------
const anglesOf = (d: Design) => (d.angles ??= []);
/** The first bar clicked with the Fix angle tool, waiting for the second. */
let angleFirst: number | null = null;

const findAngle = (a: number, b: number) => anglesOf(design).find((x) => (x.a === a && x.b === b) || (x.a === b && x.b === a));

/** Fix the angle between two bars, or free it if it is already fixed. */
function toggleAngle(a: number, b: number) {
  pushUndo();
  const found = findAngle(a, b);
  if (found) design.angles = anglesOf(design).filter((x) => x !== found);
  else anglesOf(design).push({ id: nextId(anglesOf(design)), a, b });
  designChanged();
}

// ---- joined bodies ---------------------------------------------------------------
function bodyExists(d: Design, ref: DBodyRef): boolean {
  return ref.kind === 'gear' ? d.gears.some((g) => g.id === ref.id) : d.links.some((l) => l.id === ref.id);
}
const joinsOf = (d: Design) => (d.joins ??= []);
/** The first body clicked with the Join tool, waiting for the second. */
let joinFirst: DBodyRef | null = null;

const findJoin = (a: DBodyRef, b: DBodyRef) => joinsOf(design).find((j) => (sameBody(j.a, a) && sameBody(j.b, b)) || (sameBody(j.a, b) && sameBody(j.b, a)));

/** Join two bodies into one, or separate them if they are already joined. */
function toggleJoin(a: DBodyRef, b: DBodyRef) {
  pushUndo();
  const found = findJoin(a, b);
  if (found) design.joins = joinsOf(design).filter((j) => j !== found);
  else joinsOf(design).push({ id: nextId(joinsOf(design)), a: { ...a }, b: { ...b } });
  designChanged();
}

/** The bar, rack or gear under the cursor (bars first, as they are drawn over gears). */
function bodyAt(w: Vec): DBodyRef | null {
  const l = linkAt(w);
  if (l) return { kind: 'link', id: l.id };
  const g = gearAt(w);
  return g ? { kind: 'gear', id: g.id } : null;
}

const bodySelection = (ref: DBodyRef): Selection => (ref.kind === 'gear' ? { kind: 'gear', id: ref.id } : { kind: 'link', id: ref.id });

/** Inspector rows listing a body's joins, with a picker to join it to another body. */
function joinsHtml(ref: DBodyRef, disabled: string): string {
  const mine = joinsOf(design).filter((j) => sameBody(j.a, ref) || sameBody(j.b, ref));
  const other = (j: { a: DBodyRef; b: DBodyRef }) => (sameBody(j.a, ref) ? j.b : j.a);
  const choices: DBodyRef[] = [
    ...design.links.map((l): DBodyRef => ({ kind: 'link', id: l.id })),
    ...design.gears.map((g): DBodyRef => ({ kind: 'gear', id: g.id })),
  ].filter((x) => !sameBody(x, ref) && !findJoin(ref, x));
  return `
    <div class="field"><span>One body with</span><span>${
      mine.map((j) => `${bodyName(design, other(j))} <button data-unjoin="${j.id}" title="Separate these bodies" ${disabled}>×</button>`).join('<br>') || '<span class="muted">none</span>'
    }</span></div>
    ${choices.length ? `<label class="field">Join to<select id="i-join" ${disabled}><option value="">choose a body…</option>${choices.map((x) => `<option value="${x.kind}:${x.id}">${bodyName(design, x)}</option>`).join('')}</select></label>` : ''}`;
}

function bindJoinControls(ref: DBodyRef) {
  inspectorEl.querySelectorAll<HTMLButtonElement>('[data-unjoin]').forEach((b) =>
    b.addEventListener('click', () => {
      pushUndo();
      design.joins = joinsOf(design).filter((j) => j.id !== Number(b.dataset.unjoin));
      designChanged();
    }),
  );
  inspectorEl.querySelector<HTMLSelectElement>('#i-join')?.addEventListener('change', (e) => {
    const [kind, id] = (e.target as HTMLSelectElement).value.split(':');
    const target: DBodyRef = { kind: kind === 'gear' ? 'gear' : 'link', id: Number(id) };
    if (id !== undefined && bodyExists(design, target)) toggleJoin(ref, target);
  });
}

// ---- ground contact ----------------------------------------------------------------
/** The design's ground in the simulator's units (slope in radians), or null. */
function groundSettings(): GroundSettings | null {
  const g = design.ground;
  return g ? { y: g.y, slope: (g.slope * Math.PI) / 180, restitution: g.restitution, friction: g.friction } : null;
}

function groundNote() {
  const g = design.ground;
  return g ? { ...g, zeta: dampingRatio(g.restitution), omega: CONTACT_OMEGA } : null;
}

/** Whether any joint of the design lies below its ground. */
function belowGround(): boolean {
  const g = design.ground;
  if (!g) return false;
  const t = Math.tan((g.slope * Math.PI) / 180);
  return design.points.some((p) => p.y < g.y + t * p.x - 1e-3);
}

const groundBox = $<HTMLInputElement>('#ground');
const groundPanel = $<HTMLDivElement>('#ground-settings');
const groundInputs = {
  y: $<HTMLInputElement>('#ground-y'),
  slope: $<HTMLInputElement>('#ground-slope'),
  restitution: $<HTMLInputElement>('#ground-e'),
  friction: $<HTMLInputElement>('#ground-mu'),
};
const groundOut = { restitution: $<HTMLOutputElement>('#ground-e-out'), friction: $<HTMLOutputElement>('#ground-mu-out') };

/** Show the design's ground in the settings panel. */
function syncGroundUI() {
  const g = design.ground;
  groundBox.checked = !!g;
  groundPanel.hidden = !g;
  if (!g) return;
  groundInputs.y.value = String(g.y);
  groundInputs.slope.value = String(g.slope);
  groundInputs.restitution.value = String(g.restitution);
  groundInputs.friction.value = String(g.friction);
  groundOut.restitution.textContent = g.restitution.toFixed(2);
  groundOut.friction.textContent = g.friction.toFixed(2);
}

/** The ground changed: save it, hand it to a running simulation, and redescribe it. */
function groundChanged() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(design));
  if (sim) sim.ground = groundSettings();
  syncGroundUI();
  retypeset();
}

groundBox.addEventListener('change', () => {
  pushUndo();
  if (groundBox.checked) {
    // just below the lowest joint or gear rim, so nothing starts in the ground
    const lows = [...design.points.map((p) => p.y), ...design.gears.map((g) => (design.points.find((p) => p.id === g.center)?.y ?? 0) - g.radius)];
    const y = lows.length ? Math.floor((Math.min(...lows) - 5e-4) * 100) / 100 : 0;
    design.ground = { y, slope: 0, restitution: 0.3, friction: 0.8 };
  } else delete design.ground;
  groundChanged();
});
for (const key of ['y', 'slope', 'restitution', 'friction'] as const) {
  groundInputs[key].addEventListener('input', () => {
    const g = design.ground;
    const v = Number(groundInputs[key].value);
    if (!g || !Number.isFinite(v)) return;
    g[key] = key === 'slope' ? Math.max(-45, Math.min(45, v)) : key === 'y' ? v : Math.max(0, v);
    if (sim) sim.ground = groundSettings();
    if (key === 'restitution' || key === 'friction') groundOut[key].textContent = g[key].toFixed(2);
  });
  groundInputs[key].addEventListener('change', groundChanged);
}
syncGroundUI();

// ---- groups: box selection, copy and paste ------------------------------------------
const partsKey = { point: 'points', link: 'links', spring: 'springs', gear: 'gears' } as const;
const inParts = (s: PartSet, sel: Selection) => s[partsKey[sel.kind]].has(sel.id);

/** The parts chosen together, without any that have since been deleted (null if none are left). */
function liveMulti(): PartSet | null {
  if (!multi) return null;
  const keep = (ids: Set<number>, items: readonly { id: number }[]) => new Set([...ids].filter((id) => items.some((x) => x.id === id)));
  multi = { points: keep(multi.points, design.points), links: keep(multi.links, design.links), springs: keep(multi.springs, design.springs), gears: keep(multi.gears, design.gears) };
  if (!partCount(multi)) multi = null;
  return multi;
}

/** What Copy, Cut and Delete act on: the parts chosen together, else the selected part. */
function currentParts(): PartSet | null {
  const group = liveMulti();
  if (group && !selection) return group;
  if (!selection) return null;
  const s = emptyParts();
  s[partsKey[selection.kind]].add(selection.id);
  return s;
}

/** Show a short message in the hint bar. */
let flashTimer = 0;
function flash(message: string) {
  hintEl.textContent = message;
  clearTimeout(flashTimer);
  flashTimer = window.setTimeout(refreshControls, 1800);
}

function copyParts(): boolean {
  const parts = currentParts();
  if (!parts) return false;
  clipboard = extractParts(design, parts);
  const n = clipboard.points.length + clipboard.links.length + clipboard.springs.length + clipboard.gears.length;
  flash(`Copied ${n} part${n === 1 ? '' : 's'}. Press Ctrl/⌘+V to paste them at the cursor.`);
  renderInspector();
  return true;
}

/** Paste the copied parts centred on the cursor; they become the new selection, ready to drag. */
function pasteClipboard() {
  if (!clipboard || sim) return;
  pushUndo();
  const c = fragmentCentre(clipboard);
  let off: Vec = [mouseWorld[0] - c[0], mouseWorld[1] - c[1]];
  if (settings.snap) off = [snapValue(off[0]), snapValue(off[1])];
  if (Math.hypot(off[0], off[1]) < 0.05) off = [0.2, -0.2]; // never exactly on top of the original
  setTool('select');
  selection = null;
  multi = pasteParts(design, clipboard, off[0], off[1]);
  designChanged();
}

/** Delete a group of parts, and the bars and springs attached to its joints. */
function deleteParts(parts: PartSet) {
  pushUndo();
  design.links = design.links.filter((l) => !parts.links.has(l.id) && !parts.points.has(l.a) && !parts.points.has(l.b));
  design.springs = design.springs.filter((l) => !parts.springs.has(l.id) && !parts.points.has(l.a) && !parts.points.has(l.b));
  for (const g of design.gears.filter((x) => parts.gears.has(x.id) || parts.points.has(x.center))) removeGear(g.id);
  design.points = design.points.filter((p) => !parts.points.has(p.id));
  removeOrphans();
  selection = null;
  multi = null;
  designChanged();
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
    if (pid !== null) {
      grab = { a: pid, b: pid, s: 0 };
      selection = { kind: 'point', id: pid }; // and draw its path
      renderInspector();
    } else {
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
    case 'rack':
    case 'spring':
    case 'gear': {
      if (drawing?.clickMode) {
        finishSegment();
        return;
      }
      const hit = pointAt(w);
      const from = hit !== null ? pos(hit) : snap(w);
      drawing = { kind: tool, mesh: null, fromId: hit, from, to: from, toId: null, downAt: sp, clickMode: false };
      break;
    }
    case 'select': {
      const hit = hitTest(w);
      const group = liveMulti();
      if (hit && group && inParts(group, hit)) {
        // drag the whole group
        dragging = { id: -1, pushed: false, group: { start: w, applied: [0, 0] } };
        break;
      }
      multi = null;
      selection = hit;
      if (hit?.kind === 'point') dragging = { id: hit.id, pushed: false };
      if (!hit) boxSel = { from: w, to: w, downAt: sp }; // empty space: start a selection box
      renderInspector();
      break;
    }
    case 'delete':
      deleteSelection(hitTest(w));
      break;
    case 'mass':
      toggleMass(w);
      break;
    case 'angle': {
      const hit = linkAt(w);
      if (!hit) {
        angleFirst = null;
        selection = null;
      } else if (angleFirst === null || angleFirst === hit.id || !design.links.some((l) => l.id === angleFirst)) {
        angleFirst = angleFirst === hit.id ? null : hit.id; // clicking the same bar again cancels
        selection = angleFirst === null ? null : { kind: 'link', id: hit.id };
      } else {
        const first = angleFirst;
        angleFirst = null;
        selection = { kind: 'link', id: hit.id };
        toggleAngle(first, hit.id);
      }
      refreshControls();
      break;
    }
    case 'join': {
      const hit = bodyAt(w);
      if (!hit) {
        joinFirst = null;
        selection = null;
      } else if (!joinFirst || sameBody(joinFirst, hit) || !bodyExists(design, joinFirst)) {
        joinFirst = joinFirst && sameBody(joinFirst, hit) ? null : hit; // the same body again cancels
        selection = joinFirst ? bodySelection(hit) : null;
      } else {
        const first = joinFirst;
        joinFirst = null;
        selection = bodySelection(hit);
        toggleJoin(first, hit);
      }
      refreshControls();
      break;
    }
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
    const { r, mesh } = gearRadiusAt(drawing.from, w, drawing.fromId);
    const len = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]) || 1;
    drawing.to = [drawing.from[0] + ((w[0] - drawing.from[0]) / len) * r, drawing.from[1] + ((w[1] - drawing.from[1]) / len) * r];
    drawing.mesh = mesh;
    drawing.toId = null;
  } else if (drawing) {
    const hit = pointAt(w, drawing.fromId);
    drawing.toId = hit;
    drawing.to = hit !== null ? pos(hit) : snap(w);
    drawing.mesh = null;
    if (drawing.kind === 'rack' && hit === null) {
      const t = rackTangentAt(drawing.from, w);
      if (t) [drawing.to, drawing.mesh] = [t.to, t.mesh];
    }
  }
  if (boxSel) boxSel.to = w;
  if (dragging?.group && multi) {
    const g = dragging.group;
    let off: Vec = [w[0] - g.start[0], w[1] - g.start[1]];
    if (settings.snap) off = [snapValue(off[0]), snapValue(off[1])]; // keeps grid-aligned parts on the grid
    const [dx, dy] = [off[0] - g.applied[0], off[1] - g.applied[1]];
    if (dx || dy) {
      if (!dragging.pushed) {
        pushUndo();
        dragging.pushed = true;
      }
      for (const p of design.points) if (multi.points.has(p.id)) [p.x, p.y] = [p.x + dx, p.y + dy];
      g.applied = off;
      designChanged({ keepInspector: true });
    }
  } else if (dragging) {
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
  canvas.style.cursor = sim ? (grab ? 'grabbing' : hover?.kind === 'spring' ? 'pointer' : hover ? 'grab' : 'default') : tool === 'bar' || tool === 'rack' || tool === 'spring' || tool === 'gear' ? 'crosshair' : hover ? 'pointer' : 'default';
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
  if (boxSel) {
    const box = boxSel;
    boxSel = null;
    if (Math.hypot(sp[0] - box.downAt[0], sp[1] - box.downAt[1]) > 4) {
      const parts = partsInBox(design, box.from, toWorld(view, sp));
      const only = partCount(parts) === 1 && parts.points.size === 1 ? [...parts.points][0] : null;
      if (only !== null) selection = { kind: 'point', id: only };
      else multi = partCount(parts) ? parts : null;
      renderInspector();
    }
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
    view.scale = Math.min(20000, Math.max(25, view.scale * Math.exp(-e.deltaY * 0.0015)));
    view.ox = sp[0] - wx * view.scale;
    view.oy = sp[1] + wy * view.scale;
  },
  { passive: false },
);

// ---- keyboard -------------------------------------------------------------------
const toolKeys: Record<string, Tool> = { v: 'select', b: 'bar', k: 'rack', s: 'spring', g: 'gear', m: 'mass', l: 'lockGear', a: 'angle', j: 'join', f: 'fixed', x: 'lockX', y: 'lockY', n: 'free', d: 'delete' };

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
  } else if ((e.metaKey || e.ctrlKey) && (k === 'c' || k === 'x') && !sim) {
    if (window.getSelection()?.toString()) return; // copying text from the page, e.g. the equations
    if (!copyParts()) return;
    e.preventDefault();
    if (k === 'x') deleteParts(currentParts()!);
  } else if ((e.metaKey || e.ctrlKey) && k === 'v' && !sim) {
    e.preventDefault();
    pasteClipboard();
  } else if ((e.metaKey || e.ctrlKey) && k === 'a' && !sim) {
    e.preventDefault();
    setTool('select');
    selection = null;
    multi = allParts(design);
    renderInspector();
  } else if (k === 'escape') {
    drawing = null;
    selection = null;
    multi = null;
    boxSel = null;
    renderInspector();
  } else if ((k === 'delete' || k === 'backspace') && !sim) {
    const group = liveMulti();
    if (group && !selection) deleteParts(group);
    else deleteSelection(selection);
  } else if (toolKeys[k] && !sim && !e.metaKey && !e.ctrlKey) {
    setTool(toolKeys[k]);
  }
});

// ---- toolbar ----------------------------------------------------------------------
const toolButtons = [...document.querySelectorAll<HTMLButtonElement>('#tools button')];
toolButtons.forEach((b) => b.addEventListener('click', () => setTool(b.dataset.tool as Tool)));

function setTool(t: Tool) {
  if (t !== 'select') multi = null;
  tool = t;
  angleFirst = null;
  joinFirst = null;
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
  multi = null;
  syncGroundUI();
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
  multi = null;
  syncGroundUI();
  settings.gravity = !p.topView; // a top view lies in the horizontal plane
  $<HTMLInputElement>('#gravity').checked = settings.gravity;
  settings.follow = followBox.checked = !!p.follow; // walkers leave the view otherwise
  const drag = $<HTMLInputElement>('#damping');
  drag.value = String(p.drag ?? 0); // tissue damping for the presets that ask for it
  drag.dispatchEvent(new Event('input'));
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
  retypeset();
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
const labelsBox = $<HTMLInputElement>('#labels');
labelsBox.checked = settings.labels;
labelsBox.addEventListener('change', () => {
  settings.labels = labelsBox.checked;
  localStorage.setItem(LABELS_KEY, settings.labels ? 'on' : 'off'); // remembered between visits
});
const followBox = $<HTMLInputElement>('#follow');
followBox.addEventListener('change', () => (settings.follow = followBox.checked));

/** Pan the view smoothly so the moving joints' centre stays in the middle of the free canvas. */
function followMechanism() {
  let [cx, cy, k] = [0, 0, 0];
  for (const id of sim!.sys.derivation.joints.filter((j) => !j.ground).map((j) => j.id)) {
    const p = simPositions.get(id);
    if (p) [cx, cy, k] = [cx + p[0], cy + p[1], k + 1];
  }
  if (!k) return;
  const left = 240;
  const right = W > 900 ? W - 530 : W - 20;
  const tx = (left + right) / 2 - (cx / k) * view.scale;
  const ty = H / 2 + (cy / k) * view.scale;
  view.ox += (tx - view.ox) * 0.08;
  view.oy += (ty - view.oy) * 0.08;
}

const eomWrap = $<HTMLElement>('#eom-wrap');
$<HTMLButtonElement>('#eom-toggle').addEventListener('click', (e) => {
  eomWrap.classList.toggle('collapsed');
  (e.target as HTMLButtonElement).textContent = eomWrap.classList.contains('collapsed') ? '+' : '–';
});

const hints: Record<Tool, string> = {
  select: 'Drag joints to reshape, or drag on empty space to box-select. Ctrl/⌘+C and V copy and paste.',
  bar: 'Drag, or click twice, to draw a bar. Start or end on an existing joint to pin bars together.',
  rack: 'Drag to draw a rack (a toothed bar). Pass along a gear and it snaps tangent and meshes. Lock its joints on rails to make it slide.',
  spring: 'Drag, or click twice, to draw a spring between joints. An end in empty space is anchored to the ground.',
  gear: 'Click a centre and drag out the radius. Touch another gear or a rack to mesh with it. Bars drawn on a gear are pinned to it.',
  fixed: 'Click a joint to pin it to the ground. Click again to free it.',
  lockX: 'Click a joint to lock its x position. It then slides on a vertical rail.',
  lockY: 'Click a joint to lock its y position. It then slides on a horizontal rail.',
  free: 'Click a joint to remove its constraint.',
  delete: 'Click a joint, bar, rack, spring or gear to delete it.',
  mass: 'Click a joint to hang a 1 kg mass on it (click again to remove). Click on a gear to pin a weight to it, or in empty space for a free mass.',
  lockGear: 'Click a gear to lock it so it cannot turn. Click again to unlock it.',
  angle: 'Click two bars to fix the angle between them. Click the same pair again to free it.',
  join: 'Click two bodies (bars, racks or gears) to join them into one. Click the same pair again to separate them.',
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
    : tool === 'angle' && angleFirst !== null
      ? `Now click the bar whose angle to bar ${linkLabel(angleFirst)} should stay fixed (or bar ${linkLabel(angleFirst)} again to cancel).`
      : tool === 'join' && joinFirst
        ? `Now click the body to join to ${bodyName(design, joinFirst)} (or ${bodyName(design, joinFirst)} again to cancel).`
        : hints[tool];
  renderInspector();
}

// ---- inspector ------------------------------------------------------------------
function renderInspector() {
  const disabled = sim ? 'disabled' : '';
  const group = sim ? null : liveMulti();
  if (group && !selection) {
    const n = partCount(group);
    const rows = (
      [
        ['Joints', group.points.size],
        ['Bars and racks', group.links.size],
        ['Springs', group.springs.size],
        ['Gears', group.gears.size],
      ] as const
    )
      .filter(([, k]) => k)
      .map(([name, k]) => `<div class="field"><span>${name}</span><span>${k}</span></div>`)
      .join('');
    inspectorEl.innerHTML = `
      <h4>${n} parts selected</h4>
      ${rows}
      <div class="field"><span></span><span><button id="i-copy">Copy</button> <button id="i-paste" ${clipboard ? '' : 'disabled'}>Paste</button> <button id="i-delete">Delete</button></span></div>
      <p class="muted">Drag any selected part to move them together. Ctrl/⌘+C copies them and Ctrl/⌘+V pastes a copy at the cursor, keeping the constraints among the copied parts (pins, rails, meshes, fixed angles, joins). Ctrl/⌘+X cuts, Delete removes, Esc deselects.</p>`;
    inspectorEl.querySelector('#i-copy')!.addEventListener('click', copyParts);
    inspectorEl.querySelector('#i-paste')!.addEventListener('click', pasteClipboard);
    inspectorEl.querySelector('#i-delete')!.addEventListener('click', () => deleteParts(group));
    return;
  }
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
      const meshed = design.rackMeshes.filter((m) => m.rack === l.id);
      const locks = anglesOf(design).filter((x) => x.a === l.id || x.b === l.id);
      const other = (x: { a: number; b: number }) => design.links.find((k) => k.id === (x.a === l.id ? x.b : x.a))!;
      // bars sharing a joint with this one first, as those are the usual ones to weld
      const lockable = design.links
        .filter((k) => k.id !== l.id && !findAngle(l.id, k.id))
        .sort((p, q) => Number(sharedJoint(l, q) !== null) - Number(sharedJoint(l, p) !== null) || p.id - q.id);
      const anglesHtml = `
        <div class="field"><span>Fixed angle to</span><span>${
          locks.map((x) => `bar ${linkLabel(other(x).id)} (${drawnAngle(design, l, other(x)).toFixed(1)}°) <button data-unangle="${x.id}" title="Free this angle" ${disabled}>×</button>`).join('<br>') ||
          '<span class="muted">none</span>'
        }</span></div>
        ${lockable.length ? `<label class="field">Fix angle to<select id="i-angle" ${disabled}><option value="">choose a bar…</option>${lockable.map((k) => `<option value="${k.id}">bar ${linkLabel(k.id)} (${pointLabel(k.a)}–${pointLabel(k.b)})</option>`).join('')}</select></label>` : ''}`;
      const candidates = design.gears.filter((g) => !meshed.some((m) => m.gear === g.id));
      inspectorEl.innerHTML = `
        <h4>${l.rack ? 'Rack' : 'Bar'} ${linkLabel(l.id)} <span class="muted">(${pointLabel(l.a)}–${pointLabel(l.b)})</span></h4>
        <label class="field">Mass (kg)<input id="i-mass" type="number" min="0.01" step="0.1" value="${l.mass}" ${disabled}></label>
        <div class="field"><span>Length</span><span>${linkLength(design, l).toFixed(3)} m</span></div>
        ${anglesHtml}
        ${joinsHtml({ kind: 'link', id: l.id }, disabled)}
        <label class="field">Toothed<span><input id="i-rack" type="checkbox" ${l.rack ? 'checked' : ''} ${disabled}> rack</span></label>
        ${
          l.rack
            ? `<label class="field">Teeth side<select id="i-side" ${disabled}><option value="1" ${l.rack === 1 ? 'selected' : ''}>left of ${pointLabel(l.a)}→${pointLabel(l.b)}</option><option value="-1" ${l.rack === -1 ? 'selected' : ''}>right of ${pointLabel(l.a)}→${pointLabel(l.b)}</option></select></label>
        <div class="field"><span>Meshes with</span><span>${
          meshed.map((m) => `g${gearLabel(m.gear)} <button data-unrack="${m.id}" title="Remove this mesh" ${disabled}>×</button>`).join(' ') || '<span class="muted">none</span>'
        }</span></div>
        ${candidates.length ? `<label class="field">Add mesh<select id="i-rmesh" ${disabled}><option value="">choose a gear…</option>${candidates.map((g) => `<option value="${g.id}">g${gearLabel(g.id)}</option>`).join('')}</select></label>` : ''}
        <p class="muted">A toothed bar, I = mL²/12. Its pitch line runs through ${pointLabel(l.a)} and ${pointLabel(l.b)}; a meshing gear rolls along it, turning rotation into sliding. Lock its joints on rails to make it slide.</p>`
            : '<p class="muted">Uniform rigid bar, I = mL²/12 about its centre.</p>'
        }`;
      bindField('#i-mass', (v) => (l.mass = Math.max(0.01, Number(v) || 0.01)));
      inspectorEl.querySelector<HTMLInputElement>('#i-rack')!.addEventListener('change', (e) => {
        pushUndo();
        if ((e.target as HTMLInputElement).checked) l.rack = 1;
        else {
          delete l.rack;
          design.rackMeshes = design.rackMeshes.filter((m) => m.rack !== l.id);
        }
        designChanged();
      });
      bindField('#i-side', (v) => (l.rack = v === '-1' ? -1 : 1));
      bindJoinControls({ kind: 'link', id: l.id });
      inspectorEl.querySelectorAll<HTMLButtonElement>('[data-unangle]').forEach((b) =>
        b.addEventListener('click', () => {
          pushUndo();
          design.angles = anglesOf(design).filter((x) => x.id !== Number(b.dataset.unangle));
          designChanged();
        }),
      );
      inspectorEl.querySelector<HTMLSelectElement>('#i-angle')?.addEventListener('change', (e) => {
        const k = Number((e.target as HTMLSelectElement).value);
        if (design.links.some((x) => x.id === k)) toggleAngle(l.id, k);
      });
      bindRackMeshButtons();
      inspectorEl.querySelector<HTMLSelectElement>('#i-rmesh')?.addEventListener('change', (e) => {
        const g = design.gears.find((x) => x.id === Number((e.target as HTMLSelectElement).value));
        if (!g) return;
        pushUndo();
        addRackMesh(g.id, l, sideOfRack(l, g));
        designChanged();
      });
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
        <label class="field">Tension only<span><input id="i-tension" type="checkbox" ${sp.tensionOnly ? 'checked' : ''}> slack when short</span></label>
        <p class="muted">Massless, no constraint. Adds V = ½k(ℓ − ℓ₀)²${sp.tensionOnly ? ' only while stretched (ℓ > ℓ₀), like a cable, ligament or tendon; drawn dashed while slack' : ''}.</p>`;
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
      inspectorEl.querySelector<HTMLInputElement>('#i-tension')!.addEventListener('change', (e) => {
        apply(() => {
          if ((e.target as HTMLInputElement).checked) sp.tensionOnly = true;
          else delete sp.tensionOnly;
        });
        renderInspector();
      });
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
      const racked = design.rackMeshes.filter((m) => m.gear === g.id);
      const rackCandidates = design.links.filter((l) => l.rack && !racked.some((m) => m.rack === l.id));
      const I = 0.5 * g.mass * g.radius * g.radius;
      inspectorEl.innerHTML = `
        <h4>Gear ${label(g.id)} <span class="muted">(centre ${pointLabel(g.center)})</span></h4>
        <label class="field">Radius (m)<input id="i-r" type="number" min="0.03" step="0.01" value="${g.radius}" ${disabled}></label>
        <label class="field">Teeth<input id="i-n" type="number" min="6" max="300" step="1" value="${g.teeth}" ${disabled}></label>
        <label class="field">Mass (kg)<input id="i-m" type="number" min="0.01" step="0.1" value="${g.mass}" ${disabled}></label>
        <label class="field">Locked<span><input id="i-lock" type="checkbox" ${g.locked ? 'checked' : ''} ${disabled}> does not turn</span></label>
        <div class="field"><span>Meshes with</span><span>${
          [
            ...meshed.map((m) => `${label(partner(m))} <button data-unmesh="${m.id}" title="Remove this mesh" ${disabled}>×</button>`),
            ...racked.map((m) => `rack ${linkLabel(m.rack)} <button data-unrack="${m.id}" title="Remove this mesh" ${disabled}>×</button>`),
          ].join(' ') || '<span class="muted">none</span>'
        }</span></div>
        ${
          candidates.length + rackCandidates.length
            ? `<label class="field">Add mesh<select id="i-mesh" ${disabled}><option value="">choose a gear or rack…</option>${[
                ...candidates.map((x) => `<option value="g${x.id}">${label(x.id)}</option>`),
                ...rackCandidates.map((l) => `<option value="r${l.id}">rack ${linkLabel(l.id)}</option>`),
              ].join('')}</select></label>`
            : ''
        }
        ${joinsHtml({ kind: 'gear', id: g.id }, disabled)}
        <p class="muted">Uniform disk, I = ½mR² = ${I.toFixed(4)} kg·m². Bars drawn on it are pinned to it.</p>`;
      bindJoinControls({ kind: 'gear', id: g.id });
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
      bindRackMeshButtons();
      inspectorEl.querySelector<HTMLSelectElement>('#i-mesh')?.addEventListener('change', (e) => {
        const v = (e.target as HTMLSelectElement).value;
        const id = Number(v.slice(1));
        const rack = v[0] === 'r' && design.links.find((l) => l.id === id && l.rack);
        if (rack) {
          pushUndo();
          addRackMesh(g.id, rack, sideOfRack(rack, g));
        } else if (v[0] === 'g' && design.gears.some((x) => x.id === id)) {
          pushUndo();
          design.meshes.push({ id: nextId(design.meshes), a: g.id, b: id });
        } else return;
        designChanged();
      });
      return;
    }
  }
  inspectorEl.innerHTML = `
    <h4>Inspector</h4>
    <p class="muted">Select a joint, bar, rack, spring or gear to edit it. Press <b>Play</b> to derive Lagrange's equations and integrate them with RK4. While it runs, drag any part to pull it. Right-drag pans, the wheel zooms.</p>`;
}

function bindRackMeshButtons() {
  inspectorEl.querySelectorAll<HTMLButtonElement>('[data-unrack]').forEach((b) =>
    b.addEventListener('click', () => {
      pushUndo();
      design.rackMeshes = design.rackMeshes.filter((m) => m.id !== Number(b.dataset.unrack));
      designChanged();
    }),
  );
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
  for (const [id, T] of simHeat.racks) {
    const m = design.rackMeshes.find((x) => x.id === id);
    if (T > hottest && m) [hottest, where] = [T, `gear g${gearLabel(m.gear)} on rack ${linkLabel(m.rack)}`];
  }
  const { T, V } = s.energy();
  return (
    ` · heat W = <b>${s.dissipated.toFixed(3)} J</b> · E + W = ${(T + V + s.dissipated).toFixed(4)} J` +
    (hottest > 0.01 ? ` · hottest: ${where} +${hottest.toFixed(1)} K` : '')
  );
}

function renderStatus() {
  const nBars = design.links.filter((l) => !l.rack).length;
  const nRacks = design.links.length - nBars;
  const nSprings = design.springs.length;
  const nGears = design.gears.length;
  if (simError) {
    statusEl.innerHTML = `<span class="err">${simError}</span>`;
    return;
  }
  if (!sim) {
    const dof = derivation ? derivation.coords.length - derivation.constraints.length : 0;
    statusEl.innerHTML = `Editing · <b>${nBars}</b> bar${nBars === 1 ? '' : 's'}${nRacks ? ` · <b>${nRacks}</b> rack${nRacks === 1 ? '' : 's'}` : ''}${nSprings ? ` · <b>${nSprings}</b> spring${nSprings === 1 ? '' : 's'}` : ''}${nGears ? ` · <b>${nGears}</b> gear${nGears === 1 ? '' : 's'}` : ''} · <b>${dof}</b> degree${dof === 1 ? '' : 's'} of freedom`;
    return;
  }
  const { T, V } = sim.energy();
  const F = Math.hypot(...grabPull);
  const phi = sim.constraintError();
  statusEl.innerHTML =
    `${running ? 'Running' : 'Paused'} · t = <b>${sim.t.toFixed(2)} s</b> · E = T + V = <b>${(T + V).toFixed(4)} J</b>` +
    (sim.sys.nc ? ` · max|Φ| = ${phi.toExponential(1)}` : '') +
    (grab ? ` · |F| = <b>${F.toFixed(1)} N</b>` : '') +
    (sim.ground ? ` · ${sim.contactCount()} touching the ground` : '') +
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
  const empty = !pts.length;
  if (empty) pts.push([0, 0]);
  const xs = pts.map((p) => p[0]);
  const ys = pts.map((p) => p[1]);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  // a margin in proportion to the mechanism, so a hand fills the view as a crane would
  const fit = (px: number, span: number) => px / (Math.max(span, 0.1) * 1.3 + 0.04);
  const scale = empty ? 170 : Math.min(fit(right - left, x1 - x0), fit(bottom - top, y1 - y0));
  view.scale = Math.min(20000, Math.max(25, scale));
  view.ox = (left + right) / 2 - ((x0 + x1) / 2) * view.scale;
  view.oy = (top + bottom) / 2 + ((y0 + y1) / 2) * view.scale;
}

// ---- main loop ------------------------------------------------------------------
let last = performance.now();
/** The grab spring's pull on the grabbed point this frame (N), shown by the arrow and status bar. */
let grabPull: Vec = [0, 0];
function frame(now: number) {
  const dt = Math.min((now - last) / 1000, 1 / 20);
  last = now;
  const target = trailTarget();
  if (target !== trail.id) resetTrail(target);
  if (sim && running) {
    advance(dt * settings.speed);
    if (settings.follow) followMechanism();
  }

  let grabScene = null;
  grabPull = [0, 0];
  if (sim && grab) {
    const a = pos(grab.a);
    const b = pos(grab.b);
    const gr = grab.gear && design.gears.find((x) => x.id === grab!.gear!.id);
    const at: Vec = gr ? gearPoint(gr, grab.gear!.u, grab.gear!.v) : [a[0] + (b[0] - a[0]) * grab.s, a[1] + (b[1] - a[1]) * grab.s];
    // the arrow is the spring's pull, k (cursor − point): it lies along the dashed line by construction
    const k = settings.stiffness;
    grabPull = running ? [k * (mouseWorld[0] - at[0]), k * (mouseWorld[1] - at[1])] : [0, 0];
    grabScene = { at, to: mouseWorld, force: grabPull };
  }
  drawScene(ctx, W, H, view, {
    design,
    pos,
    selection,
    hover,
    preview: drawing ? { from: drawing.from, to: drawing.to, kind: drawing.kind, meshAt: drawing.mesh?.at ?? null, side: drawing.mesh?.side } : null,
    gearAngle: gearAngleNow,
    heat: sim ? simHeat : null,
    grab: grabScene,
    simulating: !!sim,
    labels: settings.labels,
    ground: design.ground ? { y: design.ground.y, slope: design.ground.slope } : null,
    multi: sim ? null : liveMulti(),
    box: boxSel ? { from: boxSel.from, to: boxSel.to } : null,
    trail: trail.n > 1 ? trail : null,
  });
  renderStatus();
  requestAnimationFrame(frame);
}

window.addEventListener('resize', resize);
resize();
fitView();
deriveInBackground();
refreshControls();
requestAnimationFrame(frame);

// Exposed for debugging in the browser console.
Object.assign(window, { linkageLab: { get design() { return design; }, get derivation() { return derivation; }, get sim() { return sim; }, toScreen: (v: Vec) => toScreen(view, v) } });
