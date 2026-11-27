import 'katex/dist/katex.min.css';
import './style.css';
import { deriveEquations, deriveSteps, type Derivation } from './mechanics/derive';
import { bodyName, gearName, motorTorque, nextLeverColor, leverOf, pidTarget, measurable, quantityName, defaultGains, LEVER_TARGET_SCALE, QUANTITY_UNIT, type DSensor, type Quantity, type DPid, propellerAero, propellerMotor, clearConstraint, cloneDesign, motorHousingRadius, drawnAngle, emptyDesign, groundCeiling, groundHeight, lowerGroundUnder, railOf, straightGround, type DGround, sameBody, type DBodyRef, gearLabel, linkLabel, linkLength, nextId, pointLabel, rackPitch, sharedJoint, springLabel, teethForRadius, type Design, type DGear, type DLink, type PointKind } from './mechanics/model';
import { CONTACT_OMEGA, dampingRatio, defaultFriction, NumericSystem, Simulator, type ControllerSpec, type GroundSettings, type Heat } from './mechanics/simulate';
import { allParts, emptyParts, extractParts, fragmentCentre, partCount, partsInBox, pasteParts, type PartSet } from './mechanics/fragment';
import { enableLongEquations, eomPanelSteps } from './ui/eomPanel';
import { presets } from './ui/presets';
import { SoundEngine, type Cue, type MachineState } from './ui/sound';
import { drawScene, leverGeometry, pidGeometry, segmentDistance, toScreen, toWorld, type Selection, type Vec, type View } from './ui/render';

type Tool = 'select' | 'bar' | 'rack' | 'spring' | 'gear' | 'wheel' | 'propeller' | 'mass' | 'lockGear' | 'motor' | 'lever' | 'pid' | 'angle' | 'join' | 'pin' | 'slider' | 'free' | 'delete';
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
let drawing: { kind: 'bar' | 'rack' | 'spring' | 'gear' | 'wheel' | 'propeller'; mesh: MeshTarget | null; fromId: number | null; from: Vec; to: Vec; toId: number | null; downAt: Vec; clickMode: boolean } | null = null;
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
let simMotorAngles = new Map<number, number>();
let simSpinAngles = new Map<number, number>();
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
    // a ground saved as a straight line (height and slope) becomes waypoints across the machine
    const old = d.ground as unknown as { y?: number; slope?: number; restitution: number; friction: number } | undefined;
    if (old && !Array.isArray((old as unknown as DGround).points)) {
      if (typeof old.y === 'number' && typeof old.slope === 'number') {
        const xs = d.points.map((p) => p.x);
        const [lo, hi] = xs.length ? [Math.min(...xs) - 2, Math.max(...xs) + 2] : [-2, 2];
        d.ground = straightGround(old.y, old.slope, lo, hi, 9, old.restitution ?? 0.3, old.friction ?? 0.8);
      } else delete d.ground;
    }
    if (d.ground && (d.ground.points.length < 2 || d.ground.points.some((p) => !Array.isArray(p) || p.length !== 2))) delete d.ground;
    if (!Array.isArray(d.angles)) d.angles = []; // saved before fixed angles existed
    d.angles = d.angles.filter((a) => a.a !== a.b && d.links.some((l) => l.id === a.a) && d.links.some((l) => l.id === a.b));
    if (!Array.isArray(d.joins)) d.joins = []; // saved before joins existed
    d.joins = d.joins.filter((j) => bodyExists(d, j.a) && bodyExists(d, j.b));
    for (const p of d.points) {
      if (p.pin && !bodyExists(d, p.pin)) delete p.pin;
      if (p.slide && (typeof p.slide.angle !== 'number' || (p.slide.on && !bodyExists(d, p.slide.on)))) delete p.slide;
    }
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
  sfx('undo');
  design = undoStack.pop()!;
  selection = null;
  multi = null;
  constrain = null;
  groundSel = null;
  syncGroundUI();
  designChanged();
}

// ---- sound ----------------------------------------------------------------------------
const sound = new SoundEngine();
const sfx = (c: Cue) => sound.cue(c);
// browsers start audio only from a gesture
for (const ev of ['pointerdown', 'keydown'] as const) window.addEventListener(ev, () => sound.unlock(), { capture: true });

/** What the running mechanism sounds like now: its motors, propellers, gear teeth, contacts and speed. */
function machineState(s: Simulator): MachineState {
  const d = s.sys.derivation;
  // the fastest mesh: its pitch speed and how often a tooth passes (pitch speed / tooth pitch)
  let mesh: MachineState['mesh'] = null;
  if (d.meshes.length) {
    s.meshSpeeds().forEach((v, k) => {
      const g = design.gears.find((x) => x.id === d.meshes[k].a);
      if (!g || (mesh && Math.abs(v) <= mesh.speed)) return;
      mesh = { speed: Math.abs(v), rate: (Math.abs(v) * g.teeth) / (2 * Math.PI * g.radius) };
    });
  }
  return {
    motors: s.motorStates(),
    propellers: d.propellers.map((pr) => ({ id: pr.id, spin: pr.spin >= 0 ? s.qd[pr.spin] : 0, thrust: s.thrust.get(pr.id) ?? 0, blades: design.gears.find((g) => g.id === pr.id)?.propeller?.blades ?? 2 })),
    mesh,
    rolling: s.contactMotion.rolling,
    slip: s.contactMotion.slip,
    scrape: s.contactMotion.scrape,
    wind: s.maxJointSpeed(),
    friction: friction.enabled ? s.power.reduce((a, b) => a + b, 0) : 0,
    events: s.events,
  };
}

/** Call after every edit of the design. */
function designChanged(opts: { keepInspector?: boolean } = {}) {
  keepGroundUnder(); // the ground gives way under anything moved below it
  localStorage.setItem(STORAGE_KEY, JSON.stringify(design));
  scheduleDerive();
  if (!opts.keepInspector) renderInspector();
  else renderLevers();
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
  sim.setControllers(controllerSpecs());
  for (const c of sim.controllers)
    if (c.sign === 0) flash(`${gearName(design, c.motor)}'s motor cannot move what its PID controller measures, so the controller does nothing.`);
  if (sim.ground && belowGround()) flash('Some joints start below the ground, so it pushes them up.');
  simPositions = sim.jointPositions();
  simHeat = sim.heat();
  simGearAngles = sim.gearAngles();
  simMotorAngles = sim.motorAngles();
  simSpinAngles = sim.spinAngles();
  simError = null;
  running = true;
  drawing = null;
  dragging = null;
  // springs and motors stay editable while running; a selected joint draws its path
  if (selection?.kind !== 'spring' && selection?.kind !== 'point' && selection?.kind !== 'gear' && selection?.kind !== 'lever' && selection?.kind !== 'pid') selection = null;
  multi = null;
  boxSel = null;
  constrain = null;
  resetTrail();
  groundSel = null;
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
  sfx(running ? 'play' : 'pause');
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
  next.motorWork = sim.motorWork; // the energy budget carries on across a parameter change
  next.airWork = sim.airWork;
  next.setControllers(controllerSpecs(), sim.controllers);
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
      sfx('error');
      refreshControls();
      break;
    }
  }
  if (trail.id >= 0) recordTrail(sim);
  if (!grab) sim.appliedForce = [0, 0];
  simPositions = sim.jointPositions();
  simGearAngles = sim.gearAngles();
  simMotorAngles = sim.motorAngles();
  simSpinAngles = sim.spinAngles();
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

/** The motor whose lever (its knob or stick, drawn on the housing) is under w. */
function leverAt(w: Vec): DGear | null {
  const sp = toScreen(view, w);
  for (const g of design.gears) {
    if (!leverOf(g.motor) || !design.points.some((p) => p.id === g.center)) continue;
    const { base, knob, radius } = leverGeometry(view, pos(g.center), motorTurnNow(g), g);
    if (Math.hypot(sp[0] - knob[0], sp[1] - knob[1]) < radius + 4 || segmentDistance(sp, base, knob).d < 5) return g;
  }
  return null;
}

/** The turn of what a motor's lever and PID box ride on: its housing, or a propeller's hub. */
const motorTurnNow = (g: DGear) => (g.propeller ? gearAngleNow(g.id) : motorAngleNow(g.id));

/** The motor whose PID controller box is under w. */
function pidAt(w: Vec): DGear | null {
  const sp = toScreen(view, w);
  for (const g of design.gears) {
    if (!g.motor?.pid || !design.points.some((p) => p.id === g.center)) continue;
    const P = pidGeometry(view, pos(g.center), motorTurnNow(g), g);
    const [dx, dy] = [sp[0] - P.c[0], sp[1] - P.c[1]];
    const [u, v] = [dx * Math.cos(P.rot) + dy * Math.sin(P.rot), -dx * Math.sin(P.rot) + dy * Math.cos(P.rot)];
    if (Math.abs(u) <= P.w / 2 + 3 && Math.abs(v) <= P.h / 2 + 3) return g;
  }
  return null;
}

function hitTest(w: Vec): Selection | null {
  const lv = leverAt(w); // small, and drawn on top
  if (lv) return { kind: 'lever', id: lv.id };
  const pc = pidAt(w);
  if (pc) return { kind: 'pid', id: pc.id };
  const p = pointAt(w);
  if (p !== null) return { kind: 'point', id: p };
  const l = linkAt(w);
  if (l) return { kind: 'link', id: l.id };
  const sp = springAt(w);
  if (sp) return { kind: 'spring', id: sp.id };
  const g = propellerAt(w) ?? gearAt(w) ?? housingAt(w); // a motor's housing selects its gear (with the motor's settings)
  return g ? { kind: 'gear', id: g.id } : null;
}

/** The gear whose pitch disk contains w (the smallest one when several do). */
function gearAt(w: Vec): DGear | null {
  let best: DGear | null = null;
  for (const g of design.gears) {
    if (g.propeller || !design.points.some((p) => p.id === g.center)) continue; // a propeller is picked by its blades (propellerAt)
    const c = pos(g.center);
    if (Math.hypot(w[0] - c[0], w[1] - c[1]) < g.radius && (!best || g.radius < best.radius)) best = g;
  }
  return best;
}

const gearAngleNow = (id: number) => (sim ? (simGearAngles.get(id) ?? 0) : 0);
const motorAngleNow = (id: number) => (sim ? (simMotorAngles.get(id) ?? 0) : 0);

/** The gear whose motor housing (a box from the axle out to one side) contains w. */
/**
 * The propeller under w: near its blade line (the rotor disk, edge-on), its hub, or its motor
 * below the hub.
 */
function propellerAt(w: Vec): DGear | null {
  for (const g of design.gears) {
    if (!g.propeller || !design.points.some((p) => p.id === g.center)) continue;
    const c = pos(g.center);
    const th = g.propeller.angle + gearAngleNow(g.id);
    const [e, n]: [Vec, Vec] = [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]];
    const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
    const [u, v] = [dx * e[0] + dy * e[1], dx * n[0] + dy * n[1]];
    const tol = 8 / view.scale;
    const R = g.radius;
    if (Math.abs(u) <= R + tol && Math.abs(v) <= Math.max(0.12 * R, tol)) return g;
    if (g.motor && Math.abs(u) <= 0.13 * R + tol && v <= 0 && v >= -0.5 * R - tol) return g;
  }
  return null;
}

function housingAt(w: Vec): DGear | null {
  for (const g of design.gears) {
    if (!g.motor || g.propeller || !design.points.some((p) => p.id === g.center)) continue;
    const c = pos(g.center);
    const r = motorHousingRadius(g);
    const t = -motorAngleNow(g.id);
    const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
    const [u, v] = [dx * Math.cos(t) - dy * Math.sin(t), dx * Math.sin(t) + dy * Math.cos(t)];
    if (u >= -0.9 * r && u <= 2.6 * r && Math.abs(v) <= 0.9 * r) return g;
  }
  return null;
}

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
    if (g.center === centerId || g.wheel) continue; // a wheel has no teeth to mesh with
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
    if (g.wheel) continue;
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

/** Finish drawing a bar, a spring, a gear or a wheel. */
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
    sfx(other || rack ? 'mesh' : 'gear');
    if (other) design.meshes.push({ id: nextId(design.meshes), a: other.id, b: id });
    if (rack) addRackMesh(id, rack, d.mesh?.side);
    selection = { kind: 'gear', id };
    designChanged();
    return;
  }
  if (d.kind === 'propeller') {
    const r = length;
    if (r < 0.03) return;
    pushUndo();
    // in empty space its centre is a free joint: a free body of four coordinates, x, y, θ and φ
    const center = d.fromId ?? addPoint(d.from);
    const id = nextId(design.gears);
    const angle = +Math.atan2(d.to[1] - d.from[1], d.to[0] - d.from[0]).toFixed(6);
    design.gears.push({ id, center, radius: +r.toFixed(4), teeth: 2, mass: +Math.max(0.02, 2 * r * r).toFixed(3), locked: false, propeller: { angle, blades: 2 } });
    sfx('propeller');
    selection = { kind: 'gear', id };
    designChanged();
    return;
  }
  if (d.kind === 'wheel') {
    const r = length;
    if (r < 0.03) return;
    pushUndo();
    // A wheel drawn in empty space turns on a free axle (it drops and rolls); drawn on a joint it
    // turns about that joint, and inside a gear or wheel it rides on it.
    const center = d.fromId ?? newJoint(d.from);
    const id = nextId(design.gears);
    design.gears.push({ id, center, radius: +r.toFixed(4), teeth: teethForRadius(r), mass: +Math.max(0.1, 25 * r * r).toFixed(2), locked: false, wheel: true });
    sfx('wheel');
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
    sfx(d.kind === 'rack' ? (d.mesh?.gear !== undefined ? 'mesh' : 'rack') : 'bar');
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
    sfx('spring');
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

/** The hint bar while placing a pin or a rail. */
function constrainHint(): string {
  const c = constrain!;
  const j = pointLabel(c.joint);
  if (c.body === undefined)
    return c.tool === 'pin'
      ? `Now click the bar, rack or gear to pin joint ${j} on, or empty space to pin it to the ground (joint ${j} again frees it).`
      : `Now click the body joint ${j} slides on, or empty space for a ground rail (joint ${j} again frees it).`;
  if (!c.ends) return `Move the mouse to set the direction of the rail ${c.body ? `on ${bodyName(design, c.body)}` : 'on the ground'} (snaps to 15° and to the bar), then click. Esc cancels.`;
  return c.ends.length === 0
    ? `Click where the rail ends on one side, then on the other. Enter for a rail without ends.`
    : `Click where the rail ends on the other side. Enter for a stop on one side only.`;
}

function removeOrphans() {
  // sensors on bodies that are gone
  for (const g of design.gears) {
    const sn = g.motor?.pid?.sensor;
    if (!sn) continue;
    const { kind, id } = sn.body;
    const there = kind === 'point' ? design.points.some((p) => p.id === id) : kind === 'link' ? design.links.some((l) => l.id === id) : design.gears.some((x) => x.id === id);
    if (!there) g.motor!.pid!.sensor = null;
  }
  // pins and rails on bodies that are gone
  for (const p of design.points) {
    if (p.pin && !bodyExists(design, p.pin)) delete p.pin;
    if (p.slide?.on && !bodyExists(design, p.slide.on)) delete p.slide;
  }
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
  sfx(found ? 'free' : 'angle');
  designChanged();
}

// ---- joined bodies ---------------------------------------------------------------
function bodyExists(d: Design, ref: DBodyRef): boolean {
  if (ref.kind === 'motor') return d.gears.some((g) => g.id === ref.id && !!g.motor);
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
  sfx(found ? 'unjoin' : 'join');
  designChanged();
}

/** The bar, rack or gear under the cursor (bars first, as they are drawn over gears). */
function bodyAt(w: Vec): DBodyRef | null {
  const l = linkAt(w);
  if (l) return { kind: 'link', id: l.id };
  const g = propellerAt(w) ?? gearAt(w); // a propeller's motor is its hub, so it picks the propeller
  if (g) return { kind: 'gear', id: g.id };
  // the part of a motor's housing outside its gear is the motor
  const h = housingAt(w);
  return h ? { kind: 'motor', id: h.id } : null;
}

const bodySelection = (ref: DBodyRef): Selection => (ref.kind === 'link' ? { kind: 'link', id: ref.id } : { kind: 'gear', id: ref.id });

/** Inspector rows listing a body's joins, with a picker to join it to another body. */
function joinsHtml(ref: DBodyRef, disabled: string, tag = ''): string {
  const mine = joinsOf(design).filter((j) => sameBody(j.a, ref) || sameBody(j.b, ref));
  const other = (j: { a: DBodyRef; b: DBodyRef }) => (sameBody(j.a, ref) ? j.b : j.a);
  const choices: DBodyRef[] = [
    ...design.links.map((l): DBodyRef => ({ kind: 'link', id: l.id })),
    ...design.gears.map((g): DBodyRef => ({ kind: 'gear', id: g.id })),
    ...design.gears.filter((g) => g.motor && !g.propeller).map((g): DBodyRef => ({ kind: 'motor', id: g.id })), // a propeller's motor is the propeller
  ].filter((x) => !sameBody(x, ref) && !findJoin(ref, x) && !(x.id === ref.id && x.kind !== 'link' && ref.kind !== 'link')); // a motor and its own gear: that would lock it
  return `
    <div class="field"><span>One body with</span><span>${
      mine.map((j) => `${bodyName(design, other(j))} <button data-unjoin${tag}="${j.id}" title="Separate these bodies" ${disabled}>×</button>`).join('<br>') || '<span class="muted">none</span>'
    }</span></div>
    ${choices.length ? `<label class="field">Join to<select id="i-join${tag}" ${disabled}><option value="">choose a body…</option>${choices.map((x) => `<option value="${x.kind}:${x.id}">${bodyName(design, x)}</option>`).join('')}</select></label>` : ''}`;
}

function bindJoinControls(ref: DBodyRef, tag = '') {
  inspectorEl.querySelectorAll<HTMLButtonElement>(`[data-unjoin${tag}]`).forEach((b) =>
    b.addEventListener('click', () => {
      pushUndo();
      design.joins = joinsOf(design).filter((j) => j.id !== Number(b.getAttribute(`data-unjoin${tag}`)));
      designChanged();
    }),
  );
  inspectorEl.querySelector<HTMLSelectElement>(`#i-join${tag}`)?.addEventListener('change', (e) => {
    const [kind, id] = (e.target as HTMLSelectElement).value.split(':');
    const target: DBodyRef = { kind: kind === 'gear' || kind === 'motor' ? kind : 'link', id: Number(id) };
    if (id !== undefined && bodyExists(design, target)) toggleJoin(ref, target);
  });
}

// ---- ground contact ----------------------------------------------------------------
/** The design's ground for the simulator, or null. */
function groundSettings(): GroundSettings | null {
  const g = design.ground;
  return g ? { points: g.points.map((p) => [p[0], p[1]] as [number, number]), restitution: g.restitution, friction: g.friction } : null;
}

function groundNote() {
  const g = design.ground;
  return g ? { ...g, zeta: dampingRatio(g.restitution), omega: CONTACT_OMEGA } : null;
}

/** The lowest points of the machine: every joint, and the bottom of every gear rim. */
function machineLows(): Vec[] {
  return [
    ...design.points.map((p) => [p.x, p.y] as Vec),
    ...design.gears.flatMap((g) => {
      const c = design.points.find((p) => p.id === g.center);
      return c && !g.propeller ? [[c.x, c.y - g.radius] as Vec] : []; // a propeller touches nothing
    }),
  ];
}

/** How far the ground stays below the machine while editing (m). */
const GROUND_CLEARANCE = 0.002;

/** Whether any part of the design lies below its ground. */
function belowGround(): boolean {
  const g = design.ground;
  return !!g && machineLows().some(([x, y]) => y < groundHeight(g, x) - 1e-3);
}

/** The highest waypoint i can go while the ground stays under every part of the machine. */
function waypointCeiling(i: number): number {
  return groundCeiling(design.ground!, i, machineLows(), GROUND_CLEARANCE);
}

/** Lower the ground wherever a part of the machine has gone below it (editing only). */
function keepGroundUnder() {
  if (design.ground && !sim) lowerGroundUnder(design.ground, machineLows(), GROUND_CLEARANCE);
}

/** A new ground: waypoints across the machine and beyond, just below its lowest part. */
function defaultGround(): DGround {
  const lows = machineLows();
  const xs = lows.map((p) => p[0]);
  const [lo, hi] = xs.length ? [Math.min(...xs), Math.max(...xs)] : [0, 0];
  const mid = (lo + hi) / 2;
  const half = Math.max(1.5, (hi - lo) / 2 + 1);
  const y = lows.length ? Math.floor((Math.min(...lows.map((p) => p[1])) - GROUND_CLEARANCE - 5e-4) * 100) / 100 : 0;
  return straightGround(y, 0, +(mid - half).toFixed(2), +(mid + half).toFixed(2), 9, 0.3, 0.8);
}

/** The ground is selected (with the waypoint chosen, if any), or null. */
let groundSel: { waypoint: number | null } | null = null;
/** A waypoint being dragged up or down. */
let groundDrag: { index: number; pushed: boolean } | null = null;

/** Distance in pixels from a screen point to the ground's terrain line (with its level ends). */
function groundDistance(w: Vec): number {
  const g = design.ground;
  if (!g) return Infinity;
  return Math.abs(w[1] - groundHeight(g, w[0])) * view.scale;
}

/** The waypoint handle under the pointer, if the ground is selected. */
function waypointAt(w: Vec): number | null {
  const g = design.ground;
  if (!g || !groundSel) return null;
  let best: number | null = null;
  let bestD = 9 / view.scale;
  g.points.forEach(([x, y], i) => {
    const d = Math.hypot(x - w[0], y - w[1]);
    if (d < bestD) [best, bestD] = [i, d];
  });
  return best;
}

/** Move waypoint i to height y, but never above the machine. */
function setWaypointHeight(i: number, y: number) {
  const g = design.ground!;
  g.points[i][1] = +Math.min(y, waypointCeiling(i)).toFixed(4);
}

const groundBox = $<HTMLInputElement>('#ground');
const groundPanel = $<HTMLDivElement>('#ground-settings');
const groundInputs = {
  restitution: $<HTMLInputElement>('#ground-e'),
  friction: $<HTMLInputElement>('#ground-mu'),
};
const groundOut = { restitution: $<HTMLOutputElement>('#ground-e-out'), friction: $<HTMLOutputElement>('#ground-mu-out') };

/** Show the design's ground in the settings panel. */
function syncGroundUI() {
  const g = design.ground;
  groundBox.checked = !!g;
  groundPanel.hidden = !g;
  if (!g) groundSel = null;
  if (!g) return;
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
  if (groundBox.checked) design.ground = defaultGround();
  else delete design.ground;
  groundSel = null;
  groundChanged();
  renderInspector();
});
for (const key of ['restitution', 'friction'] as const) {
  groundInputs[key].addEventListener('input', () => {
    const g = design.ground;
    const v = Number(groundInputs[key].value);
    if (!g || !Number.isFinite(v)) return;
    g[key] = Math.max(0, v);
    if (sim) sim.ground = groundSettings();
    groundOut[key].textContent = g[key].toFixed(2);
  });
  groundInputs[key].addEventListener('change', groundChanged);
}
syncGroundUI();

/** Double-click on the ground (Select tool): add a waypoint there, on the terrain as it is. */
canvas.addEventListener('dblclick', (e) => {
  const g = design.ground;
  if (!g || sim || tool !== 'select') return;
  const w = toWorld(view, screenOf(e as unknown as PointerEvent));
  if (groundDistance(w) > 9) return;
  pushUndo();
  const x = +w[0].toFixed(3);
  const p: [number, number] = [x, +groundHeight(g, x).toFixed(4)];
  const at = g.points.findIndex((q) => q[0] > x);
  if (at < 0) g.points.push(p);
  else if (g.points[at][0] - x > 1e-3 && (at === 0 || x - g.points[at - 1][0] > 1e-3)) g.points.splice(at, 0, p);
  else return;
  selection = null;
  multi = null;
  groundSel = { waypoint: g.points.indexOf(p) };
  sfx('place');
  groundChanged();
  renderInspector();
});

// ---- groups: box selection, copy and paste ------------------------------------------
const partsKey = { point: 'points', link: 'links', spring: 'springs', gear: 'gears', lever: 'gears', pid: 'gears' } as const; // a lever goes with its motor's gear
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
  sfx('copy');
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
  sfx('paste');
  designChanged();
}

/** Delete a group of parts, and the bars and springs attached to its joints. */
function deleteParts(parts: PartSet) {
  pushUndo();
  sfx('delete');
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
  sfx('delete');
  if (sel.kind === 'point') {
    design.links = design.links.filter((l) => l.a !== sel.id && l.b !== sel.id);
    design.springs = design.springs.filter((l) => l.a !== sel.id && l.b !== sel.id);
    for (const g of design.gears.filter((x) => x.center === sel.id)) removeGear(g.id);
    design.points = design.points.filter((p) => p.id !== sel.id);
  } else if (sel.kind === 'link') {
    design.links = design.links.filter((l) => l.id !== sel.id);
  } else if (sel.kind === 'gear') {
    removeGear(sel.id);
  } else if (sel.kind === 'lever') {
    const m = design.gears.find((g) => g.id === sel.id)?.motor;
    if (m?.lever) {
      m.torque = motorTorque(m); // the motor keeps the torque the lever was giving
      delete m.lever;
    } else if (m?.pid?.lever) {
      m.pid.offset = pidTarget(m.pid); // the controller keeps the target the lever had set
      delete m.pid.lever;
    }
  } else if (sel.kind === 'pid') {
    const m = design.gears.find((g) => g.id === sel.id)?.motor;
    if (m) delete m.pid;
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
  sfx('mass');
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

// ---- pins and rails ------------------------------------------------------------------
/**
 * The Pin and Slider tools work in steps: the joint, then the body (or the ground), then for a
 * rail its direction.
 */
let constrain: { tool: 'pin' | 'slider'; joint: number; body?: DBodyRef | null; angle: number; ends?: number[] } | null = null;

/** Distance (m) along the rail being placed from its joint to the mouse, snapped like the grid. */
function railTravel(w: Vec): number {
  const c = constrain!;
  const p = design.points.find((pt) => pt.id === c.joint)!;
  const a = (c.angle * Math.PI) / 180;
  const t = (w[0] - p.x) * Math.cos(a) + (w[1] - p.y) * Math.sin(a);
  return +(settings.snap ? Math.round(t / 0.05) * 0.05 : t).toFixed(3);
}

/** Put the joint on the rail being placed, with stops at the ends chosen (none, one or two). */
function finishSlider() {
  const c = constrain!;
  const p = design.points.find((pt) => pt.id === c.joint);
  constrain = null;
  if (!p) return;
  pushUndo();
  sfx('pin');
  clearConstraint(p);
  p.slide = { on: c.body ?? null, angle: c.angle };
  const ends = c.ends ?? [];
  if (ends.length) {
    let [lo, hi] = ends.length === 2 ? [Math.min(...ends), Math.max(...ends)] : ends[0] < 0 ? [ends[0], null] : [null, ends[0]];
    // the joint must start inside its range: an end on the far side moves to the joint
    if (lo !== null && lo > 0) [lo, hi] = [0, hi];
    if (hi !== null && hi < 0) [lo, hi] = [lo, 0];
    if (ends.length === 2 && (Math.min(...ends) > 0 || Math.max(...ends) < 0)) flash(`Both ends were on one side of joint ${pointLabel(p.id)}, so the nearer end is at the joint.`);
    p.slide.range = { min: lo, max: hi, restitution: 0.2 };
  }
  designChanged();
  refreshControls();
}

/** Remove a joint's constraint (the Free joint tool). */
function freeJoint(id: number) {
  const p = design.points.find((pt) => pt.id === id);
  if (!p) return;
  pushUndo();
  sfx('free');
  clearConstraint(p);
  selection = { kind: 'point', id };
  designChanged();
}

/** Whether a body already carries the joint (a bar ending at it, the gear it sits on). */
function carries(ref: DBodyRef, pid: number): boolean {
  const p = design.points.find((pt) => pt.id === pid);
  if (ref.kind === 'link') return design.links.some((l) => l.id === ref.id && (l.a === pid || l.b === pid));
  if (ref.kind === 'motor') return design.gears.some((g) => g.id === ref.id && g.center === pid); // on its axle already
  return design.gears.some((g) => g.id === ref.id && g.center === pid) || p?.gear === ref.id;
}

/** The rail direction (degrees) towards the mouse, snapped to 15° steps or to the body's own bar. */
function railAngle(joint: number, w: Vec, body: DBodyRef | null | undefined): number {
  const p = design.points.find((pt) => pt.id === joint)!;
  const raw = (Math.atan2(w[1] - p.y, w[0] - p.x) * 180) / Math.PI;
  const fold = (a: number) => ((a % 180) + 180) % 180;
  const diff = (a: number, b: number) => Math.min(Math.abs(fold(a) - fold(b)), 180 - Math.abs(fold(a) - fold(b)));
  if (body?.kind === 'link') {
    const l = design.links.find((x) => x.id === body.id);
    const [a, b] = l ? [design.points.find((q) => q.id === l.a), design.points.find((q) => q.id === l.b)] : [undefined, undefined];
    if (a && b) {
      const along = (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI;
      for (const c of [along, along + 90]) if (diff(raw, c) < 6) return +fold(c).toFixed(2);
    }
  }
  return fold(Math.round(raw / 15) * 15);
}

/** One click of the Pin or Slider tool. */
function constrainClick(w: Vec) {
  const t = tool as 'pin' | 'slider';
  if (!constrain) {
    const pid = pointAt(w);
    if (pid === null) return;
    constrain = { tool: t, joint: pid, angle: 0 };
    selection = { kind: 'point', id: pid };
    refreshControls();
    return;
  }
  const p = design.points.find((pt) => pt.id === constrain!.joint);
  if (!p) {
    constrain = null;
    return;
  }
  const label = pointLabel(p.id);
  if (constrain.body === undefined) {
    // step 2: the body, or the ground
    if (pointAt(w) === p.id) {
      // the joint again: free it (or just cancel)
      const constrained = p.kind !== 'free' || p.gear !== undefined || p.pin || p.slide;
      constrain = null;
      if (constrained) freeJoint(p.id);
      else refreshControls();
      return;
    }
    const body = bodyAt(w);
    if (body && carries(body, p.id)) {
      flash(`Joint ${label} already moves with ${bodyName(design, body)}. Pick another body, or empty space for the ground.`);
      return;
    }
    if (t === 'pin') {
      pushUndo();
      clearConstraint(p);
      if (!body) p.kind = 'fixed';
      else if (body.kind === 'gear') p.gear = body.id;
      else p.pin = { ...body };
      sfx('pin');
      constrain = null;
      designChanged();
      refreshControls();
      return;
    }
    constrain.body = body ? { ...body } : null;
    constrain.angle = railAngle(p.id, w, constrain.body);
    refreshControls();
    return;
  }
  if (!constrain.ends) {
    // step 3 (slider): the direction; then the two ends of its range
    constrain.angle = railAngle(p.id, w, constrain.body);
    constrain.ends = [];
    refreshControls();
    return;
  }
  // steps 4 and 5: where the rail ends on one side, then the other
  constrain.ends.push(railTravel(w));
  if (constrain.ends.length === 2) finishSlider();
  else refreshControls();
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
  if (pidPick && !sim) {
    pickSensorAt(w, sp); // choosing what a PID controller measures, whatever the tool
    return;
  }

  if (sim) {
    const lv = leverAt(w);
    const pc = lv ? null : pidAt(w);
    const pid = lv || pc ? null : pointAt(w, null, true);
    if (lv) {
      selection = { kind: 'lever', id: lv.id }; // worked from its control in the dock
      renderInspector();
    } else if (pc) {
      selection = { kind: 'pid', id: pc.id }; // tunable while it runs
      renderInspector();
    } else if (pid !== null) {
      sfx('grab');
      grab = { a: pid, b: pid, s: 0 };
      selection = { kind: 'point', id: pid }; // and draw its path
      renderInspector();
    } else {
      const l = linkAt(w);
      const link = l && design.links.find((x) => x.id === l.id);
      const spr = springAt(w);
      const gr = gearAt(w);
      const pr = propellerAt(w);
      if (pr && sim.sys.pointIndex.has(pr.center)) {
        grab = { a: pr.center, b: pr.center, s: 0 }; // pulled by its centre
        selection = { kind: 'gear', id: pr.id };
        renderInspector();
      } else if (l && link && sim.sys.pointIndex.has(link.a)) grab = { a: link.a, b: link.b, s: l.s };
      else if (spr) {
        selection = { kind: 'spring', id: spr.id };
        renderInspector();
      } else if (gr && sim.sys.pointIndex.has(gr.center)) {
        // grab the gear disk at this material point, stored in the gear's own frame
        const c = pos(gr.center);
        const a = gearAngleNow(gr.id);
        const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
        grab = { a: gr.center, b: gr.center, s: 0, gear: { id: gr.id, u: dx * Math.cos(a) + dy * Math.sin(a), v: -dx * Math.sin(a) + dy * Math.cos(a) } };
        selection = { kind: 'gear', id: gr.id }; // its motor stays adjustable while it runs
        renderInspector();
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
    case 'gear':
    case 'wheel':
    case 'propeller': {
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
      // the selected ground's waypoints come first: grab one to drag it up or down
      const wp = waypointAt(w);
      if (wp !== null) {
        groundSel = { waypoint: wp };
        groundDrag = { index: wp, pushed: false };
        renderInspector();
        break;
      }
      const hit = hitTest(w);
      if (!hit && design.ground && groundDistance(w) < 8) {
        // the ground itself: select it, showing its waypoints
        selection = null;
        multi = null;
        groundSel = { waypoint: null };
        renderInspector();
        break;
      }
      groundSel = null;
      const group = liveMulti();
      if (hit && group && inParts(group, hit)) {
        // drag the whole group
        dragging = { id: -1, pushed: false, group: { start: w, applied: [0, 0] } };
        break;
      }
      multi = null;
      if (hit && (hit.kind !== selection?.kind || hit.id !== selection.id)) sfx('select');
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
    case 'motor': {
      const pid = pointAt(w);
      const g = propellerAt(w) ?? ((pid !== null && design.gears.find((x) => x.center === pid)) || gearAt(w));
      if (!g) break;
      pushUndo();
      sfx(g.motor ? 'motorOff' : 'motorOn');
      if (g.motor) {
        delete g.motor;
        removeOrphans();
      } else {
        // its housing starts mounted on the ground if the axle is; otherwise free, to be joined to something
        const fixed = design.points.find((p) => p.id === g.center)?.kind === 'fixed';
        g.motor = g.propeller ? propellerMotor(fixed) : { torque: 1, noLoadRpm: 60, mass: 0.5, mounted: fixed };
        g.locked = false; // a motor needs its gear free to turn
      }
      selection = { kind: 'gear', id: g.id };
      designChanged();
      break;
    }
    case 'lever': {
      const pid = pointAt(w);
      // on a PID controller (or the lever it has): the lever sets its target
      const onLever = leverAt(w);
      const onPid = pidAt(w) ?? (onLever?.motor?.pid ? onLever : null);
      if (onPid?.motor?.pid) {
        const pc = onPid.motor.pid;
        pushUndo();
        sfx(pc.lever ? 'delete' : 'lever');
        if (pc.lever) {
          pc.offset = pidTarget(pc); // the target stays where the lever had it
          delete pc.lever;
          selection = { kind: 'pid', id: onPid.id };
        } else {
          pc.lever = { color: nextLeverColor(design), scale: LEVER_TARGET_SCALE[pc.sensor?.quantity ?? 'angle'], position: 0 };
          selection = { kind: 'lever', id: onPid.id };
        }
        designChanged();
        break;
      }
      const g = onLever ?? propellerAt(w) ?? housingAt(w) ?? ((pid !== null && design.gears.find((x) => x.center === pid)) || gearAt(w));
      if (!g) break;
      pushUndo();
      sfx(g.motor?.lever ? 'delete' : 'lever');
      if (g.motor?.lever) {
        delete g.motor.lever;
        selection = { kind: 'gear', id: g.id };
      } else {
        if (g.motor?.pid) delete g.motor.pid; // a lever on the motor takes the PID controller's place
        // on a gear without a motor, the motor comes too; the lever starts at the centre
        const fixed = design.points.find((p) => p.id === g.center)?.kind === 'fixed';
        g.motor ??= g.propeller ? propellerMotor(fixed) : { torque: 1, noLoadRpm: 60, mass: 0.5, mounted: fixed };
        g.locked = false;
        g.motor.lever = { color: nextLeverColor(design), scale: +Math.max(0.1, Math.abs(g.motor.torque)).toFixed(2), position: 0 };
        selection = { kind: 'lever', id: g.id };
      }
      designChanged();
      break;
    }
    case 'pid': {
      const pid = pointAt(w);
      const g = pidAt(w) ?? leverAt(w) ?? propellerAt(w) ?? housingAt(w) ?? ((pid !== null && design.gears.find((x) => x.center === pid)) || gearAt(w));
      if (!g) break;
      pushUndo();
      if (g.motor?.pid) {
        sfx('delete');
        delete g.motor.pid;
        selection = { kind: 'gear', id: g.id };
        designChanged();
        break;
      }
      // the controller takes the lever's place; a gear without a motor gets one
      const fixed = design.points.find((p) => p.id === g.center)?.kind === 'fixed';
      g.motor ??= g.propeller ? propellerMotor(fixed) : { torque: 1, noLoadRpm: 60, mass: 0.5, mounted: fixed };
      g.locked = false;
      delete g.motor.lever;
      g.motor.pid = { kp: 0, ki: 0, kd: 0, offset: 0, sensor: null, color: nextLeverColor(design) };
      sfx('pid');
      selection = { kind: 'pid', id: g.id };
      designChanged();
      startSensorPick(g.id);
      break;
    }
    case 'lockGear': {
      const pid = pointAt(w);
      const g = propellerAt(w) ?? ((pid !== null && design.gears.find((x) => x.center === pid)) || gearAt(w));
      if (!g) break;
      pushUndo();
      g.locked = !g.locked;
      sfx(g.locked ? 'lock' : 'unlock');
      selection = { kind: 'gear', id: g.id };
      designChanged();
      break;
    }
    case 'pin':
    case 'slider':
      constrainClick(w);
      break;
    case 'free': {
      const pid = pointAt(w);
      if (pid !== null) freeJoint(pid);
      break;
    }
    default:
      break;
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
  if (drawing?.kind === 'propeller') {
    // the edge point: its distance is the radius, its direction the blade line (15° steps with snap on)
    const r0 = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]);
    const r = settings.snap ? Math.max(0.05, Math.round(r0 / 0.05) * 0.05) : r0;
    let a = Math.atan2(w[1] - drawing.from[1], w[0] - drawing.from[0]);
    if (settings.snap) a = Math.round(a / (Math.PI / 12)) * (Math.PI / 12);
    drawing.to = [drawing.from[0] + r * Math.cos(a), drawing.from[1] + r * Math.sin(a)];
    drawing.mesh = null;
    drawing.toId = null;
  } else if (drawing?.kind === 'gear' || drawing?.kind === 'wheel') {
    const free = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]);
    const { r, mesh } = drawing.kind === 'gear' ? gearRadiusAt(drawing.from, w, drawing.fromId) : { r: settings.snap ? Math.max(0.05, Math.round(free / 0.05) * 0.05) : free, mesh: null };
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
  if (groundDrag && design.ground) {
    const y = settings.snap ? Math.round(w[1] / 0.01) * 0.01 : w[1];
    if (!groundDrag.pushed) {
      pushUndo();
      groundDrag.pushed = true;
    }
    setWaypointHeight(groundDrag.index, y);
  }
  if (constrain && constrain.body !== undefined && !constrain.ends) constrain.angle = railAngle(constrain.joint, w, constrain.body);
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
  canvas.style.cursor = sim ? (grab ? 'grabbing' : hover?.kind === 'spring' ? 'pointer' : hover ? 'grab' : 'default') : tool === 'bar' || tool === 'rack' || tool === 'spring' || tool === 'gear' || tool === 'wheel' || tool === 'propeller' ? 'crosshair' : hover ? 'pointer' : 'default';
});

canvas.addEventListener('pointerup', (e) => {
  const sp = screenOf(e);
  if (groundDrag) {
    const moved = groundDrag.pushed;
    groundDrag = null;
    if (moved) groundChanged();
    renderInspector();
  }
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
const toolKeys: Record<string, Tool> = { v: 'select', b: 'bar', k: 'rack', s: 'spring', g: 'gear', w: 'wheel', p: 'propeller', m: 'mass', l: 'lockGear', o: 'motor', e: 'lever', c: 'pid', a: 'angle', j: 'join', f: 'pin', x: 'slider', n: 'free', d: 'delete' };

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
    if (sim) sfx('reset');
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
  } else if (k === 'enter' && constrain?.ends) {
    finishSlider(); // the ends chosen so far; none makes an unlimited rail
  } else if ((k === 'delete' || k === 'backspace') && !sim && groundSel?.waypoint != null && design.ground && !selection) {
    // remove the selected waypoint (a ground needs at least two)
    if (design.ground.points.length > 2) {
      pushUndo();
      design.ground.points.splice(groundSel.waypoint, 1);
      groundSel = { waypoint: null };
      groundChanged();
      renderInspector();
    } else flash('The ground needs at least two waypoints.');
  } else if (k === 'escape') {
    if (pidPick) cancelSensorPick();
    drawing = null;
    constrain = null;
    groundSel = null;
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
  if (pidPick) cancelSensorPick();
  constrain = null;
  groundSel = null;
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
resetBtn.addEventListener('click', () => {
  if (sim) sfx('reset');
  stopSim();
});
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
  constrain = null;
  groundSel = null;
  syncGroundUI();
  designChanged();
});
$<HTMLButtonElement>('#fit').addEventListener('click', () => fitView());

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
  sfx('preset');
  design = p.build();
  selection = null;
  multi = null;
  constrain = null;
  groundSel = null;
  syncGroundUI();
  settings.gravity = !p.topView; // a top view lies in the horizontal plane
  $<HTMLInputElement>('#gravity').checked = settings.gravity;
  settings.follow = followBox.checked = !!p.follow; // walkers leave the view otherwise
  const drag = $<HTMLInputElement>('#damping');
  drag.value = String(p.drag ?? 0); // tissue damping for the presets that ask for it
  drag.dispatchEvent(new Event('input'));
  designChanged();
  fitView(p.view);
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
const soundBox = $<HTMLInputElement>('#sound');
const volumeIn = $<HTMLInputElement>('#volume');
const volumeOut = $<HTMLOutputElement>('#volume-out');
soundBox.checked = sound.enabled;
volumeIn.value = String(sound.volume);
const showVolume = () => {
  volumeOut.textContent = `${Math.round(sound.volume * 100)}%`;
  $<HTMLElement>('#volume-row').hidden = !sound.enabled;
};
showVolume();
soundBox.addEventListener('change', () => {
  sound.setEnabled(soundBox.checked);
  showVolume();
  if (soundBox.checked) sfx('place');
});
volumeIn.addEventListener('input', () => {
  sound.setVolume(Number(volumeIn.value));
  showVolume();
});
volumeIn.addEventListener('change', () => sfx('place')); // a sample at the new volume

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
  wheel: 'Click a centre and drag out the radius. A wheel has no teeth: it meshes with nothing but rolls on the ground with friction. Start on a joint to mount it there (in empty space its axle is free, so it drops and rolls). Bars drawn on it are pinned to it.',
  pin: 'Click a joint, then the bar, rack or gear to pin it on (it moves with that body), or empty space to pin it to the ground.',
  slider: 'Click a joint, then the body its rail is on (or empty space for a ground rail), then set the rail\'s direction.',

  free: 'Click a joint to remove its constraint: a pin, a rail, or a pin on a body.',
  delete: 'Click a joint, bar, rack, spring or gear to delete it.',
  mass: 'Click a joint to hang a 1 kg mass on it (click again to remove). Click on a gear to pin a weight to it, or in empty space for a free mass.',
  lockGear: 'Click a gear or wheel to lock it so it cannot turn (a braked wheel). Click again to unlock it.',
  motor: 'Click a gear or wheel to put a motor on it (1 N·m, 60 rpm no-load); set its torque in the inspector. Click again to remove it.',
  pid: 'Click a motor (or a gear, which gets a motor) to put a PID controller on it, in place of a lever. Then click what it should measure (a joint, bar, rack, gear, wheel or propeller) and pick the quantity. Click the motor again to remove it.',
  propeller: 'Click the centre, then the edge of a blade: the blades turn out of the page about an axis square to that line, and push along it (the arrow). In empty space it is a free body; start on a joint to mount it there. Give it a motor (O) to spin it.',
  lever: 'Click a motor (or a gear, which gets a motor) to give it a control lever; its control appears at the bottom right. Drag the control up or down to set the torque, even while it runs. Click the motor again to remove the lever.',
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
      : pidPick
        ? `Now click what the PID controller on motor M${gearLabel(pidPick.gear)} should measure: a joint, bar, rack, gear, wheel or propeller (Esc to leave it without a sensor).`
        : constrain
        ? constrainHint()
        : tool === 'join' && joinFirst
        ? `Now click the body to join to ${bodyName(design, joinFirst)} (or ${bodyName(design, joinFirst)} again to cancel).`
        : hints[tool];
  renderInspector();
}

// ---- PID controllers: sensors and the controller's inspector ---------------------------
const esc = (t: string) => t.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

/** A PID controller waiting for the body its sensor goes on. */
let pidPick: { gear: number } | null = null;
const sensorMenu = document.createElement('div');
sensorMenu.id = 'sensor-menu';
sensorMenu.className = 'panel';
sensorMenu.hidden = true;
document.body.append(sensorMenu);

function startSensorPick(gear: number) {
  pidPick = { gear };
  sensorMenu.hidden = true;
  refreshControls();
}

function cancelSensorPick() {
  pidPick = null;
  sensorMenu.hidden = true;
  refreshControls();
}

/** A quantity's value in the drawing (display units): where things were drawn, at rest. */
function drawnValue(body: DSensor['body'], q: Quantity): number {
  if (q === 'vx' || q === 'vy' || q === 'omega' || q === 'spin') return 0;
  const P = (id: number) => design.points.find((p) => p.id === id);
  if (body.kind === 'point') {
    const p = P(body.id);
    return p ? (q === 'x' ? p.x : p.y) : 0;
  }
  if (body.kind === 'link') {
    const l = design.links.find((x) => x.id === body.id);
    const [a, b] = l ? [P(l.a), P(l.b)] : [undefined, undefined];
    if (!a || !b) return 0;
    if (q === 'angle') return (Math.atan2(b.y - a.y, b.x - a.x) * 180) / Math.PI;
    return q === 'x' ? (a.x + b.x) / 2 : (a.y + b.y) / 2;
  }
  const g = design.gears.find((x) => x.id === body.id);
  const c = g && P(g.center);
  if (!g || !c) return 0;
  if (q === 'angle') return g.propeller ? (g.propeller.angle * 180) / Math.PI : 0;
  return q === 'x' ? c.x : c.y;
}

/** The PID controller picking a sensor: the body under w, then a menu of what it can measure there. */
function pickSensorAt(w: Vec, sp: Vec) {
  if (!pidPick) return;
  const pt = pointAt(w);
  const l = pt === null ? linkAt(w) : null;
  const g = pt === null && !l ? (propellerAt(w) ?? gearAt(w)) : null;
  const body: DSensor['body'] | null = pt !== null ? { kind: 'point', id: pt } : l ? { kind: 'link', id: l.id } : g ? { kind: 'gear', id: g.id } : null;
  if (!body) {
    flash('Click a joint, bar, rack, gear, wheel or propeller for the sensor (Esc to leave it without one).');
    return;
  }
  const what = body.kind === 'point' ? `joint ${pointLabel(body.id)}` : bodyName(design, body as DBodyRef);
  const gear = pidPick.gear;
  sensorMenu.innerHTML = `<div class="sm-title">Measure on ${esc(what)}:</div>${measurable(design, body)
    .map((q) => `<button data-q="${q}">${esc(quantityName(q, body, design))}<span>${q === 'angle' || q === 'x' || q === 'y' ? fmtQ(drawnValue(body, q), q) : QUANTITY_UNIT[q]}</span></button>`)
    .join('')}<button class="sm-cancel">Cancel</button>`;
  sensorMenu.hidden = false;
  // beside the click, kept on screen
  const r = sensorMenu.getBoundingClientRect();
  sensorMenu.style.left = `${Math.min(sp[0] + 12, window.innerWidth - r.width - 8)}px`;
  sensorMenu.style.top = `${Math.min(sp[1] + 12, window.innerHeight - r.height - 8)}px`;
  sensorMenu.querySelectorAll<HTMLButtonElement>('[data-q]').forEach((b) => b.addEventListener('click', () => setSensor(gear, body, b.dataset.q as Quantity)));
  sensorMenu.querySelector('.sm-cancel')!.addEventListener('click', () => (sensorMenu.hidden = true)); // pick another body, or Esc
}

/**
 * Put the sensor on: the controller's target starts at the quantity's value as drawn (so it
 * holds the machine where it is), with gains suited to the quantity and the motor's torque.
 */
function setSensor(gear: number, body: DSensor['body'], q: Quantity) {
  const g = design.gears.find((x) => x.id === gear);
  const pc = g?.motor?.pid;
  cancelSensorPick();
  if (!g || !pc) return;
  pushUndo();
  const fresh = !pc.sensor || pc.sensor.quantity !== q;
  pc.sensor = { body: { ...body }, quantity: q };
  if (fresh) {
    pc.offset = +drawnValue(body, q).toFixed(4);
    Object.assign(pc, defaultGains(q, g.motor!.torque));
    if (pc.lever) pc.lever.scale = LEVER_TARGET_SCALE[q];
  }
  sfx('sensor');
  selection = { kind: 'pid', id: gear };
  setTool('select');
  designChanged();
}

/** The PID controllers, as the simulator takes them. */
function controllerSpecs(): ControllerSpec[] {
  return design.gears.flatMap((g): ControllerSpec[] => {
    const pc = g.motor?.pid;
    if (!pc?.sensor) return [];
    const { body, quantity } = pc.sensor;
    let sensor: ControllerSpec['sensor'];
    if (body.kind === 'point') sensor = { quantity, point: body.id };
    else if (body.kind === 'link') {
      const l = design.links.find((x) => x.id === body.id);
      if (!l) return [];
      sensor = { quantity, a: l.a, b: l.b };
    } else {
      const sg = design.gears.find((x) => x.id === body.id);
      if (!sg) return [];
      sensor = { quantity, gear: sg.id, center: sg.center, angle0: sg.propeller?.angle ?? 0 };
    }
    return [{ motor: g.id, sensor, kp: pc.kp, ki: pc.ki, kd: pc.kd, max: Math.abs(g.motor!.torque), target: pidTarget(pc) }];
  });
}

/** The controller's live line in the inspector: what it measures, aims at and puts out. */
function pidLiveText(g: DGear): string {
  const pc = g.motor!.pid!;
  const q = pc.sensor?.quantity;
  if (!pc.sensor) return 'no sensor yet';
  const c = sim?.controllers.find((x) => x.motor === g.id);
  if (!c) return `${fmtQ(drawnValue(pc.sensor.body, pc.sensor.quantity), q)} as drawn; target ${fmtQ(pidTarget(pc), q)}`;
  if (c.sign === 0) return 'its motor cannot move this quantity';
  return `${fmtQ(c.y, q)} → ${fmtQ(c.target, q)} (error ${fmtQ(c.e, q)}); torque ${c.u.toFixed(3)} N·m${Math.abs(c.u) >= Math.abs(c.max) - 1e-9 ? ' (at its limit)' : ''}`;
}

/** The inspector for a PID controller: its sensor, target, gains and limit, and what it is doing. */
function inspectorPid(g: DGear, pc: DPid, disabled: string) {
  const lab = gearLabel(g.id);
  const q = pc.sensor?.quantity;
  const unit = QUANTITY_UNIT[q ?? 'angle'];
  const num = (id: string, label: string, value: number, title: string, dis = '') =>
    `<label class="field" title="${title}"><span>${label}</span><input id="${id}" type="number" step="any" value="${+value.toPrecision(5)}" ${dis}></label>`;
  inspectorEl.innerHTML = `
    <h4><span style="color:${pc.color}">●</span> PID controller <span class="muted">(motor M${lab} on ${gearName(design, g.id)})</span></h4>
    <div class="field"><span>Sensor</span><span>${
      pc.sensor ? `${esc(quantityName(pc.sensor.quantity, pc.sensor.body, design))} of ${pc.sensor.body.kind === 'point' ? `joint ${pointLabel(pc.sensor.body.id)}` : esc(bodyName(design, pc.sensor.body as DBodyRef))}` : '<span class="muted">none</span>'
    } <button id="i-psensor" ${disabled}>${pc.sensor ? 'Change…' : 'Choose…'}</button></span></div>
    ${num('i-poff', `Target offset (${unit})`, pc.offset, 'The target with the lever centred, or without a lever')}
    ${
      pc.lever
        ? `<div class="field"><span>Target lever</span><span><button id="i-golever" style="color:${pc.lever.color}">lever L${lab}</button> ±${pc.lever.scale} ${unit}</span></div>`
        : `<div class="field"><span></span><span><button id="i-paddlever" ${disabled}>Add a lever for the target</button></span></div>`
    }
    ${num('i-kp', `K<sub>p</sub> (N·m per ${unit})`, pc.kp, 'Proportional gain')}
    ${num('i-ki', `K<sub>i</sub> (N·m per ${unit}·s)`, pc.ki, 'Integral gain: removes the steady error a constant load leaves')}
    ${num('i-kd', `K<sub>d</sub> (N·m·s per ${unit})`, pc.kd, 'Derivative gain: damps, acting on the rate of the measurement')}
    ${num('i-pmax', 'Max torque (N·m)', Math.abs(g.motor!.torque), "The motor's torque limit; the output stays within ±this")}
    <label class="field">Colour<input id="i-pcolor" type="color" value="${pc.color}"></label>
    <div class="field"><span>Now</span><span id="i-plive">${pidLiveText(g)}</span></div>
    <div class="field"><span></span><span><button id="i-premove" ${disabled}>Remove controller</button></span></div>
    <p class="muted">Torque u = s (K<sub>p</sub> e + K<sub>i</sub> ∫e dt − K<sub>d</sub> dy/dt), e = target − y, kept within ±max torque; the integral pauses while the output is at its limit, and the derivative acts on the measurement, so moving the target gives no kick. s = ±1 is the way the motor moves the quantity, found when it starts. Gains, target and limit can be changed while it runs.</p>`;
  const bindNum = (sel: string, apply: (v: number) => void, live?: () => void) =>
    inspectorEl.querySelector<HTMLInputElement>(sel)!.addEventListener('change', (e) => {
      const v = Number((e.target as HTMLInputElement).value);
      if (!Number.isFinite(v)) return;
      pushUndo();
      apply(v);
      designChanged({ keepInspector: true });
      live?.();
      renderLevers();
    });
  const tune = () => sim?.tuneController(g.id, { kp: pc.kp, ki: pc.ki, kd: pc.kd, target: pidTarget(pc) });
  bindNum('#i-poff', (v) => (pc.offset = v), tune);
  bindNum('#i-kp', (v) => (pc.kp = v), tune);
  bindNum('#i-ki', (v) => (pc.ki = v), tune);
  bindNum('#i-kd', (v) => (pc.kd = v), tune);
  bindNum(
    '#i-pmax',
    (v) => (g.motor!.torque = Math.max(1e-3, Math.abs(v))),
    () => sim && rebuildSim(), // b = max / Ω changes as well
  );
  inspectorEl.querySelector<HTMLInputElement>('#i-pcolor')!.addEventListener('input', (e) => {
    pc.color = (e.target as HTMLInputElement).value;
    inspectorEl.querySelector<HTMLElement>('h4 span')!.style.color = pc.color;
    saveSoon();
  });
  inspectorEl.querySelector('#i-psensor')!.addEventListener('click', () => startSensorPick(g.id));
  inspectorEl.querySelector('#i-golever')?.addEventListener('click', () => {
    selection = { kind: 'lever', id: g.id };
    renderInspector();
  });
  inspectorEl.querySelector('#i-paddlever')?.addEventListener('click', () => {
    pushUndo();
    pc.lever = { color: nextLeverColor(design), scale: LEVER_TARGET_SCALE[q ?? 'angle'], position: 0 };
    sfx('lever');
    designChanged();
  });
  inspectorEl.querySelector('#i-premove')!.addEventListener('click', () => deleteSelection({ kind: 'pid', id: g.id }));
}

// ---- inspector ------------------------------------------------------------------
// ---- levers: the control dock at the bottom right ------------------------------------
// One control per lever, side by side, each a vertical slot with a handle in the lever's colour:
// up is forward, the middle is zero. More than fit scroll sideways, the cut-off edges fading out.
const leverDock = $<HTMLDivElement>('#levers');
const leverScroll = $<HTMLDivElement>('#levers .lever-scroll');
let leverDrag: { id: number; pointer: number } | null = null;
let saveTimer = 0;

const leverGears = () => design.gears.filter((g) => leverOf(g.motor) && design.points.some((p) => p.id === g.center)).sort((a, b) => a.id - b.id);
/** What a lever is giving now: its motor's torque, or its PID controller's target. */
const leverTorqueText = (g: DGear) => {
  const pc = g.motor!.pid;
  if (pc?.lever) return `→ ${fmtQ(pidTarget(pc), pc.sensor?.quantity)}`;
  const t = motorTorque(g.motor!);
  return `${t > 0 ? '+' : ''}${t.toFixed(2)} N·m`;
};
const leverSettingText = (g: DGear) => {
  const { lever, owner } = leverOf(g.motor)!;
  const p = Math.round(lever.position * 100);
  if (owner === 'motor') return `${p > 0 ? '+' : ''}${p}% of ${lever.scale} N·m = ${leverTorqueText(g)}`;
  const pc = g.motor!.pid!;
  const q = pc.sensor?.quantity;
  return `offset ${fmtQ(pc.offset, q)} ${p >= 0 ? '+' : '−'} ${Math.abs(p)}% of ${lever.scale} ${QUANTITY_UNIT[q ?? 'angle']}: target ${fmtQ(pidTarget(pc), q)}`;
};
/** A quantity's value with its unit. */
function fmtQ(v: number, q: Quantity | undefined): string {
  const unit = QUANTITY_UNIT[q ?? 'angle'];
  const digits = unit === 'm' || unit === 'm/s' ? 3 : unit === 'rpm' ? 0 : 1;
  return `${v.toFixed(digits)}${unit === '°' ? '°' : ` ${unit}`}`;
}

/** Save the design shortly, once a stream of changes (a lever being dragged) pauses. */
function saveSoon() {
  clearTimeout(saveTimer);
  saveTimer = window.setTimeout(() => localStorage.setItem(STORAGE_KEY, JSON.stringify(design)), 250);
}

/**
 * Move a lever, −1 … 1 (snapping to the centre close by). While running, only the motor's
 * torque parameter changes, so the motion carries on; the derivation's value is kept in step
 * for the next run, without deriving again.
 */
function setLeverPosition(g: DGear, p: number) {
  const m = g.motor;
  const lv = leverOf(m);
  if (!m || !lv) return;
  const before = Math.round(lv.lever.position * 20);
  lv.lever.position = Math.abs(p) < 0.04 ? 0 : Math.max(-1, Math.min(1, +p.toFixed(3)));
  if (Math.round(lv.lever.position * 20) !== before) sfx('tick'); // a ratchet click every 5 %
  if (lv.owner === 'pid') sim?.tuneController(g.id, { target: pidTarget(m.pid!) }); // a new target for the controller
  else {
    const tau = motorTorque(m);
    sim?.setMotorTorque(g.id, tau);
    const prm = derivation?.params.find((x) => x.sym === `tau${g.id}`);
    if (prm) prm.value = tau;
  }
  const card = leverScroll.querySelector<HTMLElement>(`[data-lever="${g.id}"]`);
  if (card) {
    card.style.setProperty('--p', String(lv.lever.position));
    card.querySelector('.lever-value')!.textContent = leverTorqueText(g);
  }
  const shown = inspectorEl.querySelector('#i-lpos');
  if (shown && selection?.kind === 'lever' && selection.id === g.id) shown.textContent = leverSettingText(g);
  saveSoon();
}

/** Fade out whichever ends of the dock are scrolled out of view. */
function updateLeverFades() {
  const el = leverScroll;
  el.style.setProperty('--fl', el.scrollLeft > 1 ? '32px' : '0px');
  el.style.setProperty('--fr', el.scrollLeft + el.clientWidth < el.scrollWidth - 1 ? '32px' : '0px');
}

function markSelectedLever() {
  leverScroll.querySelectorAll<HTMLElement>('.lever').forEach((c) => c.classList.toggle('selected', selection?.kind === 'lever' && selection.id === Number(c.dataset.lever)));
}

function renderLevers() {
  if (leverDrag) {
    markSelectedLever(); // rebuilding would drop the control being dragged
    return;
  }
  const gs = leverGears();
  leverDock.hidden = !gs.length;
  leverScroll.innerHTML = gs
    .map((g) => {
      const { lever: lv, owner } = leverOf(g.motor)!;
      const lab = gearLabel(g.id);
      const unit = owner === 'pid' ? QUANTITY_UNIT[g.motor!.pid!.sensor?.quantity ?? 'angle'] : 'N·m';
      return `<div class="lever" data-lever="${g.id}" style="--c:${lv.color};--p:${lv.position}">
        <button class="lever-name" title="Lever L${lab} on ${owner === 'pid' ? 'the PID controller of motor' : 'motor'} M${lab}: click for its settings">L${lab}${owner === 'pid' ? '<small> PID</small>' : ''}</button>
        <div class="lever-track" title="${owner === 'pid' ? `Drag to set the controller's target: ±${lv.scale} ${unit} about its offset; double-click to centre` : `Drag up for forward (anticlockwise), down for reverse; double-click to centre. Full throw: ±${lv.scale} N·m`}">
          <div class="lever-zero"></div><div class="lever-handle"></div>
        </div>
        <div class="lever-value">${leverTorqueText(g)}</div>
      </div>`;
    })
    .join('');
  markSelectedLever();
  for (const card of leverScroll.querySelectorAll<HTMLElement>('.lever')) {
    const id = Number(card.dataset.lever);
    const gear = () => design.gears.find((x) => x.id === id);
    const track = card.querySelector<HTMLElement>('.lever-track')!;
    const fromPointer = (e: PointerEvent) => {
      const r = track.getBoundingClientRect();
      return 1 - (2 * (e.clientY - r.top - 6)) / (r.height - 12); // the slot runs 6 px in from each end
    };
    card.querySelector('.lever-name')!.addEventListener('click', () => {
      selection = { kind: 'lever', id };
      multi = null;
      renderInspector();
    });
    track.addEventListener('pointerdown', (e) => {
      const g = gear();
      if (!g || e.button !== 0) return;
      e.preventDefault();
      track.setPointerCapture(e.pointerId);
      if (!sim) pushUndo();
      leverDrag = { id, pointer: e.pointerId };
      card.classList.add('dragging');
      setLeverPosition(g, fromPointer(e));
      if (selection?.kind !== 'lever' || selection.id !== id) {
        selection = { kind: 'lever', id };
        multi = null;
        renderInspector();
      }
    });
    track.addEventListener('pointermove', (e) => {
      const g = gear();
      if (g && leverDrag?.pointer === e.pointerId) setLeverPosition(g, fromPointer(e));
    });
    const release = (e: PointerEvent) => {
      if (leverDrag?.pointer !== e.pointerId) return;
      leverDrag = null;
      card.classList.remove('dragging');
      if (!sim) scheduleDerive(); // refresh the parameters shown in the equations panel
    };
    track.addEventListener('pointerup', release);
    track.addEventListener('pointercancel', release);
    track.addEventListener('dblclick', () => {
      const g = gear();
      if (!g) return;
      if (!sim) pushUndo();
      setLeverPosition(g, 0);
    });
  }
  // the status bar makes room for the dock
  document.documentElement.style.setProperty('--dock-w', gs.length ? `${leverDock.offsetWidth + 12}px` : '0px');
  updateLeverFades();
}
leverScroll.addEventListener('scroll', updateLeverFades);
window.addEventListener('resize', () => renderLevers());
// a mouse wheel scrolls the dock sideways
leverScroll.addEventListener(
  'wheel',
  (e) => {
    if (Math.abs(e.deltaY) <= Math.abs(e.deltaX)) return;
    leverScroll.scrollLeft += e.deltaY;
    e.preventDefault();
  },
  { passive: false },
);

/** The inspector for a propeller: its shape, mass, aerodynamics, motor and joins. */
function inspectorPropeller(g: DGear, disabled: string) {
  const P = g.propeller!;
  const lab = gearLabel(g.id);
  const aero = propellerAero(P);
  const m = g.motor;
  const joined = joinsOf(design).some((j) => [j.a, j.b].some((r) => r.kind !== 'link' && r.id === g.id));
  const num = (id: string, label: string, value: number, step: string, title = '', dis = disabled) =>
    `<label class="field" ${title ? `title="${title}"` : ''}><span>${label}</span><input id="${id}" type="number" step="${step}" value="${+value.toPrecision(4)}" ${dis}></label>`;
  inspectorEl.innerHTML = `
    <h4>Propeller P${lab} <span class="muted">(centre ${pointLabel(g.center)})</span></h4>
    ${num('i-r', 'Radius (m)', g.radius, '0.01')}
    <label class="field">Blades<input id="i-pb" type="number" min="2" max="6" step="1" value="${P.blades}" ${disabled}></label>
    ${num('i-m', 'Mass (kg)', g.mass, '0.01')}
    <label class="field">Blade line (°)<input id="i-pa" type="number" step="5" value="${+((P.angle * 180) / Math.PI).toFixed(2)}" ${disabled}></label>
    <label class="field">Pitch<span><input id="i-ph" type="checkbox" ${aero.hand < 0 ? 'checked' : ''} ${disabled}> reversed (forward spin pushes the other way)</span></label>
    <label class="field">Braked<span><input id="i-lock" type="checkbox" ${g.locked ? 'checked' : ''} ${disabled}> blades do not spin</span></label>
    <details><summary class="muted">Aerodynamics (C<sub>T</sub> ${aero.ct}, C<sub>Q</sub> ${aero.cq}, ρ ${aero.rho})</summary>
      ${num('i-ct', 'C<sub>T</sub>', aero.ct, '0.01', 'Static thrust coefficient: T = ρ D⁴ C_T n²')}
      ${num('i-cq', 'C<sub>Q</sub>', aero.cq, '0.001', 'Torque coefficient: Q = ρ D⁵ C_Q n² (power coefficient C_P = 2π C_Q)')}
      ${num('i-j0', 'J<sub>0</sub>', aero.j0, '0.05', 'Advance ratio V/(nD) at which the thrust falls to zero (about pitch / diameter)')}
      ${num('i-ch', 'C<sub>H</sub>', aero.ch, '0.01', 'Rotor drag (H-force) coefficient: H = −ρ D³ C_H |n| v, across the axis')}
      ${num('i-rho', 'Air ρ (kg/m³)', aero.rho, '0.05')}
    </details>
    <label class="field">Motor<span><input id="i-motor" type="checkbox" ${m ? 'checked' : ''} ${disabled}> spins it</span></label>
    ${
      m
        ? `${
            m.pid
              ? `<div class="field"><span>Torque</span><span>set by its <button id="i-gopid" style="color:${m.pid.color}">PID controller</button></span></div><label class="field">Max torque (N·m)<input id="i-torque" type="number" step="0.1" value="${Math.abs(m.torque)}"></label>`
              : m.lever
              ? `<div class="field"><span>Torque</span><span>set by <button id="i-golever" style="color:${m.lever.color}">lever L${lab}</button>: ${motorTorque(m).toFixed(2)} N·m</span></div>`
              : `<label class="field">Torque (N·m)<input id="i-torque" type="number" step="0.1" value="${m.torque}"></label>`
          }
        <label class="field">No-load speed (rpm)<input id="i-rpm" type="number" min="0" step="100" value="${m.noLoadRpm}"></label>
        <label class="field">Motor mass (kg)<input id="i-hmass" type="number" min="0.001" step="0.05" value="${m.mass ?? 0.5}" ${disabled}></label>
        <label class="field">Hub<span><input id="i-mounted" type="checkbox" ${m.mounted !== false ? 'checked' : ''} ${disabled || joined ? 'disabled' : ''}> mounted on the ground</span></label>`
        : ''
    }
    ${joinsHtml({ kind: 'gear', id: g.id }, disabled)}
    <p class="muted">Its blades spin by φ about an axis in the plane, square to the blade line, and push along it (the arrow). It touches nothing. ${
      m ? `The motor spins the blades against the hub it is part of; ${joined ? 'joined, the hub is carried by the bodies it is joined to' : m.mounted !== false ? 'the hub is mounted on the ground' : 'the hub is free'}. ` : 'Without a motor it is a free body (x, y, θ, φ); join it to carry it on another body. '
    }Air: thrust ρD⁴C<sub>T</sub>n² (less as it moves along its axis), torque ρD⁵C<sub>Q</sub>n² against the spin, and rotor drag across the axis.</p>`;
  bindJoinControls({ kind: 'gear', id: g.id });
  bindField('#i-r', (v) => (g.radius = Math.max(0.03, Number(v) || 0.03)));
  bindField('#i-pb', (v) => (P.blades = Math.min(6, Math.max(2, Math.round(Number(v) || 2)))));
  bindField('#i-m', (v) => (g.mass = Math.max(0.001, Number(v) || 0.001)));
  bindField('#i-pa', (v) => (P.angle = (((Number(v) || 0) * Math.PI) / 180)));
  const coeff = (sel: string, key: 'ct' | 'cq' | 'j0' | 'ch' | 'rho', min: number) =>
    bindField(sel, (v) => {
      const x = Number(v);
      if (Number.isFinite(x)) P[key] = Math.max(min, x);
    });
  coeff('#i-ct', 'ct', 0);
  coeff('#i-cq', 'cq', 0);
  coeff('#i-j0', 'j0', 0.05);
  coeff('#i-ch', 'ch', 0);
  coeff('#i-rho', 'rho', 0);
  inspectorEl.querySelector<HTMLInputElement>('#i-ph')!.addEventListener('change', (e) => {
    pushUndo();
    P.hand = (e.target as HTMLInputElement).checked ? -1 : 1;
    designChanged({ keepInspector: true });
  });
  inspectorEl.querySelector<HTMLInputElement>('#i-lock')!.addEventListener('change', (e) => {
    pushUndo();
    g.locked = (e.target as HTMLInputElement).checked;
    designChanged({ keepInspector: true });
  });
  inspectorEl.querySelector<HTMLInputElement>('#i-motor')!.addEventListener('change', (e) => {
    pushUndo();
    sfx((e.target as HTMLInputElement).checked ? 'motorOn' : 'motorOff');
    if ((e.target as HTMLInputElement).checked) {
      g.motor = propellerMotor(design.points.find((p) => p.id === g.center)?.kind === 'fixed');
      g.locked = false;
    } else delete g.motor;
    designChanged();
  });
  const motorField = (sel: string, apply: (mm: NonNullable<DGear['motor']>, v: number) => void) =>
    inspectorEl.querySelector<HTMLInputElement>(sel)?.addEventListener('change', (e) => {
      const v = Number((e.target as HTMLInputElement).value);
      if (!g.motor || !Number.isFinite(v)) return;
      pushUndo();
      apply(g.motor, v);
      designChanged();
      if (sim) rebuildSim();
    });
  motorField('#i-torque', (mm, v) => (mm.torque = v));
  motorField('#i-rpm', (mm, v) => (mm.noLoadRpm = Math.max(0, v)));
  motorField('#i-hmass', (mm, v) => (mm.mass = Math.max(0.001, v)));
  inspectorEl.querySelector<HTMLInputElement>('#i-mounted')?.addEventListener('change', (e) => {
    if (!g.motor) return;
    pushUndo();
    g.motor.mounted = (e.target as HTMLInputElement).checked;
    designChanged();
  });
  inspectorEl.querySelector('#i-golever')?.addEventListener('click', () => {
    selection = { kind: 'lever', id: g.id };
    renderInspector();
  });
  inspectorEl.querySelector('#i-gopid')?.addEventListener('click', () => {
    selection = { kind: 'pid', id: g.id };
    renderInspector();
  });
}

/** A propeller's spin rate φ̇ now (rad/s), or null if it cannot spin. */
function simSpinRate(id: number): number | null {
  const pr = sim?.sys.derivation.propellers.find((x) => x.id === id);
  return sim && pr && pr.spin >= 0 ? sim.qd[pr.spin] : null;
}

/** The inspector for a lever: its colour and scale, where it is set, and removing it. */
function inspectorLever(g: DGear, lever: NonNullable<NonNullable<DGear['motor']>['lever']>, disabled: string) {
  const lab = gearLabel(g.id);
  const m = g.motor!;
  if (m.pid?.lever === lever) {
    // a lever setting a PID controller's target
    const q = m.pid.sensor?.quantity ?? 'angle';
    inspectorEl.innerHTML = `
    <h4><span style="color:${lever.color}">●</span> Lever L${lab} <span class="muted">(target of the PID controller on motor M${lab})</span></h4>
    <label class="field">Colour<input id="i-lcolor" type="color" value="${lever.color}"></label>
    <label class="field"><span>Scale (${QUANTITY_UNIT[q]})</span><input id="i-lscale" type="number" min="0" step="any" value="${lever.scale}" title="How far full throw moves the target from its offset"></label>
    <div class="field"><span>Setting</span><span id="i-lpos">${leverSettingText(g)}</span></div>
    <div class="field"><span></span><span><button id="i-lzero">Centre it</button> <button id="i-gopid">Controller…</button> <button id="i-lremove" ${disabled}>Remove lever</button></span></div>
    <p class="muted">Drag its control at the bottom right to move the controller's target: the offset (set on the controller) plus the lever's position × scale. It works while it runs.</p>`;
    inspectorEl.querySelector('#i-gopid')!.addEventListener('click', () => {
      selection = { kind: 'pid', id: g.id };
      renderInspector();
    });
  } else
  inspectorEl.innerHTML = `
    <h4><span style="color:${lever.color}">●</span> Lever L${lab} <span class="muted">(motor M${lab} on ${gearName(design, g.id)})</span></h4>
    <label class="field">Colour<input id="i-lcolor" type="color" value="${lever.color}"></label>
    <label class="field">Scale (N·m)<input id="i-lscale" type="number" min="0.01" step="0.1" value="${lever.scale}" title="Torque at full throw"></label>
    <div class="field"><span>Setting</span><span id="i-lpos">${leverSettingText(g)}</span></div>
    <div class="field"><span></span><span><button id="i-lzero">Centre it</button> <button id="i-lremove" ${disabled}>Remove lever</button></span></div>
    <p class="muted">Drag its control at the bottom right: up for forward (anticlockwise), down for reverse; double-click it to centre. The scale is the torque at full throw, ±${lever.scale} N·m. ${
      m.noLoadRpm > 0
        ? `The top speed follows the lever too, up to ${m.noLoadRpm} rpm at full throw, and at the centre the motor brakes, as in a DC motor whose voltage the lever sets: τ = position × scale − (scale/Ω) ω.`
        : 'With no no-load speed set, the torque is the same at any speed.'
    } You can work it, and change its scale and colour, while it runs.</p>`;
  inspectorEl.querySelector<HTMLInputElement>('#i-lcolor')!.addEventListener('input', (e) => {
    lever.color = (e.target as HTMLInputElement).value;
    renderLevers();
    inspectorEl.querySelector<HTMLElement>('h4 span')!.style.color = lever.color;
    saveSoon();
  });
  inspectorEl.querySelector<HTMLInputElement>('#i-lscale')!.addEventListener('change', (e) => {
    const v = Number((e.target as HTMLInputElement).value);
    if (!Number.isFinite(v) || v <= 0) return;
    pushUndo();
    lever.scale = v;
    designChanged({ keepInspector: true });
    if (m.pid?.lever === lever) sim?.tuneController(g.id, { target: pidTarget(m.pid) });
    else if (sim) rebuildSim(); // b = scale / Ω changes as well
    inspectorEl.querySelector('#i-lpos')!.textContent = leverSettingText(g);
    renderLevers();
  });
  inspectorEl.querySelector('#i-lzero')!.addEventListener('click', () => {
    if (!sim) pushUndo();
    setLeverPosition(g, 0);
  });
  inspectorEl.querySelector('#i-lremove')!.addEventListener('click', () => deleteSelection({ kind: 'lever', id: g.id }));
}

function renderInspector() {
  const disabled = sim ? 'disabled' : '';
  renderLevers();
  if (selection?.kind === 'lever') {
    const g = design.gears.find((x) => x.id === selection!.id);
    const lever = leverOf(g?.motor)?.lever;
    if (g && lever) {
      inspectorLever(g, lever, disabled);
      return;
    }
    selection = null;
  }
  if (selection?.kind === 'pid') {
    const g = design.gears.find((x) => x.id === selection!.id);
    if (g?.motor?.pid) {
      inspectorPid(g, g.motor.pid, disabled);
      return;
    }
    selection = null;
  }
  if (!sim && groundSel && design.ground && !selection) {
    const g = design.ground;
    const i = groundSel.waypoint !== null && groundSel.waypoint < g.points.length ? groundSel.waypoint : null;
    inspectorEl.innerHTML = `
      <h4>Ground</h4>
      <div class="field"><span>Waypoints</span><span>${g.points.length}</span></div>
      ${
        i !== null
          ? `<label class="field">Waypoint ${i + 1}: x (m)<input id="i-gx" type="number" step="0.05" value="${g.points[i][0]}"></label>
      <label class="field">Height (m)<input id="i-gy" type="number" step="0.01" value="${g.points[i][1]}"></label>
      <div class="field"><span></span><span><button id="i-gdel" ${g.points.length <= 2 ? 'disabled' : ''}>Remove waypoint</button></span></div>`
          : ''
      }
      <p class="muted">Drag a waypoint up or down. It stops where the ground would touch the machine, and the ground drops under anything you move below it. Double-click the ground to add a waypoint; Delete removes the selected one. Restitution and friction are in the settings on the left.</p>`;
    if (i !== null) {
      const commit = () => {
        groundChanged();
        renderInspector();
      };
      inspectorEl.querySelector<HTMLInputElement>('#i-gx')!.addEventListener('change', (e) => {
        const v = Number((e.target as HTMLInputElement).value);
        if (!Number.isFinite(v)) return;
        pushUndo();
        // stays between its neighbours, so the waypoints keep their order
        const lo = i > 0 ? g.points[i - 1][0] + 1e-3 : -Infinity;
        const hi = i < g.points.length - 1 ? g.points[i + 1][0] - 1e-3 : Infinity;
        g.points[i][0] = Math.min(hi, Math.max(lo, v));
        keepGroundUnder();
        commit();
      });
      inspectorEl.querySelector<HTMLInputElement>('#i-gy')!.addEventListener('change', (e) => {
        const v = Number((e.target as HTMLInputElement).value);
        if (!Number.isFinite(v)) return;
        pushUndo();
        setWaypointHeight(i, v);
        commit();
      });
      inspectorEl.querySelector<HTMLButtonElement>('#i-gdel')!.addEventListener('click', () => {
        if (g.points.length <= 2) return;
        pushUndo();
        g.points.splice(i, 1);
        groundSel = { waypoint: null };
        commit();
      });
    }
    return;
  }
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
      const rail = railOf(p);
      const fold = (a: number) => +((((a % 180) + 180) % 180).toFixed(2));
      const what =
        p.kind === 'fixed'
          ? 'pinned to the ground'
          : p.gear !== undefined
            ? `pinned on ${gearName(design, p.gear)}`
            : p.pin
              ? `pinned on ${bodyName(design, p.pin)}`
              : rail
                ? `slides on a rail on ${rail.on ? bodyName(design, rail.on) : 'the ground'}`
                : 'free';
      inspectorEl.innerHTML = `
        <h4>Joint ${pointLabel(p.id)}</h4>
        <div class="field"><span>Constraint</span><span>${what}${what !== 'free' ? ` <button id="i-free" ${disabled}>Free</button>` : ''}</span></div>
        ${rail ? `<label class="field">Rail angle (°)<input id="i-rail" type="number" step="5" value="${fold(rail.angle)}" ${disabled}></label>` : ''}
        ${
          rail
            ? `<label class="field">Range from (m)<input id="i-rmin" type="number" step="0.05" max="0" placeholder="no stop" value="${p.slide?.range?.min ?? ''}" ${disabled}></label>
        <label class="field">Range to (m)<input id="i-rmax" type="number" step="0.05" min="0" placeholder="no stop" value="${p.slide?.range?.max ?? ''}" ${disabled}></label>
        ${p.slide?.range ? `<label class="field">Stop restitution<input id="i-rest-stop" type="number" min="0" max="1" step="0.05" value="${p.slide.range.restitution}" ${disabled}></label>` : ''}
        <p class="muted">Travel along the rail from where the joint was drawn; past an end it hits a stop, as it would the ground. Leave blank for no stop.</p>`
            : ''
        }
        ${what === 'free' ? '<p class="muted">Use <b>Pin</b> (F) to pin it to the ground or onto a body, or <b>Slider</b> (X) to put it on a rail.</p>' : ''}
        <label class="field">Point mass (kg)<input id="i-mass" type="number" min="0" step="0.1" value="${p.mass}" ${disabled}></label>
        <label class="field">x (m)<input id="i-x" type="number" step="0.1" value="${+p.x.toFixed(4)}" ${disabled}></label>
        <label class="field">y (m)<input id="i-y" type="number" step="0.1" value="${+p.y.toFixed(4)}" ${disabled}></label>
        `;
      inspectorEl.querySelector('#i-free')?.addEventListener('click', () => freeJoint(p.id));
      const setRange = (key: 'min' | 'max' | 'restitution', raw: string) => {
        const r = railOf(p);
        if (!r) return;
        p.kind = 'free';
        p.slide = { ...r };
        const range = p.slide.range ?? { min: null, max: null, restitution: 0.2 };
        const v = raw.trim() === '' ? null : Number(raw);
        if (key === 'restitution') range.restitution = Math.min(1, Math.max(0, v ?? 0.2));
        else if (v === null || Number.isFinite(v)) range[key] = v === null ? null : key === 'min' ? Math.min(0, v) : Math.max(0, v);
        if (range.min === null && range.max === null) delete p.slide.range;
        else p.slide.range = range;
      };
      bindField('#i-rmin', (v) => setRange('min', v));
      bindField('#i-rmax', (v) => setRange('max', v));
      bindField('#i-rest-stop', (v) => setRange('restitution', v));
      bindField('#i-rail', (v) => {
        const r = railOf(p);
        if (!r || !Number.isFinite(Number(v))) return;
        p.kind = 'free'; // a lockX / lockY rail becomes an ordinary ground rail at the new angle
        p.slide = { ...r, angle: Number(v) };
      });
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
      const candidates = design.gears.filter((g) => !g.wheel && !meshed.some((m) => m.gear === g.id));
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
    if (g?.propeller) {
      inspectorPropeller(g, disabled);
      return;
    }
    if (g) {
      const label = (id: number) => `g${gearLabel(id)}`;
      const meshed = design.meshes.filter((m) => m.a === g.id || m.b === g.id);
      const partner = (m: { a: number; b: number }) => (m.a === g.id ? m.b : m.a);
      const wheel = !!g.wheel;
      const candidates = wheel ? [] : design.gears.filter((x) => x.id !== g.id && !x.wheel && !meshed.some((m) => partner(m) === x.id));
      const racked = design.rackMeshes.filter((m) => m.gear === g.id);
      const rackCandidates = wheel ? [] : design.links.filter((l) => l.rack && !racked.some((m) => m.rack === l.id));
      const I = 0.5 * g.mass * g.radius * g.radius;
      inspectorEl.innerHTML = `
        <h4>${wheel ? `Wheel w${gearLabel(g.id)}` : `Gear ${label(g.id)}`} <span class="muted">(centre ${pointLabel(g.center)})</span></h4>
        <label class="field">Radius (m)<input id="i-r" type="number" min="0.03" step="0.01" value="${g.radius}" ${disabled}></label>
        ${wheel ? '' : `<label class="field">Teeth<input id="i-n" type="number" min="6" max="300" step="1" value="${g.teeth}" ${disabled}></label>`}
        <label class="field">Mass (kg)<input id="i-m" type="number" min="0.01" step="0.1" value="${g.mass}" ${disabled}></label>
        <label class="field">Locked<span><input id="i-lock" type="checkbox" ${g.locked ? 'checked' : ''} ${disabled}> ${wheel ? 'braked: does not turn' : 'does not turn'}</span></label>
        <label class="field">Motor<span><input id="i-motor" type="checkbox" ${g.motor ? 'checked' : ''} ${disabled}> drives it</span></label>
        ${
          g.motor
            ? `${
                g.motor.pid
                  ? `<div class="field"><span>Torque</span><span>set by its <button id="i-gopid" style="color:${g.motor.pid.color}">PID controller</button></span></div><label class="field">Max torque (N·m)<input id="i-torque" type="number" step="0.1" value="${Math.abs(g.motor.torque)}"></label>`
                  : g.motor.lever
                  ? `<div class="field"><span>Torque</span><span>set by <button id="i-golever" style="color:${g.motor.lever.color}">lever L${gearLabel(g.id)}</button>: ${motorTorque(g.motor).toFixed(2)} N·m</span></div>`
                  : `<label class="field">Torque (N·m)<input id="i-torque" type="number" step="0.1" value="${g.motor.torque}"></label>`
              }
        <label class="field">No-load speed (rpm)<input id="i-rpm" type="number" min="0" step="10" value="${g.motor.noLoadRpm}"></label>
        <label class="field">Housing mass (kg)<input id="i-hmass" type="number" min="0.01" step="0.1" value="${g.motor.mass ?? 0.5}" ${disabled}></label>
        ${(() => {
          const joined = joinsOf(design).some((j) => sameBody(j.a, { kind: 'motor', id: g.id }) || sameBody(j.b, { kind: 'motor', id: g.id }));
          return `<label class="field">Housing<span><input id="i-mounted" type="checkbox" ${g.motor.mounted !== false ? 'checked' : ''} ${disabled || joined ? 'disabled' : ''}> mounted on the ground</span></label>
        ${joinsHtml({ kind: 'motor', id: g.id }, disabled, 'm')}
        <p class="muted">The motor turns the gear by τ and its own housing back by −τ. ${joined ? 'Its housing is joined to other bodies, so they take the reaction.' : g.motor.mounted !== false ? 'Its housing is mounted on the ground, which takes the reaction.' : 'Its housing is free, so it spins back the other way; join it to a frame to hold it.'}</p>`;
        })()}
        <p class="muted">${g.motor.noLoadRpm > 0 ? `Torque τ − (|τ|/Ω)ω: ${g.motor.torque} N·m at rest, falling to 0 at ${g.motor.noLoadRpm} rpm turning the way it pushes, like a DC motor.` : 'A constant torque, whatever the speed.'} Positive turns the gear anticlockwise. You can change these while it runs.</p>`
            : ''
        }
        ${
          wheel
            ? ''
            : `<div class="field"><span>Meshes with</span><span>${
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
        }`
        }
        ${joinsHtml({ kind: 'gear', id: g.id }, disabled)}
        <p class="muted">${wheel ? 'A wheel: no teeth, so it meshes with nothing; it rolls on the ground, its tyre gripping with the ground friction. ' : ''}Uniform disk, I = ½mR² = ${I.toFixed(4)} kg·m². Bars drawn on it are pinned to it.</p>`;
      bindJoinControls({ kind: 'gear', id: g.id });
      bindField('#i-r', (v) => (g.radius = Math.max(0.03, Number(v) || 0.03)));
      if (!wheel) bindField('#i-n', (v) => (g.teeth = Math.min(300, Math.max(6, Math.round(Number(v) || 6)))));
      bindField('#i-m', (v) => (g.mass = Math.max(0.01, Number(v) || 0.01)));
      inspectorEl.querySelector<HTMLInputElement>('#i-lock')!.addEventListener('change', (e) => {
        pushUndo();
        g.locked = (e.target as HTMLInputElement).checked;
        designChanged({ keepInspector: true });
      });
      inspectorEl.querySelector<HTMLInputElement>('#i-motor')!.addEventListener('change', (e) => {
        pushUndo();
        sfx((e.target as HTMLInputElement).checked ? 'motorOn' : 'motorOff');
        if ((e.target as HTMLInputElement).checked) {
          g.motor = { torque: 1, noLoadRpm: 60, mass: 0.5, mounted: design.points.find((p) => p.id === g.center)?.kind === 'fixed' };
          g.locked = false;
        } else delete g.motor;
        removeOrphans(); // joins to its housing go with it
        designChanged();
      });
      // torque and speed can change while it runs: only parameters change, so the state carries over
      const motorField = (sel: string, apply: (m: NonNullable<DGear['motor']>, v: number) => void) =>
        inspectorEl.querySelector<HTMLInputElement>(sel)?.addEventListener('change', (e) => {
          const v = Number((e.target as HTMLInputElement).value);
          if (!g.motor || !Number.isFinite(v)) return;
          pushUndo();
          apply(g.motor, v);
          designChanged();
          if (sim) rebuildSim();
        });
      motorField('#i-torque', (m, v) => (m.torque = v));
      motorField('#i-rpm', (m, v) => (m.noLoadRpm = Math.max(0, v)));
      motorField('#i-hmass', (m, v) => (m.mass = Math.max(0.01, v)));
      inspectorEl.querySelector<HTMLInputElement>('#i-mounted')?.addEventListener('change', (e) => {
        if (!g.motor) return;
        pushUndo();
        g.motor.mounted = (e.target as HTMLInputElement).checked;
        designChanged();
      });
      if (g.motor) bindJoinControls({ kind: 'motor', id: g.id }, 'm');
      inspectorEl.querySelector('#i-golever')?.addEventListener('click', () => {
        selection = { kind: 'lever', id: g.id };
        renderInspector();
      });
      inspectorEl.querySelector('#i-gopid')?.addEventListener('click', () => {
        selection = { kind: 'pid', id: g.id };
        renderInspector();
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
  const motors = s.sys.derivation.motors.length > 0;
  const air = s.sys.derivation.propellers.length > 0;
  const heat = friction.enabled || s.dissipated > 0;
  return (
    (heat ? ` · heat W = <b>${s.dissipated.toFixed(3)} J</b>` : '') +
    (motors ? ` · motor work W<sub>m</sub> = <b>${s.motorWork.toFixed(3)} J</b>` : '') +
    (air ? ` · air work W<sub>a</sub> = <b>${s.airWork.toFixed(3)} J</b>` : '') +
    ` · E${heat ? ' + W' : ''}${motors ? ' − W<sub>m</sub>' : ''}${air ? ' − W<sub>a</sub>' : ''} = ${(T + V + s.dissipated - s.motorWork - s.airWork).toFixed(4)} J` +
    (hottest > 0.01 ? ` · hottest: ${where} +${hottest.toFixed(1)} K` : '')
  );
}

function renderStatus() {
  const nBars = design.links.filter((l) => !l.rack).length;
  const nRacks = design.links.length - nBars;
  const nSprings = design.springs.length;
  const nWheels = design.gears.filter((g) => g.wheel).length;
  const nProps = design.gears.filter((g) => g.propeller).length;
  const nGears = design.gears.length - nWheels - nProps;
  if (simError) {
    statusEl.innerHTML = `<span class="err">${simError}</span>`;
    return;
  }
  if (!sim) {
    const dof = derivation ? derivation.coords.length - derivation.constraints.length : 0;
    statusEl.innerHTML = `Editing · <b>${nBars}</b> bar${nBars === 1 ? '' : 's'}${nRacks ? ` · <b>${nRacks}</b> rack${nRacks === 1 ? '' : 's'}` : ''}${nSprings ? ` · <b>${nSprings}</b> spring${nSprings === 1 ? '' : 's'}` : ''}${nGears ? ` · <b>${nGears}</b> gear${nGears === 1 ? '' : 's'}` : ''}${nWheels ? ` · <b>${nWheels}</b> wheel${nWheels === 1 ? '' : 's'}` : ''}${nProps ? ` · <b>${nProps}</b> propeller${nProps === 1 ? '' : 's'}` : ''} · <b>${dof}</b> degree${dof === 1 ? '' : 's'} of freedom`;
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
    (friction.enabled || sim.dissipated > 0 || sim.sys.derivation.motors.length ? heatStatus(sim) : '') +
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
/** Fit the mechanism in view, or the given box (an example's preferred view). */
function fitView(box?: { x: [number, number]; y: [number, number] }) {
  const left = 240;
  const right = W > 900 ? W - 530 : W - 20;
  const top = 50;
  const bottom = H - 70;
  const pts = box ? ([[box.x[0], box.y[0]], [box.x[1], box.y[1]]] as Vec[]) : design.points.map((p) => [p.x, p.y] as Vec);
  // with a ground, show the ground under the machine too
  const g = design.ground;
  if (!box && g && pts.length) {
    const xs = pts.map((p) => p[0]);
    const [lo, hi] = [Math.min(...xs), Math.max(...xs)];
    pts.push([lo, groundHeight(g, lo)], [hi, groundHeight(g, hi)], ...g.points.filter((p) => p[0] > lo && p[0] < hi));
  }
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
  sound.update(sim && running ? machineState(sim) : null);
  if (selection?.kind === 'pid' && sim) {
    const live = inspectorEl.querySelector('#i-plive');
    const g = design.gears.find((x) => x.id === selection!.id);
    if (live && g?.motor?.pid) live.textContent = pidLiveText(g);
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
    motorAngle: motorAngleNow,
    spinAngle: (id: number) => (sim ? (simSpinAngles.get(id) ?? 0) : 0),
    spinRate: (id: number) => simSpinRate(id) ?? 0,
    thrust: (id: number) => (sim ? (sim.thrust.get(id) ?? 0) : undefined),
    heat: sim ? simHeat : null,
    grab: grabScene,
    simulating: !!sim,
    labels: settings.labels,
    ground: design.ground ? design.ground.points : null,
    groundSelected: !sim && design.ground ? groundSel : null,
    multi: sim ? null : liveMulti(),
    box: boxSel ? { from: boxSel.from, to: boxSel.to } : null,
    railPreview:
      !sim && constrain && constrain.body !== undefined
        ? (() => {
            const p = design.points.find((pt) => pt.id === constrain!.joint);
            if (!p) return null;
            const ends = constrain!.ends ? [...constrain!.ends, railTravel(mouseWorld)] : undefined;
            return { at: [p.x, p.y] as Vec, angle: constrain!.angle, ends };
          })()
        : null,
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
