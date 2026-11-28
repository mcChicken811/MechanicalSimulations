import 'katex/dist/katex.min.css';
import './style.css';
import { deriveEquations, deriveSteps, type Derivation } from './mechanics/derive';
import { bodyName, gearName, motorTorque, nextLeverColor, leverOf, pidTarget, measurable, quantityName, defaultGains, LEVER_TARGET_SCALE, QUANTITY_UNIT, type DSensor, type Quantity, type DPid, type DDimension, propellerAero, propellerMotor, clearConstraint, cloneDesign, motorHousingRadius, anglesToJoins, emptyDesign, groundCeiling, groundHeight, lowerGroundUnder, railOf, straightGround, type DGround, sameBody, type DBodyRef, gearLabel, linkLabel, linkLength, nextId, pointLabel, rackPitch, springLabel, teethForRadius, type Design, type DGear, type DLink, type PointKind } from './mechanics/model';
import { CONTACT_OMEGA, dampingRatio, defaultFriction, INTEGRATORS, NumericSystem, Simulator, type Integrator, type ControllerSpec, type GroundSettings, type Heat, type Loads } from './mechanics/simulate';
import { allParts, emptyParts, extractParts, fragmentCentre, partCount, partsInBox, pasteParts, type PartSet } from './mechanics/fragment';
import { enableLongEquations, eomPanelSteps } from './ui/eomPanel';
import { presets } from './ui/presets';
import { PoseEditor, type MaterialPoint } from './mechanics/initial';
import { enableScrubbing } from './ui/scrub';
import { Palette, type Command } from './ui/palette';
import { autoHide, type AutoHide } from './ui/autohide';
import { Messages } from './ui/messages';
import { Gallery, MY_PROJECTS, type GalleryItem } from './ui/gallery';
import { Backdrop, clockText, DEFAULT_ANIM, inkFor, inkOn, inkStyleOf, NO_BACKGROUND, paperOf, SCENES, type BackgroundState, type ImageLayer } from './ui/background';
import { isWeatherMode, WEATHER_MODES, WEATHERS, type WeatherKind } from './ui/weather';
import { BackgroundEditor } from './ui/bgeditor';
import { askDialog } from './ui/dialog';
import { DESIGN_FORMAT, PROJECT_FORMAT, fileName, pickJson, readFile, saveJson, type ProjectFile } from './ui/files';
import { measuresOf, StateAt, unwrapDegrees, type Measure } from './ui/measure';
import { jointForces, type ForceItem, type ForceResult, type LoadItem } from './mechanics/forces';
import { barDiagram, deflection, peak, valueAt, type BeamDiagram } from './mechanics/beam';
import { breakBars, breakPoints, transferState } from './mechanics/fracture';
import { SoundEngine, type Cue, type MachineState } from './ui/sound';
import { dimensionAt, dimNormal, drawScene, PAGE_COLOR, gridStep, stressColor, leverGeometry, pidGeometry, segmentDistance, toScreen, toWorld, type Scene, type Selection, type Vec, type View } from './ui/render';

type Tool = 'select' | 'initial' | 'dimension' | 'bar' | 'rack' | 'spring' | 'gear' | 'wheel' | 'propeller' | 'mass' | 'lockGear' | 'motor' | 'lever' | 'pid' | 'join' | 'pin' | 'slider' | 'free' | 'delete';
/** What a part being drawn will mesh with, and where. */
type MeshTarget = { gear?: number; rack?: number; at: Vec; side?: 1 | -1 };

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;
const canvas = $<HTMLCanvasElement>('#canvas');
const ctx = canvas.getContext('2d')!;
const eomEl = $<HTMLDivElement>('#eom');
const inspectorEl = $<HTMLDivElement>('#inspector');
const statusEl = $<HTMLDivElement>('#status');
/** The message bar at the top: every instruction and notice goes through it (ui/messages.ts). */
const messages = new Messages($<HTMLDivElement>('#hint'));
/** Whether the drawing has changes not saved to a file (kept across reloads): opening an example asks first. */
const UNSAVED_KEY = 'linkage-lab:unsaved';
let unsaved = localStorage.getItem(UNSAVED_KEY) === '1';
function setUnsaved(v: boolean) {
  if (unsaved === v) return;
  unsaved = v;
  localStorage.setItem(UNSAVED_KEY, v ? '1' : '0');
}
/** The inspector's sliding away (set up below; it comes back when the part it shows changes). */
let inspectorPanel: AutoHide | undefined;
let inspectedFor = '';

const STORAGE_KEY = 'linkage-lab-design';
/** How the equations are stepped: the integrator, its step (s; the adaptive one's largest) and the adaptive one's tolerance (remembered). */
const INTEGRATOR_KEY = 'linkage-lab:integrator';
const stepping: { integrator: Integrator; h: number; tolerance: number } = (() => {
  const out = { integrator: 'rk4' as Integrator, h: 1e-3, tolerance: 1e-6 };
  try {
    const v = JSON.parse(localStorage.getItem(INTEGRATOR_KEY) ?? '');
    if (v.integrator in INTEGRATORS) out.integrator = v.integrator;
    if (Number.isFinite(v.h) && v.h >= 1e-5 && v.h <= 0.05) out.h = v.h;
    if (Number.isFinite(v.tolerance) && v.tolerance > 0) out.tolerance = v.tolerance;
  } catch {
    /* the defaults */
  }
  return out;
})();
/** Simulated time owed to the clock (s): carried from frame to frame, so any step keeps real time. */
let simDebt = 0;
const GRAVITY = 9.81;
const DEFAULT_SPRING_K = 50; // N/m
const DEFAULT_MASS = 1; // kg, for the mass tool

// ---- state -----------------------------------------------------------------
let W = 0;
let H = 0;
const view: View = { scale: 170, ox: 0, oy: 0 };
let design: Design = loadDesign() ?? presets[0].build(); // first visit: the Strandbeest leg
/** The tool in use: Select at first, and what everything goes back to (Reset, an example, Esc). */
let tool: Tool = 'select';
let selection: Selection | null = null;
let hover: Selection | null = null;
let drawing: { kind: 'bar' | 'rack' | 'spring' | 'gear' | 'wheel' | 'propeller'; mesh: MeshTarget | null; fromId: number | null; from: Vec; to: Vec; toId: number | null; downAt: Vec; clickMode: boolean } | null = null;
/** A dragged joint, or (`group`) the parts chosen together, moved by the snapped mouse offset. */
/**
 * A part being moved with the Select tool: a joint (id) or the parts chosen together (group).
 * It only starts moving once the cursor is 0.3 cm (DRAG_START px) from where it was pressed
 * (`live`), and then moves continuously by the cursor's travel since then (from `at` = where
 * the joint was), or on the grid with Alt / Option.
 */
let dragging: { id: number; pushed: boolean; downSp: Vec; downW: Vec; at: Vec; live: boolean; group?: { start: Vec; applied: Vec } } | null = null;
/** How far the cursor must go before a press becomes a move: 0.3 cm on screen (CSS px, 96 per inch). */
const DRAG_START = (0.3 / 2.54) * 96;
let lastPointer: { w: Vec; sp: Vec } | null = null;
/** Parts chosen together by a selection box, Select all or a paste. */
let multi: PartSet | null = null;
/** The selection box being dragged in Select mode. */
let boxSel: { from: Vec; to: Vec; downAt: Vec } | null = null;
/** Parts copied with Ctrl/Cmd+C, as a design of their own. */
let clipboard: Design | null = null;
/** A middle- or right-button drag panning the view; a right press that does not move opens the search list. */
let panning: { last: Vec; start: Vec; right: boolean; moved: boolean } | null = null;
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
const settings = { gravity: true, labels: localStorage.getItem(LABELS_KEY) !== 'off', follow: false, damping: 0, stiffness: 80, speed: 1 };
const friction = defaultFriction();
let simHeat: Heat = { joints: new Map(), meshes: new Map(), racks: new Map() };

// ---- persistence & undo ------------------------------------------------------
/** The design kept in the browser, if any (see normaliseDesign). */
function loadDesign(): Design | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    return raw ? normaliseDesign(JSON.parse(raw)) : null;
  } catch {
    return null;
  }
}

/**
 * A design read from storage or a file, made whole: what older versions saved is brought up to
 * date, and references to parts that are not there are dropped. Null if it is not a design.
 */
function normaliseDesign(raw: unknown): Design | null {
  try {
    const d = raw as Design;
    if (!d || !Array.isArray(d.points) || !Array.isArray(d.links)) return null;
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
    if (!Array.isArray(d.joins)) d.joins = []; // saved before joins existed
    anglesToJoins(d); // the old Fix angle tool: its fixed angles become joins
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

/** While a number box is dragged: whether the drag has taken its one undo step yet (null: no drag). */
let scrubUndo: boolean | null = null;

function pushUndo() {
  if (scrubUndo === true) return; // the rest of a drag on a number box: one undo step for it all
  if (scrubUndo === false) scrubUndo = true;
  undoStack.push(cloneDesign(persistable()));
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

// ---- number boxes: drag sideways to change them (Blender style) -------------------------------
enableScrubbing(document.body, {
  begin: () => (scrubUndo = false),
  end: () => (scrubUndo = null),
});

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
/**
 * Bars broken in this run (see mechanics/fracture.ts): the design as drawn, before the first
 * break, while the running copy has the pieces. Reset brings it back.
 */
let unbroken: Design | null = null;

/**
 * The design to save and to undo to: as drawn, never with broken bars. What can be edited while
 * it runs (springs, gears and motors, the ground) is taken from the running copy.
 */
function persistable(): Design {
  return unbroken ? { ...unbroken, springs: design.springs, gears: design.gears, ground: design.ground } : design;
}

function designChanged(opts: { keepInspector?: boolean } = {}) {
  setUnsaved(true);
  keepGroundUnder(); // the ground gives way under anything moved below it
  localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable()));
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
  simDebt = 0;
  sim.damping = settings.damping;
  sim.friction = friction;
  sim.ground = groundSettings();
  applyInitialRates(sim);
  resetHistory();
  recordState();
  sim.setControllers(controllerSpecs());
  for (const c of sim.controllers)
    if (c.sign === 0) messages.notify(`${gearName(design, c.motor)}'s motor cannot move what its PID controller measures here, so the controller does nothing. Set its pushing direction to Forward or Reverse to drive it anyway.`);
  if (sim.ground && belowGround()) messages.notify('Some joints start below the ground, so it pushes them up.');
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
  if (unbroken) {
    // the bars as drawn again (keeping what was edited while it ran)
    design = persistable();
    unbroken = null;
    if (selection && !selectionExists(selection)) selection = null;
    scheduleDerive();
    renderInspector();
  }
  resetHistory();
  closeGraph();
  inspectSettings = false;
  resetTrail();
  running = false;
  grab = null;
  simError = null;
  setTool('select'); // back to editing: with the Select tool (Reset, an example, Clear)
}

function togglePlay() {
  duplicating = null;
  if (ic || !icMenu.hidden) cancelInitial();
  if (!sim) startSim();
  else running = !running;
  sfx(running ? 'play' : 'pause');
  refreshControls();
}

/** Whether a simulation's state is all numbers (not blown up). */
const finiteState = (s: Simulator) => s.q.every(Number.isFinite) && s.qd.every(Number.isFinite);

/** Re-derive with new settings while keeping the current state (coordinates are unchanged). */
function rebuildSim() {
  derive();
  if (!sim || !derivation) return;
  if (derivation.coords.length !== sim.q.length) resetHistory(); // a different mechanism: its history no longer applies
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
  const h = stepping.h;
  sim.integrator = stepping.integrator;
  sim.tolerance = stepping.tolerance;
  // whole steps of h for the time this frame stands for (and what was left over), at most 0.1 s of them
  simDebt += seconds;
  let steps = Math.floor(simDebt / h + 1e-9);
  simDebt -= steps * h;
  const most = Math.min(4000, Math.ceil(0.1 / h));
  if (steps > most) [steps, simDebt] = [most, 0]; // falling behind: let it go rather than catch up
  sim.damping = settings.damping;
  sim.grab = grab ? { ...grab, target: mouseWorld, stiffness: settings.stiffness } : null;
  for (let i = 0; i < steps; i++) {
    if (trail.id >= 0) recordTrail(sim);
    if (!sim.step(h)) {
      running = false;
      simError = 'The solver hit a singular configuration (for example a fully stretched loop). Press Reset to edit.';
      sfx('error');
      refreshControls();
      break;
    }
  }
  if (trail.id >= 0) recordTrail(sim);
  if (!sim) return;
  if (!finiteState(sim)) {
    // blown up: stop rather than carry NaN into the drawing, the forces and the sound
    running = false;
    simError = 'The simulation became unstable (its state is no longer finite). Press Reset to edit.';
    sfx('error');
    refreshControls();
    return;
  }
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
/** How far back a path is traced (s): a selected joint's trail, and a phase plot's path. */
const TRAIL_SECONDS = 20;
const trail = { id: -1, xs: new Float64Array(TRAIL_MAX), ys: new Float64Array(TRAIL_MAX), ts: new Float64Array(TRAIL_MAX), start: 0, n: 0 };

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
  trail.ts[k] = s.t;
  if (trail.n < TRAIL_MAX) trail.n++;
  else trail.start = (trail.start + 1) % TRAIL_MAX;
  // the last TRAIL_SECONDS only
  while (trail.n > 1 && trail.ts[trail.start] < s.t - TRAIL_SECONDS) {
    trail.start = (trail.start + 1) % TRAIL_MAX;
    trail.n--;
  }
}

// ---- geometry helpers ----------------------------------------------------------
function pos(id: number): Vec {
  const s = simPositions.get(id);
  if (sim && s) return s;
  const c = icPose?.positions.get(id);
  if (c) return c; // the pose being set with the Initial state tool
  const p = design.points.find((pt) => pt.id === id)!;
  return [p.x, p.y];
}

// ---- grid mode: hold Alt (Option on a Mac) to snap to the grid, which shows only then ------
let altHeld = false;
/** Whether moves and drawing snap to the grid now (Alt / Option held, while editing). */
const gridMode = () => altHeld && !sim;
/** A coordinate on the grid as shown. */
const snapValue = (v: number) => {
  const st = gridStep(view);
  return +(Math.round(v / st) * st).toFixed(6);
};
/** A length (radius, rail end) on the grid in grid mode: to half the grid's spacing, at least that. */
const snapLength = (r: number) => {
  if (!gridMode()) return r;
  const half = gridStep(view) / 2;
  return Math.max(half, Math.round(r / half) * half);
};
function snap(w: Vec): Vec {
  return gridMode() ? [snapValue(w[0]), snapValue(w[1])] : w;
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

/** The motors whose lever (its knob or stick, drawn on the housing) is under w. */
function leversAt(w: Vec): DGear[] {
  const sp = toScreen(view, w);
  return design.gears.filter((g) => {
    if (!leverOf(g.motor) || !design.points.some((p) => p.id === g.center)) return false;
    const { base, knob, radius } = leverGeometry(view, pos(g.center), motorTurnNow(g), g);
    return Math.hypot(sp[0] - knob[0], sp[1] - knob[1]) < radius + 4 || segmentDistance(sp, base, knob).d < 5;
  });
}
const leverAt = (w: Vec): DGear | null => leversAt(w)[0] ?? null;

/** The turn of what a motor's lever and PID box ride on: its housing, or a propeller's hub. */
const motorTurnNow = (g: DGear) => (g.propeller ? gearAngleNow(g.id) : motorAngleNow(g.id));

/** The motors whose PID controller box is under w. */
function pidsAt(w: Vec): DGear[] {
  const sp = toScreen(view, w);
  return design.gears.filter((g) => {
    if (!g.motor?.pid || !design.points.some((p) => p.id === g.center)) return false;
    const P = pidGeometry(view, pos(g.center), motorTurnNow(g), g);
    const [dx, dy] = [sp[0] - P.c[0], sp[1] - P.c[1]];
    const [u, v] = [dx * Math.cos(P.rot) + dy * Math.sin(P.rot), -dx * Math.sin(P.rot) + dy * Math.cos(P.rot)];
    return Math.abs(u) <= P.w / 2 + 3 && Math.abs(v) <= P.h / 2 + 3;
  });
}
const pidAt = (w: Vec): DGear | null => pidsAt(w)[0] ?? null;

/** Every part within `tol` pixels of w along a list of segments, nearest first. */
function segmentsAt(w: Vec, segments: readonly { id: number; a: number; b: number }[], tol = 9): number[] {
  return segments
    .map((l) => ({ id: l.id, d: segmentDistance(w, pos(l.a), pos(l.b)).d }))
    .filter((x) => x.d < tol / view.scale)
    .sort((a, b) => a.d - b.d)
    .map((x) => x.id);
}

/**
 * Everything selectable under w, from the top layer down (the order hitTest picks in): levers,
 * PID boxes, joints, bars and racks, springs, then propellers, gears and motor housings (which
 * select their gear), each kind nearest or smallest first; the ground last, as 'ground'.
 */
function hitAll(w: Vec): (Selection | 'ground')[] {
  const out: (Selection | 'ground')[] = [];
  const add = (s: Selection) => {
    if (!out.some((x) => x !== 'ground' && x.kind === s.kind && x.id === s.id)) out.push(s);
  };
  leversAt(w).forEach((g) => add({ kind: 'lever', id: g.id }));
  pidsAt(w).forEach((g) => add({ kind: 'pid', id: g.id }));
  design.points
    .map((p) => ({ id: p.id, d: Math.hypot(pos(p.id)[0] - w[0], pos(p.id)[1] - w[1]) }))
    .filter((x) => x.d < 11 / view.scale)
    .sort((a, b) => a.d - b.d)
    .forEach((x) => add({ kind: 'point', id: x.id }));
  segmentsAt(w, design.links).forEach((id) => add({ kind: 'link', id }));
  segmentsAt(w, design.springs).forEach((id) => add({ kind: 'spring', id }));
  [...propellersAt(w), ...gearsAt(w), ...housingsAt(w)].forEach((g) => add({ kind: 'gear', id: g.id }));
  if (design.ground && groundDistance(w) < 8) out.push('ground');
  return out;
}

/** A selectable's name, for the hint while cycling. */
function pickName(x: Selection | 'ground'): string {
  if (x === 'ground') return 'the ground';
  switch (x.kind) {
    case 'point':
      return `joint ${pointLabel(x.id)}`;
    case 'link':
      return bodyName(design, { kind: 'link', id: x.id });
    case 'spring':
      return `spring ${springLabel(x.id)}`;
    case 'gear':
      return gearName(design, x.id);
    case 'lever':
      return `lever L${gearLabel(x.id)}`;
    case 'pid':
      return `the PID controller on motor M${gearLabel(x.id)}`;
  }
}

/** Where the last select-mode click landed, and what was under it, to cycle through on clicking there again. */
let lastPick: { at: Vec; under: string } | null = null;

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
/** The gears whose pitch disk contains w, smallest first. */
function gearsAt(w: Vec): DGear[] {
  return design.gears
    .filter((g) => {
      if (g.propeller || !design.points.some((p) => p.id === g.center)) return false; // a propeller is picked by its blades (propellerAt)
      const c = pos(g.center);
      return Math.hypot(w[0] - c[0], w[1] - c[1]) < g.radius;
    })
    .sort((a, b) => a.radius - b.radius);
}
const gearAt = (w: Vec): DGear | null => gearsAt(w)[0] ?? null;

const gearAngleNow = (id: number) => (sim ? (simGearAngles.get(id) ?? 0) : (icPose?.gearAngles.get(id) ?? 0));
const motorAngleNow = (id: number) => (sim ? (simMotorAngles.get(id) ?? 0) : (icPose?.motorAngles.get(id) ?? 0));

/** The gear whose motor housing (a box from the axle out to one side) contains w. */
/**
 * The propeller under w: near its blade line (the rotor disk, edge-on), its hub, or its motor
 * below the hub.
 */
function propellersAt(w: Vec): DGear[] {
  const out: DGear[] = [];
  for (const g of design.gears) {
    if (!g.propeller || !design.points.some((p) => p.id === g.center)) continue;
    const c = pos(g.center);
    const th = g.propeller.angle + gearAngleNow(g.id);
    const [e, n]: [Vec, Vec] = [[Math.cos(th), Math.sin(th)], [-Math.sin(th), Math.cos(th)]];
    const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
    const [u, v] = [dx * e[0] + dy * e[1], dx * n[0] + dy * n[1]];
    const tol = 8 / view.scale;
    const R = g.radius;
    if ((Math.abs(u) <= R + tol && Math.abs(v) <= Math.max(0.12 * R, tol)) || (g.motor && Math.abs(u) <= 0.13 * R + tol && v <= 0 && v >= -0.5 * R - tol)) out.push(g);
  }
  return out;
}
const propellerAt = (w: Vec): DGear | null => propellersAt(w)[0] ?? null;

function housingsAt(w: Vec): DGear[] {
  return design.gears.filter((g) => {
    if (!g.motor || g.propeller || !design.points.some((p) => p.id === g.center)) return false;
    const c = pos(g.center);
    const r = motorHousingRadius(g);
    const t = -motorAngleNow(g.id);
    const [dx, dy] = [w[0] - c[0], w[1] - c[1]];
    const [u, v] = [dx * Math.cos(t) - dy * Math.sin(t), dx * Math.sin(t) + dy * Math.cos(t)];
    return u >= -0.9 * r && u <= 2.6 * r && Math.abs(v) <= 0.9 * r;
  });
}
const housingAt = (w: Vec): DGear | null => housingsAt(w)[0] ?? null;

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
  return { r: snapLength(r), mesh: null };
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
  // dimensions of parts that are gone
  if (design.dimensions) {
    const there = { point: design.points, link: design.links, spring: design.springs, gear: design.gears };
    design.dimensions = design.dimensions.filter((x) => there[x.kind].some((p) => p.id === x.id));
    if (!design.dimensions.length) delete design.dimensions;
  }
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
  design.rackMeshes = design.rackMeshes.filter((m) => design.gears.some((g) => g.id === m.gear) && design.links.some((l) => l.id === m.rack && l.rack));
  design.points = design.points.filter(
    (p) =>
      [...design.links, ...design.springs].some((l) => l.a === p.id || l.b === p.id) ||
      design.gears.some((g) => g.center === p.id) ||
      p.mass > 0, // a particle, or a weight pinned on a gear
  );
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
let groundDrag: { index: number; pushed: boolean; downSp: Vec; downW: Vec; at: number; live: boolean } | null = null;

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
  setUnsaved(true);
  localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable()));
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
  // only on bare ground: double clicks over parts are clicks cycling through what is there
  if (hitAll(w).some((x) => x !== 'ground')) return;
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

function copyParts(): boolean {
  const parts = currentParts();
  if (!parts) return false;
  clipboard = extractParts(design, parts);
  const n = clipboard.points.length + clipboard.links.length + clipboard.springs.length + clipboard.gears.length;
  messages.notify(`Copied ${n} part${n === 1 ? '' : 's'}. Press Ctrl/⌘+V to paste them at the cursor.`);
  sfx('copy');
  renderInspector();
  return true;
}

/**
 * Move what is being dragged to follow the cursor at w: nothing until the cursor is DRAG_START
 * px from where it was pressed, then continuously by the cursor's travel, or on the grid in grid
 * mode (a joint lands on a grid point; a group moves by whole grid steps, keeping grid-aligned
 * parts on the grid; a waypoint's height lands on a grid line).
 */
function applyDrags(w: Vec, sp: Vec) {
  const started = (d: { downSp: Vec; live: boolean }) => d.live || (d.live = Math.hypot(sp[0] - d.downSp[0], sp[1] - d.downSp[1]) >= DRAG_START);
  if (groundDrag && design.ground && started(groundDrag)) {
    const y0 = groundDrag.at + (w[1] - groundDrag.downW[1]);
    if (!groundDrag.pushed) {
      pushUndo();
      groundDrag.pushed = true;
    }
    setWaypointHeight(groundDrag.index, gridMode() ? snapValue(y0) : y0);
  }
  if (!dragging || !started(dragging)) return;
  const [tx, ty] = [w[0] - dragging.downW[0], w[1] - dragging.downW[1]]; // the cursor's travel
  if (dragging.group && multi) {
    const g = dragging.group;
    let off: Vec = [tx, ty];
    if (gridMode()) off = [snapValue(off[0]), snapValue(off[1])];
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
  } else if (!dragging.group) {
    const p = design.points.find((pt) => pt.id === dragging!.id);
    if (!p) return;
    const [x, y] = snap([dragging.at[0] + tx, dragging.at[1] + ty]);
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

// Alt (Option) pressed or let go: grid mode on or off, at once, for whatever is being moved
for (const ev of ['keydown', 'keyup'] as const)
  window.addEventListener(ev, (e) => {
    if (e.key !== 'Alt') return;
    if (ev === 'keydown') e.preventDefault(); // not the browser's menu bar (Windows)
    if (altHeld === (ev === 'keydown')) return;
    altHeld = ev === 'keydown';
    if (lastPointer && !sim) applyDrags(lastPointer.w, lastPointer.sp);
  });
window.addEventListener('blur', () => (altHeld = false));
// a pointer event knows too (Alt pressed while the window was elsewhere)
for (const ev of ['pointerdown', 'pointermove'] as const) window.addEventListener(ev, (e) => (altHeld = e.altKey), { capture: true });

// ---- initial state: drive a coordinate with the mouse, the rest following ----------------
/** A coordinate being set: the editor, the point that follows the mouse, which coordinate, and whether its rate. */
let ic: { ed: PoseEditor; mp: MaterialPoint; k: number; rate: boolean; label: string; readout: string } | null = null;
/** The pose shown while setting a position: joint positions, gear and housing turns. */
let icPose: { positions: Map<number, Vec>; gearAngles: Map<number, number>; motorAngles: Map<number, number> } | null = null;
let mouseScreen: Vec = [0, 0];
const icMenu = document.createElement('div');
icMenu.id = 'ic-menu';
icMenu.className = 'panel';
icMenu.hidden = true;
document.body.append(icMenu);
/** Arrows show a velocity as the motion over this long (s). */
const ARROW_T = 0.5;

let icSys: { of: Derivation; sys: NumericSystem } | null = null;
/** A numeric system for the current derivation, built once per derivation. */
function editSystem(): NumericSystem | null {
  if (!derivation || !derivation.coords.length) return null;
  if (icSys?.of !== derivation) icSys = { of: derivation, sys: new NumericSystem(derivation) };
  return icSys.sys;
}

/** The stored initial rates, in coordinate order. */
function storedRates(d: Derivation): number[] {
  return d.coords.map((c) => design.initialRates?.[c.sym] ?? 0);
}

const hasInitialRates = () => Object.values(design.initialRates ?? {}).some((v) => Math.abs(v) > 1e-12);

/** Start the simulation moving at the initial rates, made consistent with the constraints. */
function applyInitialRates(s: Simulator) {
  if (!hasInitialRates()) return;
  storedRates(s.sys.derivation).forEach((v, i) => (s.qd[i] = v));
  s.project();
}

/** A coordinate's name: its symbol (HTML) and what it measures. */
function coordName(c: Derivation['coords'][number], rate: boolean): { sym: string; what: string } {
  const base = ({ '\\theta': 'θ', '\\varphi': 'φ', '\\psi': 'ψ' } as Record<string, string>)[c.base] ?? c.base;
  const sym = `${base}${rate ? '\u0307' : ''}<sub>${c.sub}</sub>`;
  const id = Number(c.sym.replace(/^[a-z]+/, ''));
  const g = design.gears.find((x) => x.id === id);
  const what = c.sym.startsWith('th')
    ? `angle of ${bodyName(design, { kind: 'link', id })}`
    : c.sym.startsWith('ph')
      ? g?.propeller
        ? `yaw of ${gearName(design, id)}`
        : `turn of ${gearName(design, id)}`
      : c.sym.startsWith('sp')
        ? `spin of ${gearName(design, id)}`
        : c.sym.startsWith('ms')
          ? `turn of motor M${gearLabel(id)}'s housing`
          : c.sym.startsWith('s')
            ? `travel of joint ${pointLabel(id)} along its rail`
            : `${c.kind} of joint ${pointLabel(id)}`;
  return { sym, what: rate ? `rate of the ${what}` : what };
}

/** A coordinate's value (or rate) in display units. */
function coordValue(c: Derivation['coords'][number], v: number, rate: boolean): string {
  if (c.kind === 'angle') {
    if (rate && c.sym.startsWith('sp')) return `${((v * 60) / (2 * Math.PI)).toFixed(0)} rpm`;
    return `${((v * 180) / Math.PI).toFixed(1)}°${rate ? '/s' : ''}`;
  }
  return `${v.toFixed(3)} m${rate ? '/s' : ''}`;
}

/** The point of the part under w that will follow the mouse. */
function materialPointAt(w: Vec): MaterialPoint | null {
  const pt = pointAt(w);
  if (pt !== null) return { kind: 'point', id: pt };
  const l = linkAt(w);
  if (l) {
    const link = design.links.find((x) => x.id === l.id)!;
    return { kind: 'bar', a: link.a, b: link.b, s: l.s };
  }
  const g = propellerAt(w) ?? gearAt(w);
  if (!g) return null;
  const c = pos(g.center);
  let [u, v] = [w[0] - c[0], w[1] - c[1]];
  if (Math.hypot(u, v) < 0.3 * g.radius) {
    // near the centre: a point out on the rim (on a propeller's blade line), so turning moves it
    const a = g.propeller?.angle ?? 0;
    [u, v] = [0.8 * g.radius * Math.cos(a), 0.8 * g.radius * Math.sin(a)];
  }
  return { kind: 'gear', gear: g.id, center: g.center, u, v };
}

/** The Initial state tool's click: the part's coordinates and rates, in a menu beside the mouse. */
function pickInitial(w: Vec, sp: Vec) {
  const mp = materialPointAt(w);
  if (!mp) return;
  derive(); // up to date with the drawing
  const sys = editSystem();
  if (!sys || !derivation) {
    messages.notify(deriveError ?? 'Nothing can move yet, so there is no initial state to set.');
    return;
  }
  const d = derivation;
  const ed = new PoseEditor(sys, d.coords.map((c) => c.initial), storedRates(d));
  const ks = ed.related(mp).filter((k) => !d.coords[k].sym.startsWith('ms'));
  // a propeller's spin moves no point in the plane, but its rate matters: offer it too
  if (mp.kind === 'gear') {
    const k = d.coords.findIndex((c) => c.sym === `sp${mp.gear}`);
    if (k >= 0 && !ks.includes(k)) ks.push(k);
  }
  if (!ks.length) {
    messages.notify('That part cannot move, so it has no initial state to set.');
    return;
  }
  const row = (k: number, rate: boolean) => {
    const c = d.coords[k];
    const { sym, what } = coordName(c, rate);
    return `<button data-k="${k}" data-rate="${rate ? 1 : 0}"><b>${sym}</b> ${esc(what)}<span>${coordValue(c, rate ? ed.qd[k] : ed.q[k], rate)}</span></button>`;
  };
  icMenu.innerHTML = `<div class="sm-title">Set the initial state of:</div>
    ${ks.filter((k) => !d.coords[k].sym.startsWith('sp')).map((k) => row(k, false)).join('')}
    <div class="sm-sep">or its velocity:</div>
    ${ks.map((k) => row(k, true)).join('')}
    ${hasInitialRates() ? '<button class="sm-clear">Clear all initial velocities</button>' : ''}
    <button class="sm-cancel">Cancel</button>`;
  icMenu.hidden = false;
  const r = icMenu.getBoundingClientRect();
  icMenu.style.left = `${Math.min(sp[0] + 12, window.innerWidth - r.width - 8)}px`;
  icMenu.style.top = `${Math.min(sp[1] + 12, window.innerHeight - r.height - 8)}px`;
  icMenu.querySelectorAll<HTMLButtonElement>('[data-k]').forEach((b) =>
    b.addEventListener('click', () => {
      const k = Number(b.dataset.k);
      const rate = b.dataset.rate === '1';
      const { sym } = coordName(d.coords[k], rate);
      icMenu.hidden = true;
      ic = { ed, mp, k, rate, label: sym.replace(/<\/?sub>/g, ''), readout: '' };
      sfx('select');
      if (rate) chaseInitial(mouseWorld);
      else {
        ed.startDrag(k, mp);
        showPose(ed);
      }
      refreshControls();
    }),
  );
  icMenu.querySelector('.sm-clear')?.addEventListener('click', () => {
    icMenu.hidden = true;
    pushUndo();
    delete design.initialRates;
    sfx('delete');
    designChanged();
  });
  icMenu.querySelector('.sm-cancel')!.addEventListener('click', () => (icMenu.hidden = true));
}

/** Setting a rate: it follows the mouse directly. */
function chaseInitial(w: Vec) {
  if (!ic?.rate) return;
  const { ed, mp, k } = ic;
  ic.readout = `${ic.label} = ${coordValue(ed.sys.derivation.coords[k], ed.chaseRate(k, mp, w, ARROW_T), true)}`;
}

/**
 * Setting a position, every frame: the clicked point is pulled towards the cursor through the
 * equations of motion (the chosen coordinate and those tied to it move, the rest held).
 */
function pullInitial(dt: number) {
  if (!ic || ic.rate) return;
  const { ed, k } = ic;
  if (!ed.dragStep(mouseWorld, dt)) {
    messages.notify('The mechanism locked up there; settle it or press Esc.');
    return;
  }
  ic.readout = `${ic.label} = ${coordValue(ed.sys.derivation.coords[k], ed.q[k], false)}`;
  showPose(ed);
}

/** Show the editor's pose: joint positions, gear and housing turns. */
function showPose(ed: PoseEditor) {
  const n = ed.sys.pointIndex.size;
  const p = new Float64Array(2 * n);
  ed.sys.positions(ed.q, p);
  const ga = new Float64Array(ed.sys.gearIds.length);
  ed.sys.gearAngles(ed.q, ga);
  const ma = new Float64Array(ed.sys.derivation.motorAngles.length);
  ed.sys.motorAngles(ed.q, ma);
  icPose = {
    positions: new Map([...ed.sys.pointIndex].map(([id, i]) => [id, [p[2 * i], p[2 * i + 1]] as Vec])),
    gearAngles: new Map(ed.sys.gearIds.map((id, i) => [id, ga[i]])),
    motorAngles: new Map(ed.sys.derivation.motorAngles.map((m, i) => [m.id, ma[i]])),
  };
}

/**
 * Settle: a new pose becomes the drawing (every bar keeping its length, so the geometry is the
 * same), and the rates are stored for the next run.
 */
function settleInitial() {
  if (!ic) return;
  const { ed, rate } = ic;
  ic = null;
  pushUndo();
  if (!rate) {
    const sys = ed.sys;
    const d = sys.derivation;
    // slider ranges are measured from where the joint is drawn: shift them by its travel
    if (d.sliderStops.length) {
      const buf = new Float64Array(d.sliderStops.length * (sys.n + 1));
      sys.stops(ed.q, buf);
      d.sliderStops.forEach((st, k) => {
        const range = design.points.find((p) => p.id === st.joint)?.slide?.range;
        const s = buf[k * (sys.n + 1)];
        if (!range) return;
        if (range.min !== null) range.min = +(range.min - s).toFixed(6);
        if (range.max !== null) range.max = +(range.max - s).toFixed(6);
      });
    }
    // a propeller's drawn blade line turns with its hub
    for (const g of design.gears) if (g.propeller) g.propeller.angle = +(g.propeller.angle + (icPose?.gearAngles.get(g.id) ?? 0)).toFixed(6);
    for (const [id, [x, y]] of icPose?.positions ?? []) {
      const p = design.points.find((pt) => pt.id === id);
      if (p) [p.x, p.y] = [+x.toFixed(6), +y.toFixed(6)];
    }
  }
  if (!rate) ed.projectRates(-1); // the stored rates, consistent with the new pose
  const rates: Record<string, number> = {};
  ed.sys.derivation.coords.forEach((c, i) => {
    if (Math.abs(ed.qd[i]) > 1e-9) rates[c.sym] = +ed.qd[i].toPrecision(8);
  });
  if (Object.keys(rates).length) design.initialRates = rates;
  else delete design.initialRates;
  icPose = null;
  sfx('place');
  designChanged();
  refreshControls();
}

function cancelInitial() {
  ic = null;
  icPose = null;
  icMenu.hidden = true;
  refreshControls();
}

let icArrowCache: { key: string; arrows: { from: Vec; to: Vec; color: string; width?: number }[] } | null = null;
/**
 * Velocity arrows: while setting a rate, the dragged point's (to the mouse) and every joint's;
 * otherwise, if initial velocities are set, every moving joint's, fainter.
 */
function initialArrows(): { from: Vec; to: Vec; color: string; width?: number }[] | null {
  const jointArrows = (ed: PoseEditor, color: string) => {
    const out: { from: Vec; to: Vec; color: string; width?: number }[] = [];
    for (const id of ed.sys.pointIndex.keys()) {
      const v = ed.velocity({ kind: 'point', id });
      const p = ed.point({ kind: 'point', id });
      if (v && p && Math.hypot(v[0], v[1]) > 1e-4) out.push({ from: p.r, to: [p.r[0] + v[0] * ARROW_T, p.r[1] + v[1] * ARROW_T], color, width: 1.8 });
    }
    return out;
  };
  if (ic?.rate) {
    const p = ic.ed.point(ic.mp);
    const v = ic.ed.velocity(ic.mp);
    const own = p && v ? [{ from: p.r, to: [p.r[0] + v[0] * ARROW_T, p.r[1] + v[1] * ARROW_T] as Vec, color: '#e07b39', width: 2.6 }] : [];
    return [...jointArrows(ic.ed, 'rgba(47,158,143,0.9)'), ...own];
  }
  if (ic) {
    // the pull: from the clicked point to the cursor
    const p = ic.ed.point(ic.mp);
    return p ? [{ from: p.r, to: mouseWorld, color: '#d64545', width: 1.6 }] : null;
  }
  if (!hasInitialRates() || !derivation) return null;
  const key = `${derivation.coords.length}:${JSON.stringify(design.initialRates)}:${design.points.length}`;
  if (icArrowCache?.key !== key || icSys?.of !== derivation) {
    const sys = editSystem();
    if (!sys) return null;
    const ed = new PoseEditor(sys, derivation.coords.map((c) => c.initial), storedRates(derivation));
    icArrowCache = { key, arrows: jointArrows(ed, 'rgba(47,158,143,0.75)') };
  }
  return icArrowCache.arrows;
}

// ---- duplicating: Ctrl/⌘+D, then click to place ----------------------------------------
/** A copy of the selected parts following the cursor as an outline, until a click places it (Esc cancels). */
/** Parts being placed (a duplicate, or an imported design, named by `label`): an outline that follows the mouse until a click. */
let duplicating: { frag: Design; centre: Vec; label?: string } | null = null;

/** Start duplicating the selected parts (or the parts chosen together). */
function startDuplicate(): boolean {
  const parts = currentParts();
  if (!parts || sim) return false;
  const frag = extractParts(design, parts);
  if (!frag.points.length) return false;
  duplicating = { frag, centre: fragmentCentre(frag) };
  drawing = null;
  constrain = null;
  sfx('copy');
  refreshControls();
  return true;
}

/** Where the duplicate would go now: its centre on the cursor, snapped like a paste. */
function duplicateOffset(): Vec {
  const c = duplicating!.centre;
  let off: Vec = [mouseWorld[0] - c[0], mouseWorld[1] - c[1]];
  if (gridMode()) off = [snapValue(off[0]), snapValue(off[1])];
  return off;
}

/** Place the duplicate where its outline is; it becomes the new selection. */
function placeDuplicate() {
  if (!duplicating) return;
  const off = duplicateOffset();
  const frag = duplicating.frag;
  duplicating = null;
  pushUndo();
  setTool('select');
  selection = null;
  multi = pasteParts(design, frag, off[0], off[1]);
  sfx('paste');
  designChanged();
  refreshControls();
}

function cancelDuplicate() {
  duplicating = null;
  refreshControls();
}

/** Paste the copied parts centred on the cursor; they become the new selection, ready to drag. */
function pasteClipboard() {
  if (!clipboard || sim) return;
  pushUndo();
  const c = fragmentCentre(clipboard);
  let off: Vec = [mouseWorld[0] - c[0], mouseWorld[1] - c[1]];
  if (gridMode()) off = [snapValue(off[0]), snapValue(off[1])];
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
  return +(gridMode() ? Math.round(t / (gridStep(view) / 2)) * (gridStep(view) / 2) : t).toFixed(4);
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
    if (ends.length === 2 && (Math.min(...ends) > 0 || Math.max(...ends) < 0)) messages.notify(`Both ends were on one side of joint ${pointLabel(p.id)}, so the nearer end is at the joint.`);
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
      messages.notify(`Joint ${label} already moves with ${bodyName(design, body)}. Pick another body, or empty space for the ground.`);
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
    panning = { last: sp, start: sp, right: e.button === 2, moved: false };
    return;
  }
  if (e.button !== 0 || palette.dismissedBy(e)) return; // a click that closed the search list does only that
  if (bgEditor.active) {
    bgEditor.pointerDown(sp); // editing the background: presses move its images, nothing else
    return;
  }
  if (ic && !sim) {
    settleInitial();
    return;
  }
  if (!icMenu.hidden && !sim) icMenu.hidden = true; // a click elsewhere closes the menu (and may pick again)
  if (duplicating && !sim) {
    placeDuplicate();
    return;
  }
  if (pidPick && !sim) {
    pickSensorAt(w, sp); // choosing what a PID controller measures, whatever the tool
    return;
  }

  if (sim && !running) {
    // paused: a part pressed is measured and grabbed, as while running (drag to set up a pull, Space
    // to let it act); a click on the part already measured lets it go (see pointerup)
    const hit = hitTest(w);
    pausePress = { at: sp, wasSelected: !!hit && hit.kind === selection?.kind && hit.id === selection?.id };
    if (hit && !pausePress.wasSelected) sfx('select');
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
      } else if (l && link && sim.sys.pointIndex.has(link.a)) {
        grab = { a: link.a, b: link.b, s: l.s };
        selection = { kind: 'link', id: link.id }; // and measure it
        renderInspector();
      }
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

  toolClick(w, sp);
});

/** What a click with the current tool does at w (screen point sp), editing; also run by the right-click list's actions on a part. */
function toolClick(w: Vec, sp: Vec) {
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
        groundDrag = { index: wp, pushed: false, downSp: sp, downW: w, at: design.ground!.points[wp][1], live: false };
        renderInspector();
        break;
      }
      // clicking again in the same place steps down through everything under it, then wraps
      const under = hitAll(w);
      const key = under.map((x) => (x === 'ground' ? 'ground' : `${x.kind}:${x.id}`)).join(',');
      const again = lastPick !== null && lastPick.under === key && Math.hypot(sp[0] - lastPick.at[0], sp[1] - lastPick.at[1]) < 6;
      lastPick = { at: sp, under: key };
      const current = groundSel && !selection ? under.indexOf('ground') : under.findIndex((x) => x !== 'ground' && x.kind === selection?.kind && x.id === selection.id);
      const choice = under.length === 0 ? null : again && current >= 0 && under.length > 1 ? under[(current + 1) % under.length] : under[0];
      if (choice && under.length > 1) messages.notify(`${pickName(choice)}: ${under.indexOf(choice) + 1} of ${under.length} here (click again in the same spot for the next).`);
      if (choice === 'ground') {
        // the ground itself: select it, showing its waypoints
        selection = null;
        multi = null;
        groundSel = { waypoint: null };
        sfx('select');
        renderInspector();
        break;
      }
      const hit = choice;
      groundSel = null;
      const group = liveMulti();
      if (hit && group && inParts(group, hit) && !again) {
        // drag the whole group
        dragging = { id: -1, pushed: false, downSp: sp, downW: w, at: w, live: false, group: { start: w, applied: [0, 0] } };
        break;
      }
      multi = null;
      if (hit && (hit.kind !== selection?.kind || hit.id !== selection.id)) sfx('select');
      selection = hit;
      if (hit?.kind === 'point') dragging = { id: hit.id, pushed: false, downSp: sp, downW: w, at: pos(hit.id), live: false };
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
    case 'initial':
      pickInitial(w, sp);
      break;
    case 'dimension':
      pressDimension(w, sp);
      break;
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
}

canvas.addEventListener('pointermove', (e) => {
  const sp = screenOf(e);
  const w = toWorld(view, sp);
  mouseWorld = w;
  mouseScreen = sp;
  if (bgEditor.active && bgEditor.pointerMove(sp)) return;
  if (panning) {
    // a right press is a click (the search list) until it moves a few pixels: then it pans
    if (!panning.moved) {
      if (Math.hypot(sp[0] - panning.start[0], sp[1] - panning.start[1]) < 4) return;
      panning.moved = true;
      palette.close();
    }
    view.ox += sp[0] - panning.last[0];
    view.oy += sp[1] - panning.last[1];
    panning.last = sp;
    return;
  }
  if (ic && !sim) chaseInitial(w);
  if (drawing?.kind === 'propeller') {
    // the edge point: its distance is the radius, its direction the blade line (15° steps with snap on)
    const r0 = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]);
    const r = snapLength(r0);
    let a = Math.atan2(w[1] - drawing.from[1], w[0] - drawing.from[0]);
    if (gridMode()) a = Math.round(a / (Math.PI / 12)) * (Math.PI / 12);
    drawing.to = [drawing.from[0] + r * Math.cos(a), drawing.from[1] + r * Math.sin(a)];
    drawing.mesh = null;
    drawing.toId = null;
  } else if (drawing?.kind === 'gear' || drawing?.kind === 'wheel') {
    const free = Math.hypot(w[0] - drawing.from[0], w[1] - drawing.from[1]);
    const { r, mesh } = drawing.kind === 'gear' ? gearRadiusAt(drawing.from, w, drawing.fromId) : { r: snapLength(free), mesh: null };
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
  lastPointer = { w, sp };
  applyDrags(w, sp);
  dragDimension(w, sp);
  if (constrain && constrain.body !== undefined && !constrain.ends) constrain.angle = railAngle(constrain.joint, w, constrain.body);
  hover = dragging || grab ? hover : hitTest(w);
  canvas.style.cursor = duplicating ? 'copy' : !sim && tool === 'dimension' && (dimPress?.live || (dimensionDigits() !== null && dimensionAt(view, dimScene(), sp) !== null)) ? 'move' : sim ? (grab ? 'grabbing' : hover?.kind === 'spring' ? 'pointer' : hover ? 'grab' : 'default') : tool === 'bar' || tool === 'rack' || tool === 'spring' || tool === 'gear' || tool === 'wheel' || tool === 'propeller' ? 'crosshair' : hover ? 'pointer' : 'default';
});

canvas.addEventListener('pointerup', (e) => {
  const sp = screenOf(e);
  if (bgEditor.active) bgEditor.pointerUp();
  releaseDimension();
  if (groundDrag) {
    const moved = groundDrag.pushed;
    groundDrag = null;
    if (moved) groundChanged();
    renderInspector();
  }
  if (panning) {
    const click = panning.right && !panning.moved;
    const at = panning.start;
    panning = null;
    if (click) openPalette(e.clientX, e.clientY, at); // right-click: the search list, here (a part's own, on a part)
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
  // paused: a click (no drag) on the part already measured lets it go
  if (pausePress) {
    const p = pausePress;
    pausePress = null;
    if (sim && p.wasSelected && Math.hypot(sp[0] - p.at[0], sp[1] - p.at[1]) < 4) {
      selection = null;
      renderInspector();
    }
  }
});

/** A press on the drawing while paused: where, and whether it was on the part already measured. */
let pausePress: { at: Vec; wasSelected: boolean } | null = null;

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
const toolKeys: Record<string, Tool> = { v: 'select', i: 'initial', u: 'dimension', b: 'bar', k: 'rack', s: 'spring', g: 'gear', w: 'wheel', p: 'propeller', m: 'mass', l: 'lockGear', o: 'motor', e: 'lever', c: 'pid', j: 'join', f: 'pin', x: 'slider', n: 'free', d: 'delete' };

window.addEventListener('keydown', (e) => {
  const target = e.target as HTMLElement;
  if (target.closest('input, select, textarea')) return;
  if (gallery.isOpen || document.querySelector('.dialog-backdrop')) return; // the gallery and dialogs have their own keys
  if (bgEditor.active) {
    if (bgEditor.key(e)) e.preventDefault(); // Esc leaves, Delete removes the chosen image
    if (e.key !== ' ') return; // (Space still plays: the parallax shows on a moving view too)
  }
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
  } else if ((e.metaKey || e.ctrlKey) && k === 'd') {
    e.preventDefault(); // not the browser's bookmark
    if (!sim && !startDuplicate()) messages.notify('Select something to duplicate first (click a part, or drag a box around several).');
  } else if ((e.metaKey || e.ctrlKey) && k === 'v' && !sim) {
    e.preventDefault();
    pasteClipboard();
  } else if ((e.metaKey || e.ctrlKey) && k === 's') {
    e.preventDefault(); // not the browser's Save Page
    void saveProject();
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
    } else messages.notify('The ground needs at least two waypoints.');
  } else if (k === 'escape') {
    // Esc cancels what is under way; with nothing under way, it goes back to the Select tool
    const underway = !!(ic || !icMenu.hidden || duplicating || pidPick || drawing || constrain || joinFirst || boxSel);
    if (!underway && tool !== 'select' && !sim) setTool('select');
    if (ic || !icMenu.hidden) cancelInitial();
    if (duplicating) cancelDuplicate();
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

// ---- Save & load: designs (the drawing) and projects (the drawing and every setting), see ui/files.ts ----
/** Side-panel controls that are part of the design (the ground) or of the beam colouring (saved with it): not saved as settings. */
const NOT_SETTINGS = new Set(['ground', 'ground-e', 'ground-mu', 'beam-scale', 'bg-kind', 'bg-opacity', 'bg-animate', 'bg-time', 'bg-day', 'bg-birds', 'bg-weather', 'bg-sound', 'bg-style', 'bg-trigger-kind', 'bg-trigger-for']);

/** The side panel's settings, by control id: ticked or not, or the value. */
function captureControls(): Record<string, boolean | string> {
  const out: Record<string, boolean | string> = {};
  document.querySelectorAll<HTMLInputElement | HTMLSelectElement>('#toolbar input[id], #toolbar select[id]').forEach((el) => {
    if (NOT_SETTINGS.has(el.id)) return;
    out[el.id] = el instanceof HTMLInputElement && el.type === 'checkbox' ? el.checked : el.value;
  });
  return out;
}

/** Set the side panel's controls as if by hand (their own handlers apply them), from captureControls. */
function applyControls(c: Record<string, boolean | string>) {
  for (const [id, v] of Object.entries(c)) {
    const el = document.getElementById(id);
    if (el instanceof HTMLSelectElement && el.closest('#toolbar') && typeof v === 'string') {
      if (![...el.options].some((o) => o.value === v)) el.append(new Option(v, v)); // a step not in the list
      if (el.value !== v) {
        el.value = v;
        el.dispatchEvent(new Event('change', { bubbles: true }));
      }
      continue;
    }
    if (!(el instanceof HTMLInputElement) || NOT_SETTINGS.has(id) || !el.closest('#toolbar')) continue;
    if (el.type === 'checkbox') {
      if (typeof v !== 'boolean' || el.checked === v) continue;
      el.checked = v;
      el.dispatchEvent(new Event('change', { bubbles: true }));
    } else {
      if (typeof v !== 'string' || el.value === v) continue;
      el.value = v;
      el.dispatchEvent(new Event('input', { bubbles: true }));
      el.dispatchEvent(new Event('change', { bubbles: true }));
    }
  }
}

/** A title for files: the example it came from, else "linkage". */
let designTitle = 'linkage';

/** Save the drawing as a design file; whether it was (false: cancelled). */
async function saveDesign(): Promise<boolean> {
  const ok = await saveJson({ format: DESIGN_FORMAT, version: 1, design: persistable() }, fileName(designTitle, 'design'), 'Linkage Lab design');
  if (ok) {
    setUnsaved(false);
    messages.notify('Design saved.');
  }
  return ok;
}

/** The project as it is now: the drawing, the side panel's settings, the beam colouring, the view. */
function buildProject(): ProjectFile {
  return { format: PROJECT_FORMAT, version: 1, savedAt: new Date().toISOString(), design: cloneDesign(persistable()), controls: captureControls(), beamColour: { ...beamColour, fixed: { ...beamColour.fixed } }, view: { ...view }, background: backgroundNow() };
}

async function saveProject(): Promise<boolean> {
  const project = buildProject();
  const ok = await saveJson(project, fileName(designTitle, 'project'), 'Linkage Lab project');
  if (ok) {
    setUnsaved(false);
    messages.notify('Project saved: the drawing, every setting and the view.');
  }
  return ok;
}

/** Ask for a file and read the design in it (a design, a project, or a bare design); null if cancelled or unreadable (said so). */
async function openDesignFile(want: 'design' | 'project'): Promise<{ design: Design; project: ProjectFile | null; name: string } | null> {
  let picked;
  try {
    picked = await pickJson();
  } catch (e) {
    messages.notify(errorText(e), 'error');
    return null;
  }
  if (!picked) return null;
  const read = readFile(picked.data);
  const d = read && normaliseDesign(read.design);
  if (!read || !d) {
    messages.notify(`${picked.name} is not a Linkage Lab ${want}.`, 'error');
    return null;
  }
  if (want === 'project' && !read.project) {
    messages.notify(`${picked.name} is a design, not a project: use Load design to open it (it has no settings).`, 'warn');
    return null;
  }
  return { design: d, project: read.project, name: picked.name.replace(/\.(design|project)?\.?json$/i, '') };
}

/** Put a design in place of the drawing (one undo step). */
function replaceDesign(d: Design, title: string) {
  stopSim();
  if (ic || !icMenu.hidden) cancelInitial();
  duplicating = null;
  pushUndo();
  design = d;
  designTitle = title;
  selection = null;
  multi = null;
  constrain = null;
  groundSel = null;
  syncGroundUI();
  designChanged();
}

async function loadDesignFile() {
  const f = await openDesignFile('design');
  if (!f) return;
  replaceDesign(f.design, f.name);
  setUnsaved(false); // as it is in the file
  fitView();
  sfx('preset');
  messages.notify(`Loaded ${f.name}${f.project ? ' (the drawing from that project; its settings are left as they are)' : ''}.`);
}

async function loadProjectFile() {
  const f = await openDesignFile('project');
  if (!f || !f.project) return;
  applyProject(f.project, f.design, f.name);
  messages.notify(`Loaded project ${f.name}: its drawing, settings and view.`);
}

/** Put a project in place of everything: its drawing, its settings, its beam colouring and its view. */
function applyProject(project: ProjectFile, d: Design, title: string) {
  replaceDesign(d, title);
  setUnsaved(false);
  applyControls(project.controls ?? {});
  const bc = project.beamColour as Partial<BeamColour> | undefined;
  if (bc && typeof bc === 'object') {
    if (['off', 'N', 'V', 'M'].includes(bc.q as string)) beamColour.q = bc.q!;
    if (['max', 'dist'].includes(bc.mode as string)) beamColour.mode = bc.mode!;
    if (['now', 'run', 'set'].includes(bc.scale as string)) beamColour.scale = bc.scale!;
    if (bc.fixed && typeof bc.fixed === 'object') beamColour.fixed = { ...bc.fixed };
    saveBeamColour();
    showBeamColour();
  }
  const v = project.view;
  if (v && [v.ox, v.oy, v.scale].every(Number.isFinite) && v.scale > 0) Object.assign(view, { ox: v.ox, oy: v.oy, scale: v.scale });
  else fitView();
  setBackground(validBackground(project.background) ?? { ...NO_BACKGROUND }); // a project without one has none
  sfx('preset');
}

/** Add a design to the drawing: its outline follows the mouse until a click places it (as a duplicate does). */
async function importDesignFile() {
  const f = await openDesignFile('design');
  if (!f) return;
  startImport(f.design, f.name);
}

/** Add design d (named `label`) to the drawing: its outline follows the mouse until a click places it. */
function startImport(d: Design, label: string) {
  if (!d.points.length) {
    messages.notify(`${label} is empty: nothing to import.`, 'warn');
    return;
  }
  stopSim();
  if (ic || !icMenu.hidden) cancelInitial();
  setTool('select');
  selection = null;
  multi = null;
  duplicating = { frag: d, centre: fragmentCentre(d), label };
  sfx('copy');
  refreshControls();
}

$<HTMLButtonElement>('#save-design').addEventListener('click', () => void saveDesign());
$<HTMLButtonElement>('#save-project').addEventListener('click', () => void saveProject());
$<HTMLButtonElement>('#load-design').addEventListener('click', () => void loadDesignFile());
$<HTMLButtonElement>('#load-project').addEventListener('click', () => void loadProjectFile());
$<HTMLButtonElement>('#import-design').addEventListener('click', () => void importDesignFile());

// ---- the background (the Background block): a scene, or a custom one with images at distances (ui/background.ts) ----
const BACKGROUND_KEY = 'linkage-lab:background';
const backdrop = new Backdrop();
const bgKind = $<HTMLSelectElement>('#bg-kind');
const bgOpacity = $<HTMLInputElement>('#bg-opacity');
bgKind.append(new Option('None (plain)', 'none'));
for (const sc of SCENES) bgKind.append(new Option(sc.name, sc.id));
bgKind.append(new Option('Custom…', 'custom'));

/** A background read from storage or a project, made safe (older ones brought up to date): null if it is not one. */
function validBackground(raw: unknown): BackgroundState | null {
  if (!raw || typeof raw !== 'object') return null;
  const b = raw as Partial<BackgroundState> & { image?: string; mode?: string; rect?: { x: number; y: number; w: number } };
  const scene = (id: unknown) => (SCENES.some((x) => x.id === id) ? (id as string) : 'none');
  const opacity = Number.isFinite(b.opacity) ? Math.min(1, Math.max(0.05, b.opacity!)) : 0.7;
  const a = (b.anim ?? {}) as Partial<BackgroundState['anim']>;
  const anim = {
    on: typeof a.on === 'boolean' ? a.on : DEFAULT_ANIM.on,
    dayLength: Number.isFinite(a.dayLength) && a.dayLength! >= 0 ? a.dayLength! : DEFAULT_ANIM.dayLength,
    hour: Number.isFinite(a.hour) ? ((a.hour! % 24) + 24) % 24 : DEFAULT_ANIM.hour,
    birds: typeof a.birds === 'boolean' ? a.birds : DEFAULT_ANIM.birds,
    weather: isWeatherMode(a.weather) ? a.weather : DEFAULT_ANIM.weather,
    // (saved before the scenes had sounds of their own: as "weather sounds")
    sound: typeof a.sound === 'boolean' ? a.sound : typeof (a as { weatherSound?: unknown }).weatherSound === 'boolean' ? (a as { weatherSound: boolean }).weatherSound : DEFAULT_ANIM.sound,
  };
  const okLayer = (l: ImageLayer) =>
    l && typeof l.image === 'string' && l.image.startsWith('data:image/') && Number.isFinite(l.aspect) && l.aspect >= 0 && (l.depth === null || (Number.isFinite(l.depth) && l.depth >= 1)) && l.ref && [l.ref.cx, l.ref.cy, l.ref.s].every(Number.isFinite) && l.ref.s > 0 && l.rect && [l.rect.x, l.rect.y, l.rect.w].every(Number.isFinite) && l.rect.w > 0;
  // the earlier single image, placed in the drawing: a custom background with it at distance 1
  if (b.kind === 'image' && typeof b.image === 'string' && b.image.startsWith('data:image/') && b.rect)
    return { kind: 'custom', base: 'none', opacity, anim, layers: [{ id: 'img', name: 'image', image: b.image, aspect: 0, depth: 1, ref: { cx: 0, cy: 0, s: 100 }, rect: { ...b.rect } }] }; // its shape: from the image, once loaded
  const layers = Array.isArray(b.layers) ? b.layers.filter(okLayer).map((l) => ({ ...l, name: String(l.name ?? 'image'), id: String(l.id ?? Math.random()) })) : [];
  const kind = b.kind === 'custom' ? 'custom' : scene(b.kind);
  // the machine drawn in the paper's style, by paper (only those set)
  const style = b.style && typeof b.style === 'object' ? Object.fromEntries(Object.entries(b.style).filter(([k, on]) => ['blueprint', 'paper', 'none'].includes(k) && typeof on === 'boolean')) : undefined;
  return { kind, base: scene(b.base), layers, opacity, anim, ...(style && Object.keys(style).length ? { style } : {}) };
}

/** Show the background and its controls, and keep it in the browser unless `passing` (an image too big for that stays for now, and in saved projects). */
function setBackground(b: BackgroundState, passing = false) {
  backdrop.set(b);
  bgKind.value = b.kind;
  $<HTMLElement>('#bg-edit-row').hidden = b.kind !== 'custom';
  // the animation's controls, as far as the scene has them
  const sc = SCENES.find((x) => x.id === (b.kind === 'custom' ? b.base : b.kind));
  $<HTMLElement>('#bg-animate-row').hidden = !sc?.alive;
  $<HTMLInputElement>('#bg-animate').checked = b.anim.on;
  $<HTMLElement>('#bg-time-row').hidden = !sc?.cycle || !b.anim.on;
  $<HTMLElement>('#bg-day-row').hidden = !sc?.cycle || !b.anim.on;
  $<HTMLElement>('#bg-birds-row').hidden = !sc?.birds || !b.anim.on;
  $<HTMLInputElement>('#bg-birds').checked = b.anim.birds;
  $<HTMLElement>('#bg-weather-row').hidden = !sc?.weather || !b.anim.on;
  $<HTMLElement>('#bg-trigger-row').hidden = !sc?.weather || !b.anim.on;
  $<HTMLElement>('#bg-sound-row').hidden = !sc?.sounds || !b.anim.on; // (blueprint, graph paper and plain have none)
  // the machine in the paper's style: a blueprint, or pencil on graph paper or plain paper
  const ink = inkStyleOf(b);
  $<HTMLElement>('#bg-style-row').hidden = !ink;
  $<HTMLElement>('#bg-style-text').textContent = ink === 'blueprint' ? 'Draw the machine as a blueprint' : 'Draw the machine by hand (pencil)';
  $<HTMLInputElement>('#bg-style').checked = inkOn(b);
  // the beam colours' key, in the same colours as the bars
  const on = ink && inkOn(b) ? ink : null;
  $<HTMLElement>('#beam-legend .bl-bar').style.background = on === 'blueprint' ? 'linear-gradient(90deg, rgb(38,88,168), #fff)' : on === 'sketch' ? 'linear-gradient(90deg, rgb(205,205,205), rgb(18,18,18))' : '';
  $<HTMLSelectElement>('#bg-weather').value = b.anim.weather;
  $<HTMLInputElement>('#bg-sound').checked = b.anim.sound;
  const day = $<HTMLSelectElement>('#bg-day');
  if (![...day.options].some((o) => Number(o.value) === b.anim.dayLength)) day.append(new Option(`${b.anim.dayLength} s`, String(b.anim.dayLength)));
  day.value = [...day.options].find((o) => Number(o.value) === b.anim.dayLength)!.value;
  showBackgroundTime();
  bgOpacity.value = String(b.opacity);
  $<HTMLOutputElement>('#bg-opacity-out').textContent = `${Math.round(b.opacity * 100)}%`;
  if (passing) return;
  try {
    localStorage.setItem(BACKGROUND_KEY, JSON.stringify(backgroundNow()));
  } catch {
    localStorage.removeItem(BACKGROUND_KEY);
    messages.notify('The background images are too big to keep in the browser between visits: they show now, and are kept in projects you save.', 'warn');
  }
}

/** The background as it is now, to keep: with the time of day it has reached. */
function backgroundNow(): BackgroundState {
  const b = backdrop.state;
  return { ...b, anim: { ...b.anim, hour: +backdrop.hour.toFixed(3) } };
}

/** The time-of-day slider and its clock, following the day as it turns (unless being dragged). */
/** Whether the time-of-day slider is held (it follows the day only when let go). */
let timeHeld = false;
function showBackgroundTime() {
  const slider = $<HTMLInputElement>('#bg-time');
  const h = backdrop.hour;
  if (!timeHeld) slider.value = h.toFixed(2);
  const text = clockText(h);
  const out = $<HTMLOutputElement>('#bg-time-out');
  if (out.textContent !== text) out.textContent = text;
}
$<HTMLInputElement>('#bg-animate').addEventListener('change', (e) => setBackground({ ...backgroundNow(), anim: { ...backgroundNow().anim, on: (e.target as HTMLInputElement).checked } }));
$<HTMLInputElement>('#bg-birds').addEventListener('change', (e) => setBackground({ ...backgroundNow(), anim: { ...backgroundNow().anim, birds: (e.target as HTMLInputElement).checked } }));
for (const m of WEATHER_MODES) $<HTMLSelectElement>('#bg-weather').append(new Option(m.name, m.id));
$<HTMLSelectElement>('#bg-weather').addEventListener('change', (e) => setBackground({ ...backgroundNow(), anim: { ...backgroundNow().anim, weather: (e.target as HTMLSelectElement).value as BackgroundState['anim']['weather'] } }));
$<HTMLInputElement>('#bg-style').addEventListener('change', (e) => {
  const b = backgroundNow();
  const k = paperOf(b);
  if (k) setBackground({ ...b, style: { ...b.style, [k]: (e.target as HTMLInputElement).checked } });
});
$<HTMLInputElement>('#bg-sound').addEventListener('change', (e) => setBackground({ ...backgroundNow(), anim: { ...backgroundNow().anim, sound: (e.target as HTMLInputElement).checked } }));
/** The scene the background shows (a custom one's base). */
const backgroundScene = () => SCENES.find((x) => x.id === (backdrop.state.kind === 'custom' ? backdrop.state.base : backdrop.state.kind));
/** Whether the background is to be heard: a scene with sounds, animated, its sounds wanted. */
const backgroundHeard = () => backdrop.state.anim.on && backdrop.state.anim.sound && !!backgroundScene()?.sounds;
// starting a weather by hand, and lightning
const triggerKind = $<HTMLSelectElement>('#bg-trigger-kind');
const triggerFor = $<HTMLSelectElement>('#bg-trigger-for');
for (const k of WEATHERS) triggerKind.append(new Option(k.name, k.id));
triggerKind.value = 'thunder';
/** Start a weather now (for the chosen time) and say so. */
function triggerWeather(kind: WeatherKind) {
  backdrop.weather.trigger(kind, Number(triggerFor.value));
  const name = WEATHERS.find((x) => x.id === kind)!.name.toLowerCase();
  const left = backdrop.weather.triggered!.left;
  messages.notify(`Weather: ${name} ${Number.isFinite(left) ? `for ${weatherTime(left)}` : 'until stopped'}.`);
}
/** A while in words: 45 s, 2:05. */
const weatherTime = (secs: number) => (secs < 60 ? `${Math.ceil(secs)} s` : `${Math.floor(secs / 60)}:${String(Math.floor(secs % 60)).padStart(2, '0')}`);
$<HTMLButtonElement>('#bg-trigger').addEventListener('click', () => triggerWeather(triggerKind.value as WeatherKind));
$<HTMLButtonElement>('#bg-trigger-stop').addEventListener('click', () => {
  backdrop.weather.endTrigger();
  messages.notify('Weather: back to its own ways.');
});
$<HTMLButtonElement>('#bg-strike').addEventListener('click', () => backdrop.weather.strikeNow());
/** The weather started by hand, as it goes: what it is and how long it has left (every frame, while the block shows it). */
function showWeatherTrigger() {
  const t = backdrop.weather.triggered;
  const status = $<HTMLElement>('#bg-trigger-status');
  const stop = $<HTMLElement>('#bg-trigger-stop');
  if (stop.hidden !== !t) stop.hidden = !t;
  if (status.hidden !== !t) status.hidden = !t;
  if (!t) return;
  const text = `${WEATHERS.find((x) => x.id === t.kind)!.name}, started by hand: ${Number.isFinite(t.left) ? `${weatherTime(t.left)} left` : 'until stopped'}.`;
  if (status.textContent !== text) status.textContent = text;
}
// lightning: its thunder, after the time the sound takes to arrive
backdrop.weather.onThunder = (delay, loud) => {
  if (backgroundHeard() && backgroundScene()?.weather) sound.thunder(delay, loud);
};
$<HTMLSelectElement>('#bg-day').addEventListener('change', (e) => setBackground({ ...backgroundNow(), anim: { ...backgroundNow().anim, dayLength: Number((e.target as HTMLSelectElement).value) } }));
$<HTMLInputElement>('#bg-time').addEventListener('pointerdown', () => (timeHeld = true));
window.addEventListener('pointerup', () => (timeHeld = false));
$<HTMLInputElement>('#bg-time').addEventListener('input', (e) => setBackground({ ...backdrop.state, anim: { ...backdrop.state.anim, hour: Number((e.target as HTMLInputElement).value) } }, true));
$<HTMLInputElement>('#bg-time').addEventListener('change', (e) => setBackground({ ...backdrop.state, anim: { ...backdrop.state.anim, hour: Number((e.target as HTMLInputElement).value) } }));

/** Background edit mode (ui/bgeditor.ts): its panel, and the drawing's presses and keys while it is on. */
const bgEditor = new BackgroundEditor({
  view: () => view,
  size: () => [W, H],
  state: () => backdrop.state,
  apply: (b, passing) => setBackground(b, passing),
  notify: (text, kind) => messages.notify(text, kind),
  closed: () => {
    refreshControls();
    messages.notify(`Background: ${backdrop.name}.`);
  },
});

function editBackground() {
  if (ic || !icMenu.hidden) cancelInitial();
  duplicating = null;
  drawing = null;
  bgEditor.open();
  refreshControls();
}

bgKind.addEventListener('change', () => {
  const kind = bgKind.value;
  if (kind === 'custom') {
    // build on the scene that was showing
    const was = backdrop.state.kind;
    setBackground({ ...backdrop.state, kind: 'custom', base: was !== 'none' && was !== 'custom' ? was : backdrop.state.base });
    editBackground();
    return;
  }
  const scene = SCENES.find((x) => x.id === kind);
  setBackground({ ...backdrop.state, kind, opacity: scene ? scene.strength : backdrop.state.opacity });
});
bgOpacity.addEventListener('input', () => setBackground({ ...backdrop.state, opacity: Number(bgOpacity.value) }, true));
bgOpacity.addEventListener('change', () => setBackground({ ...backdrop.state, opacity: Number(bgOpacity.value) }));
$<HTMLButtonElement>('#bg-edit').addEventListener('click', editBackground);
setBackground(validBackground((() => {
  try {
    return JSON.parse(localStorage.getItem(BACKGROUND_KEY) ?? 'null');
  } catch {
    return null;
  }
})()) ?? { ...NO_BACKGROUND }, true);

// ---- the integrator and its step (the Simulation block) ----
const integratorSel = $<HTMLSelectElement>('#integrator');
const stepSel = $<HTMLSelectElement>('#step-size');
const toleranceSel = $<HTMLSelectElement>('#tolerance');
for (const [key, it] of Object.entries(INTEGRATORS)) integratorSel.append(new Option(`${it.name}`, key));
/** A step in words: 1 ms, 0.5 ms. */
const stepText = (h: number) => `${+(h * 1000).toPrecision(3)} ms`;
/** The integrator and step for the status bar: "RK4, h = 1 ms", or the adaptive one's sub-steps. */
function steppingText(): string {
  const short = { euler: 'Euler', 'symplectic-euler': 'symplectic Euler', midpoint: 'midpoint', heun: 'Heun', verlet: 'Verlet', rk3: 'RK3', rk4: 'RK4', rk45: 'RK45' }[stepping.integrator];
  if (stepping.integrator === 'rk45') return `${short}, h ≤ ${stepText(stepping.h)} (${sim?.adaptiveSteps ?? 1} per step), tol ${stepping.tolerance.toExponential(0)}`;
  return `${short}, h = ${stepText(stepping.h)}`;
}
function showStepping() {
  integratorSel.value = stepping.integrator;
  // a step not in the list (from a file): add it
  if (![...stepSel.options].some((o) => Number(o.value) === stepping.h)) stepSel.append(new Option(stepText(stepping.h), String(stepping.h)));
  stepSel.value = [...stepSel.options].find((o) => Number(o.value) === stepping.h)!.value;
  if (![...toleranceSel.options].some((o) => Number(o.value) === stepping.tolerance)) toleranceSel.append(new Option(stepping.tolerance.toExponential(0), String(stepping.tolerance)));
  toleranceSel.value = [...toleranceSel.options].find((o) => Number(o.value) === stepping.tolerance)!.value;
  const adaptive = stepping.integrator === 'rk45';
  $<HTMLElement>('#tolerance-row').hidden = !adaptive;
  $<HTMLElement>('#step-label').textContent = adaptive ? 'Largest step' : 'Step';
  const it = INTEGRATORS[stepping.integrator];
  $<HTMLElement>('#integrator-note').textContent = `Order ${it.order}, ${it.evaluations} evaluation${it.evaluations === 1 ? '' : 's'} a step: ${it.note}.`;
}
for (const sel of [integratorSel, stepSel, toleranceSel])
  sel.addEventListener('change', () => {
    stepping.integrator = integratorSel.value as Integrator;
    stepping.h = Number(stepSel.value);
    stepping.tolerance = Number(toleranceSel.value);
    localStorage.setItem(INTEGRATOR_KEY, JSON.stringify(stepping));
    showStepping();
    simDebt = 0;
  });
showStepping();

// ---- the right-click search list: everything in the side panel, found by typing (ui/palette.ts) ----
const palette = new Palette();
const sidePanel = $<HTMLElement>('#toolbar');
// the list opens under the mouse: a right press on it works as on the drawing (drag: close it and pan; click: open it here)
palette.el.addEventListener('contextmenu', (e) => e.preventDefault());
palette.el.addEventListener('pointerdown', (e) => {
  if (e.button !== 2) return;
  e.preventDefault();
  const r = canvas.getBoundingClientRect();
  const sp: Vec = [e.clientX - r.left, e.clientY - r.top];
  canvas.setPointerCapture(e.pointerId); // its moves and release go to the drawing
  panning = { last: sp, start: sp, right: true, moved: false };
});
const textOf = (el: Element | null) => (el?.textContent ?? '').replace(/\s+/g, ' ').trim();
/** Whether a control can be used now: not disabled, not inside something hidden. */
const usable = (el: HTMLElement) => !(el as HTMLInputElement).disabled && !el.closest('[hidden]');

/** The search list's entries, as the side panel is now (what is on, what can be used). */
function paletteCommands(): Command[] {
  const out: Command[] = [];
  for (const b of toolButtons) {
    if (!usable(b)) continue;
    const label = [...b.childNodes].filter((n) => n.nodeType === Node.TEXT_NODE).map((n) => n.textContent).join('').trim();
    out.push({ label, group: 'Tool', hint: textOf(b.querySelector('kbd')), keywords: b.title, on: b.dataset.tool === tool && !sim, run: () => b.click() });
  }
  for (const [id, hint] of [['#play', 'Space'], ['#step', ''], ['#reset', 'R']] as const) {
    const b = $<HTMLButtonElement>(id);
    if (usable(b)) out.push({ label: textOf(b), group: 'Simulation', hint: hint || undefined, keywords: b.title, run: () => b.click() });
  }
  // toggles
  sidePanel.querySelectorAll<HTMLLabelElement>('label.check').forEach((l) => {
    const box = l.querySelector<HTMLInputElement>('input[type="checkbox"]');
    if (!box || !usable(box)) return;
    out.push({ label: textOf(l).replace(/[≥×]$/, '').trim(), group: l.closest('#beam-colour') ? 'Beam forces' : 'Setting', keywords: l.title || l.closest<HTMLElement>('[title]')?.title, on: box.checked, toggle: true, run: () => box.click() });
  });
  // the beam colour switches
  const segName: Record<string, string> = { 'beam-q': 'Beam colour', 'beam-mode': 'Beam colour mode', 'beam-scale-mode': 'Beam colour scale' };
  sidePanel.querySelectorAll<HTMLElement>('.seg').forEach((seg) => {
    if (!usable(seg)) return;
    seg.querySelectorAll<HTMLButtonElement>('button').forEach((b) =>
      out.push({ label: `${segName[seg.id] ?? 'Option'}: ${textOf(b)}`, group: 'Beam forces', keywords: b.title, on: b.classList.contains('on'), toggle: true, run: () => b.click() }),
    );
  });
  // sliders and number boxes: worked right in the list, driving the side panel's own (its handlers do the rest)
  /** Set a side-panel control as if worked by hand: 'input' as it moves, 'change' when done. */
  const drive = (input: HTMLInputElement, v: string, done: boolean, events: ('input' | 'change')[]) => {
    input.value = v;
    for (const ev of events) if (ev === 'input' || done) input.dispatchEvent(new Event(ev, { bubbles: true }));
  };
  sidePanel.querySelectorAll<HTMLLabelElement>('label.slider').forEach((l) => {
    const input = l.querySelector<HTMLInputElement>('input');
    if (!input || !usable(input)) return;
    const label = [...l.childNodes].filter((n) => n.nodeType === Node.TEXT_NODE).map((n) => n.textContent).join('').trim();
    const output = l.querySelector('output');
    out.push({
      label,
      group: l.closest('#friction-settings') ? 'Friction' : l.closest('#ground-settings') ? 'Ground' : 'Setting',
      keywords: 'slider',
      value: { type: 'range', min: input.min, max: input.max, step: input.step, get: () => input.value, set: (v, done) => drive(input, v, done, ['input', 'change']), shown: () => textOf(output) },
    });
  });
  // the background: each scene an entry, and choosing an image
  for (const o of bgKind.options)
    out.push({ label: `Background: ${o.textContent}`, group: 'Background', keywords: 'backdrop scene image picture', on: bgKind.value === o.value, toggle: o.value !== 'custom', run: () => {
      bgKind.value = o.value;
      bgKind.dispatchEvent(new Event('change'));
    } });
  if (backdrop.state.kind === 'custom') out.push({ label: 'Edit background…', group: 'Background', keywords: 'backdrop images layers parallax depth', run: editBackground });
  if (!$<HTMLElement>('#bg-weather-row').hidden) {
    const wsel = $<HTMLSelectElement>('#bg-weather');
    for (const m of WEATHER_MODES)
      out.push({ label: `Weather: ${m.name}`, group: 'Background', keywords: 'rain storm thunder lightning hail snow cloudy clouds sky', on: wsel.value === m.id, toggle: true, run: () => {
        wsel.value = m.id;
        wsel.dispatchEvent(new Event('change'));
      } });
    for (const k of WEATHERS) out.push({ label: `Start weather now: ${k.name}`, group: 'Background', keywords: 'trigger manual rain storm thunder lightning hail snow cloudy clouds sky', run: () => triggerWeather(k.id) });
    if (backdrop.weather.triggered) out.push({ label: 'Stop the weather started by hand', group: 'Background', keywords: 'trigger manual end weather', run: () => $<HTMLButtonElement>('#bg-trigger-stop').click() });
    out.push({ label: 'Lightning strike', group: 'Background', keywords: 'thunder bolt flash weather trigger', run: () => backdrop.weather.strikeNow() });
  }
  out.push({ label: 'Background strength', group: 'Background', keywords: 'opacity fade backdrop', value: { type: 'range', min: bgOpacity.min, max: bgOpacity.max, step: bgOpacity.step, get: () => bgOpacity.value, set: (v, done) => drive(bgOpacity, v, done, ['input', 'change']), shown: () => `${Math.round(Number(bgOpacity.value) * 100)}%` } });
  // the integrator and step: each choice an entry (a toggle, like a switch's option)
  for (const [sel, name] of [[integratorSel, 'Integrator'], [stepSel, stepping.integrator === 'rk45' ? 'Largest step' : 'Step'], [toleranceSel, 'Tolerance']] as const) {
    if (!usable(sel)) continue;
    for (const o of sel.options)
      out.push({ label: `${name}: ${o.textContent}`, group: 'Simulation', keywords: sel === integratorSel ? 'integrator solver method' : sel === stepSel ? 'time step size' : 'adaptive error tolerance', on: sel.value === o.value, toggle: true, run: () => {
        sel.value = o.value;
        sel.dispatchEvent(new Event('change'));
      } });
  }
  const numbers: [string, string, string][] = [
    ['#deflect-scale', 'Deflection drawn larger by', '×'],
    ['#beam-E', "Young's modulus E (default for bars)", 'GPa'],
    ['#beam-I', 'Second moment of area I (default for bars)', 'cm⁴'],
    ['#break-at', 'Bars break at bending moment', 'N·m'],
    ['#beam-scale', 'Beam colour: value drawn red', ''],
  ];
  for (const [id, label, unit] of numbers) {
    const input = $<HTMLInputElement>(id);
    if (usable(input))
      out.push({ label, group: 'Beam forces', value: { type: 'number', min: input.min, max: input.max, step: input.step, get: () => input.value, set: (v) => drive(input, v, true, ['change']), unit: id === '#beam-scale' ? textOf($('#beam-legend .bl-unit')) : unit } });
  }
  for (const [id, label, hint] of [['#save-design', 'Save design…', ''], ['#load-design', 'Load design…', ''], ['#import-design', 'Import design…', ''], ['#save-project', 'Save project…', 'Ctrl S'], ['#load-project', 'Load project…', '']] as const) {
    const b = $<HTMLButtonElement>(id);
    out.push({ label, group: 'File', hint: hint || undefined, keywords: `${b.title} save load open import file`, run: () => b.click() });
  }
  for (const [id, group, hint] of [['#undo', 'Edit', 'Ctrl Z'], ['#fit', 'View', ''], ['#clear', 'Edit', '']] as const) {
    const b = $<HTMLButtonElement>(id);
    if (usable(b)) out.push({ label: textOf(b), group, hint: hint || undefined, keywords: b.title, run: () => b.click() });
  }
  const eom = $<HTMLButtonElement>('#eom-toggle');
  out.push({ label: `Equations panel: ${eom.textContent === '–' ? 'collapse' : 'expand'}`, group: 'View', run: () => eom.click() });
  out.push({ label: 'Open the gallery…', group: 'File', keywords: 'gallery examples presets projects browse', run: () => gallery.open() });
  out.push({ label: 'Save project to the gallery', group: 'File', keywords: 'save browser gallery project store', run: () => {
    gallery.open();
    (document.querySelector('#gallery .g-save') as HTMLButtonElement | null)?.click();
  } });
  presets.forEach((p, i) =>
    out.push({
      label: p.name,
      group: 'Example',
      keywords: p.group,
      run: () => void requestPreset(i),
    }),
  );
  return out;
}

/**
 * Open the search list at (x, y) (viewport px) for a right-click at screen point sp: on a part, its
 * own entries first (selecting it, so it shows which); elsewhere, everything.
 */
function openPalette(x: number, y: number, sp: Vec) {
  const hit = ic || duplicating || drawing ? null : hitTest(toWorld(view, sp));
  if (!hit) {
    palette.open(x, y, paletteCommands);
    return;
  }
  const part: Selection = hit.kind === 'lever' || hit.kind === 'pid' ? { kind: 'gear', id: hit.id } : hit;
  if (!selectionExists(part)) {
    palette.open(x, y, paletteCommands);
    return;
  }
  if (part.kind !== selection?.kind || part.id !== selection?.id) {
    selection = part;
    multi = null;
    renderInspector();
  }
  palette.open(x, y, paletteCommands, { title: partTitle(part.kind, part.id).replace(/^./, (c) => c.toUpperCase()), items: () => (selectionExists(part) ? partCommands(part) : []) });
}

/** Where a part is, to click at with a tool: a joint, a bar's middle, a gear's centre, a spring's middle. */
function partAnchor(s: Selection): Vec {
  switch (s.kind) {
    case 'point':
      return pos(s.id);
    case 'link':
    case 'spring': {
      const x = (s.kind === 'link' ? design.links : design.springs).find((y) => y.id === s.id)!;
      const [a, b] = [pos(x.a), pos(x.b)];
      return [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
    }
    default:
      return pos(design.gears.find((g) => g.id === s.id)!.center);
  }
}

/**
 * A part's own entries in the search list. Editing: what the tools do to it (join, pin, motor,
 * lock, …), its numbers to set right there, its dimension, its initial motion, duplicate, delete.
 * Running: measuring it and graphing its quantities, and for a bar its internal forces (diagrams,
 * its E and I, the beam colouring).
 */
function partCommands(s: Selection): Command[] {
  const out: Command[] = [];
  const name = partTitle(s.kind, s.id);
  const group = name.replace(/^./, (c) => c.toUpperCase());
  const plain = (h: string) => h.replace(/<sub>(.*?)<\/sub>/g, '$1').replace(/<[^>]+>/g, '');
  const kind = s.kind as 'point' | 'link' | 'gear' | 'spring';
  const inspect = () => {
    selection = s;
    multi = null;
    inspectSettings = false;
    renderInspector();
  };
  /** Do what clicking it with tool t does; keep the tool when it needs more clicks (join, pin, a rail, initial state, a PID's sensor). */
  const withTool = (t: Tool, keep = false) => () => {
    const prev = tool;
    const w = partAnchor(s);
    setTool(t);
    toolClick(w, toScreen(view, w));
    if (!keep && tool === t) setTool(prev);
  };
  /** A number of the part's, set right in the list (one undo step; saved, and derived again unless `look` only). */
  const field = (label: string, get: () => number, set: (v: number) => void, step: string, unit: string, look = false): Command => ({
    label,
    group,
    value: {
      type: 'number',
      min: '0',
      max: '',
      step,
      unit,
      get: () => String(+get().toPrecision(6)),
      set: (v) => {
        const x = parseFloat(v);
        if (!Number.isFinite(x)) return;
        pushUndo();
        set(x);
        if (look) {
          localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable()));
          beamCache.clear();
        } else designChanged({ keepInspector: true });
      },
    },
  });
  const dimension = (label: string): Command => {
    const shown = !!design.dimensions?.some((x) => x.kind === s.kind && x.id === s.id);
    return { label: shown ? `Hide its ${label}` : `Show its ${label}`, group, keywords: 'dimension', on: shown, toggle: true, run: () => toggleDimension({ kind: kind as DDimension['kind'], id: s.id }) };
  };

  if (sim) {
    // running: measure it, graph it, and a bar's internal forces
    out.push({ label: `Measure ${name}`, group, keywords: 'inspector quantities', run: inspect });
    if (s.kind === 'link' && forcesNow()) {
      const l = design.links.find((x) => x.id === s.id)!;
      for (const q of ['M', 'V', 'N', 'w'] as BeamQ[])
        out.push({ label: `${BEAM_Q[q].name[0].toUpperCase()}${BEAM_Q[q].name.slice(1)} along it`, group, keywords: 'beam diagram internal force', run: () => {
          inspect();
          openBeam(s.id, q);
        } });
      if (!l.rack) {
        out.push(field("Young's modulus E (this bar)", () => (l.E ?? deflect.E) / 1e9, (v) => (l.E = v * 1e9), '1', 'GPa', true));
        out.push(field('Second moment of area I (this bar)', () => (l.I ?? deflect.I) * 1e8, (v) => (l.I = v * 1e-8), '0.01', 'cm⁴', true));
      }
      out.push(...paletteCommands().filter((c) => c.group === 'Beam forces'));
    }
    for (const m of [...measuresOf(design, kind, s.id, g0()), ...forceMeasures(kind, s.id), ...loadMeasures(kind, s.id)])
      out.push({ label: `Graph ${plain(m.sym)}: ${m.name}`, group, keywords: 'graph plot over time', run: () => {
        inspect();
        openGraph(kind, s.id, m);
      } });
    return out;
  }

  // editing
  out.push({ label: `Edit ${name} in the inspector`, group, keywords: 'select settings', run: inspect });
  if (s.kind === 'link') {
    const l = design.links.find((x) => x.id === s.id)!;
    out.push({ label: 'Join it to another body…', group, keywords: 'weld rigid join bodies', run: withTool('join', true) });
    out.push(field('Mass', () => l.mass, (v) => (l.mass = Math.max(0.01, v)), '0.1', 'kg'));
    if (!l.rack) {
      out.push(field("Young's modulus E (for deflection)", () => (l.E ?? deflect.E) / 1e9, (v) => (l.E = v * 1e9), '1', 'GPa', true));
      out.push(field('Second moment of area I (for deflection)', () => (l.I ?? deflect.I) * 1e8, (v) => (l.I = v * 1e-8), '0.01', 'cm⁴', true));
    }
    out.push({
      label: l.rack ? 'Make it a plain bar (no teeth)' : 'Make it a rack (toothed)',
      group,
      on: !!l.rack,
      toggle: true,
      run: () => {
        pushUndo();
        if (l.rack) {
          delete l.rack;
          design.rackMeshes = design.rackMeshes.filter((m) => m.rack !== l.id);
        } else l.rack = 1;
        designChanged();
      },
    });
    out.push(dimension('length'));
  } else if (s.kind === 'point') {
    const p = design.points.find((x) => x.id === s.id)!;
    const constrained = p.kind !== 'free' || !!p.pin || !!p.slide || p.gear !== undefined;
    out.push({ label: 'Pin it (onto a body, or the ground)…', group, keywords: 'pin fix ground', run: withTool('pin', true) });
    out.push({ label: 'Put it on a rail…', group, keywords: 'slider rail slide', run: withTool('slider', true) });
    if (constrained) out.push({ label: 'Make it free', group, keywords: 'free unpin', run: withTool('free') });
    out.push({ label: p.mass > 0 ? 'Remove its mass' : 'Hang a mass on it', group, keywords: 'mass weight', run: withTool('mass') });
    if (p.mass > 0) out.push(field('Point mass', () => p.mass, (v) => (p.mass = Math.max(0, v)), '0.1', 'kg'));
    out.push(field('x', () => p.x, (v) => (p.x = v), '0.01', 'm'));
    out.push(field('y', () => p.y, (v) => (p.y = v), '0.01', 'm'));
    out.push(dimension('position'));
  } else if (s.kind === 'gear') {
    const g = design.gears.find((x) => x.id === s.id)!;
    out.push({ label: g.motor ? 'Remove its motor' : 'Put a motor on it', group, keywords: 'motor drive torque', run: withTool('motor') });
    if (!g.propeller) out.push({ label: g.locked ? 'Unlock it' : 'Lock it (no turning)', group, keywords: 'lock brake', on: g.locked, toggle: true, run: withTool('lockGear') });
    out.push({ label: g.motor?.lever ? 'Remove its lever' : 'Give its motor a lever', group, keywords: 'lever throttle control', run: withTool('lever') });
    out.push({ label: g.motor?.pid ? 'Remove its PID controller' : 'Put a PID controller on it…', group, keywords: 'pid controller sensor', run: withTool('pid', !g.motor?.pid) });
    out.push({ label: 'Join it to another body…', group, keywords: 'weld rigid join bodies', run: withTool('join', true) });
    out.push(field('Mass', () => g.mass, (v) => (g.mass = Math.max(0.001, v)), '0.1', 'kg'));
    if (g.motor && !g.motor.lever && !g.motor.pid) out.push(field('Motor torque', () => g.motor!.torque, (v) => (g.motor!.torque = v), '0.1', 'N·m'));
    if (g.motor) out.push(field('Motor no-load speed (0: constant torque)', () => g.motor!.noLoadRpm, (v) => (g.motor!.noLoadRpm = Math.max(0, v)), '1', 'rpm'));
    out.push(dimension('radius'));
  } else if (s.kind === 'spring') {
    const sp = design.springs.find((x) => x.id === s.id)!;
    out.push(field('Stiffness k', () => sp.k, (v) => (sp.k = Math.max(0, v)), '1', 'N/m'));
    out.push(field('Rest length', () => sp.rest, (v) => (sp.rest = Math.max(0, v)), '0.01', 'm'));
    out.push({ label: 'Tension only (a cable)', group, keywords: 'cable rope slack', on: !!sp.tensionOnly, toggle: true, run: () => {
      pushUndo();
      sp.tensionOnly = !sp.tensionOnly;
      designChanged({ keepInspector: true });
    } });
    out.push(dimension('length'));
  }
  if (s.kind !== 'spring') out.push({ label: 'Set its initial motion…', group, keywords: 'initial state velocity', run: withTool('initial', true) });
  out.push({ label: 'Duplicate it', group, keywords: 'copy', hint: 'Ctrl D', run: () => {
    selection = s;
    multi = null;
    startDuplicate();
  } });
  out.push({ label: `Delete ${name}`, group, keywords: 'remove', run: () => deleteSelection(s) });
  return out;
}

// ---- toolbar ----------------------------------------------------------------------
const toolButtons = [...document.querySelectorAll<HTMLButtonElement>('#tools button')];
toolButtons.forEach((b) => b.addEventListener('click', () => setTool(b.dataset.tool as Tool)));

// ---- the side bar's blocks: each one folds to its heading (folded at first; remembered) ----
const BLOCKS_KEY = 'linkage-lab:blocks';
const openBlocks: Record<string, boolean> = (() => {
  try {
    const v = JSON.parse(localStorage.getItem(BLOCKS_KEY) ?? '{}');
    return v && typeof v === 'object' ? v : {};
  } catch {
    return {};
  }
})();
document.querySelectorAll<HTMLElement>('#toolbar .collapsible').forEach((block) => {
  const key = block.dataset.block!;
  const head = block.querySelector<HTMLButtonElement>('.block-head')!;
  const show = (open: boolean) => {
    block.classList.toggle('collapsed', !open);
    head.setAttribute('aria-expanded', String(open));
  };
  show(!!openBlocks[key]);
  head.addEventListener('click', () => {
    const open = block.classList.contains('collapsed');
    show(open);
    openBlocks[key] = open;
    localStorage.setItem(BLOCKS_KEY, JSON.stringify(openBlocks));
  });
});

/** What a folded block's heading says of it: the tool in use, the beam analysis that is on, friction and ground. */
function blockSummaries() {
  const put = (id: string, text: string) => {
    const el = document.getElementById(id);
    if (el && el.textContent !== text) el.textContent = text;
  };
  const b = toolButtons.find((x) => x.dataset.tool === tool);
  const toolName = b ? [...b.childNodes].filter((n) => n.nodeType === Node.TEXT_NODE).map((n) => n.textContent).join('').trim() : '';
  put('sum-tools', sim ? 'while running: pull parts' : toolName);
  const colour = beamColour.q === 'off' ? '' : { N: 'tension', V: 'shear', M: 'bending', w: 'deflection' }[beamColour.q];
  put('sum-beams', [colour, deflect.on && 'deflection drawn', breaking.on && `break ≥ ${breaking.at} N·m`].filter(Boolean).join(' · '));
  put('sum-physics', [friction.enabled && 'friction', design.ground && 'ground'].filter(Boolean).join(' · '));
  const bgScene = backgroundScene();
  const bgAnim = backdrop.state.anim.on;
  put('sum-background', [backdrop.name, bgAnim && bgScene?.cycle && clockText(backdrop.hour), bgAnim && bgScene?.weather && backdrop.weather.name.toLowerCase()].filter(Boolean).join(' · '));
  if (!$<HTMLElement>('#bg-time-row').hidden) showBackgroundTime();
  if (!$<HTMLElement>('#bg-trigger-row').hidden) showWeatherTrigger();
}

function setTool(t: Tool) {
  if (pidPick) cancelSensorPick();
  if (ic || !icMenu.hidden) cancelInitial();
  duplicating = null;
  constrain = null;
  groundSel = null;
  if (t !== 'select') multi = null;
  tool = t;
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
  recordState();
  checkBreaks();
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

// ---- the gallery (ui/gallery.ts): the user's projects, saved in this browser, and the examples ----
/** A project kept in the gallery: its name and description (the user's to edit), when, and the project itself. */
interface StoredProject {
  id: string;
  name: string;
  description: string;
  savedAt: string;
  project: ProjectFile;
}
const PROJECTS_KEY = 'linkage-lab:projects';

/** The user's projects in this browser, newest first. */
function storedProjects(): StoredProject[] {
  try {
    const list = JSON.parse(localStorage.getItem(PROJECTS_KEY) ?? '[]');
    return Array.isArray(list) ? list.filter((x) => x && typeof x.id === 'string' && x.project?.design) : [];
  } catch {
    return [];
  }
}

/** Keep the user's projects; false (and said so) if the browser has no room for them. */
function storeProjects(list: StoredProject[]): boolean {
  try {
    localStorage.setItem(PROJECTS_KEY, JSON.stringify(list));
    return true;
  } catch {
    messages.notify("The browser has no room left to keep this project: save it to a file instead (Save & load), or delete some from the gallery.", 'error');
    return false;
  }
}

/** A saved design made whole (from a copy: what is stored stays as it was). */
const designCopy = (raw: unknown) => normaliseDesign(JSON.parse(JSON.stringify(raw))) ?? emptyDesign();

const when = (iso: string) => {
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? '' : d.toLocaleString(undefined, { dateStyle: 'medium', timeStyle: 'short' });
};

function galleryItems(): GalleryItem[] {
  const own = storedProjects().map((sp): GalleryItem => ({ key: `p:${sp.id}`, group: MY_PROJECTS, name: sp.name, description: sp.description, notes: `Your project · saved ${when(sp.savedAt)}`, design: () => designCopy(sp.project.design), editable: true }));
  const examples = presets.map((p, i): GalleryItem => ({
    key: `e:${i}`,
    group: p.group,
    name: p.name,
    description: p.description ?? '',
    notes: `${p.group}${p.topView ? ' · seen from above (no gravity)' : ''}${p.follow ? ' · the view follows it' : ''}`,
    design: () => p.build(),
  }));
  return [...own, ...examples];
}

const projectOf = (key: string) => storedProjects().find((x) => `p:${x.id}` === key);

/** Save the current project to the gallery, in this browser, first in its list; its key, or null if there was no room. */
function saveToGallery(): string | null {
  const sp: StoredProject = { id: `${Date.now().toString(36)}${Math.random().toString(36).slice(2, 6)}`, name: designTitle === 'linkage' ? 'Untitled project' : designTitle, description: '', savedAt: new Date().toISOString(), project: buildProject() };
  if (!storeProjects([sp, ...storedProjects()])) return null;
  setUnsaved(false);
  sfx('place');
  messages.notify(`Saved “${sp.name}” to the gallery, in this browser: name it and describe it there.`);
  return `p:${sp.id}`;
}

const gallery = new Gallery({
  items: galleryItems,
  open: (key) => {
    if (key.startsWith('e:')) void requestPreset(Number(key.slice(2)));
    else void requestProject(key);
  },
  import: (key) => {
    const sp = projectOf(key);
    const p = key.startsWith('e:') ? presets[Number(key.slice(2))] : null;
    if (sp) startImport(designCopy(sp.project.design), sp.name);
    else if (p) startImport(p.build(), p.name);
  },
  saveCurrent: () => saveToGallery(),
  replace: (key) => {
    const list = storedProjects();
    const sp = list.find((x) => `p:${x.id}` === key);
    if (!sp) return;
    Object.assign(sp, { project: buildProject(), savedAt: new Date().toISOString() });
    if (!storeProjects(list)) return;
    setUnsaved(false);
    messages.notify(`Updated “${sp.name}” with the current project.`);
  },
  rename: (key, name, description) => {
    const list = storedProjects();
    const sp = list.find((x) => `p:${x.id}` === key);
    if (!sp) return;
    Object.assign(sp, { name, description });
    storeProjects(list);
  },
  remove: async (key) => {
    const sp = projectOf(key);
    if (!sp) return false;
    const a = await askDialog({ title: `Delete “${sp.name}”?`, body: 'It is removed from the gallery in this browser. This cannot be undone (a copy saved to a file stays).', buttons: [{ id: 'delete', label: 'Delete', primary: true }, { id: 'cancel', label: 'Cancel' }] });
    if (a.choice !== 'delete') return false;
    storeProjects(storedProjects().filter((x) => x.id !== sp.id));
    messages.notify(`Deleted “${sp.name}” from the gallery.`);
    return true;
  },
});
$<HTMLButtonElement>('#gallery-btn').addEventListener('click', () => gallery.open());
const NO_WARN_KEY = 'linkage-lab:no-unsaved-warning';

/**
 * Whether to go ahead replacing the drawing with `what`: yes at once if it has no unsaved changes
 * (or the user said not to ask); else asked, with saving first as a choice.
 */
async function confirmReplace(what: string): Promise<boolean> {
  const muted = localStorage.getItem(NO_WARN_KEY) === '1' || sessionStorage.getItem(NO_WARN_KEY) === '1';
  if (unsaved && !muted) {
    const a = await askDialog({
      title: 'Your design has unsaved changes',
      body: `Opening “${what}” replaces your drawing, which has changes not saved. Undo can bring it back until you reload the page.`,
      buttons: [
        { id: 'gallery', label: 'Save to gallery first' },
        { id: 'save', label: 'Save to a file first…' },
        { id: 'open', label: 'Open without saving', primary: true },
        { id: 'cancel', label: 'Cancel' },
      ],
      remember: true,
    });
    if (a.remember === 'forever') localStorage.setItem(NO_WARN_KEY, '1');
    else if (a.remember === 'session') sessionStorage.setItem(NO_WARN_KEY, '1');
    if (a.choice === 'cancel') return false;
    if (a.choice === 'save' && !(await saveProject())) return false; // cancelled the save: stay
    if (a.choice === 'gallery' && !saveToGallery()) return false; // no room: stay
  }
  return true;
}

/** Open example i (asking first about unsaved changes). */
async function requestPreset(i: number) {
  const p = presets[i];
  if (p && (await confirmReplace(p.name))) openPreset(i);
}

/** Open one of the user's projects from the gallery (asking first about unsaved changes). */
async function requestProject(key: string) {
  const sp = projectOf(key);
  if (!sp || !(await confirmReplace(sp.name))) return;
  applyProject(sp.project, designCopy(sp.project.design), sp.name);
  messages.notify(`Opened your project “${sp.name}”: its drawing, settings and view.`);
}

function openPreset(i: number) {
  const p = presets[i];
  if (!p) return;
  stopSim();
  duplicating = null;
  if (ic || !icMenu.hidden) cancelInitial();
  pushUndo();
  sfx('preset');
  design = p.build();
  designTitle = p.name.split(/[:(]/)[0].trim(); // files saved from it are named after it
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
  setUnsaved(false); // an example as it comes
  fitView(p.view);
  if (p.analysis) showAnalysis(p.analysis); // an example made to show the beam analysis
}

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
// Dimensions show by default while editing and not while running; each choice is remembered.
const DIMS_KEY = 'linkage-lab-dimensions';
const dimsShown: { edit: boolean; play: boolean } = (() => {
  try {
    return { edit: true, play: false, ...JSON.parse(localStorage.getItem(DIMS_KEY) ?? '{}') };
  } catch {
    return { edit: true, play: false };
  }
})();
const dimsBox = $<HTMLInputElement>('#dims');
dimsBox.addEventListener('change', () => {
  dimsShown[sim ? 'play' : 'edit'] = dimsBox.checked;
  localStorage.setItem(DIMS_KEY, JSON.stringify(dimsShown));
});
let shiftHeld = false;
for (const ev of ['keydown', 'keyup'] as const) window.addEventListener(ev, (e) => e.key === 'Shift' && (shiftHeld = ev === 'keydown'));
window.addEventListener('blur', () => (shiftHeld = false));
for (const ev of ['pointerdown', 'pointermove'] as const) window.addEventListener(ev, (e) => (shiftHeld = e.shiftKey), { capture: true });
/** Decimals for dimensions: 2, 3 with Alt / Option, 1 with Shift; null when they are hidden. */
const dimensionDigits = () => ((sim ? dimsShown.play : dimsShown.edit) && design.dimensions?.length ? (shiftHeld ? 1 : altHeld ? 3 : 2) : null);

/**
 * A press with the Dimension tool: on a dimension (its line or text) or on a part. Let go
 * without moving and it shows or hides that part's dimension; drag (past DRAG_START) and the
 * dimension follows the cursor (made first if it was not shown), as in CAD: a length's line
 * slides nearer or farther, on either side; a radius swings round with its text out along it;
 * a position label moves beside its joint.
 */
let dimPress: { target: DDimension; downSp: Vec; live: boolean } | null = null;

const dimScene = () => ({ design, pos, gearAngle: gearAngleNow });

function pressDimension(w: Vec, sp: Vec) {
  const k = dimensionDigits() !== null ? dimensionAt(view, dimScene(), sp) : null;
  if (k !== null) {
    const d = design.dimensions![k];
    dimPress = { target: { kind: d.kind, id: d.id }, downSp: sp, live: false };
    return;
  }
  const hit = hitTest(w);
  if (!hit || hit.kind === 'lever' || hit.kind === 'pid') return;
  dimPress = { target: { kind: hit.kind, id: hit.id }, downSp: sp, live: false };
}

/** Drag the pressed dimension so it passes through w. */
function dragDimension(w: Vec, sp: Vec) {
  if (!dimPress) return;
  if (!dimPress.live) {
    if (Math.hypot(sp[0] - dimPress.downSp[0], sp[1] - dimPress.downSp[1]) < DRAG_START) return;
    dimPress.live = true;
    pushUndo();
    const t = dimPress.target;
    const list = (design.dimensions ??= []);
    if (!list.some((x) => x.kind === t.kind && x.id === t.id)) {
      list.push({ kind: t.kind, id: t.id }); // dragging out a dimension that was not shown
      sfx('tick');
    }
    if (!dimsShown.edit) {
      dimsShown.edit = true; // dragging one shows them
      localStorage.setItem(DIMS_KEY, JSON.stringify(dimsShown));
      refreshControls();
    }
  }
  const t = dimPress.target;
  const d = design.dimensions?.find((x) => x.kind === t.kind && x.id === t.id);
  if (!d) return;
  if (d.kind === 'link' || d.kind === 'spring') {
    const seg = (d.kind === 'link' ? design.links : design.springs).find((x) => x.id === d.id);
    if (!seg) return;
    const [pa, pb] = [pos(seg.a), pos(seg.b)];
    const n = dimNormal(pa, pb);
    d.offset = +((w[0] - pa[0]) * n[0] + (w[1] - pa[1]) * n[1]).toFixed(5);
  } else if (d.kind === 'gear') {
    const g = design.gears.find((x) => x.id === d.id);
    if (!g) return;
    const c = pos(g.center);
    d.angle = +Math.atan2(w[1] - c[1], w[0] - c[0]).toFixed(5);
    d.dist = +Math.hypot(w[0] - c[0], w[1] - c[1]).toFixed(5);
  } else {
    const p = pos(d.id);
    [d.dx, d.dy] = [+(w[0] - p[0]).toFixed(5), +(w[1] - p[1]).toFixed(5)];
  }
  saveSoon();
}

/** Let go: a click (no drag) shows or hides the pressed part's dimension. */
function releaseDimension() {
  const pr = dimPress;
  dimPress = null;
  if (!pr || pr.live) return;
  toggleDimension(pr.target);
}

/** Show or hide a part's dimension. */
function toggleDimension(hit: { kind: DDimension['kind']; id: number }) {
  const kind = hit.kind;
  pushUndo();
  const list = (design.dimensions ??= []);
  const i = list.findIndex((x) => x.kind === kind && x.id === hit.id);
  if (i >= 0) list.splice(i, 1);
  else list.push({ kind, id: hit.id });
  if (!list.length) delete design.dimensions;
  sfx(i >= 0 ? 'free' : 'tick');
  if (i < 0 && !dimsShown.edit) messages.notify('Dimensions are hidden: tick Show dimensions to see them.');
  designChanged();
}

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

// the equations panel: folded at first (remembered); like the side bar, it slides off its edge
// when the mouse has been away from it for 3 s, and back when the mouse comes to that edge's top
const EOM_KEY = 'linkage-lab:eom-open';
const eomWrap = $<HTMLElement>('#eom-wrap');
const eomToggle = $<HTMLButtonElement>('#eom-toggle');
const showEomFolded = (folded: boolean) => {
  eomWrap.classList.toggle('collapsed', folded);
  eomToggle.textContent = folded ? '+' : '–';
  eomToggle.title = folded ? 'Expand' : 'Collapse';
};
showEomFolded(localStorage.getItem(EOM_KEY) !== 'true');
eomToggle.addEventListener('click', () => {
  const folded = !eomWrap.classList.contains('collapsed');
  showEomFolded(folded);
  localStorage.setItem(EOM_KEY, String(!folded));
});
autoHide(eomWrap, 'right', { part: 'start' });
autoHide($<HTMLElement>('#toolbar'), 'left');
// the status bar (down, off the bottom edge) and the inspector (the bottom of the right edge)
autoHide(statusEl, 'bottom');
inspectorPanel = autoHide(inspectorEl, 'right', { part: 'end' });

const hints: Record<Tool, string> = {
  select: 'Drag joints to reshape (hold Alt / Option to move on the grid), or drag on empty space to box-select. Ctrl/⌘+D duplicates; Ctrl/⌘+C and V copy and paste.',
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
  dimension: 'Click a bar, rack or spring to show its length, a gear, wheel or propeller for its radius, or a joint for its position; click again to hide it. Press on a dimension (or its part) and drag to move it nearer or farther. Hold Alt / Option for three decimals, Shift for one.',
  initial: 'Click a part to set its initial state: pick one of its coordinates (or a rate) from the list, move the mouse to set it while everything else follows, and click to settle. Esc cancels.',
  pid: 'Click a motor (or a gear, which gets a motor) to put a PID controller on it, in place of a lever. Then click what it should measure (a joint, bar, rack, gear, wheel or propeller) and pick the quantity. Click the motor again to remove it.',
  propeller: 'Click the centre, then the edge of a blade: the blades turn out of the page about an axis square to that line, and push along it (the arrow). In empty space it is a free body; start on a joint to mount it there. Give it a motor (O) to spin it.',
  lever: 'Click a motor (or a gear, which gets a motor) to give it a control lever; its control appears at the bottom right. Drag the control up or down to set the torque, even while it runs. Click the motor again to remove the lever.',
  join: 'Click two bodies (bars, racks or gears) to join them into one. Click the same pair again to separate them.',
};

function refreshControls() {
  dimsBox.checked = sim ? dimsShown.play : dimsShown.edit; // the choice for running, or for editing
  toolButtons.forEach((b) => {
    b.classList.toggle('active', b.dataset.tool === tool && !sim);
    b.disabled = !!sim;
  });
  playBtn.textContent = running ? 'Pause' : sim ? 'Resume' : 'Play';
  resetBtn.disabled = !sim;
  undoBtn.disabled = !!sim;
  clearBtn.disabled = !!sim;
  messages.instruct(bgEditor.active
    ? 'Editing the background: drag an image to move it, its corner to resize it; set how far away each is in the panel (farther moves slower as the view pans). Right-drag to pan, Esc or Done when finished.'
    : sim
    ? running
      ? 'Drag a bar, joint or gear to pull it (red arrow = force); it is measured in the inspector. Pause to pick parts to measure.'
      : 'Paused: click a part to measure it (again, or on empty space, to let it go), or drag it to set up a pull: keep holding and press Space to let it act. Click a quantity to graph it over time.'
    : ic
        ? ic.rate
          ? `Setting ${ic.label}: move the mouse to set the speed (the arrow shows the motion over half a second); the rest follows. Click to settle, Esc to cancel.`
          : `Setting ${ic.label}: the part is pulled towards the mouse; ${ic.ed.dragFree.length > 1 ? `it and the ${ic.ed.dragFree.length - 1} coordinate${ic.ed.dragFree.length === 2 ? '' : 's'} tied to it move` : 'only it moves'}, the rest is held. Click to settle, Esc to cancel.`
        : duplicating
        ? duplicating.label
          ? `Importing ${duplicating.label}: move to where it should go and click to place it (Esc to cancel).`
          : `Duplicating ${duplicating.frag.points.length} joint${duplicating.frag.points.length === 1 ? '' : 's'} and what is on them: move to where the copy should go and click to place it (Esc to cancel).`
        : pidPick
        ? `Now click what the PID controller on motor M${gearLabel(pidPick.gear)} should measure: a joint, bar, rack, gear, wheel or propeller (Esc to leave it without a sensor).`
        : constrain
        ? constrainHint()
        : tool === 'join' && joinFirst
        ? `Now click the body to join to ${bodyName(design, joinFirst)} (or ${bodyName(design, joinFirst)} again to cancel).`
        : hints[tool]);
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
    messages.notify('Click a joint, bar, rack, gear, wheel or propeller for the sensor (Esc to leave it without one).');
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
    return [{ motor: g.id, sensor, kp: pc.kp, ki: pc.ki, kd: pc.kd, max: Math.abs(g.motor!.torque), target: pidTarget(pc), direction: pc.direction ?? 'auto' }];
  });
}

/** The controller's live line in the inspector: what it measures, aims at and puts out. */
function pidLiveText(g: DGear): string {
  const pc = g.motor!.pid!;
  const q = pc.sensor?.quantity;
  if (!pc.sensor) return 'no sensor yet';
  const c = sim?.controllers.find((x) => x.motor === g.id);
  if (!c) return `${fmtQ(drawnValue(pc.sensor.body, pc.sensor.quantity), q)} as drawn; target ${fmtQ(pidTarget(pc), q)}`;
  if (c.sign === 0) return 'off: its motor cannot move this quantity here (choose Forward or Reverse to drive it anyway)';
  return `${fmtQ(c.y, q)} → ${fmtQ(c.target, q)} (error ${fmtQ(c.e, q)}); torque ${c.u.toFixed(3)} N·m${Math.abs(c.u) >= Math.abs(c.max) - 1e-9 ? ' (at its limit)' : ''}`;
}

/** What the automatic check found, for the direction menu (only known while it runs). */
function detectedText(g: DGear): string {
  const c = sim?.controllers.find((x) => x.motor === g.id);
  if (!c) return ' (checked when it starts)';
  return c.detected > 0 ? ' (found: forward)' : c.detected < 0 ? ' (found: reverse)' : ' (found: cannot move it, off)';
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
    <label class="field" title="Which way the controller pushes. Auto checks how the motor moves the quantity when it starts, and switches off if it cannot; Forward and Reverse override that."><span>Pushing direction</span><select id="i-pdir">
      <option value="auto" ${(pc.direction ?? 'auto') === 'auto' ? 'selected' : ''}>Auto${detectedText(g)}</option>
      <option value="forward" ${pc.direction === 'forward' ? 'selected' : ''}>Forward: + torque raises it</option>
      <option value="reverse" ${pc.direction === 'reverse' ? 'selected' : ''}>Reverse: + torque lowers it</option>
    </select></label>
    <label class="field">Colour<input id="i-pcolor" type="color" value="${pc.color}"></label>
    <div class="field"><span>Now</span><span id="i-plive">${pidLiveText(g)}</span></div>
    <div class="field"><span></span><span><button id="i-premove" ${disabled}>Remove controller</button></span></div>
    <p class="muted">Torque u = s (K<sub>p</sub> e + K<sub>i</sub> ∫e dt − K<sub>d</sub> dy/dt), e = target − y, kept within ±max torque; the integral pauses while the output is at its limit, and the derivative acts on the measurement, so moving the target gives no kick. s = ±1 is the pushing direction: on Auto, the way the motor moves the quantity, found when it starts (off if it cannot move it there, as at a dead point); Forward and Reverse fix it. Gains, target, limit and direction can be changed while it runs.</p>`;
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
  inspectorEl.querySelector<HTMLSelectElement>('#i-pdir')!.addEventListener('change', (e) => {
    pushUndo();
    pc.direction = (e.target as HTMLSelectElement).value as DPid['direction'];
    sim?.tuneController(g.id, { direction: pc.direction });
    designChanged({ keepInspector: true });
    const live = inspectorEl.querySelector('#i-plive');
    if (live) live.textContent = pidLiveText(g);
  });
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
  saveTimer = window.setTimeout(() => localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable())), 250);
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
  // a different part (or parts) to inspect: the inspector comes out to show it
  const showing = `${selection?.kind}:${selection?.id}|${multi ? partCount(multi) : 0}|${groundSel ? 'ground' : ''}|${inspectSettings}`;
  if (showing !== inspectedFor) {
    inspectedFor = showing;
    inspectorPanel?.show();
  }
  measureView = null;
  if (sim && selection && (selection.kind === 'point' || selection.kind === 'link' || selection.kind === 'gear' || selection.kind === 'spring')) {
    const key = `${selection.kind}:${selection.id}`;
    if (key !== inspectedKey) inspectSettings = false; // a new part: its measurements first
    inspectedKey = key;
    if (!inspectSettings) {
      renderLevers();
      inspectorMeasure(selection.kind, selection.id);
      return;
    }
    renderInspectorPanel();
    inspectorEl.insertAdjacentHTML('afterbegin', '<div class="field"><span></span><span><button id="m-back">← Measurements</button></span></div>');
    inspectorEl.querySelector('#m-back')!.addEventListener('click', () => {
      inspectSettings = false;
      renderInspector();
    });
    return;
  }
  renderInspectorPanel();
}

// ---- measuring while it runs: the inspector's measurements and their graphs over time ---------
/** Whether the inspector shows a selected part's settings rather than its measurements, while running. */
let inspectSettings = false;
let inspectedKey = '';
/** The measurements shown now, to update every frame. */
let measureView: { kind: 'point' | 'link' | 'gear' | 'spring'; id: number; measures: Measure[]; sig: string } | null = null;
/** The state at every frame since the simulation started (at most HISTORY_MAX), for graphs. */
const HISTORY_MAX = 20000;
/** The state at every frame, and the energy figures on the status bar, which are not functions of the state (heat and work add up over time). */
interface History {
  t: number[];
  q: Float64Array[];
  qd: Float64Array[];
  /** E = T + V, heat W, motor work W_m, air work W_a, and the largest constraint error max|Φ|. */
  e: number[];
  w: number[];
  wm: number[];
  wa: number[];
  phi: number[];
  /** The accelerations and the simulator's point loads, for the joint forces. */
  qdd: Float64Array[];
  loads: Loads[];
}
const emptyHistory = (): History => ({ t: [], q: [], qd: [], e: [], w: [], wm: [], wa: [], phi: [], qdd: [], loads: [] });
let history: History = emptyHistory();

function resetHistory() {
  history = emptyHistory();
  forceCache.clear();
  beamCache.clear();
  beamColourPeak = 0;
  if (graph?.beam) [graph.beam.lo, graph.beam.hi] = [0, 0];
  if (graph) for (const x of [graph, ...graph.extra]) x.vals = [];
}

function recordState() {
  if (!sim) return;
  const h = history;
  if (h.t.length && sim.t <= h.t[h.t.length - 1]) return;
  h.t.push(sim.t);
  h.q.push(sim.q.slice());
  h.qd.push(sim.qd.slice());
  const snap = sim.snapshot();
  h.qdd.push(snap.qdd);
  h.loads.push(snap.loads);
  const { T, V } = sim.energy();
  h.e.push(T + V);
  h.w.push(sim.dissipated);
  h.wm.push(sim.motorWork);
  h.wa.push(sim.airWork);
  h.phi.push(sim.constraintError());
  if (h.t.length > HISTORY_MAX) {
    const drop = 2000;
    for (const a of [h.t, h.q, h.qd, h.e, h.w, h.wm, h.wa, h.phi, h.qdd, h.loads]) a.splice(0, drop);
    forceCache.clear(); // its indices have moved
    if (graph) for (const x of [graph, ...graph.extra]) x.vals.splice(0, Math.min(drop, x.vals.length));
  }
}

// ---- joint forces: inverse dynamics at any recorded frame (see mechanics/forces.ts) ----------
const forceCache = new Map<number, ForceResult>();

/** The joint forces at history index i (cached). */
function forcesAt(i: number): ForceResult | null {
  if (!sim || i < 0 || i >= history.t.length) return null;
  let r = forceCache.get(i);
  if (!r) {
    r = jointForces(design, sim.sys, history.q[i], history.qd[i], history.qdd[i], history.loads[i], {
      gravity: forceGravity(),
      friction: friction.enabled ? { enabled: true, pin: friction.pin, rail: friction.rail } : null,
      damping: sim.damping,
    });
    if (forceCache.size > 4000) forceCache.clear();
    forceCache.set(i, r);
  }
  return r;
}
/** The gravity the simulation runs with (m/s², 0 when off). */
function forceGravity(): number {
  const gp = sim?.sys.derivation.params.find((p) => p.sym === 'g');
  return gp ? sys0(gp.sym) : 0;
}
/** A parameter's value in the running simulation. */
const sys0 = (sym: string) => {
  const k = sim!.sys.derivation.params.findIndex((p) => p.sym === sym);
  return k >= 0 ? sim!.sys.params[k] : 0;
};
/** The joint forces now (the latest recorded frame). */
const forcesNow = () => forcesAt(history.t.length - 1);

/** A force's name and symbol: what it is and where (on a body, or on a joint for a joint). */
function forceName(it: ForceItem, onJoint: boolean): { sym: string; name: string } {
  const at = it.node !== undefined ? pointLabel(it.node) : '';
  if (it.key.startsWith('ground@')) return { sym: `R<sub>${at}</sub>`, name: `ground support at ${at}` };
  if (it.key.startsWith('rail@')) return { sym: `N<sub>${at}</sub>`, name: `${it.label} at ${at}` };
  if (it.key.startsWith('brail:')) return { sym: `N<sub>${at}</sub>`, name: `slider ${at} pressing on its rail` };
  if (it.key.startsWith('pin:')) {
    const b = it.body!;
    const bn = b.kind === 'link' ? bodyName(design, b) : b.kind === 'motor' ? `motor M${gearLabel(b.id)}` : gearName(design, b.id);
    return onJoint ? { sym: `F<sub>${at}</sub>`, name: `from ${bn}` } : { sym: `F<sub>${at}</sub>`, name: `at joint ${at}` };
  }
  if (it.key.startsWith('mesh:')) return { sym: 'F<sub>t</sub>', name: `tooth force, ${it.label}` };
  if (it.key.startsWith('rack:')) return { sym: 'F<sub>r</sub>', name: `tooth force, ${it.label}` };
  if (it.key.startsWith('weld:')) {
    const other = (design.joins ?? []).find((j) => `weld:${j.id}` === it.key.slice(0, it.key.lastIndexOf(':')));
    const ref = other && (it.key.endsWith(':a') ? other.b : other.a);
    return { sym: 'F<sub>w</sub>', name: `weld to ${ref ? bodyName(design, ref) : 'its group'}` };
  }
  if (it.key.startsWith('lock:')) return { sym: 'M<sub>h</sub>', name: 'held by the ground' };
  return { sym: 'F', name: it.label };
}

/**
 * The joint forces on a part, as quantities: for a body, the force at each of its connections
 * (size and direction) and the couples of welds and locks; for a joint, the force each body puts
 * on it and its support's reaction.
 */
function forceMeasures(kind: 'point' | 'link' | 'gear' | 'spring', id: number): Measure[] {
  if (kind === 'spring') return [];
  const now = forcesNow();
  if (!now) return [];
  const onJoint = kind === 'point';
  const mine = now.items.filter((it) => (onJoint ? it.node === id && (it.body === null || it.key.startsWith('pin:')) : it.body?.kind === kind && it.body.id === id));
  const out: Measure[] = [];
  for (const it of mine) {
    const key = it.key;
    const sign = onJoint && key.startsWith('pin:') ? -1 : 1; // on the joint: the body's force back on it
    const pick = (r: ForceResult | null) => r?.items.find((x) => x.key === key);
    const F = (r: ForceResult | null): [number, number] => {
      const x = pick(r);
      return x ? [sign * x.F[0], sign * x.F[1]] : [NaN, NaN];
    };
    const { sym, name } = forceName(it, onJoint);
    const hasForce = !key.startsWith('lock:') && !(key.startsWith('weld:') && Math.hypot(...it.F) === 0 && it.M !== undefined && pick(now) && isSharedWeld(key));
    if (hasForce) {
      out.push({ key: `F:${key}:mag`, sym: `|${sym}|`, name: `${name}: size`, unit: 'N', value: () => Math.hypot(...F(forcesNow())), atHistory: (i) => Math.hypot(...F(forcesAt(i))) });
      out.push({ key: `F:${key}:dir`, sym: `∠${sym}`, name: `${name}: direction`, unit: '°', angle: true, value: () => (Math.atan2(F(forcesNow())[1], F(forcesNow())[0]) * 180) / Math.PI, atHistory: (i) => (Math.atan2(F(forcesAt(i))[1], F(forcesAt(i))[0]) * 180) / Math.PI });
    }
    if (it.M !== undefined) {
      const M = (r: ForceResult | null) => pick(r)?.M ?? NaN;
      out.push({ key: `F:${key}:M`, sym: key.startsWith('lock:') ? sym : 'M<sub>w</sub>', name: `${name}: couple`, unit: 'N·m', value: () => M(forcesNow()), atHistory: (i) => M(forcesAt(i)) });
    }
  }
  return out;
}

/** Whether a weld's bodies share a joint (it carries only a couple then). */
function isSharedWeld(key: string): boolean {
  const j = (design.joins ?? []).find((x) => key.startsWith(`weld:${x.id}:`));
  if (!j) return false;
  const nodesOf = (r: DBodyRef) => {
    if (r.kind === 'link') {
      const l = design.links.find((x) => x.id === r.id);
      return l ? [l.a, l.b] : [];
    }
    const g = design.gears.find((x) => x.id === r.id);
    return g ? [g.center, ...design.points.filter((p) => p.gear === g.id).map((p) => p.id)] : [];
  };
  const a = nodesOf(j.a);
  return nodesOf(j.b).some((x) => a.includes(x));
}

/** How each kind of load is drawn and named: colour, symbol, name. */
const LOAD_STYLE: Record<LoadItem['type'], { color: string; sym: string; name: string }> = {
  weight: { color: '#2f7a45', sym: 'W', name: 'weight' },
  drag: { color: '#3c9cc4', sym: 'F<sub>d</sub>', name: 'air drag' },
  friction: { color: '#c0392b', sym: 'F<sub>μ</sub>', name: 'friction' },
  contact: { color: '#a35a17', sym: 'F<sub>c</sub>', name: 'ground contact' },
  spring: { color: '#5b6b7d', sym: 'F<sub>k</sub>', name: 'spring' },
  motor: { color: '#3b6ea8', sym: 'τ', name: 'motor torque' },
  stop: { color: '#7a5c2e', sym: 'F<sub>s</sub>', name: 'end stop' },
  air: { color: '#2f9e8f', sym: 'F<sub>a</sub>', name: 'propeller air' },
  pull: { color: '#d64545', sym: 'F<sub>p</sub>', name: 'your pull' },
};

/** The loads on the part being measured (on a joint: those acting on the joint itself). */
const loadsOn = (r: ForceResult, kind: string, id: number) => r.loads.filter((it) => (kind === 'point' ? it.body === null && it.node === id : it.body?.kind === kind && it.body.id === id));

/** A load's row name: its kind, and which one (a spring's id, a friction's joint). */
function loadName(it: LoadItem): string {
  const st = LOAD_STYLE[it.type];
  if (it.type === 'spring') return `spring ${springLabel(Number(it.key.split(':')[1]))}`;
  if (it.type === 'friction' && it.key.includes('@')) return `pin friction at ${pointLabel(Number(it.key.split('@')[1]))}`;
  return it.label || st.name;
}

/**
 * The known loads on a part, as quantities: one per kind (and per spring or friction joint),
 * several of a kind added up (two contact points: their total), size and direction, or the
 * couple for torques and pin friction.
 */
function loadMeasures(kind: 'point' | 'link' | 'gear' | 'spring', id: number): Measure[] {
  if (kind === 'spring') return [];
  const now = forcesNow();
  if (!now) return [];
  const keys = [...new Map(loadsOn(now, kind, id).map((it) => [it.key, it])).values()];
  const total = (r: ForceResult | null, key: string): { F: Vec; M: number; n: number } => {
    const out = { F: [0, 0] as Vec, M: 0, n: 0 };
    for (const it of r ? loadsOn(r, kind, id) : []) if (it.key === key) [out.F, out.M, out.n] = [[out.F[0] + it.F[0], out.F[1] + it.F[1]], out.M + (it.M ?? 0), out.n + 1];
    return out;
  };
  const out: Measure[] = [];
  for (const it of keys) {
    const st = LOAD_STYLE[it.type];
    const name = loadName(it);
    const couple = it.type === 'motor' || (it.type === 'friction' && it.key.includes('@'));
    if (!couple) {
      out.push({ key: `L:${it.key}:mag`, sym: `|${st.sym}|`, name: `${name}: size`, unit: 'N', value: () => Math.hypot(...total(forcesNow(), it.key).F), atHistory: (i) => Math.hypot(...total(forcesAt(i), it.key).F) });
      out.push({ key: `L:${it.key}:dir`, sym: `∠${st.sym}`, name: `${name}: direction`, unit: '°', angle: true, value: () => (Math.atan2(total(forcesNow(), it.key).F[1], total(forcesNow(), it.key).F[0]) * 180) / Math.PI, atHistory: (i) => (Math.atan2(total(forcesAt(i), it.key).F[1], total(forcesAt(i), it.key).F[0]) * 180) / Math.PI });
    }
    if (couple || it.type === 'drag')
      out.push({ key: `L:${it.key}:M`, sym: it.type === 'motor' ? 'τ' : it.type === 'drag' ? 'M<sub>d</sub>' : 'M<sub>μ</sub>', name: `${name}: couple`, unit: 'N·m', value: () => total(forcesNow(), it.key).M, atHistory: (i) => total(forcesAt(i), it.key).M });
  }
  return out;
}

/** Which forces and loads act on the measured part now (the panel is rebuilt when this changes). */
function forceSignature(kind: string, id: number): string {
  const now = forcesNow();
  if (!now || kind === 'spring') return '';
  return [...new Set(loadsOn(now, kind, id).map((it) => it.key))].sort().join(',');
}

/** Arrows for the joint forces on the part being measured, scaled so the largest is about 70 px. */
/** Whether the free-body diagram drawn this frame has the mouse pull in it (then the pull's own arrow is left out). */
let fbdHasPull = false;

function forceArrowsNow(): Scene['forceArrows'] {
  fbdHasPull = false;
  if (!sim || !measureView || inspectSettings) return null;
  const { kind, id } = measureView;
  if (kind === 'spring') return null;
  const now = forcesNow();
  if (!now) return null;
  const onJoint = kind === 'point';
  const items = now.items
    .filter((it) => (onJoint ? it.node === id && (it.body === null || it.key.startsWith('pin:')) : it.body?.kind === kind && it.body.id === id))
    .map((it) => {
      const s = onJoint && it.key.startsWith('pin:') ? -1 : 1;
      const F: Vec = [s * it.F[0], s * it.F[1]];
      return { at: it.at, F, M: it.M, label: `${Math.hypot(...F).toFixed(2)} N` } as { at: Vec; F: Vec; M?: number; label: string; color?: string };
    });
  // and every other load on it, where it acts, in its own colour
  for (const it of loadsOn(now, kind, id)) {
    const st = LOAD_STYLE[it.type];
    if (it.type === 'pull') fbdHasPull = true;
    const short = st.sym.replace(/<sub>(.*?)<\/sub>/g, '$1').replace(/<[^>]+>/g, '');
    items.push({ at: it.at, F: it.F, M: it.M, label: `${short} ${Math.hypot(...it.F).toFixed(2)} N`, color: st.color });
  }
  const big = Math.max(1e-9, ...items.map((x) => Math.hypot(...x.F)));
  return { items, pxPerN: 70 / big };
}

// ---- a bar's internal forces: axial force, shear and bending moment along it (mechanics/beam.ts) ----
/** The internal forces, and the deflection w (mm) they bend it by. */
type BeamQ = 'N' | 'V' | 'M' | 'w';
const BEAM_Q: Record<BeamQ, { sym: string; name: string; unit: string }> = {
  N: { sym: 'N', name: 'axial force (+ tension)', unit: 'N' },
  V: { sym: 'V', name: 'shear force', unit: 'N' },
  M: { sym: 'M', name: 'bending moment', unit: 'N·m' },
  w: { sym: 'w', name: 'deflection (+ to the left of A→B)', unit: 'mm' },
};
/** A bar's diagram with its deflection (mm, EI w″ = M, zero at its joints). */
type BeamView = BeamDiagram & { w: number[] };
const beamCache = new Map<number, { t: number; d: BeamView | null }>();

/** Bar `link`'s internal forces and deflection now (the latest recorded frame), cached for the frame. */
function beamNow(link: number): BeamView | null {
  const t = history.t.length ? history.t[history.t.length - 1] : NaN;
  const c = beamCache.get(link);
  if (c && c.t === t) return c.d;
  const r = forcesNow();
  const l = design.links.find((x) => x.id === link);
  const d0 = r && sim && l ? barDiagram(r, link, { gravity: forceGravity(), damping: sim.damping }) : null;
  const d = d0 && l ? { ...d0, w: deflection(d0, stiffnessOf(l)).map((v) => v * 1000) } : null;
  beamCache.set(link, { t, d });
  return d;
}

// ---- bars drawn bent: Euler–Bernoulli deflection, with E and I set by the user ----
const DEFLECT_KEY = 'linkage-lab:deflection';
/** Drawing deflection: on, how many times larger, and the default E (Pa) and I (m⁴) of bars without their own. */
const deflect: { on: boolean; scale: number; E: number; I: number } = (() => {
  const out = { on: false, scale: 100, E: 200e9, I: 1e-8 };
  try {
    const v = JSON.parse(localStorage.getItem(DEFLECT_KEY) ?? '');
    if (typeof v.on === 'boolean') out.on = v.on;
    for (const k of ['scale', 'E', 'I'] as const) if (Number.isFinite(v[k]) && v[k] > 0) out[k] = v[k];
  } catch {
    /* the defaults */
  }
  return out;
})();
/** A bar's bending stiffness EI (N·m²): its own E and I, or the defaults. */
const stiffnessOf = (l: DLink) => (l.E ?? deflect.E) * (l.I ?? deflect.I);
{
  const box = $<HTMLInputElement>('#deflect');
  const scale = $<HTMLInputElement>('#deflect-scale');
  const E = $<HTMLInputElement>('#beam-E');
  const I = $<HTMLInputElement>('#beam-I');
  box.checked = deflect.on;
  scale.value = String(deflect.scale);
  E.value = String(+(deflect.E / 1e9).toPrecision(6));
  I.value = String(+(deflect.I * 1e8).toPrecision(6));
  const save = () => {
    localStorage.setItem(DEFLECT_KEY, JSON.stringify(deflect));
    beamCache.clear(); // the deflections change (the forces do not)
  };
  box.addEventListener('change', () => {
    deflect.on = box.checked;
    save();
  });
  const num = (el: HTMLInputElement, apply: (v: number) => void) =>
    el.addEventListener('change', () => {
      const v = parseFloat(el.value);
      if (Number.isFinite(v) && v > 0) apply(v);
      save();
      if (measureView) inspectorMeasure(measureView.kind, measureView.id);
    });
  num(scale, (v) => (deflect.scale = v));
  num(E, (v) => (deflect.E = v * 1e9)); // GPa
  num(I, (v) => (deflect.I = v * 1e-8)); // cm⁴
}

/** Every bar bent by its deflection (drawn `scale` times larger) while it runs; racks stay straight. */
function bendsNow(): Scene['bends'] {
  if (!deflect.on || !sim || !history.t.length) return null;
  const out: NonNullable<Scene['bends']> = [];
  for (const l of design.links) {
    if (l.rack) continue;
    const d = beamNow(l.id);
    if (d) out.push({ link: l.id, s: d.s.map((x) => x / d.L), w: d.w.map((v) => (v / 1000) * deflect.scale) });
  }
  return out;
}

/** The internal forces as rows of the inspector: each one's largest value along the bar (with its sign). */
function beamMeasures(link: number): Measure[] {
  return (['N', 'V', 'M', 'w'] as BeamQ[]).map((q) => ({
    key: `B:${q}`,
    sym: BEAM_Q[q].sym,
    name: `${BEAM_Q[q].name}: largest along it`,
    unit: BEAM_Q[q].unit,
    value: () => {
      const d = beamNow(link);
      return d ? peak(d[q]).v : NaN;
    },
  }));
}

/** Show a bar's diagram in the graph panel (it replaces what is there: no phase plots with it). */
function openBeam(link: number, q: BeamQ) {
  openGraph('link', link, beamMeasures(link).find((x) => x.key === `B:${q}`)!, () => NaN);
  graph!.beam = { link, q, lo: 0, hi: 0, hoverS: null };
  setGraphTitle();
}

/** The size that is red on the bar: the largest seen since the diagram opened, or the value axis's reach if set by hand. */
function beamScale(): number {
  const g = graph!;
  const [lo, hi] = g.view.yAuto ? [g.beam!.lo, g.beam!.hi] : [g.view.lo, g.view.hi];
  return Math.max(Math.abs(lo), Math.abs(hi), 1e-9);
}

/** The bar shown in the graph panel coloured by the size of its diagram, green (none) through yellow to red. */
function beamShadeNow(): NonNullable<Scene['beamShades']>[number] | null {
  const bm = graph?.beam;
  if (!sim || !bm) return null;
  const d = beamNow(bm.link);
  if (!d) return null;
  const scale = beamScale();
  return { link: bm.link, s: d.s.map((x) => x / d.L), f: d[bm.q].map((v) => (Number.isFinite(v) ? Math.min(1, Math.abs(v) / scale) : 0)), marker: bm.hoverS === null ? null : bm.hoverS / d.L };
}

// ---- every bar coloured by an internal force (the side panel's Beam forces as colour) ----
/**
 * Which force colours the bars (off, or V, N, M), how ('max': each bar one colour for its largest
 * size; 'dist': along each bar), and what is red, on one scale for all bars: 'now', the largest
 * across the bars at this moment; 'run', the largest any bar has reached in this run; 'set', a
 * value of the user's (kept for each force, N or N·m).
 */
interface BeamColour {
  q: 'off' | BeamQ;
  mode: 'max' | 'dist';
  scale: 'now' | 'run' | 'set';
  fixed: Partial<Record<BeamQ, number>>;
}
const BEAM_COLOUR_KEY = 'linkage-lab:beam-colour';
const beamColour: BeamColour = (() => {
  const out: BeamColour = { q: 'off', mode: 'max', scale: 'now', fixed: {} };
  try {
    const v = JSON.parse(localStorage.getItem(BEAM_COLOUR_KEY) ?? '');
    if (['off', 'N', 'V', 'M'].includes(v.q)) out.q = v.q;
    if (['max', 'dist'].includes(v.mode)) out.mode = v.mode;
    if (['now', 'run', 'set'].includes(v.scale)) out.scale = v.scale;
    for (const k of ['N', 'V', 'M'] as BeamQ[]) if (Number.isFinite(v.fixed?.[k]) && v.fixed[k] > 0) out.fixed[k] = v.fixed[k];
  } catch {
    /* the defaults */
  }
  return out;
})();
/** The largest size any bar has reached in this run: red with the 'run' scale. */
let beamColourPeak = 0;
/** The size drawn red in the last frame (to start a set scale from). */
let beamColourScale = 0;
const beamQBox = $<HTMLElement>('#beam-q');
const beamModeBox = $<HTMLElement>('#beam-mode');
const beamScaleBox = $<HTMLElement>('#beam-scale-mode');
const beamLegend = $<HTMLElement>('#beam-legend');
const beamScaleInput = $<HTMLInputElement>('#beam-scale');
const saveBeamColour = () => localStorage.setItem(BEAM_COLOUR_KEY, JSON.stringify(beamColour));

function showBeamColour() {
  const off = beamColour.q === 'off';
  const set = beamColour.scale === 'set';
  for (const [box, v] of [[beamQBox, beamColour.q], [beamModeBox, beamColour.mode], [beamScaleBox, beamColour.scale]] as const)
    box.querySelectorAll<HTMLButtonElement>('button').forEach((b) => b.classList.toggle('on', b.dataset.v === v));
  beamModeBox.hidden = beamScaleBox.hidden = beamLegend.hidden = off;
  beamScaleInput.hidden = !set;
  beamLegend.querySelector<HTMLElement>('.bl-unit')!.hidden = !set;
  beamLegend.querySelector<HTMLElement>('.bl-hi')!.hidden = set;
  const q = beamColour.q;
  if (q !== 'off') {
    beamLegend.querySelector('.bl-unit')!.textContent = q === 'M' ? 'N·m' : 'N';
    if (set && document.activeElement !== beamScaleInput) beamScaleInput.value = String(beamColour.fixed[q] ?? 1);
  }
}
for (const [box, field] of [[beamQBox, 'q'], [beamModeBox, 'mode'], [beamScaleBox, 'scale']] as const)
  box.addEventListener('click', (e) => {
    const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-v]');
    if (!b) return;
    (beamColour as unknown as Record<string, string>)[field] = b.dataset.v!;
    if (field === 'q') beamColourPeak = 0; // another force: its own scale
    // a set scale starts from what is red now, the first time for each force
    const q = beamColour.q;
    if (beamColour.scale === 'set' && q !== 'off' && beamColour.fixed[q] === undefined && beamColourScale > 0 && field !== 'q') beamColour.fixed[q] = +beamColourScale.toPrecision(3);
    saveBeamColour();
    showBeamColour();
    sfx('select');
  });
beamScaleInput.addEventListener('change', () => {
  const v = parseFloat(beamScaleInput.value);
  if (beamColour.q === 'off' || !Number.isFinite(v) || v <= 0) return;
  beamColour.fixed[beamColour.q] = v;
  saveBeamColour();
});
showBeamColour();

/** Turn on the beam analysis an example asks for: its colouring, and deflection drawn (side panel and saved). */
function showAnalysis(a: NonNullable<(typeof presets)[number]['analysis']>) {
  Object.assign(beamColour, { q: a.colour, mode: a.mode, scale: a.scale });
  beamColourPeak = 0;
  saveBeamColour();
  showBeamColour();
  if (a.deflect) {
    deflect.on = true;
    deflect.scale = a.deflect;
    $<HTMLInputElement>('#deflect').checked = true;
    $<HTMLInputElement>('#deflect-scale').value = String(a.deflect);
    localStorage.setItem(DEFLECT_KEY, JSON.stringify(deflect));
  }
}

/**
 * The bars' colours: every bar by the chosen force while it runs, and the bar whose diagram is in
 * the graph panel by that diagram (on its scale). Updates the side panel's scale as well.
 */
function beamShadesNow(): Scene['beamShades'] {
  const out: NonNullable<Scene['beamShades']> = [];
  const shown = beamShadeNow();
  const q = beamColour.q;
  const unit = q === 'M' ? 'N·m' : 'N';
  if (q !== 'off' && sim && history.t.length) {
    const ds = design.links.filter((l) => l.id !== shown?.link).map((l) => ({ link: l.id, d: beamNow(l.id) }));
    const peaks = ds.map(({ d }) => (d ? Math.abs(peak(d[q]).v) : 0)).map((v) => (Number.isFinite(v) ? v : 0));
    const frameMax = Math.max(0, ...peaks);
    beamColourPeak = Math.max(beamColourPeak, frameMax);
    const red = Math.max(beamColour.scale === 'set' ? (beamColour.fixed[q] ?? 1) : beamColour.scale === 'run' ? beamColourPeak : frameMax, 1e-9);
    beamColourScale = red;
    ds.forEach(({ link, d }, k) => {
      if (!d) return;
      if (beamColour.mode === 'max') {
        const f = Math.min(1, peaks[k] / red);
        out.push({ link, s: [0, 1], f: [f, f], marker: null });
      } else {
        // along it, on the same scale as every other bar
        out.push({ link, s: d.s.map((x) => x / d.L), f: d[q].map((v) => (Number.isFinite(v) ? Math.min(1, Math.abs(v) / red) : 0)), marker: null });
      }
    });
    beamLegend.querySelector('.bl-hi')!.textContent = `${red.toPrecision(3)} ${unit}`;
  } else if (q !== 'off') beamLegend.querySelector('.bl-hi')!.textContent = 'while running';
  if (shown) out.push(shown);
  return out;
}

// ---- bars that break (the side panel's Bars break at |M| ≥) ----
const BREAK_KEY = 'linkage-lab:breaking';
const breaking: { on: boolean; at: number } = (() => {
  try {
    const v = JSON.parse(localStorage.getItem(BREAK_KEY) ?? '');
    if (typeof v.on === 'boolean' && Number.isFinite(v.at) && v.at > 0) return v;
  } catch {
    /* the default */
  }
  return { on: false, at: 2 };
})();
const breakBox = $<HTMLInputElement>('#breakable');
const breakAt = $<HTMLInputElement>('#break-at');
breakBox.checked = breaking.on;
breakAt.value = String(breaking.at);
const saveBreaking = () => localStorage.setItem(BREAK_KEY, JSON.stringify(breaking));
breakBox.addEventListener('change', () => {
  breaking.on = breakBox.checked;
  saveBreaking();
});
breakAt.addEventListener('change', () => {
  const v = parseFloat(breakAt.value);
  if (Number.isFinite(v) && v > 0) breaking.at = v;
  saveBreaking();
});

/** Whether a selected part is still in the design. */
function selectionExists(s: Selection): boolean {
  const list = { point: design.points, link: design.links, spring: design.springs, gear: design.gears, lever: design.gears, pid: design.gears }[s.kind];
  return !!list?.some((x) => x.id === s.id);
}

/**
 * Break the bars whose bending moment has reached the threshold (every frame while it runs, at
 * the recorded state): each where its |M| is largest in every stretch over the threshold.
 */
function checkBreaks() {
  if (!sim || !breaking.on || !history.t.length) return;
  const found: { link: number; u: number[]; xs: number[]; M: number }[] = [];
  for (const l of design.links) {
    if (l.rack) continue; // a rack does not break
    const d = beamNow(l.id);
    if (!d) continue;
    const xs = breakPoints(d, breaking.at);
    if (xs.length) found.push({ link: l.id, u: xs.map((x) => x / d.L), xs, M: peak(d.M).v });
  }
  if (found.length) breakNow(found);
}

/** Cut the bars and carry on: derive the cut mechanism, and start it from the state it was in. */
function breakNow(found: { link: number; u: number[]; xs: number[]; M: number }[]) {
  const old = sim!;
  const before = design;
  const names = found.map((f) => `${bodyName(design, { kind: 'link', id: f.link })} (|M| = ${Math.abs(f.M).toFixed(2)} N·m, at ${f.xs.map((x) => `${x.toFixed(2)} m`).join(' and ')} from ${pointLabel(design.links.find((l) => l.id === f.link)!.a)})`);
  const res = breakBars(design, found);
  design = res.design;
  derive();
  if (!derivation || derivation.coords.length === 0) {
    // could not carry on with the pieces: leave it whole and stop breaking
    design = before;
    derive();
    breaking.on = breakBox.checked = false;
    saveBreaking();
    messages.notify(`${names.join('; ')} would break, but the pieces could not be simulated (${deriveError ?? 'no coordinates'}), so breaking is turned off.`);
    return;
  }
  unbroken ??= cloneDesign(before);
  const next = new Simulator(new NumericSystem(derivation));
  const st = transferState(old.sys, old.q, old.qd, next.sys, res.cuts, res.pieces);
  next.q.set(st.q);
  next.qd.set(st.qd);
  next.project();
  if (!finiteState(next)) {
    // the state could not be carried over: leave it whole and stop breaking
    design = before;
    derive();
    breaking.on = breakBox.checked = false;
    saveBreaking();
    messages.notify(`${names.join('; ')} would break, but the motion could not be carried over to the pieces, so breaking is turned off.`);
    return;
  }
  next.energyOffset = old.energy().V - next.energy().V; // V's zero moves with the mechanism: E carries on
  next.t = old.t;
  next.damping = settings.damping;
  next.friction = friction;
  next.ground = groundSettings();
  next.motorWork = old.motorWork;
  next.airWork = old.airWork;
  next.dissipated = old.dissipated;
  next.setControllers(controllerSpecs(), old.controllers);
  // heat stays with its joints and meshes
  const heatOf = new Map(old.sys.derivation.joints.map((j, i) => [j.id, old.jointHeat[i]]));
  next.sys.derivation.joints.forEach((j, i) => (next.jointHeat[i] = heatOf.get(j.id) ?? 0));
  if (next.meshHeat.length === old.meshHeat.length) next.meshHeat.set(old.meshHeat);
  // a pull on a broken bar holds the piece it was on
  if (grab && !grab.gear) {
    const g = grab;
    const from = res.cuts.find((c) => (c.a === g.a && c.b === g.b) || (c.a === g.b && c.b === g.a));
    if (from) {
      const u = from.a === g.a ? g.s : 1 - g.s;
      const pc = res.pieces.find((p) => p.from === from.link && u >= p.u0 - 1e-9 && u <= p.u1 + 1e-9)!;
      const l = design.links.find((x) => x.id === pc.id)!;
      grab = { a: l.a, b: l.b, s: (u - pc.u0) / (pc.u1 - pc.u0) };
    }
  }
  sim = next;
  resetHistory();
  recordState();
  simPositions = sim.jointPositions();
  simGearAngles = sim.gearAngles();
  simMotorAngles = sim.motorAngles();
  simSpinAngles = sim.spinAngles();
  simHeat = sim.heat();
  if (selection && !selectionExists(selection)) selection = null;
  renderInspector();
  messages.notify(`Broke: ${names.join('; ')}. Reset puts the bars back.`);
  sfx('snap');
}

const partTitle = (kind: string, id: number) =>
  kind === 'system' ? 'the whole mechanism' : kind === 'point' ? `joint ${pointLabel(id)}` : kind === 'spring' ? `spring ${springLabel(id)}` : kind === 'gear' ? gearName(design, id) : bodyName(design, { kind: 'link', id });
const g0 = () => (settings.gravity ? 9.81 : 0);

/** The inspector while it runs: the selected part's physical quantities, each one a button for its graph. */
function inspectorMeasure(kind: 'point' | 'link' | 'gear' | 'spring', id: number) {
  const forces = [...forceMeasures(kind, id), ...loadMeasures(kind, id)];
  const measures = [...measuresOf(design, kind, id, g0()), ...forces];
  const now = forcesNow();
  const beams = kind === 'link' && now ? beamMeasures(id) : [];
  measureView = { kind, id, measures: [...measures, ...beams], sig: forceSignature(kind, id) };
  const title = partTitle(kind, id);
  const link = kind === 'link' ? design.links.find((l) => l.id === id) : undefined;
  const row = (m: Measure, beam: boolean) =>
    `<button class="m-row${beam ? ' m-beam' : ''}${isGraphed(kind, id, m.key) ? ' on' : ''}" data-key="${m.key}" title="${esc(m.name)}: ${beam ? 'click to show it along the bar, and colour the bar by its size' : 'click to graph it over time; shift-click to add it to a phase plot'}"><span class="m-sym">${m.sym}</span><span class="m-name">${esc(m.name)}</span><span class="m-val" id="m-${m.key}">–</span><span class="m-unit">${m.unit}</span></button>`;
  inspectorEl.innerHTML = `
    <h4>${esc(title[0].toUpperCase() + title.slice(1))} <span class="muted">measured</span></h4>
    <div class="m-list">${measures.map((m) => row(m, false)).join('')}</div>
    ${
      beams.length && link
        ? `<h5 class="m-head">Internal forces along it</h5>
    <div class="m-list">${beams.map((m) => row(m, true)).join('')}</div>
    <p class="muted m-note">Click one for its diagram along the bar, from ${esc(pointLabel(link.a))} to ${esc(pointLabel(link.b))}, with the bar coloured by its size (green, yellow, red at the largest so far). N &gt; 0 pulls the bar apart; V and M by the beam convention with ${esc(pointLabel(link.a))} on the left (M &gt; 0 sags). Its weight, drag and inertia are spread along it. Its deflection w is from EI w″ = M, zero at its joints, with E = ${+((link.E ?? deflect.E) / 1e9).toPrecision(4)} GPa and I = ${+((link.I ?? deflect.I) * 1e8).toPrecision(4)} cm⁴ (set in its settings or the side panel).</p>`
        : ''
    }
    ${
      forces.length && now
        ? `<p class="muted m-note">Forces on it, drawn where they act: joint forces (purple) by inverse dynamics, Newton–Euler for every body, and its loads in their own colours (weight green, drag blue, friction red, contact brown, springs grey, motor blue, your pull red); model residual ${now.residual.toExponential(1)}.${now.indeterminate ? ' <b>Redundant supports:</b> the mechanism is over-constrained, so how the load splits between them is not fixed by a rigid model; this is the even (minimum-norm) split.' : ''}${friction.enabled && design.meshes.length ? ' Gear-mesh friction is left out of the forces.' : ''}</p>`
        : ''
    }
    <p class="muted">Click a quantity to graph it over time; shift-click others (here, or on other parts) for a phase plot. ${running ? 'Pause to pick another part.' : 'Click another part to measure it, or empty space to let go.'} <button id="m-settings">Settings…</button></p>`;
  inspectorEl.querySelectorAll<HTMLButtonElement>('.m-row').forEach((b) =>
    b.addEventListener('click', (e) => {
      if (b.classList.contains('m-beam')) {
        openBeam(id, b.dataset.key!.slice(2) as BeamQ); // a diagram along the bar, never part of a phase plot
        return;
      }
      const m = measures.find((x) => x.key === b.dataset.key)!;
      if (e.shiftKey) addSeries(kind, id, m);
      else openGraph(kind, id, m);
    }),
  );
  inspectorEl.querySelector('#m-settings')!.addEventListener('click', () => {
    inspectSettings = true;
    renderInspector();
  });
  updateMeasures();
}

/** Refresh the measurements shown (every frame while running). */
function updateMeasures() {
  if (!sim || !measureView) return;
  if (forceSignature(measureView.kind, measureView.id) !== measureView.sig) {
    // a load came or went (a wheel landed, the pull let go): list what acts now
    inspectorMeasure(measureView.kind, measureView.id);
    return;
  }
  const s = new StateAt(sim.sys, sim.q, sim.qd);
  for (const m of measureView.measures) {
    const el = inspectorEl.querySelector(`#${CSS.escape(`m-${m.key}`)}`);
    if (el) el.textContent = fmt3(m.value(s));
  }
}

const fmt3 = (v: number) => (Number.isFinite(v) ? (Object.is(+v.toFixed(3), -0) ? '0.000' : v.toFixed(3)) : '–');

// the graph panel: one quantity at a time, real time, with a cursor line showing the value at a time
const graphEl = document.createElement('div');
graphEl.id = 'graph';
graphEl.className = 'panel';
graphEl.hidden = true;
graphEl.innerHTML = `<header><span class="g-title"></span><span class="g-now"></span><button class="g-big" title="Enlarge">⤢</button><button class="g-close" title="Close">×</button></header><canvas></canvas>`;
document.body.append(graphEl);
const graphCanvas = graphEl.querySelector('canvas')!;
/**
 * The graph's view: a window of `span` seconds ending at `end` (null: following the latest
 * time), and the value range (fitted to what is visible unless the user set it, `yAuto` false).
 */
interface GraphView {
  span: number;
  end: number | null;
  yAuto: boolean;
  lo: number;
  hi: number;
}
/** Where the last drawing put things, for the mouse: the window, value range and plot box (CSS px). */
interface GraphLayout {
  a: number;
  b: number;
  t0: number;
  t1: number;
  lo: number;
  hi: number;
  L: number;
  R: number;
  T: number;
  B: number;
  W: number;
  H: number;
}
type GraphKind = 'point' | 'link' | 'gear' | 'spring' | 'system';
/**
 * A quantity in the graph: where it is from, what it is, its value at history index i, and its
 * values at the history's indices, worked out as they come into view (a hole: not yet; see
 * fillSeries). A joint force costs a force solve per frame, so a long run is never done all at once.
 */
interface Series {
  kind: GraphKind;
  id: number;
  key: string;
  m: Measure;
  at: (i: number) => number;
  vals: number[];
}

/** Time spent working out graph values per animation frame (ms): the rest of the frame stays smooth. */
const GRAPH_FILL_MS = 6;

/**
 * Work out series x's values at history indices lo…hi that are not known yet, coarse to fine
 * (every 2ᵏ·finest-th first, then halving down to every finest-th), so the curve shows at once and
 * sharpens, until `deadline` (performance.now()). Whether all of them are done.
 */
function fillSeries(x: Series, lo: number, hi: number, finest: number, deadline: number): boolean {
  lo = Math.max(0, lo);
  hi = Math.min(history.t.length - 1, hi);
  if (hi < lo) return true;
  let stride = Math.max(1, finest);
  while ((hi - lo) / stride > 64) stride *= 2;
  for (;;) {
    for (let i = Math.ceil(lo / stride) * stride; i <= hi; i += stride)
      if (x.vals[i] === undefined) {
        x.vals[i] = x.at(i);
        if (performance.now() > deadline) return false;
      }
    if (stride <= Math.max(1, finest)) return true;
    stride = Math.max(Math.max(1, finest), stride / 2);
  }
}

/** The time window a graph shows (s, a…b) and the history indices covering it (i0…i1). */
function graphWindow(t: number[], n: number, view: GraphView) {
  const [t0, t1] = [t[0], t[n - 1]];
  // `span` seconds ending at the latest time (following) or where it was scrolled to; early in a
  // run it starts at the beginning, the rest still to fill
  let b = view.end ?? t1;
  let a = b - view.span;
  if (a < t0) [a, b] = [t0, t0 + view.span];
  const find = (tt: number) => {
    let [lo, hi] = [0, n - 1];
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (t[mid] < tt) lo = mid;
      else hi = mid;
    }
    return lo;
  };
  return { a, b, t0, t1, find, i0: Math.max(0, find(a)), i1: Math.min(n - 1, find(b) + 1) };
}
let graph: {
  /** The second and third quantities (shift-clicked): a phase plot of the first against them. */
  extra: Series[];
  /** A 3D phase plot's view: turn about the vertical and tilt (rad). */
  rot: { yaw: number; pitch: number };
  hoverY: number | null;
  kind: GraphKind;
  id: number;
  key: string;
  m: Measure;
  /** The value at history index i. */
  at: (i: number) => number;
  vals: number[];
  big: boolean;
  hoverX: number | null;
  view: GraphView;
  layout: GraphLayout | null;
  drag: { mode: 'pan' | 'y'; x: number; y: number; end: number; lo: number; hi: number } | null;
  /**
   * A bar's internal force along it instead of a quantity over time: which bar and force, the
   * range of values seen since it opened (the value axis, and the bar's colour scale), and the
   * place along it under the mouse (m from its end A).
   */
  beam?: { link: number; q: BeamQ; lo: number; hi: number; hoverS: number | null };
} | null = null;
const GRAPH_SPAN = 10; // s shown by default: the most recent 10 seconds

/** The energy figures on the status bar, each one a quantity to graph (over the recorded history). */
function energySeries(key: string): { m: Measure; at: (i: number) => number } | null {
  const h = () => history;
  const none = () => NaN;
  const make = (sym: string, name: string, at: (i: number) => number) => ({ m: { key, sym, name, unit: 'J', value: none }, at });
  switch (key) {
    case 'E':
      return make('E', 'total energy T + V', (i) => h().e[i]);
    case 'W':
      return make('W', 'heat (energy lost to friction and impacts)', (i) => h().w[i]);
    case 'Wm':
      return make('W<sub>m</sub>', 'motor work', (i) => h().wm[i]);
    case 'Wa':
      return make('W<sub>a</sub>', 'air work', (i) => h().wa[i]);
    case 'phi':
      // tiny when all is well (round-off), so in scientific notation rather than to three decimals
      return { m: { key, sym: 'max|Φ|', name: 'largest constraint error', unit: '', value: none, fmt: (v: number) => (Number.isFinite(v) ? v.toExponential(2) : '–') }, at: (i) => h().phi[i] };
    case 'budget': {
      // named as the status bar shows it: only the terms this mechanism has
      const d = sim?.sys.derivation;
      const sym = `E${friction.enabled || (sim?.dissipated ?? 0) > 0 ? ' + W' : ''}${d?.motors.length ? ' − W<sub>m</sub>' : ''}${d?.propellers.length ? ' − W<sub>a</sub>' : ''}`;
      return make(sym, 'energy budget (constant if all is accounted for)', (i) => h().e[i] + h().w[i] - h().wm[i] - h().wa[i]);
    }
  }
  return null;
}

function openGraph(kind: GraphKind, id: number, m: Measure, at?: (i: number) => number) {
  const value = at ?? m.atHistory ?? ((i: number) => (sim ? m.value(new StateAt(sim.sys, history.q[i], history.qd[i])) : NaN));
  graph = { kind, id, key: m.key, m, at: value, vals: [], extra: [], rot: { ...PHASE_ROT }, hoverY: null, big: graph?.big ?? false, hoverX: null, view: { span: GRAPH_SPAN, end: null, yAuto: true, lo: 0, hi: 1 }, layout: null, drag: null };
  graphEl.hidden = false;
  graphEl.classList.toggle('big', graph.big);
  graphEl.querySelector('.g-big')!.textContent = graph.big ? '⤡' : '⤢';
  setGraphTitle();
  sfx('select');
  markGraphed();
}

const PHASE_ROT = { yaw: -0.6, pitch: 0.42 };

/** Whether a quantity is in the graph. */
const isGraphed = (kind: GraphKind, id: number, key: string) => !!graph && [graph, ...graph.extra].some((x) => x.kind === kind && x.id === id && x.key === key);

/** Highlight the graphed quantities in the inspector. */
function markGraphed() {
  const mv = measureView;
  inspectorEl.querySelectorAll<HTMLElement>('.m-row').forEach((b) => b.classList.toggle('on', !!mv && isGraphed(mv.kind, mv.id, b.dataset.key!)));
}

/** The graph's title: the quantity, or for a phase plot each quantity in order (x, y, z). */
function setGraphTitle() {
  if (!graph) return;
  if (graph.beam) {
    const l = design.links.find((x) => x.id === graph!.beam!.link);
    const q = BEAM_Q[graph.beam.q];
    graphEl.querySelector('.g-title')!.innerHTML = `<b>${q.sym}</b> ${esc(q.name)} <span class="muted">· along ${esc(partTitle('link', graph.beam.link))}${l ? `, ${esc(pointLabel(l.a))} → ${esc(pointLabel(l.b))}` : ''}</span>`;
    return;
  }
  const parts = [graph, ...graph.extra];
  const where = (x: Series) => esc(partTitle(x.kind, x.id));
  graphEl.querySelector('.g-title')!.innerHTML =
    parts.length === 1
      ? `<b>${graph.m.sym}</b> ${esc(graph.m.name)} <span class="muted">· ${where(graph)}</span>`
      : `${parts.length}D phase: ${parts.map((x, i) => `<b>${x.m.sym}</b><span class="muted"> (${['x', 'y', 'z'][i]}, ${where(x)})</span>`).join(' · ')}`;
}

/**
 * Shift-click a quantity: add it to the graph, making a phase plot of the first quantity
 * against it (x, y, then z); shift-click one already there to take it out. A fourth replaces
 * the third.
 */
function addSeries(kind: GraphKind, id: number, m: Measure, at?: (i: number) => number) {
  if (!graph || graph.beam) {
    // a bar's diagram makes no phase plot: the quantity replaces it
    openGraph(kind, id, m, at);
    return;
  }
  const value = at ?? m.atHistory ?? ((i: number) => (sim ? m.value(new StateAt(sim.sys, history.q[i], history.qd[i])) : NaN));
  const same = (x: { kind: GraphKind; id: number; key: string }) => x.kind === kind && x.id === id && x.key === m.key;
  if (same(graph)) {
    // the first one out: the next becomes the first
    const next = graph.extra.shift();
    if (!next) {
      closeGraph();
      return;
    }
    Object.assign(graph, { kind: next.kind, id: next.id, key: next.key, m: next.m, at: next.at, vals: next.vals });
  } else if (graph.extra.some(same)) graph.extra = graph.extra.filter((x) => !same(x));
  else {
    const sr: Series = { kind, id, key: m.key, m, at: value, vals: [] };
    if (graph.extra.length < 2) graph.extra.push(sr);
    else graph.extra[1] = sr;
  }
  graph.rot = { ...PHASE_ROT };
  setGraphTitle();
  markGraphed();
  sfx('select');
}

function closeGraph() {
  graph = null;
  graphEl.hidden = true;
  markGraphed();
}

graphEl.querySelector('.g-close')!.addEventListener('click', closeGraph);
graphEl.querySelector('.g-big')!.addEventListener('click', () => {
  if (!graph) return;
  graph.big = !graph.big;
  graphEl.classList.toggle('big', graph.big);
  graphEl.querySelector('.g-big')!.textContent = graph.big ? '⤡' : '⤢';
  graphEl.querySelector<HTMLButtonElement>('.g-big')!.title = graph.big ? 'Shrink' : 'Enlarge';
});
// ---- the graph's mouse: scroll through time, and set either axis's scale ----
/** Which part of the graph a point (CSS px) is over: the plot, the value axis or the time axis. */
function graphZone(x: number, y: number): 'plot' | 'y' | 'x' | null {
  const l = graph?.layout;
  if (!l) return null;
  if (x < l.L && y >= l.T && y <= l.H - l.B) return 'y';
  if (y > l.H - l.B && x >= l.L && x <= l.W - l.R) return 'x';
  if (x >= l.L && x <= l.W - l.R && y >= l.T && y <= l.H - l.B) return 'plot';
  return null;
}

/** Show the window ending at `end` (seconds), or follow the latest time if that reaches it. */
function setGraphEnd(end: number) {
  const g = graph!;
  const l = g.layout!;
  const latest = l.t1;
  const earliest = l.t0 + Math.min(g.view.span, latest - l.t0);
  g.view.end = end >= latest - 1e-9 ? null : Math.max(earliest, end); // at the latest: follow it again
}

graphCanvas.addEventListener(
  'wheel',
  (e) => {
    const g = graph;
    const l = g?.layout;
    if (!g || !l) return;
    e.preventDefault();
    if (g.extra.length) return; // a phase plot has no time axis to scroll
    const zone = graphZone(e.offsetX, e.offsetY);
    if (g.beam && zone !== 'y') return; // a bar's diagram: its length is fixed, only the values zoom
    const f = Math.exp((e.deltaY || e.deltaX) * 0.0025);
    if (zone === 'y') {
      // zoom the value scale about the value under the mouse
      const v = l.hi - ((e.offsetY - l.T) / (l.H - l.T - l.B)) * (l.hi - l.lo);
      g.view = { ...g.view, yAuto: false, lo: v + (l.lo - v) * f, hi: v + (l.hi - v) * f };
    } else if (zone === 'x') {
      // zoom the time scale about the time under the mouse (keeping the latest in view while following)
      const t = l.a + ((e.offsetX - l.L) / (l.W - l.L - l.R)) * (l.b - l.a);
      const span = Math.min(Math.max(g.view.span * f, 0.05), 600);
      if (g.view.end === null) g.view.span = span;
      else {
        const end = t + (l.b - t) * (span / g.view.span);
        g.view.span = span;
        setGraphEnd(end);
      }
    } else {
      // over the plot: scroll through time
      const d = Math.abs(e.deltaX) > Math.abs(e.deltaY) ? e.deltaX : e.deltaY;
      setGraphEnd(l.b + (d / (l.W - l.L - l.R)) * (l.b - l.a));
    }
  },
  { passive: false },
);
graphCanvas.addEventListener('pointerdown', (e) => {
  const g = graph;
  const l = g?.layout;
  if (!g || e.button !== 0) return;
  if (g.extra.length) {
    // a 3D phase plot: drag to turn it
    if (g.extra.length === 2) {
      graphCanvas.setPointerCapture(e.pointerId);
      phaseDrag = { x: e.offsetX, y: e.offsetY, yaw: g.rot.yaw, pitch: g.rot.pitch };
    }
    return;
  }
  if (!l) return;
  const zone = graphZone(e.offsetX, e.offsetY);
  if (!zone || (g.beam && zone !== 'y')) return;
  graphCanvas.setPointerCapture(e.pointerId);
  // the plot or the time axis: drag to move through time; the value axis: drag to slide the values
  g.drag = { mode: zone === 'y' ? 'y' : 'pan', x: e.offsetX, y: e.offsetY, end: l.b, lo: l.lo, hi: l.hi };
});
graphCanvas.addEventListener('pointermove', (e) => {
  const g = graph;
  if (!g) return;
  g.hoverX = e.offsetX;
  g.hoverY = e.offsetY;
  if (g.extra.length) {
    graphCanvas.style.cursor = g.extra.length === 2 ? (phaseDrag ? 'grabbing' : 'grab') : 'crosshair';
    if (phaseDrag) {
      g.rot = { yaw: phaseDrag.yaw + (e.offsetX - phaseDrag.x) * 0.01, pitch: Math.max(-1.5, Math.min(1.5, phaseDrag.pitch + (e.offsetY - phaseDrag.y) * 0.01)) };
    }
    return;
  }
  const l = g.layout;
  const zone = graphZone(e.offsetX, e.offsetY);
  graphCanvas.style.cursor = g.drag ? (g.drag.mode === 'y' ? 'ns-resize' : 'grabbing') : zone === 'y' ? 'ns-resize' : zone === 'x' && !g.beam ? 'ew-resize' : 'crosshair';
  if (!g.drag || !l) return;
  const pw = l.W - l.L - l.R;
  if (g.drag.mode === 'pan') setGraphEnd(g.drag.end - ((e.offsetX - g.drag.x) / pw) * (l.b - l.a)); // the plot moves with the mouse
  else {
    // drag the value axis: slide the range up and down
    const dv = ((e.offsetY - g.drag.y) / (l.H - l.T - l.B)) * (g.drag.hi - g.drag.lo);
    g.view = { ...g.view, yAuto: false, lo: g.drag.lo + dv, hi: g.drag.hi + dv };
  }
});
let phaseDrag: { x: number; y: number; yaw: number; pitch: number } | null = null;
const endGraphDrag = () => {
  if (graph) graph.drag = null;
  phaseDrag = null;
};
graphCanvas.addEventListener('pointerup', endGraphDrag);
graphCanvas.addEventListener('pointercancel', endGraphDrag);
graphCanvas.addEventListener('pointerleave', () => {
  if (graph) [graph.hoverX, graph.hoverY] = [null, null];
});
graphCanvas.addEventListener('dblclick', (e) => {
  const g = graph;
  if (!g) return;
  if (g.extra.length) {
    g.rot = { ...PHASE_ROT }; // a 3D phase plot: back to its first view
    return;
  }
  const zone = graphZone(e.offsetX, e.offsetY);
  if (g.beam) {
    // a bar's diagram: the value axis (and the colour scale) start again from what it carries now
    g.view.yAuto = true;
    [g.beam.lo, g.beam.hi] = [0, 0];
    return;
  }
  // double-click: the value axis fits the values again; the time axis (or the plot) comes back to the current time
  if (zone === 'y' || zone === 'plot') g.view.yAuto = true;
  if (zone === 'x' || zone === 'plot') g.view = { ...g.view, end: null };
});

/** Bring the graph's values up to date with the history, and draw it (every frame while open). */
function updateGraph() {
  if (!graph || !sim) return;
  placeGraph();
  if (graph.beam) {
    updateBeamGraph();
    return;
  }
  const { m } = graph;
  const h = history;
  const n = h.t.length;
  const series = [graph, ...graph.extra];
  // only what is in view is worked out, a little each frame (the latest always, for the readout)
  const deadline = performance.now() + GRAPH_FILL_MS;
  for (const x of series) {
    if (x.vals.length < n) x.vals.length = n;
    if (n && x.vals[n - 1] === undefined) x.vals[n - 1] = x.at(n - 1);
  }
  let complete = true;
  if (n && graph.extra.length) {
    // a phase plot: its path, as far back as a joint's trail
    let i0 = n - 1;
    while (i0 > 0 && h.t[i0 - 1] >= h.t[n - 1] - TRAIL_SECONDS) i0--;
    for (const x of series) complete = fillSeries(x, i0, n - 1, 1, deadline) && complete;
  } else if (n) {
    const w = graphWindow(h.t, n, graph.view);
    // at most about four values per pixel of the plot
    const finest = Math.max(1, Math.floor((w.i1 - w.i0) / (4 * Math.max(50, graphCanvas.clientWidth - 68))));
    complete = fillSeries(graph, w.i0, w.i1, finest, deadline);
    // then half a window either side, ready for scrolling
    if (complete) {
      const margin = Math.round((w.i1 - w.i0) / 2);
      fillSeries(graph, w.i0 - margin, w.i1 + margin, finest, deadline);
    }
  }
  const vals = m.angle ? unwrapDegrees(graph.vals) : graph.vals;
  const all = [graph, ...graph.extra].map((x) => ({ m: x.m, vals: x.m.angle ? unwrapDegrees(x.vals) : x.vals }));
  graphEl.querySelector('.g-now')!.textContent = all
    .map((x) => (x.vals.length ? `${(x.m.fmt ?? fmt3)(x.vals[x.vals.length - 1])}${x.m.unit ? ` ${x.m.unit}` : ''}` : ''))
    .join(', ');
  if (graph.extra.length) {
    graph.layout = null;
    drawPhase(graphCanvas, h.t, all, graph.rot, graph.hoverX, graph.hoverY);
  } else graph.layout = drawGraph(graphCanvas, h.t, vals, m, graph.view, graph.hoverX, !complete);
}

/** The graph panel's place: above the inspector, or beside it if the inspector is too tall. */
function placeGraph() {
  if (!graph) return;
  if (!graph.big) {
    const top = inspectorEl.getBoundingClientRect().top;
    if (top > 300) {
      graphEl.style.right = '12px';
      graphEl.style.bottom = `${window.innerHeight - top + 10}px`;
    } else {
      graphEl.style.right = '284px';
      graphEl.style.bottom = `${leverDock.hidden ? 12 : leverDock.getBoundingClientRect().height + 24}px`;
    }
  } else {
    graphEl.style.right = '';
    graphEl.style.bottom = '';
  }
}

/** A bar's diagram in the graph panel, as it is now: the range seen so far grows with it. */
function updateBeamGraph() {
  const g = graph!;
  const bm = g.beam!;
  const d = beamNow(bm.link);
  const l = design.links.find((x) => x.id === bm.link);
  const nowEl = graphEl.querySelector('.g-now')!;
  if (!d || !l) {
    nowEl.textContent = '';
    g.layout = null;
    bm.hoverS = null;
    graphCanvas.getContext('2d')!.clearRect(0, 0, graphCanvas.width, graphCanvas.height);
    return;
  }
  const vals = d[bm.q];
  for (const v of vals) if (Number.isFinite(v)) [bm.lo, bm.hi] = [Math.min(bm.lo, v), Math.max(bm.hi, v)];
  const pk = peak(vals);
  nowEl.textContent = `peak ${fmt3(pk.v)} ${g.m.unit}`;
  g.layout = drawBeam(graphCanvas, d, vals, g.m.unit, g.view, bm, beamScale(), g.hoverX, [pointLabel(l.a), pointLabel(l.b)]);
  const lay = g.layout;
  bm.hoverS = lay && g.hoverX !== null && g.hoverX >= lay.L && g.hoverX <= lay.W - lay.R ? ((g.hoverX - lay.L) / (lay.W - lay.L - lay.R)) * d.L : null;
}

/**
 * A bar's diagram: the internal force along it (m from its end A to B), filled to the zero line in
 * the bar's colours (green, yellow, red as its size nears `scale`), steps at the point loads
 * (their places dashed), and the value under the mouse. Returns the layout, for the mouse.
 */
function drawBeam(cv: HTMLCanvasElement, d: BeamDiagram, vals: number[], unit: string, view: GraphView, seen: { lo: number; hi: number }, scale: number, hoverX: number | null, ends: [string, string]): GraphLayout | null {
  const dpr = window.devicePixelRatio || 1;
  const [W, H] = [cv.clientWidth, cv.clientHeight];
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) [cv.width, cv.height] = [Math.round(W * dpr), Math.round(H * dpr)];
  const c = cv.getContext('2d')!;
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, W, H);
  const [L, R, T, B] = [58, 12, 8, 22];
  const len = d.L;
  // the value range: what it has reached since it opened (and zero), unless set by the user
  let [lo, hi] = [view.lo, view.hi];
  if (view.yAuto) {
    [lo, hi] = [Math.min(0, seen.lo), Math.max(0, seen.hi)];
    if (hi - lo < 1e-9) [lo, hi] = [-1e-3, 1e-3];
    const pad = (hi - lo) * 0.08;
    [lo, hi] = [lo - pad, hi + pad];
  }
  const pw = W - L - R;
  const X = (s: number) => L + (s / len) * pw;
  const Y = (v: number) => T + ((hi - v) / (hi - lo)) * (H - T - B);
  c.font = '10px system-ui, sans-serif';
  c.strokeStyle = '#e8e8e2';
  c.fillStyle = '#7b7e86';
  c.lineWidth = 1;
  const ys = niceStep(hi - lo, 4);
  c.textAlign = 'right';
  c.textBaseline = 'middle';
  for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) {
    const y = Math.round(Y(v)) + 0.5;
    c.beginPath();
    c.moveTo(L, y);
    c.lineTo(W - R, y);
    c.stroke();
    c.fillText(Math.abs(v) < ys * 1e-6 ? '0' : `${+v.toPrecision(4)}`, L - 5, y);
  }
  // distance along it, and the joints at its ends
  const xs = niceStep(len, Math.max(2, Math.floor(pw / 70)));
  c.textAlign = 'center';
  c.textBaseline = 'top';
  for (let s = Math.ceil(1e-9 / xs) * xs; s < len - 1e-9; s += xs) {
    const x = Math.round(X(s)) + 0.5;
    c.beginPath();
    c.moveTo(x, T);
    c.lineTo(x, H - B);
    c.stroke();
    if (x - L > 24 && W - R - x > 24) c.fillText(`${+s.toPrecision(4)} m`, x, H - B + 4);
  }
  c.font = '600 10px system-ui, sans-serif';
  c.fillStyle = '#2b2d31';
  c.fillText(ends[0], X(0), H - B + 4);
  c.fillText(ends[1], X(len), H - B + 4);
  c.save();
  c.beginPath();
  c.rect(L, T, pw, H - T - B);
  c.clip();
  // where the point loads act
  c.strokeStyle = '#c9c9c0';
  c.setLineDash([3, 3]);
  for (const s of d.at) {
    const x = Math.round(X(s)) + 0.5;
    c.beginPath();
    c.moveTo(x, T);
    c.lineTo(x, H - B);
    c.stroke();
  }
  c.setLineDash([]);
  // the area to the zero line, coloured by size along it
  const y0 = Math.min(H - B, Math.max(T, Y(0)));
  const grad = c.createLinearGradient(L, 0, W - R, 0);
  d.s.forEach((s, k) => grad.addColorStop(Math.min(1, Math.max(0, s / len)), stressColor(Number.isFinite(vals[k]) ? Math.min(1, Math.abs(vals[k]) / scale) : 0)));
  c.beginPath();
  c.moveTo(X(0), y0);
  d.s.forEach((s, k) => c.lineTo(X(s), Y(Number.isFinite(vals[k]) ? vals[k] : 0)));
  c.lineTo(X(len), y0);
  c.closePath();
  c.globalAlpha = 0.45;
  c.fillStyle = grad;
  c.fill();
  c.globalAlpha = 1;
  c.strokeStyle = '#9a9a92';
  c.beginPath();
  c.moveTo(L, Math.round(Y(0)) + 0.5);
  c.lineTo(W - R, Math.round(Y(0)) + 0.5);
  c.stroke();
  c.strokeStyle = '#3b6ea8';
  c.lineWidth = 1.6;
  c.beginPath();
  d.s.forEach((s, k) => (k ? c.lineTo(X(s), Y(vals[k])) : c.moveTo(X(s), Y(vals[k]))));
  c.stroke();
  c.restore();
  // the value under the mouse
  if (hoverX !== null && hoverX >= L && hoverX <= W - R) {
    const s = ((hoverX - L) / pw) * len;
    const v = valueAt(d, vals, s);
    c.strokeStyle = '#e07b39';
    c.lineWidth = 1;
    c.beginPath();
    c.moveTo(hoverX, T);
    c.lineTo(hoverX, H - B);
    c.stroke();
    const py = Y(v);
    if (Number.isFinite(v) && py >= T && py <= H - B) {
      c.fillStyle = '#e07b39';
      c.beginPath();
      c.arc(hoverX, py, 3.5, 0, Math.PI * 2);
      c.fill();
    }
    const text = `s = ${s.toFixed(3)} m   ${fmt3(v)} ${unit}`;
    c.font = '600 11px system-ui, sans-serif';
    const w = c.measureText(text).width + 12;
    const bx = Math.min(Math.max(hoverX + 8, L), W - R - w);
    const by = Math.max(T, Math.min((Number.isFinite(py) ? py : T) - 26, H - B - 22));
    c.fillStyle = 'rgba(255,255,255,0.94)';
    c.strokeStyle = '#e07b39';
    c.beginPath();
    c.roundRect(bx, by, w, 20, 5);
    c.fill();
    c.stroke();
    c.fillStyle = '#2b2d31';
    c.textAlign = 'left';
    c.textBaseline = 'middle';
    c.fillText(text, bx + 6, by + 10);
  }
  return { a: 0, b: len, t0: 0, t1: len, lo, hi, L, R, T, B, W, H };
}

/** Axis label for a phase plot: the quantity's symbol (plain text) and unit. */
const phaseLabel = (m: Measure) => `${m.sym.replace(/<sub>(.*?)<\/sub>/g, '$1').replace(/<[^>]+>/g, '')}${m.unit ? ` (${m.unit})` : ''}`;

/**
 * A phase plot: the path the quantities have traced over the last TRAIL_SECONDS (as long as a
 * joint's trail), fading with age, and a dot at the state now. Two quantities: x against y, on
 * axes with gridlines. Three: a turnable 3D box (drag to turn), each axis scaled to the path.
 * Hovering shows the time and the values at the nearest point of the path.
 */
function drawPhase(cv: HTMLCanvasElement, t: number[], series: { m: Measure; vals: number[] }[], rot: { yaw: number; pitch: number }, hx: number | null, hy: number | null) {
  const dpr = window.devicePixelRatio || 1;
  const [W, H] = [cv.clientWidth, cv.clientHeight];
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) [cv.width, cv.height] = [Math.round(W * dpr), Math.round(H * dpr)];
  const c = cv.getContext('2d')!;
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, W, H);
  const n = Math.min(t.length, ...series.map((x) => x.vals.length));
  if (n < 1) return;
  const tNow = t[n - 1];
  let i0 = n - 1;
  while (i0 > 0 && t[i0 - 1] >= tNow - TRAIL_SECONDS) i0--;
  // each axis's range over the path shown
  const ranges = series.map((x) => {
    let [lo, hi] = [Infinity, -Infinity];
    for (let i = i0; i < n; i++) if (Number.isFinite(x.vals[i])) [lo, hi] = [Math.min(lo, x.vals[i]), Math.max(hi, x.vals[i])];
    if (!Number.isFinite(lo)) [lo, hi] = [-1, 1];
    const mag = Math.max(Math.abs(lo), Math.abs(hi));
    if (hi - lo <= mag * 1e-6) {
      const d = mag > 0 ? mag * 0.05 : 1e-3;
      [lo, hi] = [lo - d, hi + d];
    }
    const pad = (hi - lo) * 0.06;
    return [lo - pad, hi + pad] as const;
  });
  const three = series.length === 3;
  let project: (i: number) => [number, number];
  c.font = '10px system-ui, sans-serif';
  if (!three) {
    const [L, R, T, B] = [58, 12, 10, 34];
    const [[xl, xh], [yl, yh]] = ranges;
    const X = (v: number) => L + ((v - xl) / (xh - xl)) * (W - L - R);
    const Y = (v: number) => T + ((yh - v) / (yh - yl)) * (H - T - B);
    project = (i) => [X(series[0].vals[i]), Y(series[1].vals[i])];
    // gridlines and labels
    c.strokeStyle = '#e8e8e2';
    c.fillStyle = '#7b7e86';
    c.lineWidth = 1;
    const ys = niceStep(yh - yl, 4);
    c.textAlign = 'right';
    c.textBaseline = 'middle';
    for (let v = Math.ceil(yl / ys) * ys; v <= yh; v += ys) {
      const y = Math.round(Y(v)) + 0.5;
      c.beginPath();
      c.moveTo(L, y);
      c.lineTo(W - R, y);
      c.stroke();
      c.fillText(Math.abs(v) < ys * 1e-6 ? '0' : `${+v.toPrecision(4)}`, L - 5, y);
    }
    const xs = niceStep(xh - xl, Math.max(2, Math.floor((W - L - R) / 70)));
    c.textAlign = 'center';
    c.textBaseline = 'top';
    for (let v = Math.ceil(xl / xs) * xs; v <= xh; v += xs) {
      const x = Math.round(X(v)) + 0.5;
      c.beginPath();
      c.moveTo(x, T);
      c.lineTo(x, H - B);
      c.stroke();
      c.fillText(Math.abs(v) < xs * 1e-6 ? '0' : `${+v.toPrecision(4)}`, x, H - B + 3);
    }
    c.fillStyle = '#2b2d31';
    c.font = 'italic 600 10.5px system-ui, sans-serif';
    c.fillText(phaseLabel(series[0].m), (L + W - R) / 2, H - 13);
    c.save();
    c.translate(10, (T + H - B) / 2);
    c.rotate(-Math.PI / 2);
    c.textBaseline = 'middle';
    c.fillText(phaseLabel(series[1].m), 0, 0);
    c.restore();
  } else {
    // a unit cube, each axis scaled to its range, turned by yaw and pitch, seen straight on
    const norm = (k: number, v: number) => (2 * (v - ranges[k][0])) / (ranges[k][1] - ranges[k][0]) - 1;
    const [cy, sy, cp, sp] = [Math.cos(rot.yaw), Math.sin(rot.yaw), Math.cos(rot.pitch), Math.sin(rot.pitch)];
    const size = Math.min(W, H) * 0.3;
    const [ox, oy] = [W / 2, H / 2 + 4];
    // the world's z is up on screen: turn about it (yaw), then tilt towards the viewer (pitch)
    const P = (x: number, y: number, z: number): [number, number] => {
      const [x1, y1] = [x * cy - y * sy, x * sy + y * cy];
      const [y2, z2] = [y1 * cp - z * sp, y1 * sp + z * cp];
      void y2;
      return [ox + x1 * size, oy - z2 * size];
    };
    project = (i) => P(norm(0, series[0].vals[i]), norm(1, series[1].vals[i]), norm(2, series[2].vals[i]));
    c.strokeStyle = '#e2e2dc';
    c.lineWidth = 1;
    const corners = [-1, 1];
    for (const a of corners)
      for (const b of corners) {
        for (const [p, q] of [
          [P(-1, a, b), P(1, a, b)],
          [P(a, -1, b), P(a, 1, b)],
          [P(a, b, -1), P(a, b, 1)],
        ])
          seg(c, p, q);
      }
    // the three axes from the low corner, labelled at their ends with their ranges
    const axes: [string, [number, number, number], number][] = [
      [phaseLabel(series[0].m), [1, -1, -1], 0],
      [phaseLabel(series[1].m), [-1, 1, -1], 1],
      [phaseLabel(series[2].m), [-1, -1, 1], 2],
    ];
    const o = P(-1, -1, -1);
    c.strokeStyle = '#9a9ca3';
    c.lineWidth = 1.4;
    for (const [label, end, k] of axes) {
      const e = P(...end);
      seg(c, o, e);
      c.fillStyle = '#2b2d31';
      c.font = 'italic 600 10.5px system-ui, sans-serif';
      c.textAlign = 'center';
      c.textBaseline = 'middle';
      const [dx, dy] = [e[0] - o[0], e[1] - o[1]];
      const d = Math.hypot(dx, dy) || 1;
      c.fillText(label, e[0] + (dx / d) * 16, e[1] + (dy / d) * 12);
      c.fillStyle = '#7b7e86';
      c.font = '9.5px system-ui, sans-serif';
      c.fillText(`${+ranges[k][1].toPrecision(3)}`, e[0] - (dy / d) * 10, e[1] + (dx / d) * 10);
    }
    c.fillStyle = '#7b7e86';
    c.font = '9.5px system-ui, sans-serif';
    c.textAlign = 'right';
    c.fillText('drag to turn · double-click to reset', W - 6, H - 8);
  }
  // the path, fading with age, at most a few thousand points
  const step = Math.max(1, Math.floor((n - i0) / 3000));
  const chunks = 24;
  const len = n - i0;
  c.lineWidth = 1.6;
  c.lineJoin = 'round';
  for (let k = 0; k < chunks; k++) {
    const a = i0 + Math.floor((len * k) / chunks);
    const b = Math.min(n - 1, i0 + Math.floor((len * (k + 1)) / chunks));
    if (b <= a) continue;
    c.strokeStyle = `rgba(59,110,168,${0.12 + 0.88 * ((k + 1) / chunks) ** 1.4})`;
    c.beginPath();
    let first = true;
    for (let i = a; i <= b; i = i + step > b && i < b ? b : i + step) {
      const [x, y] = project(i);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      if (first) c.moveTo(x, y);
      else c.lineTo(x, y);
      first = false;
    }
    c.stroke();
  }
  // now
  const [nx, ny] = project(n - 1);
  if (Number.isFinite(nx) && Number.isFinite(ny)) {
    c.fillStyle = '#e07b39';
    c.beginPath();
    c.arc(nx, ny, 4, 0, Math.PI * 2);
    c.fill();
  }
  // hover: the nearest point of the path, its time and values
  if (hx !== null && hy !== null) {
    let [best, bd] = [-1, 14];
    for (let i = i0; i < n; i += step) {
      const [x, y] = project(i);
      const d = Math.hypot(x - hx, y - hy);
      if (d < bd) [best, bd] = [i, d];
    }
    if (best >= 0) {
      const [x, y] = project(best);
      c.strokeStyle = '#e07b39';
      c.lineWidth = 1.5;
      c.beginPath();
      c.arc(x, y, 5, 0, Math.PI * 2);
      c.stroke();
      const text = `t = ${t[best].toFixed(3)} s   ${series.map((sr) => `${(sr.m.fmt ?? fmt3)(sr.vals[best])}`).join(', ')}`;
      c.font = '600 11px system-ui, sans-serif';
      const w = c.measureText(text).width + 12;
      const bx = Math.min(Math.max(x + 10, 4), W - w - 4);
      const by = Math.max(4, Math.min(y - 28, H - 26));
      c.fillStyle = 'rgba(255,255,255,0.94)';
      c.beginPath();
      c.roundRect(bx, by, w, 20, 5);
      c.fill();
      c.stroke();
      c.fillStyle = '#2b2d31';
      c.textAlign = 'left';
      c.textBaseline = 'middle';
      c.fillText(text, bx + 6, by + 10);
    }
  }
}

/** A straight line on a canvas. */
function seg(c: CanvasRenderingContext2D, p: [number, number], q: [number, number]) {
  c.beginPath();
  c.moveTo(p[0], p[1]);
  c.lineTo(q[0], q[1]);
  c.stroke();
}

/** A round step for about `n` ticks across `span`. */
function niceStep(span: number, n: number): number {
  const raw = span / n;
  const p = 10 ** Math.floor(Math.log10(raw || 1));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
}

/**
 * The graph: value against time over the view's window (the latest seconds while following),
 * with gridlines and labels, and a cursor line where the mouse is. Returns the layout, for the
 * mouse.
 */
function drawGraph(cv: HTMLCanvasElement, t: number[], vals: number[], m: Measure, view: GraphView, hoverX: number | null, loading = false): GraphLayout | null {
  const dpr = window.devicePixelRatio || 1;
  const [W, H] = [cv.clientWidth, cv.clientHeight];
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) [cv.width, cv.height] = [Math.round(W * dpr), Math.round(H * dpr)];
  const c = cv.getContext('2d')!;
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, W, H);
  const [L, R, T, B] = [58, 10, 8, 22];
  const n = Math.min(t.length, vals.length);
  if (n < 1) return null;
  const { a, b, t0, t1, find, i0, i1 } = graphWindow(t, n, view);
  // the value range: fitted to what is visible, unless set by the user
  let [lo, hi] = [view.lo, view.hi];
  if (view.yAuto) {
    [lo, hi] = [Infinity, -Infinity];
    for (let i = i0; i <= i1; i++) if (Number.isFinite(vals[i])) [lo, hi] = [Math.min(lo, vals[i]), Math.max(hi, vals[i])];
    if (!Number.isFinite(lo)) [lo, hi] = [-1, 1];
    // a flat line gets some room, relative to its size (tiny values such as max|Φ| keep their own scale)
    const mag = Math.max(Math.abs(lo), Math.abs(hi));
    if (hi - lo <= mag * 1e-6) {
      const d = mag > 0 ? mag * 0.05 : 1e-3;
      [lo, hi] = [lo - d, hi + d];
    }
    const pad = (hi - lo) * 0.08;
    [lo, hi] = [lo - pad, hi + pad];
  }
  const pw = W - L - R;
  const X = (tt: number) => L + ((tt - a) / (b - a)) * pw;
  const Y = (v: number) => T + ((hi - v) / (hi - lo)) * (H - T - B);
  // gridlines and labels
  c.font = '10px system-ui, sans-serif';
  c.strokeStyle = '#e8e8e2';
  c.fillStyle = '#7b7e86';
  c.lineWidth = 1;
  const ys = niceStep(hi - lo, 4);
  c.textAlign = 'right';
  c.textBaseline = 'middle';
  for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) {
    const y = Math.round(Y(v)) + 0.5;
    c.beginPath();
    c.moveTo(L, y);
    c.lineTo(W - R, y);
    c.stroke();
    c.fillText(Math.abs(v) < ys * 1e-6 ? '0' : `${+v.toPrecision(4)}`, L - 5, y);
  }
  const xs = niceStep(b - a, Math.max(2, Math.floor(pw / 70)));
  c.textAlign = 'center';
  c.textBaseline = 'top';
  for (let tt = Math.ceil(a / xs) * xs; tt <= b + 1e-9; tt += xs) {
    const x = Math.round(X(tt)) + 0.5;
    c.beginPath();
    c.moveTo(x, T);
    c.lineTo(x, H - B);
    c.stroke();
    c.fillText(`${+tt.toPrecision(4)} s`, x, H - B + 4);
  }
  if (lo < 0 && hi > 0) {
    c.strokeStyle = '#c9c9c0';
    c.beginPath();
    c.moveTo(L, Math.round(Y(0)) + 0.5);
    c.lineTo(W - R, Math.round(Y(0)) + 0.5);
    c.stroke();
  }
  // the curve, clipped to the plot: at most two points per pixel column (its lowest and highest)
  c.save();
  c.beginPath();
  c.rect(L, T, pw, H - T - B);
  c.clip();
  c.strokeStyle = '#3b6ea8';
  c.lineWidth = 1.6;
  c.beginPath();
  let started = false;
  const per = Math.max(1, Math.floor((i1 - i0 + 1) / (2 * Math.max(1, pw))));
  for (let i = i0; i <= i1; i += per) {
    // (values not worked out yet are holes: skipped, the line joins the known ones)
    let [imin, imax] = [-1, -1];
    for (let j = i; j < Math.min(i1 + 1, i + per); j++) {
      if (!Number.isFinite(vals[j])) continue;
      if (imin < 0 || vals[j] < vals[imin]) imin = j;
      if (imax < 0 || vals[j] > vals[imax]) imax = j;
    }
    if (imin < 0) continue;
    for (const j of imin < imax ? [imin, imax] : [imax, imin]) {
      if (!Number.isFinite(vals[j])) continue;
      if (!started) c.moveTo(X(t[j]), Y(vals[j]));
      else c.lineTo(X(t[j]), Y(vals[j]));
      started = true;
    }
  }
  c.stroke();
  c.restore();
  // still working out the values in view (a long run's joint forces): say so
  if (loading) {
    c.font = '600 10px system-ui, sans-serif';
    c.textAlign = 'left';
    c.textBaseline = 'top';
    c.fillStyle = '#9a9ca3';
    c.fillText('working out…', L + 4, T + 2);
  }
  // scrolled back from the current time: say so, at the top right
  if (view.end !== null) {
    c.font = '600 10px system-ui, sans-serif';
    c.textAlign = 'right';
    c.textBaseline = 'top';
    c.fillStyle = '#3b6ea8';
    c.fillText('◂ scrolled back · double-click for now', W - R - 2, T + 2);
  }
  // the cursor line: the value at the time under the mouse (else the latest shown)
  let k = Math.min(i1, n - 1);
  if (hoverX !== null && hoverX >= L && hoverX <= W - R) {
    const tt = a + ((hoverX - L) / pw) * (b - a);
    const j = find(tt);
    k = j + 1 < n && Math.abs(t[j + 1] - tt) < Math.abs(t[j] - tt) ? j + 1 : j;
    c.strokeStyle = '#e07b39';
    c.lineWidth = 1;
    c.beginPath();
    c.moveTo(hoverX, T);
    c.lineTo(hoverX, H - B);
    c.stroke();
  }
  // the nearest value worked out, if that one is not yet
  for (let d = 0; !Number.isFinite(vals[k]) && d <= i1 - i0; d++) {
    if (k - d >= i0 && Number.isFinite(vals[k - d])) k -= d;
    else if (k + d <= i1 && Number.isFinite(vals[k + d])) k += d;
  }
  const [px, py] = [X(t[k]), Y(vals[k])];
  if (Number.isFinite(vals[k]) && px >= L - 1 && px <= W - R + 1 && py >= T && py <= H - B) {
    c.fillStyle = '#e07b39';
    c.beginPath();
    c.arc(px, py, 3.5, 0, Math.PI * 2);
    c.fill();
  }
  if (hoverX !== null && hoverX >= L && hoverX <= W - R && Number.isFinite(vals[k])) {
    const text = `t = ${t[k].toFixed(3)} s   ${(m.fmt ?? fmt3)(vals[k])} ${m.unit}`;
    c.font = '600 11px system-ui, sans-serif';
    const w = c.measureText(text).width + 12;
    const bx = Math.min(Math.max(hoverX + 8, L), W - R - w);
    const by = Math.max(T, Math.min((Number.isFinite(py) ? py : T) - 26, H - B - 22));
    c.fillStyle = 'rgba(255,255,255,0.94)';
    c.strokeStyle = '#e07b39';
    c.beginPath();
    c.roundRect(bx, by, w, 20, 5);
    c.fill();
    c.stroke();
    c.fillStyle = '#2b2d31';
    c.textAlign = 'left';
    c.textBaseline = 'middle';
    c.fillText(text, bx + 6, by + 10);
  }
  return { a, b, t0, t1, lo, hi, L, R, T, B, W, H };
}

function renderInspectorPanel() {
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
      <div class="field"><span></span> <span><button id="i-dup" title="Ctrl/⌘+D: the copy follows the cursor until you click to place it">Duplicate</button> <button id="i-copy">Copy</button> <button id="i-paste" ${clipboard ? '' : 'disabled'}>Paste</button> <button id="i-delete">Delete</button></span></div>
      <p class="muted">Drag any selected part to move them together. Ctrl/⌘+D duplicates them: the copy follows the cursor as an outline until you click to place it (Esc cancels). Ctrl/⌘+C copies them and Ctrl/⌘+V pastes a copy at the cursor, keeping the constraints among the copied parts (pins, rails, meshes, joins). Ctrl/⌘+X cuts, Delete removes, Esc deselects.</p>`;
    inspectorEl.querySelector('#i-copy')!.addEventListener('click', copyParts);
    inspectorEl.querySelector('#i-dup')!.addEventListener('click', () => startDuplicate());
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
      const candidates = design.gears.filter((g) => !g.wheel && !meshed.some((m) => m.gear === g.id));
      inspectorEl.innerHTML = `
        <h4>${l.rack ? 'Rack' : 'Bar'} ${linkLabel(l.id)} <span class="muted">(${pointLabel(l.a)}–${pointLabel(l.b)})</span></h4>
        <label class="field">Mass (kg)<input id="i-mass" type="number" min="0.01" step="0.1" value="${l.mass}" ${disabled}></label>
        <div class="field"><span>Length</span><span>${linkLength(design, l).toFixed(3)} m</span></div>
        ${
          l.rack
            ? ''
            : `<label class="field" title="Young's modulus, for drawing how it bends (Draw deflection); empty: the side panel's ${+(deflect.E / 1e9).toPrecision(6)} GPa">E (GPa)<input id="i-E" type="number" min="0" step="1" placeholder="${+(deflect.E / 1e9).toPrecision(6)}" value="${l.E !== undefined ? +(l.E / 1e9).toPrecision(6) : ''}"></label>
        <label class="field" title="Second moment of area of its section, for drawing how it bends; empty: the side panel's ${+(deflect.I * 1e8).toPrecision(6)} cm⁴">I (cm⁴)<input id="i-I" type="number" min="0" step="0.01" placeholder="${+(deflect.I * 1e8).toPrecision(6)}" value="${l.I !== undefined ? +(l.I * 1e8).toPrecision(6) : ''}"></label>`
        }
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
      // E and I only change how it is drawn bent: editable while it runs, no new equations
      for (const [sel, key, unit] of [['#i-E', 'E', 1e9], ['#i-I', 'I', 1e-8]] as const)
        inspectorEl.querySelector<HTMLInputElement>(sel)?.addEventListener('change', (e) => {
          const v = parseFloat((e.target as HTMLInputElement).value);
          pushUndo();
          if (Number.isFinite(v) && v > 0) l[key] = v * unit;
          else delete l[key];
          localStorage.setItem(STORAGE_KEY, JSON.stringify(persistable()));
          beamCache.clear();
        });
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
    <p class="muted">Select a joint, bar, rack, spring or gear to edit it. Press <b>Play</b> to derive Lagrange's equations and integrate them with RK4. While it runs, drag any part to pull it. Right-click for a search list of every tool, setting and example; right-drag pans, the wheel zooms.</p>`;
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
    (heat ? ` · ${statQ('W', `heat W = <b>${s.dissipated.toFixed(3)} J</b>`)}` : '') +
    (motors ? ` · ${statQ('Wm', `motor work W<sub>m</sub> = <b>${s.motorWork.toFixed(3)} J</b>`)}` : '') +
    (air ? ` · ${statQ('Wa', `air work W<sub>a</sub> = <b>${s.airWork.toFixed(3)} J</b>`)}` : '') +
    ` · ${statQ('budget', `E${heat ? ' + W' : ''}${motors ? ' − W<sub>m</sub>' : ''}${air ? ' − W<sub>a</sub>' : ''} = ${(T + V + s.dissipated - s.motorWork - s.airWork).toFixed(4)} J`)}` +
    (hottest > 0.01 ? ` · hottest: ${where} +${hottest.toFixed(1)} K` : '')
  );
}

/** An energy figure on the status bar, a button for its graph (highlighted while graphed). */
const statQ = (key: string, html: string) => `<button class="st-q${isGraphed('system', 0, key) ? ' on' : ''}" data-q="${key}" title="Graph it over time (shift-click: add it to a phase plot)">${html}</button>`;
// the status bar is redrawn every frame, so its buttons are caught as they are pressed
statusEl.addEventListener('pointerdown', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-q]');
  const sr = b && energySeries(b.dataset.q!);
  if (!sr || !sim) return;
  e.preventDefault();
  if (e.shiftKey) addSeries('system', 0, sr.m, sr.at);
  else if (graph && !graph.extra.length && graph.kind === 'system' && graph.key === sr.m.key) closeGraph();
  else openGraph('system', 0, sr.m, sr.at);
});

/** The last simulation error put in the message bar. */
let toldError: string | null = null;

function renderStatus() {
  const nBars = design.links.filter((l) => !l.rack).length;
  const nRacks = design.links.length - nBars;
  const nSprings = design.springs.length;
  const nWheels = design.gears.filter((g) => g.wheel).length;
  const nProps = design.gears.filter((g) => g.propeller).length;
  const nGears = design.gears.length - nWheels - nProps;
  if (simError) {
    statusEl.innerHTML = `<span class="err">${simError}</span>`;
    if (simError !== toldError) messages.notify(simError, 'error'); // up top as well, once
    toldError = simError;
    return;
  }
  toldError = null;
  if (!sim) {
    const dof = derivation ? derivation.coords.length - derivation.constraints.length : 0;
    statusEl.innerHTML = `Editing · <b>${nBars}</b> bar${nBars === 1 ? '' : 's'}${nRacks ? ` · <b>${nRacks}</b> rack${nRacks === 1 ? '' : 's'}` : ''}${nSprings ? ` · <b>${nSprings}</b> spring${nSprings === 1 ? '' : 's'}` : ''}${nGears ? ` · <b>${nGears}</b> gear${nGears === 1 ? '' : 's'}` : ''}${nWheels ? ` · <b>${nWheels}</b> wheel${nWheels === 1 ? '' : 's'}` : ''}${nProps ? ` · <b>${nProps}</b> propeller${nProps === 1 ? '' : 's'}` : ''} · <b>${dof}</b> degree${dof === 1 ? '' : 's'} of freedom${hasInitialRates() ? ' · starts moving (initial velocities set)' : ''}`;
    return;
  }
  const { T, V } = sim.energy();
  const F = Math.hypot(...grabPull);
  const phi = sim.constraintError();
  statusEl.innerHTML =
    `${running ? 'Running' : 'Paused'} · t = <b>${sim.t.toFixed(2)} s</b> · ${statQ('E', `E = T + V = <b>${(T + V).toFixed(4)} J</b>`)}` +
    (sim.sys.nc ? ` · ${statQ('phi', `max|Φ| = ${phi.toExponential(1)}`)}` : '') +
    (grab ? ` · |F| = <b>${F.toFixed(1)} N</b>` : '') +
    (sim.ground ? ` · ${sim.contactCount()} touching the ground` : '') +
    (friction.enabled || sim.dissipated > 0 || sim.sys.derivation.motors.length ? heatStatus(sim) : '') +
    ` · ${steppingText()}`;
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
  // whatever goes wrong in a frame, the next one still comes (else the whole page stops)
  try {
    drawFrame(now);
  } catch (e) {
    console.error(e);
    if (sim && running) {
      running = false;
      simError = `Something went wrong while it ran (${errorText(e)}). Press Reset to edit.`;
      refreshControls();
    }
  }
  requestAnimationFrame(frame);
}

function drawFrame(now: number) {
  const dt = Math.min((now - last) / 1000, 1 / 20);
  last = now;
  const target = trailTarget();
  if (target !== trail.id) resetTrail(target);
  if (sim && running) {
    advance(dt * settings.speed);
    recordState();
    checkBreaks();
    if (settings.follow) followMechanism();
  }
  sound.update(sim && running ? machineState(sim) : null);
  sound.ambient(backgroundHeard() ? backdrop.soundscape : null);
  if (sim) {
    updateMeasures();
    updateGraph();
  }
  if (ic && !sim) pullInitial(dt);
  if (selection?.kind === 'pid' && sim) {
    const live = inspectorEl.querySelector('#i-plive');
    const g = design.gears.find((x) => x.id === selection!.id);
    if (live && g?.motor?.pid) live.textContent = pidLiveText(g);
  }

  let grabScene = null;
  grabPull = [0, 0];
  const fbd = forceArrowsNow();
  if (sim && grab) {
    const a = pos(grab.a);
    const b = pos(grab.b);
    const gr = grab.gear && design.gears.find((x) => x.id === grab!.gear!.id);
    let at: Vec = gr ? gearPoint(gr, grab.gear!.u, grab.gear!.v) : [a[0] + (b[0] - a[0]) * grab.s, a[1] + (b[1] - a[1]) * grab.s];
    // the arrow is the force the pull applies, as recorded for this frame (spring and damper: the
    // free-body diagram's "your pull" is the very same); the dashed line runs to the cursor
    const pull = running ? history.loads[history.loads.length - 1]?.points.find((p) => p.type === 'pull') : undefined;
    if (pull) [grabPull, at] = [[pull.F[0], pull.F[1]], [pull.at[0], pull.at[1]]];
    // the measured part's free-body diagram draws it already, to its own scale: not twice
    grabScene = { at, to: mouseWorld, force: fbdHasPull ? ([0, 0] as Vec) : grabPull };
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
    arrows: sim ? null : initialArrows(),
    cursorLabel: ic && !sim ? { at: mouseScreen, text: ic.readout } : null,
    grid: gridMode() && (!!dragging || !!groundDrag || !!drawing || !!duplicating || !!constrain?.ends || ['bar', 'rack', 'spring', 'gear', 'wheel', 'propeller'].includes(tool)), // only where things snap
    dimensionDigits: dimensionDigits(),
    ghost: duplicating && !sim ? (([dx, dy]) => ({ design: duplicating!.frag, dx, dy }))(duplicateOffset()) : null,
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
    forceArrows: fbd,
    backdrop: (c, w, h, v) => backdrop.draw(c, w, h, v),
    ink: inkFor(backdrop.state, PAGE_COLOR),
    overlay: bgEditor.active ? (c, w, h, v) => bgEditor.drawOverlay(c, w, h, v) : null,
    beamShades: beamShadesNow(),
    bends: bendsNow(),
  });
  renderStatus();
  blockSummaries();
}

window.addEventListener('resize', resize);
resize();
fitView();
deriveInBackground();
refreshControls();
requestAnimationFrame(frame);

// Exposed for debugging in the browser console.
Object.assign(window, { linkageLab: { get design() { return design; }, get derivation() { return derivation; }, get sim() { return sim; }, toScreen: (v: Vec) => toScreen(view, v) } });
