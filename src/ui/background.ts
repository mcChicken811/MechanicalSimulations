/**
 * Backgrounds behind the drawing, in layers at a distance: a scene drawn by the app (sky with
 * clouds, a city, a night sky, a sunset, mountains, a blueprint, graph paper), or a custom one,
 * a scene to build on with images of the user's own placed over it.
 *
 * Every layer lies at a distance z from the viewer, in units of the drawing's own distance:
 *  - z = 1 moves with the drawing exactly (an image to trace over, at its true size);
 *  - z = 2 moves half as far when the view pans, and zooms less, as something twice as far away
 *    does; and so on: parallax, moving by 1/z of the view;
 *  - z = ∞ (null) stays put: a sky.
 * For a layer placed with the view centred on c₀ at s₀ px/m, a point p of it (m, in its own
 * plane as if at z = 1 then) is drawn at
 *     screen = centre + (p − c₀) s₀ f − (c − c₀) s / z,   f = (s/s₀)^(1/z),
 * for the view now centred on c at s px/m: for z = 1 that is the drawing's own map, for z = ∞
 * the place it was put on the screen. The scenes' layers are at the distances their parts would
 * be (clouds far, near buildings close), each a tile in the scene's own space placed in the world
 * the same way (so they zoom with the view as well as pan, by their distance), repeating sideways
 * and running on below, so the view never runs out of them.
 *
 * The state is plain data (the images as data URLs), saved with projects.
 */
import { toWorld, type View } from './render';
import { Weather, type WeatherMode } from './weather';

/** An image layer of a custom background. */
export interface ImageLayer {
  id: string;
  name: string;
  /** The image, as a data URL, and its shape (height over width). */
  image: string;
  aspect: number;
  /** Its distance (1: with the drawing; null: infinitely far, fixed on the screen). */
  depth: number | null;
  /** The view it was placed in: its centre (m) and scale (px/m). */
  ref: { cx: number; cy: number; s: number };
  /** Its top-left corner and width (m), in that view's frame. */
  rect: { x: number; y: number; w: number };
}

/**
 * The background brought to life: the time of day turning (the sky, the sun and moon, stars,
 * the scenery lit warm at dawn and dusk and dark at night, windows lit after dark), clouds
 * drifting, birds flocking, shooting stars at night.
 */
export interface BackgroundAnim {
  on: boolean;
  /** How long a day lasts (s); 0: the time stands still. */
  dayLength: number;
  /** The time of day (hours, 0–24) when it was set. */
  hour: number;
  birds: boolean;
  /** The weather (ui/weather.ts): changing by itself, or one kind all the time; and whether it is heard. */
  weather: WeatherMode;
  weatherSound: boolean;
}

export const DEFAULT_ANIM: BackgroundAnim = { on: true, dayLength: 300, hour: 10, birds: true, weather: 'random', weatherSound: true };

export interface BackgroundState {
  /** 'none', a scene's id, or 'custom' (the scene `base` with the `layers` over it). */
  kind: string;
  base: string;
  layers: ImageLayer[];
  /** How strongly it shows, 0–1. */
  opacity: number;
  anim: BackgroundAnim;
}

export const NO_BACKGROUND: BackgroundState = { kind: 'none', base: 'none', layers: [], opacity: 0.7, anim: { ...DEFAULT_ANIM } };

// ---- the time of day: the sky's colours, the sun and moon, how dark and how warm it is ----
/** The sky through the day: hour → colours at the top, the middle and the horizon. */
const SKY_KEYS: [number, string, string, string][] = [
  [0, '#08122b', '#13234a', '#1f3462'],
  [4.5, '#0d1a3c', '#1d2d5c', '#34477a'],
  [5.8, '#2d3a6e', '#6a5f93', '#e48b6a'],
  [6.8, '#5f8fcf', '#a8c3e3', '#fbcf9a'],
  [8.5, '#4f9be3', '#9fcff3', '#e3f2fb'],
  [12, '#4796e2', '#99ccf3', '#e6f4fc'],
  [16.5, '#5592d6', '#a2cbec', '#f4e0bc'],
  [18, '#5e6fb0', '#c98fa0', '#f59a62'],
  [19.2, '#2b2f63', '#6b4f7f', '#c0647a'],
  [20.5, '#0f1a3e', '#1d2b58', '#2f3e6c'],
  [24, '#08122b', '#13234a', '#1f3462'],
];

/** A colour's red, green and blue: from #rrggbb or rgb(r,g,b). */
const hex = (c: string) => (c.startsWith('#') ? [1, 3, 5].map((i) => parseInt(c.slice(i, i + 2), 16)) : c.match(/\d+/g)!.slice(0, 3).map(Number));
const mix = (a: string, b: string, t: number) => {
  const [x, y] = [hex(a), hex(b)];
  return `rgb(${x.map((v, i) => Math.round(v + (y[i] - v) * t)).join(',')})`;
};

/** The sky's three colours at an hour. */
function skyAt(hour: number): [string, string, string] {
  const h = ((hour % 24) + 24) % 24;
  let k = 0;
  while (k < SKY_KEYS.length - 2 && SKY_KEYS[k + 1][0] <= h) k++;
  const [a, b] = [SKY_KEYS[k], SKY_KEYS[k + 1]];
  const t = (h - a[0]) / (b[0] - a[0] || 1);
  const ease = t * t * (3 - 2 * t);
  return [mix(a[1], b[1], ease), mix(a[2], b[2], ease), mix(a[3], b[3], ease)];
}

const ramp = (x: number, a: number, b: number) => Math.min(1, Math.max(0, (x - a) / (b - a)));
/** How dark it is, 0 (day) to 1 (night). */
export function darkness(hour: number): number {
  const h = ((hour % 24) + 24) % 24;
  return h < 12 ? 1 - ramp(h, 5, 7.2) : ramp(h, 17.6, 20.4);
}
/** How warm the light is (dawn and dusk), 0–1. */
function warmth(hour: number): number {
  const h = ((hour % 24) + 24) % 24;
  return Math.max(Math.exp(-(((h - 6.4) / 0.8) ** 2)), Math.exp(-(((h - 18.4) / 0.9) ** 2)));
}

/** Time of day in words: 10:30. */
export const clockText = (hour: number) => {
  const h = ((hour % 24) + 24) % 24;
  return `${Math.floor(h)}:${String(Math.floor((h % 1) * 60)).padStart(2, '0')}`;
};

// ---- birds: a flock by Reynolds' boids rules, in the scene's space at a distance ----
interface Bird {
  x: number;
  y: number;
  vx: number;
  vy: number;
  flap: number;
  size: number;
}

/**
 * A flock: each bird steers by what its neighbours do (Reynolds 1987). It keeps clear of those
 * too close (separation), heads the way those near it head (alignment) and towards their middle
 * (cohesion), stays in the sky's band of the scene and in view (sideways, it wraps round what is
 * seen), and wanders a little; its speed kept between a glide and a dash.
 */
class Flock {
  birds: Bird[] = [];
  private rnd = seeded(77);

  spawn(n: number, u0: number, u1: number) {
    const cx = (u0 + u1) / 2;
    this.birds = Array.from({ length: n }, () => ({ x: cx + (this.rnd() - 0.5) * 220, y: 150 + this.rnd() * 120, vx: 45 + this.rnd() * 20, vy: (this.rnd() - 0.5) * 10, flap: this.rnd() * 6.28, size: 0.8 + this.rnd() * 0.5 }));
  }

  /** One step of dt s; u0…u1 the units in view across, the sky's band top…bottom down. */
  step(dt: number, u0: number, u1: number, top: number, bottom: number) {
    const B = this.birds;
    const [SEP, ALIGN, COH] = [24, 70, 95];
    for (const b of B) {
      let [sx, sy, ax, ay, cx, cy, na, nc] = [0, 0, 0, 0, 0, 0, 0, 0];
      for (const o of B) {
        if (o === b) continue;
        const [dx, dy] = [o.x - b.x, o.y - b.y];
        const d = Math.hypot(dx, dy);
        if (d < SEP && d > 0) [sx, sy] = [sx - dx / d / d, sy - dy / d / d];
        if (d < ALIGN) [ax, ay, na] = [ax + o.vx, ay + o.vy, na + 1];
        if (d < COH) [cx, cy, nc] = [cx + o.x, cy + o.y, nc + 1];
      }
      let [fx, fy] = [sx * 1800, sy * 1800]; // separation
      if (na) [fx, fy] = [fx + (ax / na - b.vx) * 0.9, fy + (ay / na - b.vy) * 0.9]; // alignment
      if (nc) [fx, fy] = [fx + (cx / nc - b.x) * 0.35, fy + (cy / nc - b.y) * 0.35]; // cohesion
      // stay in the sky's band (a soft wall), and wander
      if (b.y < top) fy += (top - b.y) * 2;
      if (b.y > bottom) fy -= (b.y - bottom) * 2;
      fx += (this.rnd() - 0.5) * 30;
      fy += (this.rnd() - 0.5) * 30;
      b.vx += fx * dt;
      b.vy += fy * dt;
      const v = Math.hypot(b.vx, b.vy) || 1;
      const k = v > 85 ? 85 / v : v < 38 ? 38 / v : 1;
      [b.vx, b.vy] = [b.vx * k, b.vy * k * (1 - 1.5 * dt)]; // (birds fly level more than not)
    }
    for (const b of B) {
      b.x += b.vx * dt;
      b.y += b.vy * dt;
      b.flap += dt * (9 + Math.abs(b.vy) * 0.1);
      // round the view sideways: out on one side, in on the other
      const span = u1 - u0 + 160;
      if (b.x > u1 + 80) b.x -= span;
      if (b.x < u0 - 80) b.x += span;
    }
  }
}

/** A small seeded random generator (mulberry32): the same scene every time. */
function seeded(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const vertical = (c: CanvasRenderingContext2D, h: number, stops: [number, string][]) => {
  const g = c.createLinearGradient(0, 0, 0, h);
  for (const [at, col] of stops) g.addColorStop(at, col);
  return g;
};

/** A soft cloud of overlapping puffs, about `size` px across, its base at y. */
function cloud(c: CanvasRenderingContext2D, x: number, y: number, size: number, rnd: () => number, tint = '255,255,255') {
  const puffs = 5 + Math.floor(rnd() * 4);
  c.fillStyle = `rgba(${tint},0.92)`;
  c.beginPath();
  for (let i = 0; i < puffs; i++) {
    const u = i / (puffs - 1) - 0.5;
    const r = size * (0.16 + 0.14 * Math.sin(Math.PI * (u + 0.5)) + 0.05 * rnd());
    c.moveTo(x + u * size * 0.8 + r, y - r * 0.6);
    c.arc(x + u * size * 0.8, y - r * 0.6, r, 0, Math.PI * 2);
  }
  c.fill();
  c.fillStyle = 'rgba(160,180,205,0.18)';
  c.beginPath();
  c.ellipse(x, y - size * 0.04, size * 0.46, size * 0.07, 0, 0, Math.PI * 2);
  c.fill();
}

/** Rolling hills: a smooth band of colour from height `base` (fraction of h) down, seamless across w. */
function hills(c: CanvasRenderingContext2D, w: number, h: number, base: number, amp: number, color: string, rnd: () => number) {
  const k = 2 + Math.floor(rnd() * 3); // whole waves across: it repeats without a seam
  const ph = rnd() * Math.PI * 2;
  c.fillStyle = color;
  c.beginPath();
  c.moveTo(0, h);
  for (let x = 0; x <= w; x += 6) c.lineTo(x, h * base - amp * h * (0.5 + 0.5 * Math.sin((x / w) * Math.PI * 2 * k + ph)) * (0.75 + 0.25 * Math.sin((x / w) * Math.PI * 2 * (k + 2) + ph * 2)));
  c.lineTo(w, h);
  c.closePath();
  c.fill();
}

/** Clouds across a strip, repeating every w. */
function cloudStrip(c: CanvasRenderingContext2D, w: number, h: number, seed: number, count: number, top: number, spread: number, tint?: string) {
  const rnd = seeded(seed);
  for (let i = 0; i < count; i++) {
    const size = 110 + 120 * rnd();
    const [x, y] = [((i + 0.3 + 0.5 * rnd()) / count) * w, Math.max(size * 0.45, h * (top + spread * rnd()))]; // all of it inside its tile
    for (const dx of [-w, 0, w]) cloud(c, x + dx, y, size, seeded(seed * 31 + i), tint); // across the seam too
  }
}

/** A layer of a scene: its distance (null: fixed), its painter (w × h px), and the colour it runs on with below. */
interface SceneLayer {
  depth: number | null;
  paint: (c: CanvasRenderingContext2D, w: number, h: number) => void;
  below?: string;
  /** The sky itself: when the day turns (animated), the moving sky is drawn in its place. */
  sky?: boolean;
  /** Drifting (animated): units per second, sideways. */
  drift?: number;
  /** Drawn over the layer by night (animated), as bright as it is dark: lit windows. */
  glow?: (c: CanvasRenderingContext2D, w: number, h: number) => void;
}

interface SceneDef {
  name: string;
  strength: number;
  layers: SceneLayer[];
  /** Animated: whether its sky turns with the time of day, whether birds fly in it, whether it has weather. */
  cycle?: boolean;
  birds?: boolean;
  weather?: boolean;
}

const SCENE_DEFS: Record<string, SceneDef> = {
  sky: {
    name: 'Blue sky with clouds',
    weather: true,
    strength: 0.75,
    cycle: true,
    birds: true,
    layers: [
      { depth: null, sky: true, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#5aa5e6'], [0.55, '#a9d4f5'], [1, '#eaf6fd']])), c.fillRect(0, 0, w, h)) },
      { depth: 14, drift: 5, paint: (c, w, h) => cloudStrip(c, w, h, 7, Math.max(3, Math.round(w / 420)), 0.12, 0.25) },
      { depth: 6, drift: 9, paint: (c, w, h) => cloudStrip(c, w, h, 9, Math.max(3, Math.round(w / 380)), 0.38, 0.3) },
    ],
  },
  city: {
    name: 'City skyline',
    weather: true,
    strength: 0.7,
    cycle: true,
    birds: true,
    layers: [
      { depth: null, sky: true, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#8fbbe6'], [0.6, '#d9e6f2'], [1, '#f6e3c8']])), c.fillRect(0, 0, w, h)) },
      { depth: 16, drift: 6, paint: (c, w, h) => cloudStrip(c, w, h, 11, 2, 0.12, 0.12) },
      { depth: 6, paint: (c, w, h) => buildings(c, w, h, 21, '#b4c3d4', 0.42, false), below: '#b4c3d4', glow: (c, w, h) => buildings(c, w, h, 21, '#b4c3d4', 0.42, false, true) },
      { depth: 3, paint: (c, w, h) => buildings(c, w, h, 22, '#73859c', 0.55, true), below: '#73859c', glow: (c, w, h) => buildings(c, w, h, 22, '#73859c', 0.55, true, true) },
    ],
  },
  night: {
    name: 'Night sky',
    weather: true,
    strength: 0.55,
    layers: [
      {
        depth: null,
        paint: (c, w, h) => {
          const rnd = seeded(23);
          c.fillStyle = vertical(c, h, [[0, '#0b1633'], [0.7, '#1f3462'], [1, '#3b4f7e']]);
          c.fillRect(0, 0, w, h);
          for (let i = 0; i < (w * h) / 2600; i++) {
            c.fillStyle = `rgba(255,255,240,${0.35 + 0.65 * rnd()})`;
            c.beginPath();
            c.arc(rnd() * w, rnd() * h * 0.85, 0.4 + 1.1 * rnd() ** 3, 0, Math.PI * 2);
            c.fill();
          }
          const [mx, my, r] = [w * 0.8, h * 0.18, Math.min(w, h) * 0.05];
          const glow = c.createRadialGradient(mx, my, r, mx, my, r * 4);
          glow.addColorStop(0, 'rgba(255,250,220,0.25)');
          glow.addColorStop(1, 'rgba(255,250,220,0)');
          c.fillStyle = glow;
          c.fillRect(mx - r * 4, my - r * 4, r * 8, r * 8);
          c.fillStyle = '#f7f1d8';
          c.beginPath();
          c.arc(mx, my, r, 0, Math.PI * 2);
          c.fill();
          c.fillStyle = 'rgba(200,190,160,0.35)';
          for (const [dx, dy, s] of [[-0.3, -0.2, 0.22], [0.25, 0.2, 0.16], [0.1, -0.35, 0.1]]) (c.beginPath(), c.arc(mx + dx * r, my + dy * r, s * r, 0, Math.PI * 2), c.fill());
        },
      },
      { depth: 4, paint: (c, w, h) => hills(c, w, h, 0.94, 0.08, '#16223f', seeded(24)), below: '#16223f' },
    ],
  },
  sunset: {
    name: 'Sunset over hills',
    weather: true,
    strength: 0.7,
    birds: true,
    layers: [
      {
        depth: null,
        paint: (c, w, h) => {
          c.fillStyle = vertical(c, h, [[0, '#f28a5b'], [0.45, '#ffc58f'], [0.75, '#ffe6c2'], [1, '#fff4e3']]);
          c.fillRect(0, 0, w, h);
          const [sx, sy, r] = [w * 0.62, h * 0.66, Math.min(w, h) * 0.11];
          const glow = c.createRadialGradient(sx, sy, r * 0.8, sx, sy, r * 4);
          glow.addColorStop(0, 'rgba(255,235,170,0.7)');
          glow.addColorStop(1, 'rgba(255,235,170,0)');
          c.fillStyle = glow;
          c.fillRect(0, 0, w, h);
          c.fillStyle = '#fff1c4';
          c.beginPath();
          c.arc(sx, sy, r, 0, Math.PI * 2);
          c.fill();
        },
      },
      { depth: 18, drift: 5, paint: (c, w, h) => cloudStrip(c, w, h, 5, 2, 0.22, 0.12, '255,214,190') },
      { depth: 9, paint: (c, w, h) => hills(c, w, h, 0.8, 0.1, '#d9876e', seeded(51)), below: '#d9876e' },
      { depth: 5, paint: (c, w, h) => hills(c, w, h, 0.88, 0.09, '#a8607c', seeded(52)), below: '#a8607c' },
      { depth: 2.5, paint: (c, w, h) => hills(c, w, h, 0.96, 0.07, '#6e4a72', seeded(53)), below: '#6e4a72' },
    ],
  },
  mountains: {
    name: 'Mountains',
    weather: true,
    strength: 0.7,
    cycle: true,
    birds: true,
    layers: [
      { depth: null, sky: true, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#7fb2e0'], [0.6, '#cfe3f3'], [1, '#eef5ea']])), c.fillRect(0, 0, w, h)) },
      { depth: 20, drift: 4, paint: (c, w, h) => cloudStrip(c, w, h, 3, 2, 0.1, 0.15) },
      { depth: 10, paint: (c, w, h) => range(c, w, h, 31, '#9fb3c8', 0.3, 0.7, 0), below: '#9fb3c8' },
      { depth: 6, paint: (c, w, h) => range(c, w, h, 32, '#7a92ac', 0.42, 0.8, 1), below: '#7a92ac' },
      { depth: 4, paint: (c, w, h) => range(c, w, h, 33, '#5b7389', 0.6, 0.9, 2), below: '#5b7389' },
      { depth: 2, paint: (c, w, h) => hills(c, w, h, 0.96, 0.05, '#7fa56a', seeded(34)), below: '#7fa56a' },
    ],
  },
  blueprint: {
    name: 'Blueprint',
    strength: 0.6,
    layers: [{ depth: null, paint: (c, w, h) => grid(c, w, h, '#2a5d9c', '255,255,255', [[20, 0.1], [100, 0.22]]) }],
  },
  paper: {
    name: 'Graph paper',
    strength: 0.8,
    layers: [{ depth: null, paint: (c, w, h) => grid(c, w, h, '#fbf8ef', '110,160,140', [[10, 0.12], [50, 0.3]]) }],
  },
};

function grid(c: CanvasRenderingContext2D, w: number, h: number, fill: string, ink: string, steps: [number, number][]) {
  c.fillStyle = fill;
  c.fillRect(0, 0, w, h);
  c.lineWidth = 1;
  for (const [step, alpha] of steps) {
    c.strokeStyle = `rgba(${ink},${alpha})`;
    c.beginPath();
    for (let x = 0.5; x < w; x += step) (c.moveTo(x, 0), c.lineTo(x, h));
    for (let y = 0.5; y < h; y += step) (c.moveTo(0, y), c.lineTo(w, y));
    c.stroke();
  }
}

/** A row of buildings across w (repeating), rising from the bottom to as high as `top` (fraction of h). */
function buildings(c: CanvasRenderingContext2D, w: number, h: number, seed: number, color: string, top: number, lit: boolean, glowOnly = false) {
  // (glowOnly: just the windows lit at night, in the same places: the same random draws, nothing else painted)
  const rnd = seeded(seed);
  const night = seeded(seed + 1000); // which windows are lit at night
  let x = 0;
  while (x < w) {
    const bw = Math.min(40 + rnd() * 70, w - x);
    const bh = h * (1 - top) * (0.35 + 0.65 * rnd());
    const y = h - bh;
    c.fillStyle = color;
    if (!glowOnly) c.fillRect(x, y, bw, bh);
    if (rnd() < 0.3 && !glowOnly) c.fillRect(x + bw * 0.4, y - 18, 3, 18);
    for (let wy = y + 10; wy < h - 12; wy += lit ? 16 : 22)
      for (let wx = x + 7; wx < x + bw - 10; wx += lit ? 13 : 17) {
        const day = lit ? rnd() : 1;
        if (glowOnly) {
          if (night() < (lit ? 0.45 : 0.25)) {
            c.fillStyle = lit ? 'rgba(255,214,120,0.95)' : 'rgba(255,220,150,0.7)';
            c.fillRect(wx, wy, lit ? 6 : 4, lit ? 8 : 5);
          }
        } else if (lit) {
          c.fillStyle = day < 0.35 ? 'rgba(255,233,160,0.85)' : 'rgba(225,232,240,0.45)';
          c.fillRect(wx, wy, 6, 8);
        }
      }
    x += bw + 2 + rnd() * 10;
  }
}

/** A mountain range across w (repeating), peaks between `top` and `low` (fractions of h), snow on the far ones. */
function range(c: CanvasRenderingContext2D, w: number, h: number, seed: number, color: string, top: number, low: number, k: number) {
  const rnd = seeded(seed);
  const peaks: [number, number][] = [];
  // (the last at least a normal gap before the next tile's first, at w: no needle-thin peak at the seam)
  for (let x = 0; x < w - 90; x += 90 + rnd() * 140) peaks.push([x, h * (top + (low - top) * rnd() * 0.6)]);
  const all = [...peaks.map(([x, y]) => [x - w, y] as [number, number]), ...peaks, ...peaks.map(([x, y]) => [x + w, y] as [number, number])]; // across the seam
  const valley = h * (low - 0.05 * k);
  // each peak's two flanks run down to the valleys halfway to its neighbours
  const flanks = all.map(([x, y], i) => ({ x, y, left: ((all[i - 1]?.[0] ?? x - 200) + x) / 2, right: (x + (all[i + 1]?.[0] ?? x + 200)) / 2 }));
  c.fillStyle = color;
  c.beginPath();
  c.moveTo(-w, h);
  for (const f of flanks) {
    c.lineTo(f.x, f.y);
    c.lineTo(f.right, valley);
  }
  c.lineTo(2 * w, h);
  c.closePath();
  c.fill();
  if (k < 2) {
    // snow on the far ranges: a cap down each peak's own flanks (so it fits however steep the
    // peak is), its lower edge jagged and dipping into the mountain, never past its sides
    c.fillStyle = 'rgba(255,255,255,0.85)';
    const capRnd = seeded(seed + 7);
    for (const f of flanks) {
      const tall = valley - f.y;
      if (tall < 8) continue;
      const depth = Math.min(tall * 0.32, (h * 0.07) / (1 + k)); // how far down the snow reaches
      const t = depth / tall;
      const [lx, ly] = [f.x + (f.left - f.x) * t, f.y + tall * t]; // on the left flank
      const [rx, ry] = [f.x + (f.right - f.x) * t, f.y + tall * t]; // on the right flank
      // the snow line's teeth, all downwards: below the line between the flanks the mountain is
      // only wider, so they stay inside it (the middle one shallower: a notch between two teeth)
      const dip = (u: number) => depth * (u === 0.5 ? 0.06 + 0.08 * capRnd() : 0.2 + 0.3 * capRnd());
      c.beginPath();
      c.moveTo(f.x, f.y);
      c.lineTo(lx, ly);
      for (const u of [0.25, 0.5, 0.75]) c.lineTo(lx + (rx - lx) * u, ly + (ry - ly) * u + dip(u));
      c.lineTo(rx, ry);
      c.closePath();
      c.fill();
    }
  }
}

/**
 * The scenes' own space: each layer at a distance is a tile TILE_W × TILE_H units across,
 * repeating sideways, placed in the world as an image would be at that distance, centred on the
 * world's origin at a zoom of SCENE_SCALE px per metre: one unit is one pixel at that zoom.
 */
const TILE_W = 1200;
const TILE_H = 800;
const SCENE_SCALE = 170;
/** The birds' distance: in front of the far scenery, behind the near. */
const BIRD_DEPTH = 5;
/** The most canvas px across a tile is drawn with (re-drawn sharper when zoomed in, within this). */
const MAX_TILE_PX = 2400;

/** A scene tile at distance z, as an image layer: where it lies in the world, for layerRect. */
function sceneTile(z: number): ImageLayer {
  return { id: '', name: '', image: '', aspect: TILE_H / TILE_W, depth: z, ref: { cx: 0, cy: 0, s: SCENE_SCALE }, rect: { x: -TILE_W / 2 / SCENE_SCALE, y: TILE_H / 2 / SCENE_SCALE, w: TILE_W / SCENE_SCALE } };
}

/** The scenes: id, name, their usual strength, and how many layers deep. */
export const SCENES = Object.entries(SCENE_DEFS).map(([id, s]) => ({ id, name: s.name, strength: s.strength, layers: s.layers.length, cycle: !!s.cycle, birds: !!s.birds, weather: !!s.weather, alive: !!s.cycle || !!s.birds || !!s.weather || s.layers.some((l) => l.drift) || id === 'night' }));

/**
 * An image file for a layer, as a data URL: an SVG as it is; a picture scaled down to at most
 * `most` px across (kept in the browser and in projects, so not too big), as PNG if it may have
 * transparency to keep, else JPEG. Its shape too.
 */
export async function imageFileToDataUrl(file: File, most = 2048): Promise<{ url: string; aspect: number }> {
  const raw = await new Promise<string>((res, rej) => {
    const r = new FileReader();
    r.onload = () => res(String(r.result));
    r.onerror = () => rej(new Error(`could not read ${file.name}`));
    r.readAsDataURL(file);
  });
  const img = await loadImage(raw);
  const [w0, h0] = [img.naturalWidth || 1000, img.naturalHeight || 750];
  if (file.type === 'image/svg+xml' || /\.svg$/i.test(file.name)) return { url: raw, aspect: h0 / w0 };
  const k = Math.min(1, most / Math.max(w0, h0));
  if (k === 1 && file.size < 1.5e6) return { url: raw, aspect: h0 / w0 };
  const cv = document.createElement('canvas');
  [cv.width, cv.height] = [Math.round(w0 * k), Math.round(h0 * k)];
  cv.getContext('2d')!.drawImage(img, 0, 0, cv.width, cv.height);
  const png = file.type === 'image/png' || file.type === 'image/webp' || file.type === 'image/gif';
  return { url: cv.toDataURL(png ? 'image/png' : 'image/jpeg', 0.86), aspect: cv.height / cv.width };
}

function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((res, rej) => {
    const img = new Image();
    img.onload = () => res(img);
    img.onerror = () => rej(new Error('not an image this browser can show'));
    img.src = url;
  });
}

/** Where a layer is on a w × h screen in view v (CSS px): its top-left corner and size. */
export function layerRect(l: ImageLayer, v: View, w: number, h: number): { x: number; y: number; w: number; h: number } {
  const [cx, cy] = toWorld(v, [w / 2, h / 2]);
  const z = l.depth ?? Infinity;
  const far = !Number.isFinite(z);
  const k = l.ref.s * (far ? 1 : (v.scale / l.ref.s) ** (1 / z)); // px per metre of the layer
  const [sx, sy] = far ? [0, 0] : [((cx - l.ref.cx) * v.scale) / z, ((cy - l.ref.cy) * v.scale) / z];
  return { x: w / 2 + (l.rect.x - l.ref.cx) * k - sx, y: h / 2 - (l.rect.y - l.ref.cy) * k + sy, w: l.rect.w * k, h: l.rect.w * l.aspect * k };
}

/** Put a layer at screen rectangle r (its shape kept) in view v: placed afresh, from this view. */
export function placeLayer(l: ImageLayer, r: { x: number; y: number; w: number }, v: View, w: number, h: number) {
  const [cx, cy] = toWorld(v, [w / 2, h / 2]);
  const s = v.scale;
  l.ref = { cx, cy, s };
  l.rect = { x: cx + (r.x - w / 2) / s, y: cy - (r.y - h / 2) / s, w: Math.max(r.w, 4) / s };
}

/** A layer as drawn: its distance, its place among those at that distance, how it draws, whether it is lit by the time of day and the weather, and its windows. */
type Item = { depth: number; k: number; draw: (c: CanvasRenderingContext2D) => void; lit?: boolean; glow?: (c: CanvasRenderingContext2D) => void; cloud?: boolean };

/**
 * The weather's clouds: a ceiling of grey cloud across the top of the tile, its underside ragged
 * with heavy clouds hanging from it, all one shape in one shading (no seams): paler above,
 * darker beneath.
 */
function overcast(c: CanvasRenderingContext2D, w: number, h: number, seed: number) {
  const rnd = seeded(seed);
  const base = h * 0.14;
  c.fillStyle = vertical(c, h, [[0, 'rgb(152,160,172)'], [0.14, 'rgb(136,144,158)'], [0.3, 'rgb(98,106,120)'], [1, 'rgb(98,106,120)']]);
  c.beginPath();
  c.rect(0, 0, w, base);
  const count = Math.max(7, Math.round(w / 120));
  for (let i = 0; i < count; i++) {
    const size = 130 + 160 * rnd();
    const x = ((i + rnd()) / count) * w;
    const y = base + h * 0.13 * rnd();
    const puffs = 6 + Math.floor(rnd() * 4);
    const shape = Array.from({ length: puffs }, (_, k) => {
      const u = k / (puffs - 1) - 0.5;
      return [u * size * 0.85, size * (0.15 + 0.13 * Math.sin(Math.PI * (u + 0.5)) + 0.05 * rnd())] as const;
    });
    for (const dx of [-w, 0, w]) {
      for (const [ux, r] of shape) (c.moveTo(x + dx + ux + r, y - r * 0.55), c.arc(x + dx + ux, y - r * 0.55, r, 0, Math.PI * 2));
      c.rect(x + dx - size * 0.42, 0, size * 0.84, y - size * 0.2); // (hanging from the ceiling)
    }
  }
  c.fill();
}

/** Where the weather goes among the layers (distances): its clouds, the bolt, and the rain, hail and snow far and near. */
const WEATHER_DEPTH = { highCloud: 18, lowCloud: 8, bolt: 13, farFall: 7, nearFall: 1.6 };

/** The background as drawn: its state, its images once loaded, and each scene's layers drawn once per window size. */
export class Backdrop {
  state: BackgroundState = { ...NO_BACKGROUND };
  private images = new Map<string, HTMLImageElement>();
  private cache = new Map<string, HTMLCanvasElement>();
  private layered: HTMLCanvasElement | null = null;
  private apart: HTMLCanvasElement | null = null;
  /** The time of day: what it was at `at` (ms), running on at the day's length. */
  private clock = { at: performance.now(), hour: DEFAULT_ANIM.hour, dayLength: DEFAULT_ANIM.dayLength };
  private flock = new Flock();
  private lastDraw = performance.now();
  private meteor: { x: number; y: number; vx: number; vy: number; life: number } | null = null;
  private meteorClock = 3;
  /** The weather (ui/weather.ts), stepped as it is drawn. */
  readonly weather = new Weather();

  /** Set the background (images load in the background; until then they do not show). */
  set(state: BackgroundState) {
    const anim = { ...DEFAULT_ANIM, ...state.anim };
    // a new time of day, or day length: the clock starts from it
    if (anim.hour !== this.state.anim?.hour || anim.dayLength !== this.clock.dayLength || !this.state.anim) this.clock = { at: performance.now(), hour: anim.hour, dayLength: anim.dayLength };
    this.state = { ...state, anim, layers: state.layers.map((l) => ({ ...l, ref: { ...l.ref }, rect: { ...l.rect } })) };
    for (const l of this.state.layers) {
      const fix = (img: HTMLImageElement) => {
        // a layer whose shape was not known (an older save): from the image itself
        if (!(l.aspect > 0) && img.naturalWidth) l.aspect = img.naturalHeight / img.naturalWidth;
      };
      const have = this.images.get(l.image);
      if (have) fix(have);
      else
        loadImage(l.image).then(
          (img) => {
            this.images.set(l.image, img);
            fix(img);
          },
          () => {},
        );
    }
  }

  get name(): string {
    const s = this.state;
    if (s.kind === 'custom') {
      const base = SCENE_DEFS[s.base]?.name;
      const n = s.layers.length;
      return `custom: ${[base, n && `${n} image${n === 1 ? '' : 's'}`].filter(Boolean).join(' + ') || 'empty'}`;
    }
    return s.kind === 'none' ? '' : (SCENE_DEFS[s.kind]?.name ?? '');
  }

  /**
   * A scene layer drawn: a fixed one at the screen's size; one at a distance as its tile in the
   * scene's own space (TILE_W × TILE_H units) at `res` canvas px per unit.
   */
  private sceneCanvas(id: string, i: number, w: number, h: number, res: number, painter?: SceneLayer['paint']): HTMLCanvasElement {
    const key = `${id}:${i}:${w}x${h}@${res}`;
    let cv = this.cache.get(key);
    if (cv) return cv;
    if (this.cache.size > 40) this.cache.clear();
    cv = document.createElement('canvas');
    [cv.width, cv.height] = [Math.round(w * res), Math.round(h * res)];
    const c = cv.getContext('2d')!;
    c.scale(res, res);
    (painter ?? SCENE_DEFS[id].layers[i].paint)(c, w, h);
    this.cache.set(key, cv);
    return cv;
  }

  /**
   * Draw it over the plain paper (w × h CSS px), for view v, living if animated: every layer in
   * one list sorted by distance and drawn far to near, so a nearer one always covers a farther
   * one. The list holds the sky (∞: when the day turns, its colours, stars, the sun or moon and
   * shooting stars), the scene's layers, the user's images and the birds (at BIRD_DEPTH); at the
   * same distance, the sky first, then the scene's layers in their order, the birds, the images
   * (the first in the editor's list last, on top: as it picks and lists them). Each piece of scenery is lit on its own (warm at dawn and dusk, dark at night,
   * and its windows lit over that) before it goes in its place, so the lighting keeps the order.
   */
  draw(ctx: CanvasRenderingContext2D, w: number, h: number, v: View) {
    const s = this.state;
    const now = performance.now();
    const dt = Math.min(0.05, Math.max(0, (now - this.lastDraw) / 1000));
    this.lastDraw = now;
    const sceneId = s.kind === 'custom' ? s.base : s.kind;
    const scene = SCENE_DEFS[sceneId];
    const anim = s.anim.on && !!scene;
    const turning = anim && !!scene?.cycle; // its sky follows the time of day
    const hour = this.hour;
    const dark = turning ? darkness(hour) : sceneId === 'night' ? 1 : 0;
    const warm = turning ? warmth(hour) : 0;
    const dpr = window.devicePixelRatio || 1;
    const secs = now / 1000;
    // the weather: where the scene has it and it is animated
    const weatherOn = anim && !!scene?.weather;
    const wx = this.weather;
    if (weatherOn) wx.step(dt, s.anim.weather);
    else wx.clear();
    const wet = weatherOn && wx.active;
    const gloom = wet ? wx.now.gloom : 0;
    const lit = turning || wet; // the scenery in the light of the time of day and the weather
    const items: Item[] = [];

    if (turning)
      items.push({
        depth: Infinity,
        k: -1,
        draw: (c) => {
          // the sky itself, then (in front) its stars, the sun or moon, shooting stars
          const [top, mid, low] = skyAt(hour);
          c.fillStyle = vertical(c, h, [[0, top], [0.55, mid], [1, low]]);
          c.fillRect(0, 0, w, h);
          if (dark > 0.02) {
            c.globalAlpha = dark;
            c.drawImage(this.starCanvas(w, h), 0, 0, w, h);
            c.globalAlpha = 1;
          }
          this.drawSunMoon(c, w, h, hour);
          if (dark > 0.3) this.drawNightSky(c, w, h, dt, dark);
        },
      });
    scene?.layers.forEach((layer, i) => {
      const z = layer.depth ?? Infinity;
      if (turning && layer.sky) return; // the moving sky above stands in for it
      if (!Number.isFinite(z)) {
        items.push({ depth: z, k: i, draw: (c) => c.drawImage(this.sceneCanvas(sceneId, i, w, h, dpr), 0, 0, w, h) }); // infinitely far: the screen
        return;
      }
      // in the scene's own space: its tile placed in the world like an image at that distance
      // (so it pans by 1/z and zooms by (s/s₀)^(1/z)), repeating across, its ground running on below
      const r = layerRect(sceneTile(z), v, w, h);
      const unit = r.w / TILE_W;
      if (anim && layer.drift) r.x += ((secs * layer.drift) % TILE_W) * unit; // drifting along
      const res = Math.min(unit * dpr > 1.4 ? 2 : 1, MAX_TILE_PX / TILE_W) * Math.min(dpr, 2);
      const x0 = r.x - Math.ceil(r.x / r.w) * r.w;
      const tiles = (c: CanvasRenderingContext2D, cv: HTMLCanvasElement) => {
        for (let x = x0; x < w; x += r.w) c.drawImage(cv, x, r.y, r.w + 0.6, r.h); // (a hair over: no seams)
      };
      items.push({
        depth: z,
        k: i,
        lit,
        cloud: !!layer.drift, // the scene's clouds: greyed as the sky clouds over
        draw: (c) => {
          tiles(c, this.sceneCanvas(sceneId, i, TILE_W, TILE_H, res));
          if (layer.below && r.y + r.h < h) {
            c.fillStyle = layer.below;
            c.fillRect(0, r.y + r.h - 0.5, w, h - (r.y + r.h) + 0.5);
          }
        },
        // lit windows: over this layer's darkening, as bright as it is dark (and under anything nearer)
        glow: anim && layer.glow && dark > 0.02 ? (c) => tiles(c, this.sceneCanvas(sceneId, -1 - i, TILE_W, TILE_H, res, layer.glow)) : undefined,
      });
    });
    // the night scene's sky: twinkling and shooting stars, just in front of it
    if (anim && !turning && dark > 0.3) items.push({ depth: Infinity, k: 0.5, draw: (c) => this.drawNightSky(c, w, h, dt, dark) });
    // birds by day, flying at their distance in the scene's space
    // (sheltering from the rain, hail and snow)
    const shelter = wet ? Math.min(1, 1.6 * (wx.now.rain + wx.now.hail) + 1.1 * wx.now.snow) : 0;
    if (anim && scene?.birds && s.anim.birds && dark < 0.9 && shelter < 0.98) items.push({ depth: BIRD_DEPTH, k: 1000, draw: (c) => this.drawBirds(c, v, w, h, dt, (1 - dark) * (1 - shelter)) });
    if (wet) this.weatherItems(items, v, w, h, dt, dark);
    if (s.kind === 'custom')
      s.layers.forEach((l, n) => {
        const img = this.images.get(l.image);
        if (!img) return;
        items.push({
          depth: l.depth ?? Infinity,
          k: 2000 + (s.layers.length - 1 - n), // first in the editor's list: on top of the others at its distance
          lit, // in the same light as the scenery
          draw: (c) => {
            const r = layerRect(l, v, w, h);
            c.imageSmoothingQuality = 'high';
            c.drawImage(img, r.x, r.y, r.w, r.h);
          },
        });
      });
    if (!items.length) return;

    const out = this.composite(w, h); // the layers at full strength, then the whole at its strength
    const lighting = warm > 0.01 || dark > 0.01 || gloom > 0.01;
    items.sort((a, b) => b.depth - a.depth || a.k - b.k);
    for (const it of items) {
      if (it.lit && lighting) {
        // lit on its own: drawn apart, tinted where it is, its windows over that, then put in its place
        const sc = this.scratch(w, h);
        it.draw(sc);
        sc.save();
        sc.globalCompositeOperation = 'source-atop';
        if (warm > 0.01) (sc.fillStyle = `rgba(255,128,70,${0.2 * warm})`), sc.fillRect(0, 0, w, h);
        if (gloom > 0.01) (sc.fillStyle = `rgba(58,66,80,${0.5 * gloom})`), sc.fillRect(0, 0, w, h);
        if (it.cloud && wet) (sc.fillStyle = `rgba(112,120,134,${0.7 * wx.now.cover})`), sc.fillRect(0, 0, w, h);
        // snow lying on the land: whiter as it settles (before the night darkens it too)
        if (!it.cloud && wet && wx.settled > 0.01) (sc.fillStyle = `rgba(238,242,248,${0.55 * wx.settled})`), sc.fillRect(0, 0, w, h);
        if (dark > 0.01) (sc.fillStyle = `rgba(14,22,58,${0.62 * dark})`), sc.fillRect(0, 0, w, h);
        sc.restore();
        if (it.glow) {
          sc.globalAlpha = dark;
          it.glow(sc);
          sc.globalAlpha = 1;
        }
        out.drawImage(sc.canvas, 0, 0, w, h);
      } else {
        it.draw(out);
        if (it.glow) {
          out.globalAlpha = dark;
          it.glow(out);
          out.globalAlpha = 1;
        }
      }
    }
    // lightning: a flash over everything
    if (wet && wx.flash > 0.01) (out.fillStyle = `rgba(232,238,255,${0.28 * wx.flash})`), out.fillRect(0, 0, w, h);
    ctx.save();
    ctx.globalAlpha = Math.min(1, Math.max(0, s.opacity));
    ctx.drawImage(out.canvas, 0, 0, w, h);
    ctx.restore();
  }

  /**
   * The weather's layers, each at its distance: the overcast sky (in front of the sky, hiding the
   * sun and stars) and the sky lit by lightning behind the scenery, the clouds high and low
   * (drifting with the wind, lit like the scenery), the bolt, and the rain, hail and snow far and near.
   */
  private weatherItems(items: Item[], v: View, w: number, h: number, dt: number, dark: number) {
    const wx = this.weather;
    const n = wx.now;
    const dpr = window.devicePixelRatio || 1;
    items.push({
      depth: Infinity,
      k: 0.6,
      draw: (c) => {
        // the sky clouded over: grey by day, near black by night, darker as the storm is
        const day = mix('#9aa4b0', '#59626e', Math.min(1, n.gloom * 1.3));
        c.fillStyle = mix(day, '#151a24', dark);
        c.globalAlpha = Math.min(1, n.cover);
        c.fillRect(0, 0, w, h);
        c.globalAlpha = 1;
        if (wx.flash > 0.01) {
          // lightning lights the clouds from within: brightest high up
          const g = c.createLinearGradient(0, 0, 0, h);
          g.addColorStop(0, `rgba(225,232,255,${0.75 * wx.flash})`);
          g.addColorStop(1, `rgba(225,232,255,${0.25 * wx.flash})`);
          c.fillStyle = g;
          c.fillRect(0, 0, w, h);
        }
      },
    });
    const place = (z: number) => {
      const r = layerRect(sceneTile(z), v, w, h);
      return { r, unit: r.w / TILE_W };
    };
    ([[WEATHER_DEPTH.highCloud, 0.6, 61], [WEATHER_DEPTH.lowCloud, 1, 63]] as const).forEach(([z, speed, seed]) => {
      const { r, unit } = place(z);
      r.x += ((wx.drift * speed) % TILE_W) * unit;
      const res = Math.min(unit * dpr > 1.4 ? 2 : 1, MAX_TILE_PX / TILE_W) * Math.min(dpr, 2);
      const x0 = r.x - Math.ceil(r.x / r.w) * r.w;
      items.push({
        depth: z,
        k: 900,
        lit: true,
        draw: (c) => {
          const cv = this.sceneCanvas('weather', seed, TILE_W, TILE_H, res, (cc, tw, th) => overcast(cc, tw, th, seed));
          c.globalAlpha = Math.min(1, 1.15 * n.cover * (z === WEATHER_DEPTH.lowCloud ? Math.max(0, n.cover - 0.4) / 0.6 : 1));
          for (let x = x0; x < w; x += r.w) c.drawImage(cv, x, r.y, r.w + 0.6, r.h);
          // above the tile: the ceiling carries on
          if (r.y > 0) (c.fillStyle = 'rgb(150,158,170)'), c.fillRect(0, 0, w, r.y + 1);
          c.globalAlpha = 1;
        },
      });
    });
    if (wx.bolt) {
      const { r, unit } = place(WEATHER_DEPTH.bolt);
      items.push({ depth: WEATHER_DEPTH.bolt, k: 900, draw: (c) => wx.drawBolt(c, r.x, r.y, unit, -r.x / unit, (w - r.x) / unit) });
    }
    if (n.rain > 0.01 || n.hail > 0.01 || n.snow > 0.01)
      for (const near of [false, true]) {
        const z = near ? WEATHER_DEPTH.nearFall : WEATHER_DEPTH.farFall;
        const { r, unit } = place(z);
        items.push({ depth: z, k: 950, draw: (c) => wx.drawFall(c, w, h, r.x, r.y, unit, near, dt, dark) });
      }
  }

  /** A second canvas the screen's size, cleared: for lighting one layer on its own. */
  private scratch(w: number, h: number): CanvasRenderingContext2D {
    const dpr = window.devicePixelRatio || 1;
    this.apart ??= document.createElement('canvas');
    const cv = this.apart;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * dpr)];
    const c = cv.getContext('2d')!;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    return c;
  }

  /** The sun by day and the moon by night, each along its arc across the sky (with a glow). */
  private drawSunMoon(c: CanvasRenderingContext2D, w: number, h: number, hour: number) {
    const H = ((hour % 24) + 24) % 24;
    const arc = (t: number) => [w * (0.08 + 0.84 * t), h * 0.78 - Math.sin(Math.PI * t) * h * 0.62] as const;
    const sunT = (H - 5.6) / 13.4; // up from 5:36 to 19:00
    if (sunT > 0 && sunT < 1) {
      const [x, y] = arc(sunT);
      const r = Math.min(w, h) * 0.045;
      const low = 1 - Math.sin(Math.PI * sunT); // redder near the horizon
      const glow = c.createRadialGradient(x, y, r * 0.6, x, y, r * 5);
      glow.addColorStop(0, `rgba(255,${Math.round(230 - 60 * low)},${Math.round(160 - 80 * low)},0.55)`);
      glow.addColorStop(1, 'rgba(255,220,150,0)');
      c.fillStyle = `rgb(255,${Math.round(244 - 70 * low)},${Math.round(200 - 110 * low)})`;
      c.beginPath();
      c.arc(x, y, r, 0, Math.PI * 2);
      c.fill();
      c.fillStyle = glow;
      c.fillRect(x - r * 5, y - r * 5, r * 10, r * 10);
    }
    const moonT = (((H - 18.6 + 24) % 24) / 11.4); // up from 18:36 to 6:00
    if (moonT > 0 && moonT < 1) {
      const [x, y] = arc(moonT);
      const r = Math.min(w, h) * 0.035;
      c.fillStyle = '#f4efd6';
      c.beginPath();
      c.arc(x, y, r, 0, Math.PI * 2);
      c.fill();
      const glow = c.createRadialGradient(x, y, r, x, y, r * 4);
      glow.addColorStop(0, 'rgba(255,250,220,0.22)');
      glow.addColorStop(1, 'rgba(255,250,220,0)');
      c.fillStyle = glow;
      c.fillRect(x - r * 4, y - r * 4, r * 8, r * 8);
    }
  }

  /** The stars of a turning sky, drawn once per screen size (shown as dark as it is). */
  private starCanvas(w: number, h: number): HTMLCanvasElement {
    const dpr = window.devicePixelRatio || 1;
    const key = `stars:${w}x${h}@${dpr}`;
    let cv = this.cache.get(key);
    if (cv) return cv;
    cv = document.createElement('canvas');
    [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * dpr)];
    const c = cv.getContext('2d')!;
    c.scale(dpr, dpr);
    const rnd = seeded(29);
    for (let i = 0; i < (w * h) / 2400; i++) {
      c.fillStyle = `rgba(255,255,240,${0.35 + 0.65 * rnd()})`;
      c.beginPath();
      c.arc(rnd() * w, rnd() * h * 0.8, 0.4 + 1.1 * rnd() ** 3, 0, Math.PI * 2);
      c.fill();
    }
    this.cache.set(key, cv);
    return cv;
  }

  /** Twinkling stars and, now and then, a shooting star (screen space, as the night sky). */
  private drawNightSky(c: CanvasRenderingContext2D, w: number, h: number, dt: number, dark: number) {
    const t = performance.now() / 1000;
    const rnd = seeded(41);
    c.save();
    // a few stars that twinkle (over the still ones): drawn as part of the sky, in its place in the order
    for (let i = 0; i < 40; i++) {
      const [x, y, f, ph] = [rnd() * w, rnd() * h * 0.6, 1 + rnd() * 3, rnd() * 6.28];
      const a = dark * (0.5 + 0.5 * Math.sin(t * f + ph)) ** 3;
      if (a < 0.05) continue;
      c.fillStyle = `rgba(255,255,235,${a})`;
      c.beginPath();
      c.arc(x, y, 1.6, 0, Math.PI * 2);
      c.fill();
    }
    // shooting stars: one every few seconds, a streak across the upper sky
    this.meteorClock -= dt;
    if (this.meteorClock <= 0 && !this.meteor) {
      this.meteor = { x: Math.random() * w * 0.8 + w * 0.1, y: Math.random() * h * 0.3, vx: (Math.random() < 0.5 ? -1 : 1) * (500 + Math.random() * 300), vy: 180 + Math.random() * 120, life: 0.9 };
      this.meteorClock = 4 + Math.random() * 8;
    }
    const m = this.meteor;
    if (m) {
      m.life -= dt;
      m.x += m.vx * dt;
      m.y += m.vy * dt;
      const a = Math.max(0, m.life / 0.9) * dark;
      const g = c.createLinearGradient(m.x, m.y, m.x - m.vx * 0.18, m.y - m.vy * 0.18);
      g.addColorStop(0, `rgba(255,255,240,${a})`);
      g.addColorStop(1, 'rgba(255,255,240,0)');
      c.strokeStyle = g;
      c.lineWidth = 2;
      c.beginPath();
      c.moveTo(m.x, m.y);
      c.lineTo(m.x - m.vx * 0.18, m.y - m.vy * 0.18);
      c.stroke();
      if (m.life <= 0) this.meteor = null;
    }
    c.restore();
  }

  /** The flock, stepped and drawn at its distance (BIRD_DEPTH) in the scene's space: wings beating as they fly. */
  private drawBirds(c: CanvasRenderingContext2D, v: View, w: number, h: number, dt: number, light: number) {
    const r = layerRect(sceneTile(BIRD_DEPTH), v, w, h);
    const unit = r.w / TILE_W;
    // the units in view across, and the sky's band of the tile
    const [u0, u1] = [(0 - r.x) / unit, (w - r.x) / unit];
    if (!this.flock.birds.length || this.flock.birds.every((b) => b.x < u0 - 400 || b.x > u1 + 400)) this.flock.spawn(16, u0, u1);
    this.flock.step(dt, u0, u1, TILE_H * 0.1, TILE_H * 0.42);
    c.save();
    c.strokeStyle = `rgba(38,42,52,${0.85 * light})`;
    c.lineCap = 'round';
    c.lineJoin = 'round';
    c.lineWidth = Math.max(1, 1.6 * unit);
    for (const b of this.flock.birds) {
      const [x, y] = [r.x + b.x * unit, r.y + b.y * unit];
      const span = 8 * b.size * unit;
      const lift = Math.sin(b.flap) * span * 0.55; // wings up and down
      const dir = Math.sign(b.vx) || 1;
      c.beginPath();
      c.moveTo(x - span, y - lift);
      c.quadraticCurveTo(x - span * 0.45, y - lift * 0.2 - span * 0.15, x + dir * span * 0.08, y);
      c.quadraticCurveTo(x + span * 0.45, y - lift * 0.2 - span * 0.15, x + span, y - lift);
      c.stroke();
    }
    c.restore();
  }

  /** The time of day now (hours, 0–24). */
  get hour(): number {
    const a = this.state.anim;
    const passed = a.on && a.dayLength > 0 ? ((performance.now() - this.clock.at) / 1000 / a.dayLength) * 24 : 0;
    return (((this.clock.hour + passed) % 24) + 24) % 24;
  }

  /** The canvas the layers are put together on, cleared, at the screen's size (sharp at its pixel density). */
  private composite(w: number, h: number): CanvasRenderingContext2D {
    const dpr = window.devicePixelRatio || 1;
    this.layered ??= document.createElement('canvas');
    const cv = this.layered;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * dpr)];
    const c = cv.getContext('2d')!;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, w, h);
    return c;
  }
}
