/**
 * The weather over a background scene: clear, cloudy, rain, storm, thunderstorm, hail or snow, held
 * fixed or changing by itself (spells of clear sky of random length, broken by weather of a random
 * kind lasting a random time, which now and then turns into another before it clears). Any
 * weather can also be started by hand, for a random while, a set time or until stopped, after
 * which the weather goes back to its own ways; and lightning can be made to strike.
 *
 * The weather is a handful of levels, each easing towards what the weather of the moment wants
 * (so a storm rolls in and clears away rather than switching): how much of the sky is overcast,
 * how gloomy the light is, how hard it rains, hails and snows, how windy it is, and whether lightning
 * strikes. ui/background.ts draws them in their places among the scene's layers: the overcast
 * sky and its clouds, the scenery greyed (and whitened as snow settles on it, melting after),
 * rain, hail and snow falling at two distances (slanted by the
 * wind, the nearer faster and longer), and lightning: a forked bolt at a distance, the sky lit
 * up behind the scenery and a flash over all, the thunder following it after the time its sound
 * takes to arrive.
 */

export type WeatherKind = 'clear' | 'cloudy' | 'rain' | 'storm' | 'thunder' | 'hail' | 'snow';
/** 'random': changing by itself; otherwise that weather all the time. */
export type WeatherMode = 'random' | WeatherKind;

export const WEATHERS: { id: WeatherKind; name: string }[] = [
  { id: 'clear', name: 'Clear' },
  { id: 'cloudy', name: 'Cloudy' },
  { id: 'rain', name: 'Rain' },
  { id: 'storm', name: 'Storm' },
  { id: 'thunder', name: 'Thunderstorm' },
  { id: 'hail', name: 'Hail' },
  { id: 'snow', name: 'Snow' },
];
export const WEATHER_MODES: { id: WeatherMode; name: string }[] = [{ id: 'random', name: 'Changing (random)' }, ...WEATHERS];
export const isWeatherMode = (m: unknown): m is WeatherMode => WEATHER_MODES.some((x) => x.id === m);

/** The levels of a weather, each 0–1. */
export interface WeatherLevels {
  /** How much of the sky is overcast. */
  cover: number;
  /** How grey and dim the light is. */
  gloom: number;
  rain: number;
  hail: number;
  snow: number;
  wind: number;
}

const TARGETS: Record<WeatherKind, WeatherLevels> = {
  clear: { cover: 0, gloom: 0, rain: 0, hail: 0, snow: 0, wind: 0.1 },
  cloudy: { cover: 0.75, gloom: 0.3, rain: 0, hail: 0, snow: 0, wind: 0.3 },
  rain: { cover: 0.9, gloom: 0.45, rain: 0.55, hail: 0, snow: 0, wind: 0.25 },
  storm: { cover: 1, gloom: 0.65, rain: 1, hail: 0, snow: 0, wind: 1 },
  thunder: { cover: 1, gloom: 0.75, rain: 0.9, hail: 0, snow: 0, wind: 0.75 },
  hail: { cover: 0.95, gloom: 0.55, rain: 0.25, hail: 1, snow: 0, wind: 0.5 },
  snow: { cover: 1, gloom: 0.3, rain: 0, hail: 0, snow: 1, wind: 0.2 },
};
/** How quickly each level follows (s): the sky clouds over slowly, the rain starts sooner. */
const EASE: Record<keyof WeatherLevels, number> = { cover: 7, gloom: 7, rain: 4, hail: 3, snow: 6, wind: 5 };
/** How long snow takes to settle fully, and to melt away (s). */
const SETTLE = 90;
const MELT = 120;

const between = (a: number, b: number) => a + Math.random() * (b - a);
function pick<T extends string>(weights: Partial<Record<T, number>>): T {
  const all = Object.entries(weights) as [T, number][];
  let r = Math.random() * all.reduce((s, [, p]) => s + p, 0);
  for (const [k, p] of all) if ((r -= p) <= 0) return k;
  return all[all.length - 1][0];
}

/** After a spell of weather: what it may turn into (the rest of the time it clears). */
const NEXT: Partial<Record<WeatherKind, Partial<Record<WeatherKind, number>>>> = {
  cloudy: { rain: 0.3, clear: 0.7 },
  rain: { thunder: 0.15, storm: 0.1, cloudy: 0.25, clear: 0.5 },
  storm: { thunder: 0.25, rain: 0.25, clear: 0.5 },
  thunder: { rain: 0.3, hail: 0.1, clear: 0.6 },
  hail: { rain: 0.3, snow: 0.15, clear: 0.55 },
  snow: { cloudy: 0.3, clear: 0.7 },
};

/** A lightning bolt: its path (scene units, from where it strikes), how old it is (s) and how far (for the thunder). */
interface Bolt {
  /** Where it strikes: across the view (0–1), fixed in the scene (units) once first drawn. */
  x: number;
  u?: number;
  paths: { pts: [number, number][]; w: number }[];
  age: number;
  far: number;
}

/** Rain, hail or snow at one distance: how far it has fallen and drifted (px, kept as the speed changes). */
interface Fall {
  y: number;
  x: number;
}

/** A small seeded random generator (mulberry32): the same drops in the same places every frame. */
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
const mod = (a: number, n: number) => ((a % n) + n) % n;

export class Weather {
  /** The weather of the moment, and how long it has left (s; changing weather only). */
  kind: WeatherKind = 'clear';
  left = between(20, 60);
  /** The levels now, easing towards the weather's. */
  now: WeatherLevels = { ...TARGETS.clear };
  /** The clouds' drift (scene units), faster in the wind. */
  drift = 0;
  bolt: Bolt | null = null;
  /** The lightning's brightness now, 0–1 (a flash, a flicker, then gone). */
  flash = 0;
  private nextStrike = between(2, 6);
  private falls: Fall[] = [0, 1, 2, 3, 4, 5].map(() => ({ x: 0, y: 0 }));
  /** How much snow lies on the scenery, 0–1: settling while it snows, melting after. */
  settled = 0;
  /** Thunder, when lightning strikes: after `delay` s, `loud` 0–1. */
  onThunder: ((delay: number, loud: number) => void) | null = null;
  private mode: WeatherMode = 'random';
  /** A weather started by hand, and how long it has left (s; Infinity: until stopped); then back to the mode's. */
  triggered: { kind: WeatherKind; left: number } | null = null;

  /** Whether there is any weather to draw (anything but a clear sky, or still clearing). */
  get active(): boolean {
    const n = this.now;
    return n.cover > 0.005 || n.rain > 0.005 || n.hail > 0.005 || n.snow > 0.005 || this.settled > 0.005 || !!this.bolt;
  }

  get name(): string {
    return WEATHERS.find((x) => x.id === this.kind)!.name;
  }

  /** Step dt s on, in mode: change the weather when its time is up, ease the levels, strike lightning. */
  step(dt: number, mode: WeatherMode) {
    if (mode !== this.mode) {
      this.mode = mode;
      this.triggered = null; // a weather chosen ends one started by hand
      // a set weather is that weather; changing weather starts from the weather it is
      if (mode !== 'random') this.kind = mode;
      this.left = this.kind === 'clear' ? between(20, 60) : between(30, 80);
    }
    if (this.triggered) {
      this.kind = this.triggered.kind;
      if ((this.triggered.left -= dt) <= 0) this.endTrigger();
    } else if (mode === 'random' && (this.left -= dt) <= 0) {
      this.kind = this.kind === 'clear' ? pick<WeatherKind>({ cloudy: 0.3, rain: 0.24, storm: 0.1, thunder: 0.14, hail: 0.1, snow: 0.12 }) : pick<WeatherKind>(NEXT[this.kind]!);
      // clear spells last longer than the weather between them
      this.left = spell(this.kind);
    }
    const target = TARGETS[this.kind];
    for (const k of Object.keys(EASE) as (keyof WeatherLevels)[]) this.now[k] += (target[k] - this.now[k]) * (1 - Math.exp(-dt / EASE[k]));
    this.drift += (6 + 40 * this.now.wind) * dt;
    // snow settles while it snows hard enough, and melts away (rain melts it faster)
    const n = this.now;
    this.settled = Math.min(1, Math.max(0, this.settled + (n.snow > 0.3 ? (n.snow * dt) / SETTLE : -((1 + 3 * n.rain) * dt) / MELT)));
    // lightning: in a thunderstorm once the sky has clouded over, every few seconds
    if (this.bolt) {
      this.bolt.age += dt;
      if (this.bolt.age > 1.2) this.bolt = null;
    }
    if (this.kind === 'thunder' && this.now.cover > 0.8 && (this.nextStrike -= dt) <= 0) {
      this.nextStrike = Math.random() < 0.25 ? between(0.6, 1.5) : between(3, 11); // sometimes one after another
      this.strike();
    }
    this.flash = this.bolt ? flashAt(this.bolt.age) * (1 - 0.6 * this.bolt.far) : 0;
  }

  /**
   * Start a weather now, by hand: for `secs` s (Infinity: until stopped; by default, a while as
   * long as it would last by itself). It eases in as ever; when it ends, the weather goes back to
   * its mode (changing weather: a clear spell; a set one: that weather).
   */
  trigger(kind: WeatherKind, secs?: number) {
    this.triggered = { kind, left: secs && secs > 0 ? secs : spell(kind) };
    this.kind = kind;
    if (kind === 'thunder') this.nextStrike = Math.min(this.nextStrike, between(1, 3)); // (the first strike soon after it has clouded over)
  }

  /** End the weather started by hand, at once (it eases out as ever). */
  endTrigger() {
    if (!this.triggered) return;
    this.triggered = null;
    this.kind = this.mode === 'random' ? 'clear' : this.mode;
    this.left = between(40, 180);
  }

  /** Lightning, now, whatever the weather: a bolt from the blue if need be. */
  strikeNow() {
    this.strike();
  }

  /** A bolt: a jagged path down from the cloud base with a few forks, somewhere in view (set by the drawing). */
  private strike() {
    const rnd = Math.random;
    const paths: Bolt['paths'] = [];
    const branch = (x: number, y: number, bottom: number, w: number, depth: number) => {
      const pts: [number, number][] = [[x, y]];
      while (y < bottom) {
        y += 14 + rnd() * 26;
        x += (rnd() - 0.5) * 38;
        pts.push([x, y]);
        if (depth < 2 && rnd() < 0.12) branch(x, y, y + 60 + rnd() * 160, w * 0.5, depth + 1);
      }
      paths.push({ pts, w });
    };
    branch(0, 70, 560 + rnd() * 120, 3, 0);
    const far = rnd();
    this.bolt = { x: rnd(), paths, age: 0, far };
    // the thunder: the nearer, the sooner and louder (sound takes ~3 s a km)
    this.onThunder?.(0.15 + far * 3.5, 1 - 0.65 * far);
  }

  /**
   * Rain or hail at one distance, over the screen (w × h px). (ox, oy): where the layer's tile is
   * on the screen (so it pans with its distance), unit: px per scene unit there; near: the nearer
   * of the two distances (faster, longer, bigger, fewer). dark: how dark it is (0–1).
   */
  drawFall(c: CanvasRenderingContext2D, w: number, h: number, ox: number, oy: number, unit: number, near: boolean, dt: number, dark: number) {
    const n = this.now;
    const slant = n.wind * 0.55; // sideways per down
    if (n.rain > 0.01) {
      const f = this.falls[near ? 1 : 0];
      const speed = Math.max(near ? 700 : 420, (near ? 1500 : 900) * unit);
      const len = Math.min(70, Math.max(near ? 9 : 5, (near ? 38 : 18) * unit));
      f.y += speed * dt;
      f.x += speed * slant * dt;
      const count = Math.round(n.rain * (near ? 0.00028 : 0.0005) * w * h);
      const pad = slant * len + 30;
      const [W, H] = [w + pad, h + len];
      const rnd = seeded(near ? 211 : 199);
      c.save();
      c.strokeStyle = dark > 0.5 ? `rgba(160,175,200,${(near ? 0.45 : 0.3) * (1 - 0.4 * dark)})` : `rgba(215,225,240,${near ? 0.55 : 0.35})`;
      c.lineWidth = near ? Math.min(2, Math.max(1, 1.4 * unit)) : 1;
      c.lineCap = 'round';
      c.beginPath();
      for (let i = 0; i < count; i++) {
        const [x0, y0, k] = [rnd() * W, rnd() * H, 0.8 + 0.4 * rnd()];
        const x = mod(x0 + f.x * k + ox, W) - pad;
        const y = mod(y0 + f.y * k + oy, H) - len;
        c.moveTo(x, y);
        c.lineTo(x + slant * len * k, y + len * k);
      }
      c.stroke();
      c.restore();
    }
    if (n.hail > 0.01) {
      const f = this.falls[near ? 3 : 2];
      const speed = Math.max(near ? 520 : 320, (near ? 1050 : 650) * unit);
      const r = Math.min(4.5, Math.max(near ? 1.6 : 1, (near ? 3.6 : 2) * unit));
      f.y += speed * dt;
      f.x += speed * slant * 0.5 * dt;
      const count = Math.round(n.hail * (near ? 0.00012 : 0.0002) * w * h);
      const [W, H] = [w + 40, h + 20];
      const rnd = seeded(near ? 307 : 301);
      const t = performance.now() / 1000;
      c.save();
      c.fillStyle = `rgba(${dark > 0.5 ? '190,198,212' : '246,249,252'},${near ? 0.9 : 0.6})`;
      c.strokeStyle = `rgba(235,240,248,${near ? 0.25 : 0.15})`;
      c.lineWidth = r * 0.9;
      c.beginPath();
      for (let i = 0; i < count; i++) {
        const [x0, y0, k, ph] = [rnd() * W, rnd() * H, 0.85 + 0.3 * rnd(), rnd() * 6.28];
        const x = mod(x0 + f.x * k + ox + Math.sin(t * 7 + ph) * r, W) - 20; // tumbling a little as it falls
        const y = mod(y0 + f.y * k + oy, H) - 10;
        c.moveTo(x + r, y);
        c.arc(x, y, r * (0.75 + 0.25 * k), 0, Math.PI * 2);
      }
      c.fill();
      c.restore();
    }
    if (n.snow > 0.01) {
      // flakes drifting down slowly, swaying side to side, carried by the wind
      const f = this.falls[near ? 5 : 4];
      const speed = Math.max(near ? 55 : 32, (near ? 120 : 65) * unit);
      const r = Math.min(4.5, Math.max(near ? 1.5 : 0.9, (near ? 3 : 1.6) * unit));
      f.y += speed * dt;
      f.x += speed * (0.3 + 2.5 * n.wind) * dt;
      const count = Math.round(n.snow * (near ? 0.00016 : 0.00035) * w * h);
      const sway = (near ? 18 : 9) * Math.max(0.5, unit);
      const [W, H] = [w + 2 * sway + 20, h + 20];
      const rnd = seeded(near ? 401 : 397);
      const t = performance.now() / 1000;
      c.save();
      c.fillStyle = `rgba(${dark > 0.5 ? '200,208,224' : '255,255,255'},${near ? 0.92 : 0.7})`;
      c.beginPath();
      for (let i = 0; i < count; i++) {
        const [x0, y0, k, ph, fr] = [rnd() * W, rnd() * H, 0.7 + 0.6 * rnd(), rnd() * 6.28, 0.5 + rnd()];
        const x = mod(x0 + f.x * k + ox + Math.sin(t * fr + ph) * sway, W) - sway - 10;
        const y = mod(y0 + f.y * k + oy, H) - 10;
        const rr = r * (0.6 + 0.4 * k);
        c.moveTo(x + rr, y);
        c.arc(x, y, rr, 0, Math.PI * 2);
      }
      c.fill();
      c.restore();
    }
  }

  /** The bolt at its distance: (ox, oy, unit) as for drawFall, u0…u1 the scene units in view across. */
  drawBolt(c: CanvasRenderingContext2D, ox: number, oy: number, unit: number, u0: number, u1: number) {
    const b = this.bolt;
    if (!b) return;
    const a = boltAt(b.age);
    if (a < 0.02) return;
    b.u ??= u0 + (0.12 + 0.76 * b.x) * (u1 - u0);
    const x0 = ox + b.u * unit;
    c.save();
    c.lineCap = 'round';
    c.lineJoin = 'round';
    for (const pass of [0, 1]) {
      for (const p of b.paths) {
        c.strokeStyle = pass ? `rgba(255,255,255,${a})` : `rgba(170,190,255,${0.35 * a})`;
        c.lineWidth = Math.max(pass ? 0.8 : 3, p.w * unit * (pass ? 1 : 4));
        c.beginPath();
        p.pts.forEach(([x, y], i) => (i ? c.lineTo(x0 + x * unit, oy + y * unit) : c.moveTo(x0 + x * unit, oy + y * unit)));
        c.stroke();
      }
    }
    c.restore();
  }

  /** What it sounds like: the rain, the hail and the wind, 0–1 (none while the sky is clear). */
  get sound(): { rain: number; hail: number; wind: number } {
    const n = this.now;
    return { rain: n.rain, hail: n.hail, wind: Math.max(0, n.wind - 0.15) * n.cover };
  }

  /** Back to a clear sky at once (a new background). */
  clear() {
    this.now = { ...TARGETS.clear };
    this.settled = 0;
    this.bolt = null;
    this.flash = 0;
    this.triggered = null;
    if (this.mode === 'random') {
      this.kind = 'clear';
      this.left = between(20, 60);
    }
  }
}

/** How long a spell of a weather lasts by itself (s): clear spells longest, snow longer than showers. */
function spell(kind: WeatherKind): number {
  return kind === 'clear' ? between(40, 180) : kind === 'cloudy' ? between(30, 100) : kind === 'snow' ? between(40, 120) : between(25, 90);
}

/** The lightning's brightness over its life: a flash, a dimmer second one, gone in a second. */
function flashAt(age: number): number {
  return Math.min(1, Math.exp(-age / 0.06) + (age > 0.16 ? 0.7 * Math.exp(-(age - 0.16) / 0.09) : 0) + (age > 0.42 ? 0.35 * Math.exp(-(age - 0.42) / 0.12) : 0));
}
/** The bolt's own brightness: lit while it flashes, flickering. */
function boltAt(age: number): number {
  if (age > 0.75) return 0;
  return Math.min(1, flashAt(age) * 1.6 + (age < 0.5 ? 0.25 : 0));
}
