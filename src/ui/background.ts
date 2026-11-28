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
 * be (clouds far, near buildings close), repeating sideways and running on below, so panning
 * never runs out of them.
 *
 * The state is plain data (the images as data URLs), saved with projects.
 */
import { toWorld, type View } from './render';

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

export interface BackgroundState {
  /** 'none', a scene's id, or 'custom' (the scene `base` with the `layers` over it). */
  kind: string;
  base: string;
  layers: ImageLayer[];
  /** How strongly it shows, 0–1. */
  opacity: number;
}

export const NO_BACKGROUND: BackgroundState = { kind: 'none', base: 'none', layers: [], opacity: 0.7 };

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
    const [x, y, size] = [((i + 0.3 + 0.5 * rnd()) / count) * w, h * (top + spread * rnd()), 110 + 120 * rnd()];
    for (const dx of [-w, 0, w]) cloud(c, x + dx, y, size, seeded(seed * 31 + i), tint); // across the seam too
  }
}

/** A layer of a scene: its distance (null: fixed), its painter (w × h px), and the colour it runs on with below. */
interface SceneLayer {
  depth: number | null;
  paint: (c: CanvasRenderingContext2D, w: number, h: number) => void;
  below?: string;
}

const SCENE_DEFS: Record<string, { name: string; strength: number; layers: SceneLayer[] }> = {
  sky: {
    name: 'Blue sky with clouds',
    strength: 0.75,
    layers: [
      { depth: null, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#5aa5e6'], [0.55, '#a9d4f5'], [1, '#eaf6fd']])), c.fillRect(0, 0, w, h)) },
      { depth: 14, paint: (c, w, h) => cloudStrip(c, w, h, 7, Math.max(3, Math.round(w / 420)), 0.12, 0.25) },
      { depth: 6, paint: (c, w, h) => cloudStrip(c, w, h, 9, Math.max(3, Math.round(w / 380)), 0.38, 0.3) },
    ],
  },
  city: {
    name: 'City skyline',
    strength: 0.7,
    layers: [
      { depth: null, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#8fbbe6'], [0.6, '#d9e6f2'], [1, '#f6e3c8']])), c.fillRect(0, 0, w, h)) },
      { depth: 16, paint: (c, w, h) => cloudStrip(c, w, h, 11, 2, 0.12, 0.12) },
      { depth: 6, paint: (c, w, h) => buildings(c, w, h, 21, '#b4c3d4', 0.42, false), below: '#b4c3d4' },
      { depth: 3, paint: (c, w, h) => buildings(c, w, h, 22, '#73859c', 0.55, true), below: '#73859c' },
    ],
  },
  night: {
    name: 'Night sky',
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
    strength: 0.7,
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
      { depth: 18, paint: (c, w, h) => cloudStrip(c, w, h, 5, 2, 0.22, 0.12, '255,214,190') },
      { depth: 9, paint: (c, w, h) => hills(c, w, h, 0.8, 0.1, '#d9876e', seeded(51)), below: '#d9876e' },
      { depth: 5, paint: (c, w, h) => hills(c, w, h, 0.88, 0.09, '#a8607c', seeded(52)), below: '#a8607c' },
      { depth: 2.5, paint: (c, w, h) => hills(c, w, h, 0.96, 0.07, '#6e4a72', seeded(53)), below: '#6e4a72' },
    ],
  },
  mountains: {
    name: 'Mountains',
    strength: 0.7,
    layers: [
      { depth: null, paint: (c, w, h) => ((c.fillStyle = vertical(c, h, [[0, '#7fb2e0'], [0.6, '#cfe3f3'], [1, '#eef5ea']])), c.fillRect(0, 0, w, h)) },
      { depth: 20, paint: (c, w, h) => cloudStrip(c, w, h, 3, 2, 0.1, 0.15) },
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
function buildings(c: CanvasRenderingContext2D, w: number, h: number, seed: number, color: string, top: number, lit: boolean) {
  const rnd = seeded(seed);
  let x = 0;
  while (x < w) {
    const bw = Math.min(40 + rnd() * 70, w - x);
    const bh = h * (1 - top) * (0.35 + 0.65 * rnd());
    const y = h - bh;
    c.fillStyle = color;
    c.fillRect(x, y, bw, bh);
    if (rnd() < 0.3) c.fillRect(x + bw * 0.4, y - 18, 3, 18);
    if (lit)
      for (let wy = y + 10; wy < h - 12; wy += 16)
        for (let wx = x + 7; wx < x + bw - 10; wx += 13) {
          c.fillStyle = rnd() < 0.35 ? 'rgba(255,233,160,0.85)' : 'rgba(225,232,240,0.45)';
          c.fillRect(wx, wy, 6, 8);
        }
    x += bw + 2 + rnd() * 10;
  }
}

/** A mountain range across w (repeating), peaks between `top` and `low` (fractions of h), snow on the far ones. */
function range(c: CanvasRenderingContext2D, w: number, h: number, seed: number, color: string, top: number, low: number, k: number) {
  const rnd = seeded(seed);
  const peaks: [number, number][] = [];
  for (let x = 0; x < w; x += 90 + rnd() * 140) peaks.push([x, h * (top + (low - top) * rnd() * 0.6)]);
  const all = [...peaks.map(([x, y]) => [x - w, y] as [number, number]), ...peaks, ...peaks.map(([x, y]) => [x + w, y] as [number, number])]; // across the seam
  c.fillStyle = color;
  c.beginPath();
  c.moveTo(-w, h);
  all.forEach(([x, y], i) => {
    const nx = all[i + 1]?.[0] ?? 2 * w;
    c.lineTo(x, y);
    c.lineTo((x + nx) / 2, h * (low - 0.05 * k));
  });
  c.lineTo(2 * w, h);
  c.closePath();
  c.fill();
  if (k < 2) {
    c.fillStyle = 'rgba(255,255,255,0.85)';
    for (const [x, y] of all) {
      const s = (h * 0.045) / (1 + k);
      c.beginPath();
      c.moveTo(x, y);
      c.lineTo(x - s * 1.2, y + s * 1.4);
      c.lineTo(x - s * 0.3, y + s);
      c.lineTo(x + s * 0.4, y + s * 1.5);
      c.lineTo(x + s * 1.2, y + s * 1.3);
      c.closePath();
      c.fill();
    }
  }
}

/** The scenes: id, name, their usual strength, and how many layers deep. */
export const SCENES = Object.entries(SCENE_DEFS).map(([id, s]) => ({ id, name: s.name, strength: s.strength, layers: s.layers.length }));

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

/** The background as drawn: its state, its images once loaded, and each scene's layers drawn once per window size. */
export class Backdrop {
  state: BackgroundState = { ...NO_BACKGROUND };
  private images = new Map<string, HTMLImageElement>();
  private cache = new Map<string, HTMLCanvasElement>();

  /** Set the background (images load in the background; until then they do not show). */
  set(state: BackgroundState) {
    this.state = { ...state, layers: state.layers.map((l) => ({ ...l, ref: { ...l.ref }, rect: { ...l.rect } })) };
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

  /** A scene layer drawn for this screen size: w wide (it repeats), the height and half again below (it runs on). */
  private sceneCanvas(id: string, i: number, w: number, h: number): HTMLCanvasElement {
    const dpr = window.devicePixelRatio || 1;
    const key = `${id}:${i}:${w}x${h}@${dpr}`;
    let cv = this.cache.get(key);
    if (cv) return cv;
    if (this.cache.size > 40) this.cache.clear();
    const layer = SCENE_DEFS[id].layers[i];
    cv = document.createElement('canvas');
    [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * 1.5 * dpr)];
    const c = cv.getContext('2d')!;
    c.scale(dpr, dpr);
    layer.paint(c, w, h);
    if (layer.below) {
      c.fillStyle = layer.below;
      c.fillRect(0, h - 1, w, h * 0.5 + 1);
    }
    this.cache.set(key, cv);
    return cv;
  }

  /** Draw it over the plain paper (w × h CSS px), for view v: the layers far to near. */
  draw(ctx: CanvasRenderingContext2D, w: number, h: number, v: View) {
    const s = this.state;
    const sceneId = s.kind === 'custom' ? s.base : s.kind;
    const scene = SCENE_DEFS[sceneId];
    const items: { depth: number; draw: () => void }[] = [];
    const [cx, cy] = toWorld(v, [w / 2, h / 2]);
    scene?.layers.forEach((layer, i) => {
      const z = layer.depth ?? Infinity;
      items.push({
        depth: z,
        draw: () => {
          const cv = this.sceneCanvas(sceneId, i, w, h);
          if (!Number.isFinite(z)) return ctx.drawImage(cv, 0, 0, w, h * 1.5);
          // it moves 1/z as far as the view: across (repeating), and up and down within what it has
          const dx = -(((cx * v.scale) / z) % w);
          const dy = Math.max(-h * 0.45, Math.min(h * 0.6, (cy * v.scale) / z));
          for (const x of [dx - w, dx, dx + w]) if (x < w && x + w > 0) ctx.drawImage(cv, x, dy, w, h * 1.5);
        },
      });
    });
    if (s.kind === 'custom')
      for (const l of s.layers) {
        const img = this.images.get(l.image);
        if (!img) continue;
        items.push({
          depth: l.depth ?? Infinity,
          draw: () => {
            const r = layerRect(l, v, w, h);
            ctx.imageSmoothingQuality = 'high';
            ctx.drawImage(img, r.x, r.y, r.w, r.h);
          },
        });
      }
    if (!items.length) return;
    ctx.save();
    ctx.globalAlpha = Math.min(1, Math.max(0, s.opacity));
    // far to near; a scene's layers before the images at the same distance, the images in their order
    items.map((it, k) => ({ ...it, k })).sort((a, b) => b.depth - a.depth || a.k - b.k).forEach((it) => it.draw());
    ctx.restore();
  }
}
