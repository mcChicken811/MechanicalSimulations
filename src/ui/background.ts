/**
 * Backgrounds behind the drawing: a scene drawn by the app (sky with clouds, a city, a night sky,
 * a sunset, mountains, a blueprint, graph paper), or the user's own image (PNG, JPEG, SVG…).
 *
 *  - A scene fills the screen and stays put as the view moves; it is drawn once per window size
 *    (from a fixed seed, so its clouds and stars do not jump about) and kept.
 *  - An image either fills the screen too ('cover': scaled to cover it, centred) or lies in the
 *    drawing ('world'): a rectangle in metres, so it pans and zooms with the mechanism, to trace
 *    over or to stand things on.
 *  - Its strength (0–1) is how strongly it shows over the plain paper.
 *
 * The state is plain data (the image as a data URL), saved with projects.
 */
import { toScreen, type View } from './render';

export interface BackgroundState {
  /** 'none', a scene's id, or 'image'. */
  kind: string;
  /** The image, as a data URL. */
  image?: string;
  /** How an image is placed: filling the screen, or in the drawing at `rect`. */
  mode: 'cover' | 'world';
  /** In the drawing: its left and top edges and its width (m); its height follows its shape. */
  rect?: { x: number; y: number; w: number };
  /** How strongly it shows, 0–1. */
  opacity: number;
}

export const NO_BACKGROUND: BackgroundState = { kind: 'none', mode: 'world', opacity: 0.7 };

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

type Painter = (c: CanvasRenderingContext2D, w: number, h: number) => void;

const vertical = (c: CanvasRenderingContext2D, h: number, stops: [number, string][]) => {
  const g = c.createLinearGradient(0, 0, 0, h);
  for (const [at, col] of stops) g.addColorStop(at, col);
  return g;
};

/** A soft white cloud of overlapping puffs, about `size` px across, its base at y. */
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
  // a flat, faintly shaded base
  c.fillStyle = 'rgba(160,180,205,0.18)';
  c.beginPath();
  c.ellipse(x, y - size * 0.04, size * 0.46, size * 0.07, 0, 0, Math.PI * 2);
  c.fill();
}

/** Rolling hills along the bottom: a smooth band of colour from height `base` (fraction of h). */
function hills(c: CanvasRenderingContext2D, w: number, h: number, base: number, amp: number, color: string, rnd: () => number) {
  const k = 3 + Math.floor(rnd() * 3);
  const ph = rnd() * Math.PI * 2;
  c.fillStyle = color;
  c.beginPath();
  c.moveTo(0, h);
  for (let x = 0; x <= w; x += 8) c.lineTo(x, h * base - amp * h * (0.5 + 0.5 * Math.sin((x / w) * Math.PI * k + ph)) * (0.7 + 0.3 * Math.sin((x / w) * 7 + ph * 2)));
  c.lineTo(w, h);
  c.closePath();
  c.fill();
}

const SCENE_PAINTERS: Record<string, { name: string; strength: number; paint: Painter }> = {
  sky: {
    name: 'Blue sky with clouds',
    strength: 0.75,
    paint: (c, w, h) => {
      const rnd = seeded(7);
      c.fillStyle = vertical(c, h, [[0, '#5aa5e6'], [0.55, '#a9d4f5'], [1, '#eaf6fd']]);
      c.fillRect(0, 0, w, h);
      const n = Math.max(5, Math.round(w / 220));
      for (let i = 0; i < n; i++) cloud(c, ((i + 0.3 + 0.5 * rnd()) / n) * w, h * (0.15 + 0.45 * rnd()), 110 + 120 * rnd(), rnd);
    },
  },
  city: {
    name: 'City skyline',
    strength: 0.7,
    paint: (c, w, h) => {
      const rnd = seeded(11);
      c.fillStyle = vertical(c, h, [[0, '#8fbbe6'], [0.6, '#d9e6f2'], [1, '#f6e3c8']]);
      c.fillRect(0, 0, w, h);
      cloud(c, w * 0.22, h * 0.2, 160, rnd);
      cloud(c, w * 0.7, h * 0.14, 120, rnd);
      // two rows of buildings: the far ones pale, the near ones darker with lit windows
      for (const [row, color, top, lit] of [
        [0, '#b4c3d4', 0.42, false],
        [1, '#73859c', 0.55, true],
      ] as const) {
        let x = -20 + rnd() * 30 * row;
        while (x < w) {
          const bw = 40 + rnd() * 70;
          const bh = h * (1 - top) * (0.35 + 0.65 * rnd());
          const y = h - bh;
          c.fillStyle = color;
          c.fillRect(x, y, bw, bh);
          if (rnd() < 0.3) c.fillRect(x + bw * 0.4, y - 18, 3, 18); // an antenna
          if (lit)
            for (let wy = y + 10; wy < h - 12; wy += 16)
              for (let wx = x + 7; wx < x + bw - 10; wx += 13) {
                c.fillStyle = rnd() < 0.35 ? 'rgba(255,233,160,0.85)' : 'rgba(225,232,240,0.45)';
                c.fillRect(wx, wy, 6, 8);
              }
          x += bw + 2 + rnd() * 10;
        }
      }
    },
  },
  night: {
    name: 'Night sky',
    strength: 0.55,
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
      // the moon
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
      for (const [dx, dy, s] of [[-0.3, -0.2, 0.22], [0.25, 0.2, 0.16], [0.1, -0.35, 0.1]])
        c.beginPath(), c.arc(mx + dx * r, my + dy * r, s * r, 0, Math.PI * 2), c.fill();
      hills(c, w, h, 0.94, 0.08, '#16223f', rnd);
    },
  },
  sunset: {
    name: 'Sunset over hills',
    strength: 0.7,
    paint: (c, w, h) => {
      const rnd = seeded(5);
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
      cloud(c, w * 0.25, h * 0.28, 180, rnd, '255,214,190');
      hills(c, w, h, 0.8, 0.1, '#d9876e', rnd);
      hills(c, w, h, 0.88, 0.09, '#a8607c', rnd);
      hills(c, w, h, 0.96, 0.07, '#6e4a72', rnd);
    },
  },
  mountains: {
    name: 'Mountains',
    strength: 0.7,
    paint: (c, w, h) => {
      const rnd = seeded(3);
      c.fillStyle = vertical(c, h, [[0, '#7fb2e0'], [0.6, '#cfe3f3'], [1, '#eef5ea']]);
      c.fillRect(0, 0, w, h);
      cloud(c, w * 0.3, h * 0.18, 150, rnd);
      // three ranges, far to near; the far ones have snow
      for (const [k, color, top, low, snow] of [
        [0, '#9fb3c8', 0.3, 0.7, true],
        [1, '#7a92ac', 0.42, 0.8, true],
        [2, '#5b7389', 0.6, 0.9, false],
      ] as const) {
        const peaks: [number, number][] = [];
        for (let x = -60; x < w + 120; x += 90 + rnd() * 140) peaks.push([x, h * (top + (low - top) * rnd() * 0.6)]);
        c.fillStyle = color;
        c.beginPath();
        c.moveTo(-60, h);
        peaks.forEach(([x, y], i) => {
          const nx = peaks[i + 1]?.[0] ?? w + 120;
          c.lineTo(x, y);
          c.lineTo((x + nx) / 2, h * (low - 0.05 * k));
        });
        c.lineTo(w + 120, h);
        c.closePath();
        c.fill();
        if (snow) {
          c.fillStyle = 'rgba(255,255,255,0.85)';
          for (const [x, y] of peaks) {
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
      hills(c, w, h, 0.96, 0.05, '#7fa56a', rnd);
    },
  },
  blueprint: {
    name: 'Blueprint',
    strength: 0.6,
    paint: (c, w, h) => {
      c.fillStyle = '#2a5d9c';
      c.fillRect(0, 0, w, h);
      c.lineWidth = 1;
      for (const [step, alpha] of [[20, 0.1], [100, 0.22]] as const) {
        c.strokeStyle = `rgba(255,255,255,${alpha})`;
        c.beginPath();
        for (let x = 0.5; x < w; x += step) (c.moveTo(x, 0), c.lineTo(x, h));
        for (let y = 0.5; y < h; y += step) (c.moveTo(0, y), c.lineTo(w, y));
        c.stroke();
      }
    },
  },
  paper: {
    name: 'Graph paper',
    strength: 0.8,
    paint: (c, w, h) => {
      c.fillStyle = '#fbf8ef';
      c.fillRect(0, 0, w, h);
      c.lineWidth = 1;
      for (const [step, alpha] of [[10, 0.12], [50, 0.3]] as const) {
        c.strokeStyle = `rgba(110,160,140,${alpha})`;
        c.beginPath();
        for (let x = 0.5; x < w; x += step) (c.moveTo(x, 0), c.lineTo(x, h));
        for (let y = 0.5; y < h; y += step) (c.moveTo(0, y), c.lineTo(w, y));
        c.stroke();
      }
    },
  },
};

/** The scenes: id, name and their usual strength. */
export const SCENES = Object.entries(SCENE_PAINTERS).map(([id, s]) => ({ id, name: s.name, strength: s.strength }));

/**
 * An image file for a background, as a data URL: an SVG as it is; a picture scaled down to at
 * most `most` px across (kept in the browser and in projects, so not too big), as PNG if it has
 * transparency to keep, else JPEG.
 */
export async function imageFileToDataUrl(file: File, most = 2048): Promise<{ url: string; width: number; height: number }> {
  const raw = await new Promise<string>((res, rej) => {
    const r = new FileReader();
    r.onload = () => res(String(r.result));
    r.onerror = () => rej(new Error(`could not read ${file.name}`));
    r.readAsDataURL(file);
  });
  const img = await loadImage(raw);
  const [w0, h0] = [img.naturalWidth || 1000, img.naturalHeight || 750];
  if (file.type === 'image/svg+xml' || /\.svg$/i.test(file.name)) return { url: raw, width: w0, height: h0 };
  const k = Math.min(1, most / Math.max(w0, h0));
  if (k === 1 && file.size < 1.5e6) return { url: raw, width: w0, height: h0 };
  const cv = document.createElement('canvas');
  [cv.width, cv.height] = [Math.round(w0 * k), Math.round(h0 * k)];
  cv.getContext('2d')!.drawImage(img, 0, 0, cv.width, cv.height);
  const png = file.type === 'image/png' || file.type === 'image/webp' || file.type === 'image/gif';
  return { url: cv.toDataURL(png ? 'image/png' : 'image/jpeg', 0.86), width: cv.width, height: cv.height };
}

function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((res, rej) => {
    const img = new Image();
    img.onload = () => res(img);
    img.onerror = () => rej(new Error('not an image this browser can show'));
    img.src = url;
  });
}

/** The background as drawn: its state, its image once loaded, and each scene drawn once per window size. */
export class Backdrop {
  state: BackgroundState = { ...NO_BACKGROUND };
  private img: HTMLImageElement | null = null;
  private imgUrl = '';
  private cache: { key: string; canvas: HTMLCanvasElement } | null = null;

  /** Set the background (the image loads in the background; until then nothing shows). */
  set(state: BackgroundState) {
    this.state = { ...state, rect: state.rect ? { ...state.rect } : undefined };
    if (state.kind === 'image' && state.image && state.image !== this.imgUrl) {
      this.imgUrl = state.image;
      this.img = null;
      loadImage(state.image).then(
        (img) => {
          if (this.imgUrl === state.image) this.img = img;
        },
        () => {},
      );
    }
  }

  /** The image's shape (height over width), once loaded. */
  get aspect(): number {
    const i = this.img;
    return i && i.naturalWidth ? i.naturalHeight / i.naturalWidth : 0.75;
  }

  get name(): string {
    const k = this.state.kind;
    return k === 'image' ? 'your image' : k === 'none' ? '' : (SCENE_PAINTERS[k]?.name ?? '');
  }

  /** Draw it over the plain paper (w × h CSS px, the view for an image in the drawing). */
  draw(ctx: CanvasRenderingContext2D, w: number, h: number, v: View) {
    const s = this.state;
    if (s.kind === 'none') return;
    ctx.save();
    ctx.globalAlpha = Math.min(1, Math.max(0, s.opacity));
    const scene = SCENE_PAINTERS[s.kind];
    if (scene) {
      const dpr = window.devicePixelRatio || 1;
      const key = `${s.kind}:${w}x${h}@${dpr}`;
      if (this.cache?.key !== key) {
        const cv = document.createElement('canvas');
        [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * dpr)];
        const c = cv.getContext('2d')!;
        c.scale(dpr, dpr);
        scene.paint(c, w, h);
        this.cache = { key, canvas: cv };
      }
      ctx.drawImage(this.cache.canvas, 0, 0, w, h);
    } else if (s.kind === 'image' && this.img) {
      const img = this.img;
      const [iw, ih] = [img.naturalWidth || 1000, img.naturalHeight || 750];
      if (s.mode === 'world' && s.rect) {
        const [x0, y0] = toScreen(v, [s.rect.x, s.rect.y]);
        const pw = s.rect.w * v.scale;
        ctx.imageSmoothingQuality = 'high';
        ctx.drawImage(img, x0, y0, pw, (pw * ih) / iw);
      } else {
        const k = Math.max(w / iw, h / ih); // cover the screen, centred
        ctx.drawImage(img, (w - iw * k) / 2, (h - ih * k) / 2, iw * k, ih * k);
      }
    }
    ctx.restore();
  }
}
