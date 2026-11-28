/**
 * The machine drawn in the style of its paper: as a blueprint (white lines on blue) or by hand in
 * pencil (graphite on graph paper or plain paper).
 *
 * The drawing code stays as it is: it draws through a stand-in for its canvas that restyles what
 * it draws.
 *  - Colours are re-inked by how dark they are: dark ink becomes the style's ink (white on a
 *    blueprint, graphite in pencil) and light fills and halos become the paper, so outlines stay
 *    outlines and what was filled to hide what is behind still hides it. Thin lines and text are
 *    inked fully; bodies (fills, thick strokes) only lightly (a blueprint's faint wash, pencil's
 *    hatching). A strongly coloured part keeps a hint of its hue (a coloured pencil), so bodies
 *    joined in one colour, the selection and force arrows still tell apart. See-through strokes
 *    (highlights) keep their colour, like a highlighter. Gradients and patterns pass as they are
 *    (a heat glow, a bar coloured by its internal force).
 *  - Text takes the style's lettering: condensed draughting letters, or handwriting.
 *  - In pencil, lines wobble a little and are gone over twice, circles don't quite close, and
 *    fills are hatched. The wobble follows each shape's own size and shape (not where it is), so
 *    a part keeps its wobble as it moves: lines don't shimmer.
 */
import type { Vec } from './render';

export type InkStyleName = 'blueprint' | 'sketch';
/** The style, and the colour of the paper the machine is drawn on (the background as it shows). */
export interface InkStyle {
  style: InkStyleName;
  paper: string;
}

type RGBA = [number, number, number, number];
const BLUEPRINT_INK: RGBA = [244, 250, 255, 1];
const GRAPHITE: RGBA = [40, 40, 46, 1];
const LETTERING: Record<InkStyleName, string> = {
  blueprint: '"Avenir Next Condensed", "Roboto Condensed", "Arial Narrow", "Helvetica Neue", sans-serif',
  sketch: '"Bradley Hand", "Segoe Print", "Comic Sans MS", "Chalkboard SE", cursive',
};

// ---- colours ----
let parser: CanvasRenderingContext2D | null = null;
const parsed = new Map<string, RGBA>();
/** Any CSS colour as red, green, blue (0–255) and alpha (0–1). */
export function parseColor(c: string): RGBA {
  let v = parsed.get(c);
  if (v) return v;
  const t = c.trim().toLowerCase();
  const n = t.match(/-?[\d.]+/g)?.map(Number) ?? [];
  if (/^#[0-9a-f]{6}$/.test(t)) v = [parseInt(t.slice(1, 3), 16), parseInt(t.slice(3, 5), 16), parseInt(t.slice(5, 7), 16), 1];
  else if (/^#[0-9a-f]{3}$/.test(t)) v = [1, 2, 3].map((i) => parseInt(t[i] + t[i], 16)).concat(1) as RGBA;
  else if (t.startsWith('rgb') && n.length >= 3) v = [n[0], n[1], n[2], n[3] ?? 1];
  else if (t.startsWith('hsl') && n.length >= 3) {
    // hsl(h, s%, l%[, a])
    const [h, S, L] = [((n[0] % 360) + 360) % 360, n[1] / 100, n[2] / 100];
    const k = (m: number) => (m + h / 30) % 12;
    const a = S * Math.min(L, 1 - L);
    const f = (m: number) => 255 * (L - a * Math.max(-1, Math.min(k(m) - 3, 9 - k(m), 1)));
    v = [f(0), f(8), f(4), n[3] ?? 1];
  } else if (typeof document !== 'undefined') {
    // a colour by name: as the browser reads it
    parser ??= document.createElement('canvas').getContext('2d')!;
    parser.fillStyle = '#000';
    parser.fillStyle = c;
    v = parseColor(String(parser.fillStyle));
  } else v = [0, 0, 0, 1];
  if (parsed.size > 4000) parsed.clear();
  parsed.set(c, v);
  return v;
}
const mix = (a: RGBA, b: RGBA, t: number): RGBA => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t, a[3]];
const css = (c: RGBA, a: number) => `rgba(${Math.round(c[0])},${Math.round(c[1])},${Math.round(c[2])},${+a.toFixed(3)})`;
/** How light (0–1) and how strongly coloured (0–1) a colour is. */
function lightness([r, g, b]: RGBA): { L: number; sat: number } {
  const L = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255;
  const [hi, lo] = [Math.max(r, g, b), Math.min(r, g, b)];
  return { L, sat: hi ? (hi - lo) / hi : 0 };
}

/** How a colour is used: a thin line or text ('line'), or a body (a fill, a thick stroke). */
type Use = 'line' | 'body';

/**
 * A colour re-inked in a style on paper `G`: the colour, and (pencil bodies) how strongly to
 * hatch over it. `seeThrough`: drawn faintly (a highlight), so kept in its colour.
 */
export function reink(c: string, use: Use, ink: InkStyleName, G: RGBA, seeThrough = false): { color: string; hatch: number } {
  const o = parseColor(c);
  const a = o[3];
  if (seeThrough || a < 0.6) return { color: c, hatch: 0 };
  const { L, sat } = lightness(o);
  const d = 1 - L;
  const K = ink === 'blueprint' ? BLUEPRINT_INK : GRAPHITE;
  // a hint of the colour: pastel on a blueprint, a coloured pencil on paper
  const hue = ink === 'blueprint' ? mix(o, BLUEPRINT_INK, 0.6) : mix(o, GRAPHITE, 0.2);
  if (use === 'line' || d > 0.72) {
    const base = mix(K, G, Math.min(0.95, L * L * L)); // dark → ink, white (a halo) → paper
    return { color: css(mix(base, hue, (ink === 'blueprint' ? 0.3 : 0.3) * sat), a), hatch: 0 };
  }
  // a body: the paper, washed a little with ink (and in pencil, hatched), the darker the more
  const base = ink === 'blueprint' ? mix(G, K, 0.1 + 0.32 * d) : mix(G, K, 0.05 + 0.2 * d);
  return { color: css(mix(base, hue, (ink === 'blueprint' ? 0.25 : 0.18) * sat), a), hatch: ink === 'sketch' && d > 0.15 ? 0.1 + 0.35 * d : 0 };
}

/** The colour of a size from 0 to 1 in a style: blue to white on a blueprint, grey to black in pencil. */
export function inkStressColor(f: number, ink: InkStyleName): string {
  const x = Math.min(1, Math.max(0, f));
  const [lo, hi]: [RGBA, RGBA] = ink === 'blueprint' ? [[38, 88, 168, 1], [255, 255, 255, 1]] : [[205, 205, 205, 1], [18, 18, 18, 1]];
  return css(mix(lo, hi, ink === 'blueprint' ? Math.sqrt(x) : x), 1);
}

/** A font in the style's lettering, at its size and weight (handwriting a little larger: it runs small). */
export const letter = (font: string, ink: InkStyleName) => font.replace(/(\d+(?:\.\d+)?)px\s+.*$/, (_, px) => `${+(Number(px) * (ink === 'sketch' ? 1.2 : 1)).toFixed(1)}px ${LETTERING[ink]}`);

// ---- the pencil's wobble ----
interface Sub {
  pts: Vec[];
  closed: boolean;
}

/** A path's wobble seed: from its shape (length and points), not its place, so it keeps it as it moves. */
function shapeSeed(subs: Sub[]): number {
  let len = 0;
  let n = 0;
  for (const s of subs) {
    n += s.pts.length;
    for (let i = 1; i < s.pts.length; i++) len += Math.hypot(s.pts[i][0] - s.pts[i - 1][0], s.pts[i][1] - s.pts[i - 1][1]);
  }
  return (Math.round(len / 6) * 7919 + n * 104729 + subs.length * 31) % 100003;
}

/** Build the path on the canvas, each line wobbling sideways a little (seed: how). */
function wobblePath(ctx: CanvasRenderingContext2D, subs: Sub[], seed: number, shift = 0) {
  ctx.beginPath();
  const [p1, p2] = [(seed % 628) / 100, ((seed * 7) % 628) / 100];
  for (const s of subs) {
    const P = s.pts;
    if (!P.length) continue;
    let total = 0;
    for (let i = 1; i < P.length; i++) total += Math.hypot(P[i][0] - P[i - 1][0], P[i][1] - P[i - 1][1]);
    const amp = Math.min(0.8, 0.2 + total * 0.0015);
    const noise = (u: number) => amp * (0.45 * Math.sin(u * 0.02 + p1) + 0.35 * Math.sin(u * 0.09 + p2) + 0.2 * Math.sin(u * 0.31 + p1 * 2));
    let u = 0;
    ctx.moveTo(P[0][0] + shift, P[0][1] + shift * 0.6 + noise(0));
    const last = s.closed ? [...P, P[0]] : P;
    for (let i = 1; i < last.length; i++) {
      const [a, b] = [last[i - 1], last[i]];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (L < 1e-9) continue;
      const [nx, ny] = [-(b[1] - a[1]) / L, (b[0] - a[0]) / L];
      const k = Math.max(1, Math.ceil(L / 8));
      for (let j = 1; j <= k; j++) {
        const t = j / k;
        const off = noise(u + L * t);
        ctx.lineTo(a[0] + (b[0] - a[0]) * t + nx * off + shift, a[1] + (b[1] - a[1]) * t + ny * off + shift * 0.6);
      }
      u += L;
    }
    // (a closed shape is left a hair open or over, as by hand)
  }
}

/** Points along an arc of an ellipse (canvas angle conventions). */
function arcPoints(x: number, y: number, rx: number, ry: number, rot: number, a0: number, a1: number, ccw: boolean): Vec[] {
  const TAU = Math.PI * 2;
  let sweep = a1 - a0;
  if (!ccw) {
    if (sweep < 0) sweep = (sweep % TAU) + TAU;
    if (a1 - a0 >= TAU) sweep = TAU;
  } else {
    if (sweep > 0) sweep = (sweep % TAU) - TAU;
    if (a0 - a1 >= TAU) sweep = -TAU;
  }
  const n = Math.max(6, Math.ceil((Math.abs(sweep) * Math.max(rx, ry)) / 5));
  const [c, s] = [Math.cos(rot), Math.sin(rot)];
  const out: Vec[] = [];
  for (let i = 0; i <= n; i++) {
    const t = a0 + (sweep * i) / n;
    const [ex, ey] = [rx * Math.cos(t), ry * Math.sin(t)];
    out.push([x + ex * c - ey * s, y + ex * s + ey * c]);
  }
  return out;
}

/** Diagonal pencil hatching, as a pattern on that canvas. */
const hatches = new WeakMap<CanvasRenderingContext2D, CanvasPattern>();
function hatchOf(ctx: CanvasRenderingContext2D): CanvasPattern | null {
  let p = hatches.get(ctx);
  if (p) return p;
  const cv = document.createElement('canvas');
  cv.width = cv.height = 7;
  const c = cv.getContext('2d')!;
  c.strokeStyle = css(GRAPHITE, 1);
  c.lineWidth = 0.9;
  c.beginPath();
  for (const k of [-7, 0, 7]) (c.moveTo(k, 7), c.lineTo(k + 7, 0));
  c.stroke();
  p = ctx.createPattern(cv, 'repeat') ?? undefined;
  if (p) hatches.set(ctx, p);
  return p ?? null;
}

/**
 * A stand-in for `ctx` that draws in the style: hand it to drawing code in place of the canvas.
 * Anything it does not restyle goes straight to the canvas.
 */
export function inkedContext(ctx: CanvasRenderingContext2D, ink: InkStyle): CanvasRenderingContext2D {
  const G = parseColor(ink.paper);
  const sketch = ink.style === 'sketch';
  type Paint = string | CanvasGradient | CanvasPattern;
  let fill: Paint = '#000';
  let stroke: Paint = '#000';
  let font = '10px sans-serif';
  const stack: [Paint, Paint, string][] = [];
  let subs: Sub[] = [];
  let cur: Sub | null = null;
  const at = (p: Vec) => {
    if (!cur) {
      cur = { pts: [], closed: false };
      subs.push(cur);
    }
    cur.pts.push(p);
  };
  /** Drawn faintly: a highlight (strokes keep their colour). */
  const seeThrough = () => ctx.globalAlpha < 0.6;
  const strokeColor = () => {
    if (typeof stroke !== 'string') return { style: stroke, body: false };
    const body = ctx.lineWidth >= 6;
    return { style: reink(stroke, body ? 'body' : 'line', ink.style, G, seeThrough()).color, body };
  };

  const own: Record<string, unknown> = {
    save() {
      stack.push([fill, stroke, font]);
      ctx.save();
    },
    restore() {
      const top = stack.pop();
      if (top) [fill, stroke, font] = top;
      ctx.restore();
    },
    stroke() {
      const { style, body } = strokeColor();
      ctx.strokeStyle = style;
      if (!sketch) return ctx.stroke();
      const seed = shapeSeed(subs);
      wobblePath(ctx, subs, seed);
      ctx.stroke();
      if (body || seeThrough() || typeof stroke !== 'string') return;
      // gone over again, a little lighter and off the first line
      const [a, w] = [ctx.globalAlpha, ctx.lineWidth];
      ctx.globalAlpha = a * 0.45;
      ctx.lineWidth = w * 0.75;
      wobblePath(ctx, subs, seed + 101, 0.5);
      ctx.stroke();
      ctx.globalAlpha = a;
      ctx.lineWidth = w;
    },
    fill(rule?: CanvasFillRule) {
      if (typeof fill !== 'string') {
        ctx.fillStyle = fill;
        if (sketch) wobblePath(ctx, subs, shapeSeed(subs));
        return ctx.fill(rule);
      }
      // (a faint fill is a body's tint, not a highlight: washed and hatched like any body)
      const { color, hatch } = reink(fill, 'body', ink.style, G);
      ctx.fillStyle = color;
      if (!sketch) return ctx.fill(rule);
      wobblePath(ctx, subs, shapeSeed(subs));
      ctx.fill(rule);
      const pattern = hatch > 0 ? hatchOf(ctx) : null;
      if (pattern) {
        const a = ctx.globalAlpha;
        ctx.globalAlpha = Math.min(1, a * 2.5) * hatch;
        ctx.fillStyle = pattern;
        ctx.fill(rule);
        ctx.globalAlpha = a;
      }
    },
    fillRect(x: number, y: number, w: number, h: number) {
      ctx.fillStyle = typeof fill === 'string' ? reink(fill, 'body', ink.style, G).color : fill;
      ctx.fillRect(x, y, w, h);
    },
    strokeRect(x: number, y: number, w: number, h: number) {
      if (!sketch) {
        ctx.strokeStyle = strokeColor().style;
        return ctx.strokeRect(x, y, w, h);
      }
      // as a path of its own, keeping the path being built
      const keep = [subs, cur] as const;
      subs = [{ pts: [[x, y], [x + w, y], [x + w, y + h], [x, y + h]], closed: true }];
      (own.stroke as () => void)();
      [subs, cur] = [keep[0], keep[1]];
    },
    fillText(text: string, x: number, y: number, max?: number) {
      ctx.fillStyle = typeof fill === 'string' ? reink(fill, 'line', ink.style, G, seeThrough()).color : fill;
      ctx.fillText(text, x, y, max);
    },
    strokeText(text: string, x: number, y: number, max?: number) {
      ctx.strokeStyle = typeof stroke === 'string' ? reink(stroke, 'line', ink.style, G, seeThrough()).color : stroke;
      ctx.strokeText(text, x, y, max);
    },
  };
  if (sketch) {
    // the pencil keeps the path itself, to draw it wobbling
    Object.assign(own, {
      beginPath() {
        subs = [];
        cur = null;
      },
      moveTo(x: number, y: number) {
        cur = { pts: [[x, y]], closed: false };
        subs.push(cur);
      },
      lineTo(x: number, y: number) {
        at([x, y]);
      },
      closePath() {
        if (!cur) return;
        cur.closed = true;
        const start = cur.pts[0];
        cur = { pts: start ? [start] : [], closed: false };
        subs.push(cur);
      },
      arc(x: number, y: number, r: number, a0: number, a1: number, ccw = false) {
        for (const p of arcPoints(x, y, r, r, 0, a0, a1, ccw)) at(p);
      },
      ellipse(x: number, y: number, rx: number, ry: number, rot: number, a0: number, a1: number, ccw = false) {
        for (const p of arcPoints(x, y, rx, ry, rot, a0, a1, ccw)) at(p);
      },
      quadraticCurveTo(cx: number, cy: number, x: number, y: number) {
        const p0 = cur?.pts[cur.pts.length - 1] ?? [cx, cy];
        for (let i = 1; i <= 10; i++) {
          const t = i / 10;
          at([(1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * cx + t * t * x, (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * cy + t * t * y]);
        }
      },
      bezierCurveTo(c1x: number, c1y: number, c2x: number, c2y: number, x: number, y: number) {
        const p0 = cur?.pts[cur.pts.length - 1] ?? [c1x, c1y];
        for (let i = 1; i <= 14; i++) {
          const t = i / 14;
          const m = 1 - t;
          at([m ** 3 * p0[0] + 3 * m * m * t * c1x + 3 * m * t * t * c2x + t ** 3 * x, m ** 3 * p0[1] + 3 * m * m * t * c1y + 3 * m * t * t * c2y + t ** 3 * y]);
        }
      },
      rect(x: number, y: number, w: number, h: number) {
        subs.push({ pts: [[x, y], [x + w, y], [x + w, y + h], [x, y + h]], closed: true });
        cur = { pts: [[x, y]], closed: false };
        subs.push(cur);
      },
      roundRect(x: number, y: number, w: number, h: number, radii: number | DOMPointInit | (number | DOMPointInit)[] = 0) {
        const r0 = Array.isArray(radii) ? radii[0] : radii;
        const r = Math.max(0, Math.min(typeof r0 === 'number' ? r0 : (r0?.x ?? 0), Math.abs(w) / 2, Math.abs(h) / 2));
        const pts: Vec[] = [
          ...arcPoints(x + w - r, y + r, r, r, 0, -Math.PI / 2, 0, false),
          ...arcPoints(x + w - r, y + h - r, r, r, 0, 0, Math.PI / 2, false),
          ...arcPoints(x + r, y + h - r, r, r, 0, Math.PI / 2, Math.PI, false),
          ...arcPoints(x + r, y + r, r, r, 0, Math.PI, Math.PI * 1.5, false),
        ];
        subs.push({ pts, closed: true });
        cur = { pts: [[x, y]], closed: false };
        subs.push(cur);
      },
    });
  }
  return new Proxy(ctx, {
    get(target, prop) {
      if (prop === 'fillStyle') return fill;
      if (prop === 'strokeStyle') return stroke;
      if (prop === 'font') return font;
      if (typeof prop === 'string' && prop in own) return own[prop];
      const v = Reflect.get(target, prop, target);
      return typeof v === 'function' ? v.bind(target) : v;
    },
    set(target, prop, value) {
      if (prop === 'fillStyle') fill = value;
      else if (prop === 'strokeStyle') stroke = value;
      else if (prop === 'font') {
        font = value;
        target.font = letter(String(value), ink.style);
      } else if (prop === 'shadowColor') target.shadowColor = reink(String(value), 'line', ink.style, G).color;
      else Reflect.set(target, prop, value, target);
      return true;
    },
  });
}
