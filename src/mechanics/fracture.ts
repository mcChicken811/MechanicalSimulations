/**
 * Bars that break: where a bar's bending moment reaches a threshold it snaps, into rigid pieces
 * that carry on with the motion they had.
 *
 *  - breakPoints: where along a bar it breaks, from its diagram: the place of largest |M| in
 *    each stretch where |M| reaches the threshold (one bar may break in several places).
 *  - breakBars: the design with those bars cut there. The first piece keeps the bar's id; each
 *    cut makes two free joints, one at the end of each piece, where the cut was drawn. Joints
 *    pinned onto the bar, rails on it and welds to it go with the piece they are on.
 *  - transferState: the state of the cut mechanism that matches the old one, every joint where
 *    it was, moving as it was (each piece moves on as part of the bar did), every gear, motor
 *    housing and propeller turned as it was. Nothing jumps: the energy carries on unchanged.
 */
import { solveInPlace } from './linalg';
import { cloneDesign, nextId, type DBodyRef, type Design } from './model';
import type { BeamDiagram } from './beam';
import type { NumericSystem } from './simulate';

/**
 * Where a bar breaks (m from its end A): in each stretch where |M| ≥ threshold, the place of the
 * largest |M|. Kept a little off its ends (a stub at the joint rather than a cut through it) and
 * apart from each other, by `margin` (a fraction of its length).
 */
export function breakPoints(d: BeamDiagram, threshold: number, margin = 0.04): number[] {
  const [lo, hi] = [margin * d.L, (1 - margin) * d.L];
  const found: number[] = [];
  let best = -1;
  for (let k = 0; k < d.s.length; k++) {
    const over = Number.isFinite(d.M[k]) && Math.abs(d.M[k]) >= threshold;
    if (over && (best < 0 || Math.abs(d.M[k]) > Math.abs(d.M[best]))) best = k;
    if ((!over || k === d.s.length - 1) && best >= 0) {
      found.push(Math.min(hi, Math.max(lo, d.s[best])));
      best = -1;
    }
  }
  const out: number[] = [];
  for (const x of found.sort((a, b) => a - b)) if (!out.length || x - out[out.length - 1] >= margin * d.L) out.push(x);
  return out;
}

/** A cut: the free joint made at it, the bar it was cut from (its ends then), and where (fraction from a). */
export interface Cut {
  point: number;
  link: number;
  a: number;
  b: number;
  u: number;
}

/** A piece of a broken bar: its id, the bar it came from, and its span along it (fractions from a). */
export interface Piece {
  id: number;
  from: number;
  u0: number;
  u1: number;
}

/** Cut bars at fractions `u` of the way from their joint a to b (racks do not break). */
export function breakBars(src: Design, breaks: { link: number; u: number[] }[]): { design: Design; cuts: Cut[]; pieces: Piece[] } {
  const d = cloneDesign(src);
  const cuts: Cut[] = [];
  const pieces: Piece[] = [];
  let pid = nextId(d.points);
  let lid = nextId(d.links);
  const pt = (id: number) => d.points.find((p) => p.id === id)!;
  for (const br of breaks) {
    const l = d.links.find((x) => x.id === br.link);
    if (!l || l.rack) continue;
    const us = [...new Set(br.u)].filter((u) => u > 1e-6 && u < 1 - 1e-6).sort((a, b) => a - b);
    if (!us.length) continue;
    const [A, B] = [pt(l.a), pt(l.b)];
    const [a0, b0] = [{ x: A.x, y: A.y }, { x: B.x, y: B.y }];
    const len2 = (b0.x - a0.x) ** 2 + (b0.y - a0.y) ** 2 || 1e-12;
    const along = (x: number, y: number) => ((x - a0.x) * (b0.x - a0.x) + (y - a0.y) * (b0.y - a0.y)) / len2;
    const bounds = [0, ...us, 1];
    const ids = bounds.slice(1).map((_, i) => (i === 0 ? l.id : lid++));
    const owner = (u: number) => {
      let i = 0;
      while (i < us.length && u > us[i]) i++;
      return ids[i];
    };
    const [origA, origB, mass] = [l.a, l.b, l.mass];
    const cutPoint = (u: number) => {
      const id = pid++;
      d.points.push({ id, x: a0.x + u * (b0.x - a0.x), y: a0.y + u * (b0.y - a0.y), kind: 'free', mass: 0 });
      cuts.push({ point: id, link: l.id, a: origA, b: origB, u });
      return id;
    };
    // what rides on the bar goes with the piece it is on (decided before the bar changes)
    const onBar = (r: DBodyRef | null | undefined) => r?.kind === 'link' && r.id === l.id;
    const riders = d.points.filter((p) => onBar(p.pin) || onBar(p.slide?.on)).map((p) => ({ p, piece: owner(along(p.x, p.y)) }));
    const nodePiece = (id: number): number | undefined => (id === origA ? ids[0] : id === origB ? ids[ids.length - 1] : riders.find((r) => r.p.id === id)?.piece);
    for (let i = 0; i < ids.length; i++) {
      const [u0, u1] = [bounds[i], bounds[i + 1]];
      const a = i === 0 ? origA : cutPoint(u0);
      const b = i === ids.length - 1 ? origB : cutPoint(u1);
      const m = mass * (u1 - u0);
      if (i === 0) Object.assign(l, { b, mass: m });
      else d.links.push({ ...l, id: ids[i], a, b, mass: m });
      pieces.push({ id: ids[i], from: l.id, u0, u1 });
    }
    for (const { p, piece } of riders) {
      if (onBar(p.pin)) p.pin = { kind: 'link', id: piece };
      if (p.slide && onBar(p.slide.on)) p.slide = { ...p.slide, on: { kind: 'link', id: piece } };
    }
    // a weld goes with the piece at the joint it shares with the other body, else the longest piece
    const longest = ids[bounds.slice(1).map((u, i) => u - bounds[i]).reduce((bi, w, i, ws) => (w > ws[bi] ? i : bi), 0)];
    for (const j of d.joins ?? [])
      for (const side of ['a', 'b'] as const) {
        if (!onBar(j[side])) continue;
        const other = j[side === 'a' ? 'b' : 'a'];
        const nodes = other.kind === 'link' ? (() => {
          const o = d.links.find((x) => x.id === other.id);
          return o ? [o.a, o.b] : [];
        })() : d.points.filter((p) => p.id === d.gears.find((g) => g.id === other.id)?.center || p.gear === other.id).map((p) => p.id);
        const shared = nodes.map(nodePiece).find((x) => x !== undefined);
        j[side] = { kind: 'link', id: shared ?? longest };
      }
  }
  return { design: d, cuts, pieces };
}

/** What a state is matched on: joint positions, gear turns, motor housing turns, propeller spins. */
function features(sys: NumericSystem): { keys: string[]; eval: (q: ArrayLike<number>, out: Float64Array) => void } {
  const der = sys.derivation;
  const pts = [...sys.pointIndex.entries()];
  const keys = [...pts.flatMap(([id]) => [`x${id}`, `y${id}`]), ...sys.gearIds.map((id) => `g${id}`), ...der.motorAngles.map((m) => `m${m.id}`), ...der.propellers.filter((p) => p.spin >= 0).map((p) => `s${p.id}`)];
  const pos = new Float64Array(2 * sys.pointIndex.size);
  const ga = new Float64Array(sys.gearIds.length);
  const ma = new Float64Array(der.motorAngles.length);
  const spins = der.propellers.filter((p) => p.spin >= 0);
  return {
    keys,
    eval: (q, out) => {
      sys.positions(q, pos);
      let k = 0;
      for (const [, i] of pts) {
        out[k++] = pos[2 * i];
        out[k++] = pos[2 * i + 1];
      }
      sys.gearAngles(q, ga);
      for (let i = 0; i < ga.length; i++) out[k++] = ga[i];
      sys.motorAngles(q, ma);
      for (let i = 0; i < ma.length; i++) out[k++] = ma[i];
      for (const p of spins) out[k++] = q[p.spin];
    },
  };
}

/**
 * The state of `next` (the cut mechanism) matching state q, q̇ of `prev`: positions by Gauss–Newton
 * from a guess (same coordinates where they exist, a piece's angle that of its bar), then rates by
 * least squares. The cut joints sit and move where their bar's points were.
 */
export function transferState(prev: NumericSystem, q: ArrayLike<number>, qd: ArrayLike<number>, next: NumericSystem, cuts: Cut[], pieces: Piece[]): { q: Float64Array; qd: Float64Array } {
  // the old features and their rates (along q̇, by central differences)
  const fo = features(prev);
  const no = fo.keys.length;
  const [v0, vp, vm] = [new Float64Array(no), new Float64Array(no), new Float64Array(no)];
  fo.eval(q, v0);
  const speed = Math.sqrt(Array.from(qd).reduce((s, v) => s + v * v, 0));
  const h = 1e-6 / Math.max(1, speed);
  fo.eval(Array.from(q, (v, i) => v + h * qd[i]), vp);
  fo.eval(Array.from(q, (v, i) => v - h * qd[i]), vm);
  const target = new Map<string, [number, number]>(fo.keys.map((k, i) => [k, [v0[i], (vp[i] - vm[i]) / (2 * h)]]));
  for (const c of cuts) {
    // a point of the bar, a fraction u of the way: position and velocity in between its ends'
    for (const ax of ['x', 'y']) {
      const [a, b] = [target.get(`${ax}${c.a}`), target.get(`${ax}${c.b}`)];
      if (a && b) target.set(`${ax}${c.point}`, [a[0] + c.u * (b[0] - a[0]), a[1] + c.u * (b[1] - a[1])]);
    }
  }
  // the new features, those with a target
  const fn = features(next);
  const rows = fn.keys.map((k, i) => ({ i, t: target.get(k) })).filter((r): r is { i: number; t: [number, number] } => !!r.t);
  const n = next.n;
  const m = rows.length;
  const buf = new Float64Array(fn.keys.length);
  const F = (x: ArrayLike<number>, out: Float64Array) => {
    fn.eval(x, buf);
    rows.forEach((r, k) => (out[k] = buf[r.i]));
  };
  // the guess
  const prevIdx = new Map(prev.derivation.coords.map((c, i) => [c.sym, i]));
  const pieceOf = new Map(pieces.map((p) => [p.id, p.from]));
  const x = new Float64Array(n);
  next.derivation.coords.forEach((c, i) => {
    const same = prevIdx.get(c.sym);
    if (same !== undefined) x[i] = q[same];
    else if (/^th\d+$/.test(c.sym) && prevIdx.has(`th${pieceOf.get(Number(c.sym.slice(2)))}`)) x[i] = q[prevIdx.get(`th${pieceOf.get(Number(c.sym.slice(2)))}`)!];
    else if (/^[xy]\d+$/.test(c.sym) && target.has(c.sym)) x[i] = target.get(c.sym)![0];
  });
  const r = new Float64Array(m);
  const fp = new Float64Array(m);
  const J = new Float64Array(m * n);
  const jacobian = () => {
    F(x, r);
    for (let j = 0; j < n; j++) {
      const e = 1e-7 * Math.max(1, Math.abs(x[j]));
      const xj = x[j];
      x[j] = xj + e;
      F(x, fp);
      x[j] = xj;
      for (let k = 0; k < m; k++) J[k * n + j] = (fp[k] - r[k]) / e;
    }
  };
  /** Least squares J z = rhs (regularised a little, for coordinates nothing here depends on). */
  const solve = (rhs: Float64Array) => {
    const A = new Float64Array(n * n);
    const z = new Float64Array(n);
    let tr = 0;
    for (let a = 0; a < n; a++) {
      let s = 0;
      for (let k = 0; k < m; k++) s += J[k * n + a] * rhs[k];
      z[a] = s;
      for (let b = a; b < n; b++) {
        let t = 0;
        for (let k = 0; k < m; k++) t += J[k * n + a] * J[k * n + b];
        A[a * n + b] = A[b * n + a] = t;
      }
      tr += A[a * n + a];
    }
    const eps = 1e-12 * (tr / Math.max(n, 1)) + 1e-15;
    for (let a = 0; a < n; a++) A[a * n + a] += eps;
    return solveInPlace(A, z, n) ? z : new Float64Array(n);
  };
  for (let it = 0; it < 30; it++) {
    jacobian();
    const res = new Float64Array(m);
    for (let k = 0; k < m; k++) res[k] = rows[k].t[0] - r[k];
    const dx = solve(res);
    let step = 0;
    for (let j = 0; j < n; j++) {
      x[j] += dx[j];
      step = Math.max(step, Math.abs(dx[j]));
    }
    if (step < 1e-13) break;
  }
  jacobian();
  const rates = Float64Array.from(rows, (row) => row.t[1]);
  return { q: x, qd: solve(rates) };
}
