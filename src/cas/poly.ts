/**
 * A small computer-algebra kernel specialised for Lagrangian mechanics.
 *
 * Every expression that arises when deriving equations of motion for planar
 * linkages is a polynomial in
 *   - plain symbols (parameters such as m₁, L₁, g; coordinates; velocities), and
 *   - sin(u), cos(u) where u is a linear combination of coordinate symbols,
 * with real (in practice rational) coefficients. Representing expressions in
 * this canonical "trig-polynomial" form makes like-term collection automatic,
 * differentiation closed-form, and code generation trivial.
 */

export type SymKind = 'param' | 'coord' | 'vel';

export interface SymInfo {
  name: string;
  latex: string;
  kind: SymKind;
}

const symbols = new Map<string, SymInfo>();

/** Register (or re-register) a symbol with its LaTeX rendering. */
export function defineSymbol(name: string, latex: string, kind: SymKind): string {
  symbols.set(name, { name, latex, kind });
  return name;
}

export function symbolInfo(name: string): SymInfo {
  return symbols.get(name) ?? { name, latex: name, kind: 'param' };
}

/** Linear combination Σ cᵢ·symᵢ, sorted by symbol name, no zero coefficients. */
export type LinComb = ReadonlyArray<readonly [string, number]>;

export interface SymAtom {
  kind: 'sym';
  name: string;
}
export interface TrigAtom {
  kind: 'sin' | 'cos';
  arg: LinComb;
}
export type Atom = SymAtom | TrigAtom;

const atoms = new Map<string, Atom>();

function linKey(u: LinComb): string {
  return u.map(([n, c]) => `${n}*${c}`).join('+');
}

function internAtom(a: Atom): string {
  const key = a.kind === 'sym' ? `s:${a.name}` : `${a.kind}:${linKey(a.arg)}`;
  if (!atoms.has(key)) atoms.set(key, a);
  return key;
}

export function atomOf(key: string): Atom {
  const a = atoms.get(key);
  if (!a) throw new Error(`unknown atom ${key}`);
  return a;
}

/** Monomial: sorted list of [atomKey, power ≥ 1]. */
export type Mono = ReadonlyArray<readonly [string, number]>;

export interface Term {
  mono: Mono;
  coef: number;
}

const EPS = 1e-12;

function monoKey(m: Mono): string {
  return m.map(([k, p]) => (p === 1 ? k : `${k}^${p}`)).join('·');
}

function mulMono(a: Mono, b: Mono): Mono {
  const out: [string, number][] = [];
  let i = 0;
  let j = 0;
  while (i < a.length || j < b.length) {
    if (j >= b.length || (i < a.length && a[i][0] < b[j][0])) out.push([a[i][0], a[i++][1]]);
    else if (i >= a.length || b[j][0] < a[i][0]) out.push([b[j][0], b[j++][1]]);
    else {
      out.push([a[i][0], a[i][1] + b[j][1]]);
      i++;
      j++;
    }
  }
  return out;
}

/** Multiply a monomial by atom^k (k may be negative); null if a power would go negative. */
function shiftMono(m: Mono, atomKey: string, k: number): Mono | null {
  const out: [string, number][] = [];
  let placed = false;
  for (const [key, p] of m) {
    if (!placed && key === atomKey) {
      placed = true;
      const np = p + k;
      if (np < 0) return null;
      if (np > 0) out.push([key, np]);
    } else {
      if (!placed && key > atomKey) {
        if (k < 0) return null;
        out.push([atomKey, k]);
        placed = true;
      }
      out.push([key, p]);
    }
  }
  if (!placed) {
    if (k < 0) return null;
    if (k > 0) out.push([atomKey, k]);
  }
  return out;
}

export class Poly {
  readonly terms: ReadonlyMap<string, Term>;

  private constructor(terms: Map<string, Term>) {
    this.terms = terms;
  }

  static readonly zero = new Poly(new Map());

  static fromTerms(list: Iterable<Term>): Poly {
    const map = new Map<string, Term>();
    for (const t of list) addTerm(map, t.mono, t.coef);
    return new Poly(map);
  }

  static const(c: number): Poly {
    return Poly.fromTerms([{ mono: [], coef: c }]);
  }

  static sym(name: string): Poly {
    return Poly.fromTerms([{ mono: [[internAtom({ kind: 'sym', name }), 1]], coef: 1 }]);
  }

  static sin(u: LinComb): Poly {
    return trig('sin', u);
  }

  static cos(u: LinComb): Poly {
    return trig('cos', u);
  }

  isZero(): boolean {
    return this.terms.size === 0;
  }

  /** Numeric value if the polynomial is a constant, else null. */
  constantValue(): number | null {
    if (this.terms.size === 0) return 0;
    if (this.terms.size === 1) {
      const t = this.terms.values().next().value as Term;
      if (t.mono.length === 0) return t.coef;
    }
    return null;
  }

  add(o: Poly): Poly {
    const map = new Map(this.terms);
    for (const t of o.terms.values()) addTerm(map, t.mono, t.coef);
    return new Poly(map);
  }

  sub(o: Poly): Poly {
    return this.add(o.scale(-1));
  }

  scale(c: number): Poly {
    if (c === 0) return Poly.zero;
    const map = new Map<string, Term>();
    for (const [k, t] of this.terms) map.set(k, { mono: t.mono, coef: t.coef * c });
    return new Poly(map);
  }

  mul(o: Poly): Poly {
    const map = new Map<string, Term>();
    for (const a of this.terms.values())
      for (const b of o.terms.values()) addTerm(map, mulMono(a.mono, b.mono), a.coef * b.coef);
    return new Poly(map);
  }

  /** ∂/∂name, treating every other symbol as independent. */
  diff(name: string): Poly {
    const map = new Map<string, Term>();
    for (const t of this.terms.values()) {
      for (const [key, p] of t.mono) {
        const atom = atomOf(key);
        const rest = shiftMono(t.mono, key, -1)!;
        if (atom.kind === 'sym') {
          if (atom.name === name) addTerm(map, rest, t.coef * p);
          continue;
        }
        const c = atom.arg.find(([n]) => n === name)?.[1] ?? 0;
        if (c === 0) continue;
        // d sin u = c cos u, d cos u = −c sin u
        const other = trig(atom.kind === 'sin' ? 'cos' : 'sin', atom.arg);
        const sign = atom.kind === 'sin' ? 1 : -1;
        for (const ot of other.terms.values())
          addTerm(map, mulMono(rest, ot.mono), t.coef * p * c * sign * ot.coef);
      }
    }
    return new Poly(map);
  }

  /** Does the expression depend on the symbol (directly or through a trig argument)? */
  dependsOn(name: string): boolean {
    for (const t of this.terms.values())
      for (const [key] of t.mono) {
        const a = atomOf(key);
        if (a.kind === 'sym' ? a.name === name : a.arg.some(([n]) => n === name)) return true;
      }
    return false;
  }

  evaluate(env: Record<string, number>): number {
    let sum = 0;
    for (const t of this.terms.values()) {
      let v = t.coef;
      for (const [key, p] of t.mono) v *= atomValue(atomOf(key), env) ** p;
      sum += v;
    }
    return sum;
  }
}

function atomValue(a: Atom, env: Record<string, number>): number {
  if (a.kind === 'sym') {
    const v = env[a.name];
    if (v === undefined) throw new Error(`no value for ${a.name}`);
    return v;
  }
  let u = 0;
  for (const [n, c] of a.arg) {
    const v = env[n];
    if (v === undefined) throw new Error(`no value for ${n}`);
    u += c * v;
  }
  return a.kind === 'sin' ? Math.sin(u) : Math.cos(u);
}

function addTerm(map: Map<string, Term>, mono: Mono, coef: number): void {
  if (coef === 0) return;
  const key = monoKey(mono);
  const prev = map.get(key);
  const c = (prev?.coef ?? 0) + coef;
  if (Math.abs(c) < EPS) map.delete(key);
  else map.set(key, { mono, coef: c });
}

function normaliseLin(u: Iterable<readonly [string, number]>): [string, number][] {
  const acc = new Map<string, number>();
  for (const [n, c] of u) acc.set(n, (acc.get(n) ?? 0) + c);
  return [...acc].filter(([, c]) => Math.abs(c) > EPS).sort((a, b) => (a[0] < b[0] ? -1 : 1));
}

/** sin/cos with canonical argument: sorted, leading coefficient positive. */
function trig(kind: 'sin' | 'cos', u: LinComb): Poly {
  let arg = normaliseLin(u);
  if (arg.length === 0) return kind === 'sin' ? Poly.zero : Poly.const(1);
  let sign = 1;
  if (arg[0][1] < 0) {
    arg = arg.map(([n, c]) => [n, -c]);
    if (kind === 'sin') sign = -1;
  }
  const key = internAtom({ kind, arg });
  return Poly.fromTerms([{ mono: [[key, 1]], coef: sign }]);
}

export function linAdd(a: LinComb, b: LinComb, sb = 1): LinComb {
  return normaliseLin([...a, ...b.map(([n, c]) => [n, c * sb] as const)]);
}

/** Total time derivative of an expression in the coordinates: Σ ∂P/∂qⱼ · q̇ⱼ. */
export function timeDerivative(p: Poly, coords: readonly string[], vels: readonly string[]): Poly {
  let out = Poly.zero;
  coords.forEach((q, j) => {
    if (p.dependsOn(q)) out = out.add(p.diff(q).mul(Poly.sym(vels[j])));
  });
  return out;
}

// ---------------------------------------------------------------------------
// Trigonometric simplification
// ---------------------------------------------------------------------------

function close(a: number, b: number): boolean {
  return Math.abs(a - b) <= 1e-9 * Math.max(Math.abs(a), Math.abs(b));
}

/**
 * Apply sin²u + cos²u = 1 and the angle-addition identities
 *   cos u cos v ± sin u sin v = cos(u ∓ v),  sin u cos v ∓ cos u sin v = sin(u ∓ v)
 * wherever two terms match exactly, until nothing changes.
 */
export function trigSimplify(p: Poly): Poly {
  let cur = p;
  for (let iter = 0; iter < 50; iter++) {
    const next = simplifyPass(cur);
    if (!next) return cur;
    cur = next;
  }
  return cur;
}

function simplifyPass(p: Poly): Poly | null {
  const map = new Map(p.terms);
  let changed = false;
  const keys = [...map.keys()].sort();

  const take = (mono: Mono | null): Term | undefined => (mono ? map.get(monoKey(mono)) : undefined);
  const remove = (mono: Mono) => map.delete(monoKey(mono));

  for (const key of keys) {
    const t = map.get(key);
    if (!t) continue;
    const trigs = t.mono.filter(([k]) => atomOf(k).kind !== 'sym');
    let done = false;

    // Pythagorean identity, driven from the sin² side.
    for (const [k, pw] of trigs) {
      const a = atomOf(k) as TrigAtom;
      if (a.kind !== 'sin' || pw < 2) continue;
      const rest = shiftMono(t.mono, k, -2)!;
      const cosKey = internAtom({ kind: 'cos', arg: a.arg });
      const partnerMono = shiftMono(rest, cosKey, 2)!;
      const partner = take(partnerMono);
      if (partner && close(partner.coef, t.coef)) {
        remove(t.mono);
        remove(partnerMono);
        addTerm(map, rest, t.coef);
        changed = done = true;
        break;
      }
    }
    if (done) continue;

    // Angle-addition identities over pairs of distinct trig atoms.
    outer: for (let i = 0; i < trigs.length; i++) {
      for (let j = 0; j < trigs.length; j++) {
        if (i === j) continue;
        const [k1] = trigs[i];
        const [k2] = trigs[j];
        const a1 = atomOf(k1) as TrigAtom;
        const a2 = atomOf(k2) as TrigAtom;
        const rest = shiftMono(shiftMono(t.mono, k1, -1)!, k2, -1)!;
        const u = a1.arg;
        const v = a2.arg;
        if (linKey(u) === linKey(v)) continue; // sin u · cos u is not an addition pattern
        const sinU = internAtom({ kind: 'sin', arg: u });
        const sinV = internAtom({ kind: 'sin', arg: v });
        const cosU = internAtom({ kind: 'cos', arg: u });
        if (a1.kind === 'cos' && a2.kind === 'cos' && i < j) {
          const partnerMono = shiftMono(shiftMono(rest, sinU, 1)!, sinV, 1)!;
          const partner = take(partnerMono);
          if (!partner) continue;
          const minus = close(partner.coef, t.coef);
          const plus = close(partner.coef, -t.coef);
          if (!minus && !plus) continue;
          remove(t.mono);
          remove(partnerMono);
          for (const nt of trig('cos', linAdd(u, v, minus ? -1 : 1)).mul(Poly.fromTerms([{ mono: rest, coef: t.coef }])).terms.values())
            addTerm(map, nt.mono, nt.coef);
          changed = done = true;
          break outer;
        }
        if (a1.kind === 'sin' && a2.kind === 'cos') {
          const partnerMono = shiftMono(shiftMono(rest, cosU, 1)!, sinV, 1)!;
          const partner = take(partnerMono);
          if (!partner) continue;
          const minus = close(partner.coef, -t.coef);
          const plus = close(partner.coef, t.coef);
          if (!minus && !plus) continue;
          remove(t.mono);
          remove(partnerMono);
          for (const nt of trig('sin', linAdd(u, v, minus ? -1 : 1)).mul(Poly.fromTerms([{ mono: rest, coef: t.coef }])).terms.values())
            addTerm(map, nt.mono, nt.coef);
          changed = done = true;
          break outer;
        }
      }
    }
  }
  return changed ? Poly.fromTerms(map.values()) : null;
}
