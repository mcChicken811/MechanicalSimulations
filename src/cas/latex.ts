import { atomOf, Poly, symbolInfo, type LinComb, type Mono, type Term } from './poly';

/** Best rational approximation p/q (q ≤ 10⁴) of x, or null if none is close. */
export function toRational(x: number): [number, number] | null {
  const sign = x < 0 ? -1 : 1;
  let v = Math.abs(x);
  let [h0, h1, k0, k1] = [0, 1, 1, 0];
  for (let i = 0; i < 20; i++) {
    const a = Math.floor(v);
    [h0, h1] = [h1, a * h1 + h0];
    [k0, k1] = [k1, a * k1 + k0];
    if (k1 > 10000) return null;
    if (Math.abs(h1 / k1 - Math.abs(x)) <= 1e-9 * Math.max(1, Math.abs(x))) return [sign * h1, k1];
    const frac = v - a;
    if (frac < 1e-12) break;
    v = 1 / frac;
  }
  return Math.abs(h1 / k1 - Math.abs(x)) <= 1e-9 * Math.max(1, Math.abs(x)) ? [sign * h1, k1] : null;
}

/** Magnitude of a coefficient as LaTeX; '' when it is 1 and a monomial follows. */
function magnitudeLatex(c: number, hasMono: boolean): string {
  const r = toRational(Math.abs(c));
  if (r) {
    const [p, q] = r;
    if (q === 1) return p === 1 && hasMono ? '' : `${p}`;
    return `\\frac{${p}}{${q}}`;
  }
  return Math.abs(c).toPrecision(4);
}

function linLatex(u: LinComb): string {
  return u
    .map(([n, c], i) => {
      const s = symbolInfo(n).latex;
      const mag = Math.abs(c) === 1 ? '' : magnitudeLatex(c, true);
      const sign = c < 0 ? ' - ' : i === 0 ? '' : ' + ';
      return `${sign}${mag}${s}`;
    })
    .join('');
}

function rank(key: string): number {
  const a = atomOf(key);
  if (a.kind !== 'sym') return 5;
  const info = symbolInfo(a.name);
  if (info.kind === 'vel') return 7;
  if (info.kind === 'coord') return 6;
  const l = info.latex;
  if (l.startsWith('m')) return 0;
  if (l === 'g') return 1;
  if (l.startsWith('L')) return 2;
  return 3;
}

function factorLatex(key: string, power: number): string {
  const a = atomOf(key);
  const pw = power === 1 ? '' : `^{${power}}`;
  if (a.kind === 'sym') {
    const l = symbolInfo(a.name).latex;
    return power === 1 ? l : `{${l}}${pw}`;
  }
  const fn = a.kind === 'sin' ? '\\sin' : '\\cos';
  const simple = a.arg.length === 1 && a.arg[0][1] === 1;
  return simple ? `${fn}${pw} ${linLatex(a.arg)}` : `${fn}${pw}\\left(${linLatex(a.arg)}\\right)`;
}

export function monoLatex(m: Mono): string {
  return [...m]
    .sort((x, y) => rank(x[0]) - rank(y[0]) || (x[0] < y[0] ? -1 : 1))
    .map(([k, p]) => factorLatex(k, p))
    .join(' ');
}

function velocityDegree(t: Term): number {
  let d = 0;
  for (const [k, p] of t.mono) {
    const a = atomOf(k);
    if (a.kind === 'sym' && symbolInfo(a.name).kind === 'vel') d += p;
  }
  return d;
}

function sortedTerms(p: Poly): Term[] {
  return [...p.terms.entries()]
    .sort(([ka, a], [kb, b]) => velocityDegree(b) - velocityDegree(a) || (ka < kb ? -1 : 1))
    .map(([, t]) => t);
}

/** Each term as LaTeX with its sign attached: ['\\frac{1}{2} m_1', '- g m_1 …']. */
export function termLatexList(p: Poly): string[] {
  return sortedTerms(p).map((t, i) => {
    const mono = monoLatex(t.mono);
    const mag = magnitudeLatex(t.coef, t.mono.length > 0);
    const body = [mag, mono].filter(Boolean).join(' ');
    if (t.coef < 0) return `- ${body}`;
    return i === 0 ? body : `+ ${body}`;
  });
}

/** Render a polynomial; with factor=true pull out the common monomial and sign. */
export function polyLatex(p: Poly, opts: { factor?: boolean } = {}): string {
  if (p.isZero()) return '0';
  const terms = [...p.terms.values()];
  if (opts.factor && terms.length > 1) {
    let common: Map<string, number> | null = null;
    for (const t of terms) {
      const here = new Map(t.mono);
      if (!common) common = here;
      else for (const [k, pw] of common) common.set(k, Math.min(pw, here.get(k) ?? 0));
    }
    const commonMono: [string, number][] = [...(common ?? [])].filter(([, pw]) => pw > 0).sort((a, b) => (a[0] < b[0] ? -1 : 1));
    const allNeg = terms.every((t) => t.coef < 0);
    if (commonMono.length > 0 || allNeg) {
      const sign = allNeg ? -1 : 1;
      const inner = Poly.fromTerms(
        terms.map((t) => ({
          mono: t.mono
            .map(([k, pw]) => [k, pw - (commonMono.find(([ck]) => ck === k)?.[1] ?? 0)] as [string, number])
            .filter(([, pw]) => pw > 0),
          coef: t.coef * sign,
        })),
      );
      const outer = monoLatex(commonMono);
      return `${allNeg ? '-' : ''}${outer}${outer ? ' ' : ''}\\left(${termLatexList(inner).join(' ')}\\right)`;
    }
  }
  return termLatexList(p).join(' ');
}

/** Number of terms, used to decide whether a coefficient needs parentheses. */
export function termCount(p: Poly): number {
  return p.terms.size;
}
