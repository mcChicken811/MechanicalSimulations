/** Turn a Derivation into LaTeX strings for display (no DOM here, so it is unit-testable). */
import { polyLatex, termLatexList } from '../cas/latex';
import type { Poly } from '../cas/poly';
import type { Derivation } from '../mechanics/derive';

/** A polynomial written so it can be multiplied by something on its right. */
export function productLatex(p: Poly): string {
  if (p.terms.size <= 1) return polyLatex(p);
  const plain = polyLatex(p);
  const factored = polyLatex(p, { factor: true });
  return factored === plain ? `\\left(${plain}\\right)` : factored;
}

/** Join pre-signed parts ('x', '- y', '+ z') making sure every part after the first has a sign. */
function signed(part: string, first: boolean): string {
  const t = part.trim();
  if (first) return t.startsWith('+ ') ? t.slice(2) : t;
  return t.startsWith('-') || t.startsWith('+') ? t : `+ ${t}`;
}

function lines(parts: string[], perLine: number): string[] {
  const out: string[] = [];
  for (let i = 0; i < parts.length; i += perLine)
    out.push(parts.slice(i, i + perLine).map((p, k) => signed(p, i + k === 0)).join(' '));
  return out;
}

function timesSymbol(coef: Poly, symbol: string): string {
  const c = coef.constantValue();
  if (c === 1) return symbol;
  if (c === -1) return `- ${symbol}`;
  return `${productLatex(coef)}\\, ${symbol}`;
}

export function equationLatex(d: Derivation, i: number): string {
  const lhs: string[] = [];
  d.M[i].forEach((m, j) => {
    if (m.isZero()) return;
    const c = d.coords[j];
    lhs.push(timesSymbol(m, `\\ddot{${c.base}}_{${c.sub}}`));
  });
  const rhs = termLatexList(d.f[i]);
  for (const s of d.springs) rhs.push(...springForceLatex(d, s, i));
  rhs.push(`+ Q_{${d.coords[i].latex}}`);
  d.phiQ.forEach((row, k) => {
    if (row[i].isZero()) return;
    // Long constraint gradients are written symbolically to keep the equation readable.
    if (row[i].terms.size > 2) rhs.push(`\\frac{\\partial \\Phi_{${k + 1}}}{\\partial ${d.coords[i].latex}}\\,\\lambda_{${k + 1}}`);
    else rhs.push(timesSymbol(row[i], `\\lambda_{${k + 1}}`));
  });
  const L = lines(lhs.length ? lhs : ['0'], 2);
  const R = lines(rhs, 3);
  const body = [`&${L[0]}`, ...L.slice(1).map((l) => `&\\quad ${l}`), `&= ${R[0]}`, ...R.slice(1).map((l) => `&\\qquad ${l}`)];
  return `\\begin{aligned}${body.join(' \\\\ ')}\\end{aligned}`;
}

/** −∂V_s/∂q_i for one spring, or nothing if the spring does not act on q_i. */
function springForceLatex(d: Derivation, s: Derivation['springs'][number], i: number): string[] {
  const g = s.grad[i];
  if (g.isZero()) return [];
  // Short gradients are written out: −k (1 − ℓ₀/ℓ) · ½∂(ℓ²)/∂q. Long ones as −k (ℓ − ℓ₀) ∂ℓ/∂q.
  if (g.terms.size <= 3) {
    const neg = g.terms.size === 1 && [...g.terms.values()][0].coef < 0;
    const gg = neg ? g.scale(-1) : g;
    const inner = `1 - \\frac{${s.restLatex}}{${s.lLatex}}`;
    const factor = `${neg ? '+' : '-'} ${s.kLatex}${s.tensionOnly ? `\\max\\left(0,\\ ${inner}\\right)` : `\\left(${inner}\\right)`}`;
    return [gg.constantValue() === 1 ? factor : `${factor}${productLatex(gg)}`];
  }
  const stretch = `${s.lLatex} - ${s.restLatex}`;
  return [`- ${s.kLatex}${s.tensionOnly ? `\\max\\left(0,\\ ${stretch}\\right)` : `\\left(${stretch}\\right)`}\\frac{\\partial ${s.lLatex}}{\\partial ${d.coords[i].latex}}`];
}

function springEnergyParts(d: Derivation): string[] {
  return d.springs.map((s) =>
    s.tensionOnly
      ? `+ \\tfrac{1}{2} ${s.kLatex}\\max\\left(0,\\ ${s.lLatex} - ${s.restLatex}\\right)^{2}`
      : `+ \\tfrac{1}{2} ${s.kLatex}\\left(${s.lLatex} - ${s.restLatex}\\right)^{2}`,
  );
}

/** Potential energy: gravity terms plus ½ k (ℓ − ℓ₀)² for every spring. */
export function potentialLatex(d: Derivation): string {
  return energyLatex('V', d.V, springEnergyParts(d));
}

/** ℓ = √(Δx² + Δy²) for one spring. */
export function springLatex(d: Derivation, j: number): string {
  const s = d.springs[j];
  const sq = (e: Poly) => `\\left(${polyLatex(e)}\\right)^{2}`;
  const parts = [s.dx, s.dy].filter((e) => !e.isZero()).map(sq);
  return parts.length === 2 && s.dx.terms.size + s.dy.terms.size > 4
    ? `\\begin{aligned}${s.lLatex}^{2} &= ${parts[0]} \\\\ &\\quad + ${parts[1]}\\end{aligned}`
    : `${s.lLatex} = \\sqrt{${parts.join(' + ')}}`;
}

export function energyLatex(name: string, p: Poly, extra: string[] = []): string {
  const parts = [...termLatexList(p), ...extra];
  if (parts.length === 0) return `${name} = 0`;
  const L = lines(parts, 3);
  return `\\begin{aligned}${name} &= ${L[0]}${L.slice(1).map((l) => ` \\\\ &\\quad ${l}`).join('')}\\end{aligned}`;
}

export function constraintLatex(d: Derivation, k: number): string {
  const c = d.constraints[k];
  if (c.loop) {
     const sq = (e: Poly) => `\\left(${polyLatex(e)}\\right)^{2}`;
    return `\\begin{aligned}\\Phi_{${k + 1}} &= ${sq(c.loop.dx)} \\\\ &\\quad + ${sq(c.loop.dy)} - ${productLatex(c.loop.L.mul(c.loop.L))} = 0\\end{aligned}`;
  }
  if (c.gear) {
    const { Na, Nb, ta, tb, tc, C } = c.gear;
    const side = (N: Poly, t: Poly) => {
      const rel = t.sub(tc);
      if (rel.isZero()) return null;
      const single = rel.terms.size === 1 && [...rel.terms.values()][0].coef === 1;
      return `${polyLatex(N)}${single ? `\\,${polyLatex(rel)}` : `\\left(${polyLatex(rel)}\\right)`}`;
    };
    const lhs = [side(Na, ta), side(Nb, tb)].filter((x): x is string => x !== null).join(' + ') || '0';
    return `\\Phi_{${k + 1}} = ${lhs} - ${polyLatex(C)} = 0`;
  }
  if (c.rack) {
    const { xi, sR, rel, C } = c.rack;
    const parts = termLatexList(xi);
    if (!rel.isZero()) {
      const neg = sR.terms.size === 1 && [...sR.terms.values()][0].coef < 0;
      const single = rel.terms.size === 1 && [...rel.terms.values()][0].coef === 1;
      const r = polyLatex(neg ? sR.scale(-1) : sR);
      parts.push(`${neg ? '-' : '+'} ${r}${single ? `\\,${polyLatex(rel)}` : `\\left(${polyLatex(rel)}\\right)`}`);
    }
    parts.push(`- ${polyLatex(C)}`);
    const L = lines(parts, 3);
    return `\\begin{aligned}\\Phi_{${k + 1}} &= ${L[0]}${L.slice(1).map((l) => ` \\\\ &\\quad ${l}`).join('')} = 0\\end{aligned}`;
  }
  const parts = termLatexList(c.phi);
  const L = lines(parts, 4);
  return `\\begin{aligned}\\Phi_{${k + 1}} &= ${L[0]}${L.slice(1).map((l) => ` \\\\ &\\quad ${l}`).join('')} = 0\\end{aligned}`;
}

export function coordinatesLatex(d: Derivation): string {
  return `q = \\left(${d.coords.map((c) => c.latex).join(',\\ ')}\\right)`;
}

export function generalFormLatex(d: Derivation, friction = false): string {
  const R = friction ? ` - \\frac{\\partial R}{\\partial \\dot q}` : '';
  return d.constraints.length
    ? `M(q)\\,\\ddot q = f(q,\\dot q) + Q${R} + \\Phi_q^{\\mathsf T}\\lambda,\\quad \\Phi(q) = 0`
    : `M(q)\\,\\ddot q = f(q,\\dot q) + Q${R}`;
}
