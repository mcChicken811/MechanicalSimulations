import { atomOf, type Poly } from './poly';

export type CompiledFn = (q: ArrayLike<number>, qd: ArrayLike<number>, p: ArrayLike<number>, out: Float64Array) => void;

export interface CodegenInputs {
  coords: readonly string[];
  vels: readonly string[];
  params: readonly string[];
}

/**
 * Statements per generated function. V8 does not optimize very large functions, and a big
 * mechanism (a 30-link chain has thousands of terms in its mass matrix) would otherwise run
 * in the interpreter, about ten times slower. So the code is split into chunks this size.
 */
const CHUNK_TERMS = 300;

type Chunk = (q: ArrayLike<number>, qd: ArrayLike<number>, p: ArrayLike<number>, out: Float64Array, t: Float64Array) => void;

/**
 * Compile a list of polynomials into one JavaScript function
 *   (q, qd, p, out) => { out[i] = outputs[i](q, qd, p) }.
 *
 * Common subexpressions are eliminated at the atom level: every sin/cos and every power that
 * appears is computed once into a scratch array t, which the output chunks then read.
 */
export function compile(outputs: readonly Poly[], inputs: CodegenInputs): CompiledFn {
  const symVar = new Map<string, string>();
  inputs.coords.forEach((n, i) => symVar.set(n, `q[${i}]`));
  inputs.vels.forEach((n, i) => symVar.set(n, `qd[${i}]`));
  inputs.params.forEach((n, i) => symVar.set(n, `p[${i}]`));

  const atomLines: string[] = [];
  const slot = new Map<string, string>();
  let nSlots = 0;

  const argExpr = (arg: ReadonlyArray<readonly [string, number]>) =>
    arg
      .map(([n, c]) => {
        const v = symVar.get(n);
        if (!v) throw new Error(`codegen: unbound symbol ${n}`);
        return c === 1 ? v : `${c}*${v}`;
      })
      .join(' + ');

  const atomRef = (key: string): string => {
    const known = slot.get(key);
    if (known) return known;
    const a = atomOf(key);
    if (a.kind === 'sym') {
      // plain symbols are read straight from the inputs
      const src = symVar.get(a.name);
      if (!src) throw new Error(`codegen: unbound symbol ${a.name}`);
      slot.set(key, src);
      return src;
    }
    const v = `t[${nSlots++}]`;
    atomLines.push(`${v} = Math.${a.kind}(${argExpr(a.arg)});`);
    slot.set(key, v);
    return v;
  };

  const powRef = (key: string, pw: number): string => {
    const base = atomRef(key);
    if (pw === 1) return base;
    const id = `${key}^${pw}`;
    let v = slot.get(id);
    if (!v) {
      v = `t[${nSlots++}]`;
      atomLines.push(`${v} = ${pw === 2 ? `${base}*${base}` : `${base}**${pw}`};`);
      slot.set(id, v);
    }
    return v;
  };

  // Output statements, long polynomials split into partial sums.
  const bodyLines: string[] = [];
  outputs.forEach((poly, i) => {
    const terms = [...poly.terms.values()].map((t) => [String(t.coef), ...t.mono.map(([k, pw]) => powRef(k, pw))].join('*'));
    if (!terms.length) bodyLines.push(`out[${i}] = 0;`);
    for (let k = 0; k < terms.length; k += CHUNK_TERMS / 3) {
      bodyLines.push(`out[${i}] ${k ? '+=' : '='} ${terms.slice(k, k + CHUNK_TERMS / 3).join(' + ')};`);
    }
  });

  // Group statements into functions of about CHUNK_TERMS terms each.
  const chunks: Chunk[] = [];
  const group = (lines: string[]) => {
    let cur: string[] = [];
    let cost = 0;
    const flush = () => {
      if (!cur.length) return;
      chunks.push(new Function('q', 'qd', 'p', 'out', 't', `"use strict";\n${cur.join('\n')}`) as Chunk);
      cur = [];
      cost = 0;
    };
    for (const line of lines) {
      const c = line.split('*').length; // ~ number of factors
      if (cost + c > CHUNK_TERMS * 3) flush();
      cur.push(line);
      cost += c;
    }
    flush();
  };
  group(atomLines);
  group(bodyLines);

  const t = new Float64Array(Math.max(nSlots, 1));
  if (chunks.length === 1) {
    const only = chunks[0];
    return (q, qd, p, out) => only(q, qd, p, out, t);
  }
  return (q, qd, p, out) => {
    for (let k = 0; k < chunks.length; k++) chunks[k](q, qd, p, out, t);
  };
}
