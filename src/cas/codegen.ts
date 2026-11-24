import { atomOf, type Poly } from './poly';

export type CompiledFn = (q: ArrayLike<number>, qd: ArrayLike<number>, p: ArrayLike<number>, out: Float64Array) => void;

export interface CodegenInputs {
  coords: readonly string[];
  vels: readonly string[];
  params: readonly string[];
}

/**
 * Compile a list of polynomials into one JavaScript function
 *   (q, qd, p, out) => { out[i] = outputs[i](q, qd, p) }.
 *
 * Common subexpressions are eliminated at the atom level: every symbol,
 * every sin/cos and every power that appears is computed once into a local.
 */
export function compile(outputs: readonly Poly[], inputs: CodegenInputs): CompiledFn {
  const symVar = new Map<string, string>();
  inputs.coords.forEach((n, i) => symVar.set(n, `q[${i}]`));
  inputs.vels.forEach((n, i) => symVar.set(n, `qd[${i}]`));
  inputs.params.forEach((n, i) => symVar.set(n, `p[${i}]`));

  const lines: string[] = [];
  const atomVar = new Map<string, string>();
  const powVar = new Map<string, string>();

  const argExpr = (arg: ReadonlyArray<readonly [string, number]>) =>
    arg
      .map(([n, c]) => {
        const v = symVar.get(n);
        if (!v) throw new Error(`codegen: unbound symbol ${n}`);
        return c === 1 ? v : `${c}*${v}`;
      })
      .join(' + ');

  const atomRef = (key: string): string => {
    let v = atomVar.get(key);
    if (v) return v;
    const a = atomOf(key);
    v = `a${atomVar.size}`;
    if (a.kind === 'sym') {
      const src = symVar.get(a.name);
      if (!src) throw new Error(`codegen: unbound symbol ${a.name}`);
      lines.push(`const ${v} = ${src};`);
    } else {
      lines.push(`const ${v} = Math.${a.kind}(${argExpr(a.arg)});`);
    }
    atomVar.set(key, v);
    return v;
  };

  const powRef = (key: string, pw: number): string => {
    const base = atomRef(key);
    if (pw === 1) return base;
    const id = `${key}^${pw}`;
    let v = powVar.get(id);
    if (!v) {
      v = `${base}_${pw}`;
      lines.push(`const ${v} = ${pw === 2 ? `${base}*${base}` : `${base}**${pw}`};`);
      powVar.set(id, v);
    }
    return v;
  };

  const body: string[] = [];
  outputs.forEach((poly, i) => {
    const terms = [...poly.terms.values()].map((t) => {
      const factors = t.mono.map(([k, pw]) => powRef(k, pw));
      return [String(t.coef), ...factors].join('*');
    });
    body.push(`out[${i}] = ${terms.length ? terms.join(' + ') : '0'};`);
  });

  const src = `"use strict";\n${lines.join('\n')}\n${body.join('\n')}`;
  return new Function('q', 'qd', 'p', 'out', src) as CompiledFn;
}
