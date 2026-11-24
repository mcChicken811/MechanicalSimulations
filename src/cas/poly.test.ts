import { describe, expect, it } from 'vitest';
import { compile } from './codegen';
import { polyLatex, toRational } from './latex';
import { defineSymbol, Poly, trigSimplify } from './poly';

defineSymbol('a', '\\alpha', 'coord');
defineSymbol('b', '\\beta', 'coord');
defineSymbol('k', 'k', 'param');
const a = Poly.sym('a');
const k = Poly.sym('k');
const sa = Poly.sin([['a', 1]]);
const ca = Poly.cos([['a', 1]]);
const sb = Poly.sin([['b', 1]]);
const cb = Poly.cos([['b', 1]]);

describe('Poly', () => {
  it('collects like terms and cancels', () => {
    expect(a.add(a).sub(a.scale(2)).isZero()).toBe(true);
    expect(a.mul(k).add(k.mul(a)).terms.size).toBe(1);
  });

  it('differentiates trig of linear combinations', () => {
    const u = Poly.sin([['a', 1], ['b', -1]]);
    const du = u.diff('b');
    const expected = Poly.cos([['a', 1], ['b', -1]]).scale(-1);
    expect(du.sub(expected).isZero()).toBe(true);
  });

  it('canonicalises argument signs: sin(b − a) = −sin(a − b)', () => {
    expect(Poly.sin([['b', 1], ['a', -1]]).add(Poly.sin([['a', 1], ['b', -1]])).isZero()).toBe(true);
    expect(Poly.cos([['b', 1], ['a', -1]]).sub(Poly.cos([['a', 1], ['b', -1]])).isZero()).toBe(true);
  });

  it('simplifies sin² + cos² and angle-addition patterns', () => {
    expect(trigSimplify(k.mul(sa.mul(sa).add(ca.mul(ca)))).sub(k).isZero()).toBe(true);
    const e = trigSimplify(ca.mul(cb).add(sa.mul(sb)).scale(3));
    expect(e.sub(Poly.cos([['a', 1], ['b', -1]]).scale(3)).isZero()).toBe(true);
    const s = trigSimplify(sa.mul(cb).sub(ca.mul(sb)));
    expect(s.sub(Poly.sin([['a', 1], ['b', -1]])).isZero()).toBe(true);
    // sin a cos a must not be mistaken for an addition pattern
    expect(trigSimplify(sa.mul(ca)).terms.size).toBe(1);
  });

  it('compiles to code that agrees with direct evaluation', () => {
    const e = k.mul(sa).mul(cb).add(a.mul(a).scale(0.5)).sub(Poly.cos([['a', 2], ['b', -1]]));
    const fn = compile([e], { coords: ['a', 'b'], vels: [], params: ['k'] });
    const out = new Float64Array(1);
    fn([0.3, -1.1], [], [2.5], out);
    expect(out[0]).toBeCloseTo(e.evaluate({ a: 0.3, b: -1.1, k: 2.5 }), 12);
  });

  it('renders rational coefficients and factors common terms', () => {
    expect(toRational(1 / 12)).toEqual([1, 12]);
    expect(toRational(-2 / 3)).toEqual([-2, 3]);
    expect(polyLatex(k.mul(k).scale(1 / 3).add(k.mul(k).mul(a)), { factor: true })).toBe('{k}^{2} \\left(\\frac{1}{3} + \\alpha\\right)');
  });
});
