import { describe, expect, it } from 'vitest';
import { barDiagram, beamDiagram, peak, valueAt } from './beam';
import { deriveEquations } from './derive';
import { jointForces } from './forces';
import type { Design } from './model';
import { NumericSystem, Simulator } from './simulate';
import { presets } from '../ui/presets';

const G = 9.81;
const still = (A: [number, number], B: [number, number], m: number) => ({ id: 0, m, A, B, vA: [0, 0] as [number, number], vB: [0, 0] as [number, number], aA: [0, 0] as [number, number], aB: [0, 0] as [number, number] });

describe('internal forces along a bar', () => {
  it('a massless beam, simply supported, loaded in the middle: V = ±P/2, M = P L / 4', () => {
    const d = beamDiagram(still([0, 0], [2, 0], 0), [{ at: [0, 0], F: [0, 5] }, { at: [2, 0], F: [0, 5] }, { at: [1, 0], F: [0, -10] }], { gravity: G });
    expect(valueAt(d, d.V, 0.5)).toBeCloseTo(5, 9);
    expect(valueAt(d, d.V, 1.5)).toBeCloseTo(-5, 9);
    expect(valueAt(d, d.M, 1)).toBeCloseTo(5, 9);
    expect(valueAt(d, d.M, 0.5)).toBeCloseTo(2.5, 9);
    expect(peak(d.N).v).toBeCloseTo(0, 9);
    expect(d.closure.map(Math.abs).every((x) => x < 1e-9)).toBe(true);
    // the middle load is a step: V jumps from +5 to −5 there
    const k = d.s.findIndex((s) => Math.abs(s - 1) < 1e-12);
    expect([d.V[k], d.V[k + 1]].map((v) => +v.toFixed(9))).toEqual([5, -5]);
  });

  it('a heavy beam on two supports: M = m g L / 8 in the middle, under its own weight', () => {
    const m = 2;
    const d = beamDiagram(still([0, 0], [2, 0], m), [{ at: [0, 0], F: [0, (m * G) / 2] }, { at: [2, 0], F: [0, (m * G) / 2] }], { gravity: G });
    expect(valueAt(d, d.M, 1)).toBeCloseTo((m * G * 2) / 8, 9);
    expect(valueAt(d, d.V, 1e-9)).toBeCloseTo((m * G) / 2, 6);
    expect(valueAt(d, d.V, 1)).toBeCloseTo(0, 9);
  });

  it('a bar pulled at both ends is in tension; pushed, in compression; an offset load bends it', () => {
    const pull = beamDiagram(still([0, 0], [0, 1], 0), [{ at: [0, 0], F: [0, -3] }, { at: [0, 1], F: [0, 3] }], { gravity: 0 });
    expect(valueAt(pull, pull.N, 0.5)).toBeCloseTo(3, 9);
    const push = beamDiagram(still([0, 0], [0, 1], 0), [{ at: [0, 0], F: [0, 3] }, { at: [0, 1], F: [0, -3] }], { gravity: 0 });
    expect(valueAt(push, push.N, 0.5)).toBeCloseTo(-3, 9);
    // a cantilever (held at A by a force and a couple) with an end load: M = −P (L − s)
    const cant = beamDiagram(still([0, 0], [1, 0], 0), [{ at: [0, 0], F: [0, 2], M: 2 }, { at: [1, 0], F: [0, -2] }], { gravity: 0 });
    expect(valueAt(cant, cant.M, 0.25)).toBeCloseTo(-1.5, 9);
    expect(cant.closure.map(Math.abs).every((x) => x < 1e-9)).toBe(true);
  });

  const rod = (): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 1, y: 0, kind: 'free', mass: 0 }],
    links: [{ id: 0, a: 0, b: 1, mass: 2 }], springs: [], gears: [], meshes: [], rackMeshes: [],
  });

  it('a swinging rod with drag: the diagram closes at its free end (its motion accounts for every load)', () => {
    const d = rod();
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    sim.damping = 0.4;
    for (let i = 0; i < 350; i++) sim.step(1e-3);
    const { qdd, loads } = sim.snapshot();
    const r = jointForces(d, sim.sys, sim.q, sim.qd, qdd, loads, { gravity: G, friction: null, damping: 0.4 });
    const bd = barDiagram(r, 0, { gravity: G, damping: 0.4 })!;
    for (const k of [0, 1, 2]) expect(Math.abs(bd.closure[k])).toBeLessThan(1e-6);
    // at the pivot, the axial force is the pin force along the rod (pulling it towards the pivot: tension)
    const th = sim.q[0];
    const F = r.items.find((x) => x.key === 'pin:link:0@0')!.F;
    const along = F[0] * Math.cos(th) + F[1] * Math.sin(th);
    expect(valueAt(bd, bd.N, 1e-6)).toBeCloseTo(-along, 4);
    // and no moment at either pinned end (a free end, a frictionless pivot)
    expect(Math.abs(bd.M[0])).toBeLessThan(1e-9);
    expect(Math.abs(bd.M[bd.M.length - 1])).toBeLessThan(1e-6);
  });

  it('every bar of every preset: the diagram closes (the forces balance the motion)', () => {
    let checked = 0;
    for (const p of presets) {
      const d = p.build();
      const g = p.topView ? 0 : G;
      const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: g })));
      if (d.ground) sim.ground = { points: d.ground.points, restitution: d.ground.restitution, friction: d.ground.friction };
      for (let i = 0; i < 300; i++) sim.step(1e-3);
      const { qdd, loads } = sim.snapshot();
      const r = jointForces(d, sim.sys, sim.q, sim.qd, qdd, loads, { gravity: g, friction: null });
      for (const l of d.links) {
        const bd = barDiagram(r, l.id, { gravity: g });
        checked++;
        if (!bd) continue;
        const scale = 1 + Math.max(...bd.N.map(Math.abs), ...bd.V.map(Math.abs));
        expect(Math.abs(bd.closure[0]) / scale, `${p.name} bar ${l.id} N`).toBeLessThan(1e-5);
        expect(Math.abs(bd.closure[1]) / scale, `${p.name} bar ${l.id} V`).toBeLessThan(1e-5);
        expect(Math.abs(bd.closure[2]) / (scale * bd.L), `${p.name} bar ${l.id} M`).toBeLessThan(1e-5);
      }
    }
    expect(checked).toBeGreaterThan(50);
  }, 60000);
});
