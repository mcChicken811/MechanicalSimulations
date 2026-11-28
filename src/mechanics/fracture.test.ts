import { describe, expect, it } from 'vitest';
import { beamDiagram } from './beam';
import { deriveEquations } from './derive';
import { breakBars, breakPoints, transferState } from './fracture';
import type { Design } from './model';
import { NumericSystem, Simulator } from './simulate';

const G = 9.81;
const pendulum2 = (): Design => ({
  points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 1, y: 0, kind: 'free', mass: 0 }, { id: 2, x: 1.8, y: 0, kind: 'free', mass: 0.3 }],
  links: [{ id: 0, a: 0, b: 1, mass: 1 }, { id: 1, a: 1, b: 2, mass: 0.6 }], springs: [], gears: [], meshes: [], rackMeshes: [], joins: [],
});

/** Break bars of a running simulation as the app does: cut, derive, carry the state over. */
function breakSim(d: Design, sim: Simulator, breaks: { link: number; u: number[] }[]) {
  const res = breakBars(d, breaks);
  const next = new Simulator(new NumericSystem(deriveEquations(res.design, { gravity: G })));
  const st = transferState(sim.sys, sim.q, sim.qd, next.sys, res.cuts, res.pieces);
  next.q.set(st.q);
  next.qd.set(st.qd);
  next.project();
  next.energyOffset = sim.energy().V - next.energy().V; // as the app does: V's dropped constants differ
  return { next, ...res };
}

describe('breaking bars', () => {
  it('finds the largest |M| in each stretch over the threshold, off the ends', () => {
    const still = { id: 0, m: 0, A: [0, 0] as [number, number], B: [3, 0] as [number, number], vA: [0, 0] as [number, number], vB: [0, 0] as [number, number], aA: [0, 0] as [number, number], aB: [0, 0] as [number, number] };
    // supports at 0, 1.5 and 3 with loads at 0.75 and 2.25 (two sagging humps, a hogging middle)
    const d = beamDiagram(still, [{ at: [0, 0], F: [0, 4] }, { at: [0.75, 0], F: [0, -10] }, { at: [1.5, 0], F: [0, 12] }, { at: [2.25, 0], F: [0, -10] }, { at: [3, 0], F: [0, 4] }], { gravity: 0 });
    expect(breakPoints(d, 2.5)).toEqual([0.75, 2.25].map((v) => expect.closeTo(v, 9)) as unknown as number[]);
    expect(breakPoints(d, 100)).toEqual([]);
  });

  it('cuts a bar into pieces, the joints on it going with their piece', () => {
    const d = pendulum2();
    d.points.push({ id: 3, x: 0.8, y: 0, kind: 'free', mass: 0, pin: { kind: 'link', id: 0 } });
    const { design, cuts, pieces } = breakBars(d, [{ link: 0, u: [0.25, 0.6] }]);
    expect(pieces.map((p) => [p.u0, p.u1])).toEqual([[0, 0.25], [0.25, 0.6], [0.6, 1]]);
    expect(cuts.length).toBe(4);
    const masses = design.links.filter((l) => pieces.some((p) => p.id === l.id)).map((l) => l.mass);
    expect(masses.reduce((a, b) => a + b, 0)).toBeCloseTo(1, 12);
    expect(design.points.find((p) => p.id === 3)!.pin).toEqual({ kind: 'link', id: pieces[2].id }); // at 0.8: on the last piece
    expect(design.links.find((l) => l.id === 0)).toMatchObject({ a: 0, b: cuts[0].point });
  });

  it('a weld goes with the piece at the joint the bodies share', () => {
    const d = pendulum2();
    d.joins = [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 1 } }]; // a bent lever, joined at joint 1
    const { design, pieces } = breakBars(d, [{ link: 0, u: [0.5] }]);
    expect(design.joins![0].a).toEqual({ kind: 'link', id: pieces[1].id }); // the outer piece, which has joint 1
  });

  it('a swinging double pendulum breaks mid-bar without a jump: joints, velocities and energy carry on', () => {
    const d = pendulum2();
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < 400; i++) sim.step(1e-3);
    const before = sim.jointPositions();
    const e0 = sim.energy();
    const { next, cuts } = breakSim(d, sim, [{ link: 0, u: [0.5] }]);
    const after = next.jointPositions();
    for (const [id, p] of before) {
      expect(after.get(id)![0]).toBeCloseTo(p[0], 9);
      expect(after.get(id)![1]).toBeCloseTo(p[1], 9);
    }
    // the cut joints sit at the middle of where the bar was
    for (const c of cuts) {
      const mid = [(before.get(0)![0] + before.get(1)![0]) / 2, (before.get(0)![1] + before.get(1)![1]) / 2];
      expect(after.get(c.point)![0]).toBeCloseTo(mid[0], 9);
      expect(after.get(c.point)![1]).toBeCloseTo(mid[1], 9);
    }
    const e1 = next.energy();
    expect(e1.T).toBeCloseTo(e0.T, 6);
    expect(e1.V).toBeCloseTo(e0.V, 6);
    // and it carries on: the two halves part, energy still kept
    for (let i = 0; i < 300; i++) expect(next.step(1e-3)).toBe(true);
    const e2 = next.energy();
    expect(e2.T + e2.V).toBeCloseTo(e0.T + e0.V, 3);
    const p = next.jointPositions();
    expect(Math.hypot(p.get(cuts[0].point)![0] - p.get(cuts[1].point)![0], p.get(cuts[0].point)![1] - p.get(cuts[1].point)![1])).toBeGreaterThan(0.01);
  });

  it('two cuts leave a free piece in the middle, flying on with its motion', () => {
    const d = pendulum2();
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < 250; i++) sim.step(1e-3);
    const e0 = sim.energy();
    const { next } = breakSim(d, sim, [{ link: 0, u: [0.3, 0.7] }, { link: 1, u: [0.5] }]);
    const e1 = next.energy();
    expect(e1.T).toBeCloseTo(e0.T, 6);
    expect(e1.V).toBeCloseTo(e0.V, 6);
    for (let i = 0; i < 300; i++) expect(next.step(1e-3)).toBe(true);
    const e2 = next.energy();
    expect(e2.T + e2.V).toBeCloseTo(e0.T + e0.V, 3);
  });
});
