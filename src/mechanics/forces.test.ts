import { describe, expect, it } from 'vitest';
import { deriveEquations } from './derive';
import { jointForces } from './forces';
import type { Design } from './model';
import { NumericSystem, Simulator } from './simulate';
import { presets } from '../ui/presets';

const G = 9.81;
const solve = (d: Design, sim: Simulator) => {
  const { qdd, loads } = sim.snapshot();
  return jointForces(d, sim.sys, sim.q, sim.qd, qdd, loads, { gravity: G, friction: null });
};
const item = (r: ReturnType<typeof jointForces>, key: string) => r.items.find((x) => x.key === key)!;

describe('joint forces (inverse dynamics)', () => {
  const rod = (): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 1, y: 0, kind: 'free', mass: 0 }],
    links: [{ id: 0, a: 0, b: 1, mass: 2 }], springs: [], gears: [], meshes: [], rackMeshes: [],
  });

  it('a rod pendulum let go level: its pivot carries m g / 4', () => {
    const d = rod();
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    const r = solve(d, sim);
    const F = item(r, 'pin:link:0@0').F;
    expect(F[0]).toBeCloseTo(0, 9);
    expect(F[1]).toBeCloseTo((2 * G) / 4, 6);
    expect(item(r, 'ground@0').F[1]).toBeCloseTo((2 * G) / 4, 6); // the ground holds the joint the same
    expect(item(r, 'pin:link:0@1').F).toEqual([0, 0].map((v) => expect.closeTo(v, 9)) as unknown as [number, number]); // the free end carries nothing
    expect(r.residual).toBeLessThan(1e-9);
    expect(r.indeterminate).toBe(false);
  });

  it('a swinging rod: along it m (g cos θ + ω² L/2), across it m g sin θ / 4', () => {
    const d = rod();
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < 300; i++) sim.step(1e-3);
    const r = solve(d, sim);
    const F = item(r, 'pin:link:0@0').F;
    const th = sim.q[0]; // the bar's angle from +x
    const w = sim.qd[0];
    const u: [number, number] = [Math.cos(th), Math.sin(th)]; // from the pivot along the rod
    const phi = th + Math.PI / 2; // angle from straight down
    const along = -(F[0] * u[0] + F[1] * u[1]); // pulling towards the pivot
    expect(along).toBeCloseTo(2 * (G * Math.cos(phi) + (w * w * 1) / 2), 4);
    expect(Math.abs(F[0] * -u[1] + F[1] * u[0])).toBeCloseTo((2 * G * Math.abs(Math.sin(phi))) / 4, 4);
    expect(r.residual).toBeLessThan(1e-6);
  });

  it('a four-bar with a nearly massless coupler: the coupler is pushed only along itself', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 0.3, y: 0.6, kind: 'free', mass: 0 }, { id: 2, x: 1.4, y: 0.8, kind: 'free', mass: 0 }, { id: 3, x: 1.5, y: 0, kind: 'fixed', mass: 0 }],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }, { id: 1, a: 1, b: 2, mass: 1e-6 }, { id: 2, a: 2, b: 3, mass: 1 }], springs: [], gears: [], meshes: [], rackMeshes: [],
    };
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < 400; i++) sim.step(1e-3);
    const r = solve(d, sim);
    const pos = sim.jointPositions();
    const dir = [pos.get(2)![0] - pos.get(1)![0], pos.get(2)![1] - pos.get(1)![1]];
    const L = Math.hypot(dir[0], dir[1]);
    for (const end of [1, 2]) {
      const F = item(r, `pin:link:1@${end}`).F;
      expect(Math.abs(F[0] * dir[1] - F[1] * dir[0]) / L).toBeLessThan(1e-4 * Math.hypot(...F) + 1e-9);
    }
    const [Fa, Fb] = [item(r, 'pin:link:1@1').F, item(r, 'pin:link:1@2').F];
    expect(Fa[0] + Fb[0]).toBeCloseTo(0, 4); // equal and opposite at its two ends
    expect(r.residual).toBeLessThan(1e-6);
  });

  it('a motor-driven gear pair: the tooth force turns the driven gear, F R_b = I_b α_b', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 0.5, y: 0, kind: 'fixed', mass: 0 }],
      links: [], springs: [], rackMeshes: [],
      gears: [{ id: 0, center: 0, radius: 0.2, teeth: 20, mass: 1, locked: false, motor: { torque: 0.8, noLoadRpm: 0, mounted: true } }, { id: 1, center: 1, radius: 0.3, teeth: 30, mass: 2, locked: false }],
      meshes: [{ id: 0, a: 0, b: 1 }],
    };
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    sim.step(1e-3);
    const { qdd, loads } = sim.snapshot();
    const r = jointForces(d, sim.sys, sim.q, sim.qd, qdd, loads, { gravity: G, friction: null });
    const F = Math.hypot(...item(r, 'mesh:0:b').F);
    const alphaB = Math.abs(qdd[sim.sys.gearCoord.get(1)!]);
    expect(F * 0.3).toBeCloseTo(0.5 * 2 * 0.09 * alphaB, 6);
    // the driving gear: τ − F R_a = I_a α_a
    const alphaA = Math.abs(qdd[sim.sys.gearCoord.get(0)!]);
    expect(0.8 - F * 0.2).toBeCloseTo(0.5 * 1 * 0.04 * alphaA, 6);
    expect(r.residual).toBeLessThan(1e-9);
  });

  it('flags a parallelogram braced by a third parallel bar as indeterminate', () => {
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 0, y: 1, kind: 'free', mass: 0 },
        { id: 2, x: 1, y: 0, kind: 'fixed', mass: 0 }, { id: 3, x: 1, y: 1, kind: 'free', mass: 0, pin: { kind: 'link', id: 3 } },
        { id: 4, x: 2, y: 0, kind: 'fixed', mass: 0 }, { id: 5, x: 2, y: 1, kind: 'free', mass: 0 },
      ],
      // one rigid coupler across all three rockers (the middle one pinned onto it): one rocker too many
      links: [{ id: 0, a: 0, b: 1, mass: 1 }, { id: 1, a: 2, b: 3, mass: 1 }, { id: 2, a: 4, b: 5, mass: 1 }, { id: 3, a: 1, b: 5, mass: 1 }],
      springs: [], gears: [], meshes: [], rackMeshes: [],
    };
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    sim.qd[0] = 0.5;
    const r = solve(d, sim);
    expect(r.indeterminate).toBe(true);
    expect(r.residual).toBeLessThan(1e-6); // still a valid split
  });

  it('accounts for the motion of every example (residual near zero, friction off)', () => {
    for (const p of presets) {
      const d = p.build();
      const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: p.topView ? 0 : G })));
      if (d.ground) sim.ground = { points: d.ground.points, restitution: d.ground.restitution, friction: d.ground.friction };
      for (let i = 0; i < 300; i++) sim.step(1e-3);
      const { qdd, loads } = sim.snapshot();
      const r = jointForces(d, sim.sys, sim.q, sim.qd, qdd, loads, { gravity: p.topView ? 0 : G, friction: null });
      const scale = 1 + Math.max(...r.items.map((x) => Math.hypot(...x.F) + Math.abs(x.M ?? 0)), 0);
      expect(r.residual / scale, p.name).toBeLessThan(1e-5);
    }
  }, 60000);
});
