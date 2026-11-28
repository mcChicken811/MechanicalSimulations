import { describe, expect, it } from 'vitest';
import { deriveEquations } from './derive';
import type { Design } from './model';
import { INTEGRATORS, NumericSystem, Simulator, type Integrator } from './simulate';

const G = 9.81;
const pendulum = (): Design => ({
  points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 1, y: 0, kind: 'free', mass: 0 }],
  links: [{ id: 0, a: 0, b: 1, mass: 1 }], springs: [], gears: [], meshes: [], rackMeshes: [],
});
const run = (method: Integrator, h: number, T = 1, d = pendulum(), setup?: (s: Simulator) => void) => {
  const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
  sim.integrator = method;
  setup?.(sim);
  const e0 = sim.energy();
  for (let i = 0; i < Math.round(T / h); i++) expect(sim.step(h)).toBe(true);
  const e1 = sim.energy();
  return { sim, angle: sim.q[0], drift: e1.T + e1.V - (e0.T + e0.V) };
};

describe('integrators', () => {
  // the reference: RK45 at a tight tolerance
  const reference = () => run('rk45', 1e-3, 1, pendulum(), (s) => (s.tolerance = 1e-12)).angle;

  it('each one converges at its order (the error falls by about 2^order as the step halves)', () => {
    const ref = reference();
    for (const m of ['euler', 'symplectic-euler', 'midpoint', 'heun', 'verlet', 'rk3', 'rk4'] as Integrator[]) {
      const [e1, e2] = [4e-3, 2e-3].map((h) => Math.abs(run(m, h).angle - ref));
      const order = Math.log2(e1 / e2);
      expect(order, `${m}: ${e1} → ${e2}`).toBeGreaterThan(INTEGRATORS[m].order - 0.35);
    }
  }, 60000);

  it('RK45 keeps to its tolerance: a 1 ms step taken in sub-steps where needed', () => {
    const ref = reference();
    const loose = run('rk45', 1e-3, 1, pendulum(), (s) => (s.tolerance = 1e-5));
    const tight = run('rk45', 1e-3, 1, pendulum(), (s) => (s.tolerance = 1e-9));
    expect(Math.abs(tight.angle - ref)).toBeLessThan(1e-7);
    expect(Math.abs(tight.angle - ref)).toBeLessThan(Math.abs(loose.angle - ref) + 1e-12);
    // a 20 ms step still comes out right: it takes the sub-steps it needs
    const big = run('rk45', 20e-3, 1, pendulum(), (s) => (s.tolerance = 1e-9));
    expect(Math.abs(big.angle - ref)).toBeLessThan(1e-6);
  }, 60000);

  it('energy over 10 s at 5 ms: explicit Euler gains it, Verlet and symplectic Euler keep it bounded, RK4 holds it', () => {
    const euler = run('euler', 5e-3, 10).drift;
    const verlet = run('verlet', 5e-3, 10).drift;
    const sympl = run('symplectic-euler', 5e-3, 10).drift;
    const rk4 = run('rk4', 5e-3, 10).drift;
    expect(euler).toBeGreaterThan(0.5); // J: it pumps energy in
    expect(Math.abs(verlet)).toBeLessThan(0.05);
    expect(Math.abs(sympl)).toBeLessThan(0.3);
    expect(Math.abs(rk4)).toBeLessThan(1e-3);
  }, 60000);

  it('the energy budget holds with friction and a motor, with every integrator (its own stage weights)', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 0.5, y: 0, kind: 'free', mass: 0 }],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }], springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 0, radius: 0.1, teeth: 12, mass: 0.2, locked: false, motor: { torque: 0.5, noLoadRpm: 60, mass: 0.2, mounted: true } }],
      joins: [{ id: 0, a: { kind: 'gear', id: 0 }, b: { kind: 'link', id: 0 } }],
    };
    for (const m of Object.keys(INTEGRATORS) as Integrator[]) {
      const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
      sim.integrator = m;
      sim.friction = { ...sim.friction, enabled: true, pin: 0.05 };
      const e0 = sim.energy();
      for (let i = 0; i < 2000; i++) sim.step(5e-4);
      const e1 = sim.energy();
      const budget = e1.T + e1.V + sim.dissipated - sim.motorWork - (e0.T + e0.V);
      const scale = Math.max(1, sim.motorWork);
      expect(Math.abs(budget) / scale, `${m}`).toBeLessThan(INTEGRATORS[m].order >= 3 ? 1e-4 : 2e-2);
    }
  }, 60000);
});
