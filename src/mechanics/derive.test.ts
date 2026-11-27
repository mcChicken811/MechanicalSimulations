import { describe, expect, it } from 'vitest';
import { deriveEquations } from './derive';
import { cloneDesign, propellerInertia, type Design } from './model';
import { NumericSystem, Simulator, straightGroundSettings } from './simulate';
import { motorCar, twinRotor } from '../ui/presets';

const G = 9.81;

function doublePendulum(): Design {
  return {
    points: [
      { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
      { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
      { id: 2, x: 1, y: -0.5, kind: 'free', mass: 0 },
    ],
    springs: [],
    gears: [],
    meshes: [], rackMeshes: [],
    links: [
      { id: 0, a: 0, b: 1, mass: 2 },
      { id: 1, a: 1, b: 2, mass: 1 },
    ],
  };
}

function fourBar(): Design {
  return {
    points: [
      { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
      { id: 1, x: 0.3, y: 0.6, kind: 'free', mass: 0 },
      { id: 2, x: 1.4, y: 0.8, kind: 'free', mass: 0 },
      { id: 3, x: 1.5, y: 0, kind: 'fixed', mass: 0 },
    ],
    springs: [],
    gears: [],
    meshes: [], rackMeshes: [],
    links: [
      { id: 0, a: 0, b: 1, mass: 1 },
      { id: 1, a: 1, b: 2, mass: 1 },
      { id: 2, a: 2, b: 3, mass: 1 },
    ],
  };
}

function sliderCrank(): Design {
  return {
    points: [
      { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
      { id: 1, x: 0.4, y: 0.3, kind: 'free', mass: 0 },
      { id: 2, x: 1.6, y: 0, kind: 'lockY', mass: 0.5 },
    ],
    springs: [],
    gears: [],
    meshes: [], rackMeshes: [],
    links: [
      { id: 0, a: 0, b: 1, mass: 1 },
      { id: 1, a: 1, b: 2, mass: 1 },
    ],
  };
}

function runAndMeasureEnergyDrift(design: Design, seconds: number, kick: number[] = []) {
  const der = deriveEquations(design, { gravity: G });
  const sim = new Simulator(new NumericSystem(der));
  kick.forEach((v, i) => (sim.qd[i] = v));
  sim.project();
  const e0 = sim.energy();
  let maxDrift = 0;
  let maxPhi = 0;
  for (let i = 0; i < seconds / 1e-3; i++) {
    expect(sim.step(1e-3)).toBe(true);
    const e = sim.energy();
    maxDrift = Math.max(maxDrift, Math.abs(e.T + e.V - e0.T - e0.V));
    maxPhi = Math.max(maxPhi, sim.constraintError());
  }
  return { der, sim, maxDrift, maxPhi, scale: Math.abs(e0.T) + Math.abs(e0.V) + 1 };
}

describe('deriveEquations', () => {
  it('reproduces the textbook double pendulum of uniform rods', () => {
    const der = deriveEquations(doublePendulum(), { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['th0', 'th1']);
    expect(der.constraints).toHaveLength(0);

    const env: Record<string, number> = { th0: 0.7, th1: -0.4, th0_d: 1.3, th1_d: -2.1, m0: 2, m1: 1, L0: 1, L1: 0.5, g: G };
    const { th0: t1, th1: t2, th0_d: w1, th1_d: w2, m0: m1, m1: m2, L0: L1, L1: L2 } = env;
    const M = [
      [(m1 / 3 + m2) * L1 ** 2, 0.5 * m2 * L1 * L2 * Math.cos(t1 - t2)],
      [0.5 * m2 * L1 * L2 * Math.cos(t1 - t2), (m2 * L2 ** 2) / 3],
    ];
    const f = [
      -0.5 * m2 * L1 * L2 * Math.sin(t1 - t2) * w2 ** 2 - (m1 / 2 + m2) * G * L1 * Math.cos(t1),
      0.5 * m2 * L1 * L2 * Math.sin(t1 - t2) * w1 ** 2 - 0.5 * m2 * G * L2 * Math.cos(t2),
    ];
    for (let i = 0; i < 2; i++) {
      for (let j = 0; j < 2; j++) expect(der.M[i][j].evaluate(env)).toBeCloseTo(M[i][j], 12);
      expect(der.f[i].evaluate(env)).toBeCloseTo(f[i], 12);
    }
    // the trig simplifier should have collapsed the coupling term to a single cos(θ₁ − θ₂)
    expect(der.M[0][1].terms.size).toBe(1);
  });

  it('conserves energy for the double pendulum under RK4', () => {
    const { maxDrift, scale } = runAndMeasureEnergyDrift(doublePendulum(), 3, [2, -1]);
    expect(maxDrift / scale).toBeLessThan(1e-6);
  });

  it('turns a closed loop into one length constraint and keeps it satisfied', () => {
    const { der, maxDrift, maxPhi, scale } = runAndMeasureEnergyDrift(fourBar(), 3, [3, 0, 0]);
    expect(der.coords).toHaveLength(2); // θ of the two ground bars
    expect(der.constraints).toHaveLength(1);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-3);
  });

  it('turns a locked axis on a tree joint into a constraint (slider-crank)', () => {
    const { der, sim, maxPhi } = runAndMeasureEnergyDrift(sliderCrank(), 2, [4, 0]);
    expect(der.constraints).toHaveLength(1);
    expect(der.constraints[0].reason).toMatch(/y locked/);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(sim.jointPositions().get(2)![1]).toBeCloseTo(0, 8);
  });

  it('seeds a floating component at its locked joint (cart-pole has no constraints)', () => {
    const der = deriveEquations(
      {
        points: [
          { id: 0, x: 0, y: 0, kind: 'lockY', mass: 2 },
          { id: 1, x: 0, y: -1, kind: 'free', mass: 0 },
        ],
        springs: [],
        gears: [],
        meshes: [], rackMeshes: [],
        links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      },
      { gravity: G },
    );
    expect(der.coords.map((c) => c.kind)).toEqual(['x', 'angle']);
    expect(der.constraints).toHaveLength(0);
  });

  it('applies a mouse-spring force through Q = Jᵀ F', () => {
    const der = deriveEquations(doublePendulum(), { gravity: 0 });
    const sim = new Simulator(new NumericSystem(der));
    sim.grab = { a: 2, b: 2, s: 0, target: [1, 1], stiffness: 100 };
    for (let i = 0; i < 200; i++) sim.step(1e-3);
    // the tip was pulled upward from y = −0.5
    expect(sim.jointPositions().get(2)![1]).toBeGreaterThan(-0.5);
  });
});

describe('springs', () => {
  it('add no coordinates or constraints, and give F = −k(ℓ − ℓ₀) on a rail', () => {
    const k = 40;
    const m = 2;
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.8, y: 0, kind: 'lockY', mass: m },
      ],
      links: [],
      springs: [{ id: 0, a: 0, b: 1, k, rest: 0.5 }],
      gears: [],
      meshes: [], rackMeshes: [],
    };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.map((c) => c.kind)).toEqual(['x']);
    expect(der.constraints).toHaveLength(0);
    const sim = new Simulator(new NumericSystem(der));
    const a = new Float64Array(1);
    sim.accelerations(sim.q, sim.qd, a);
    expect(a[0]).toBeCloseTo((-k * (0.8 - 0.5)) / m, 12);
    expect(sim.energy().V).toBeCloseTo(0.5 * k * 0.3 ** 2, 12);
  });

  it('give a generalized force equal to −∂V/∂q (spring on a bar tip)', () => {
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0.8, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.6, y: 0, kind: 'free', mass: 0.5 },
        { id: 2, x: 1.3, y: 0.3, kind: 'fixed', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [{ id: 0, a: 1, b: 2, k: 30, rest: 0.4 }],
      gears: [],
      meshes: [], rackMeshes: [],
    };
    const sys = new NumericSystem(deriveEquations(design, { gravity: G }));
    const q = Float64Array.from([-0.9]);
    const f = new Float64Array(1);
    sys.force(q, new Float64Array(1), f);
    const h = 1e-6;
    const V = (th: number) => sys.energy([th], [0]).V;
    expect(f[0]).toBeCloseTo(-(V(q[0] + h) - V(q[0] - h)) / (2 * h), 6);
  });

  it('conserve energy for an elastic pendulum', () => {
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.5, y: -0.7, kind: 'free', mass: 1 },
      ],
      links: [],
      springs: [{ id: 0, a: 0, b: 1, k: 60, rest: 0.6 }],
      gears: [],
      meshes: [], rackMeshes: [],
    };
    const { der, maxDrift, scale } = runAndMeasureEnergyDrift(design, 3, [1, 0]);
    expect(der.coords.map((c) => c.kind)).toEqual(['x', 'y']);
    expect(maxDrift / scale).toBeLessThan(1e-6);
  });

  it('hold a massless joint that only springs touch in place', () => {
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 2, y: 0, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [{ id: 0, a: 1, b: 2, k: 10, rest: 0.5 }],
      gears: [],
      meshes: [], rackMeshes: [],
    };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords).toHaveLength(1);
    expect(der.springs).toHaveLength(1);
    expect(der.warnings.join(' ')).toMatch(/Joint C .* held in place/);
  });
});

describe('Strandbeest leg preset', () => {
  it('has Jansen\'s bar lengths and one degree of freedom', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets[0].build();
    const { linkLength } = await import('./model');
    const lengths = design.links.map((l) => +(linkLength(design, l) * 100).toFixed(6));
    expect(lengths).toEqual([15, 50, 41.5, 55.8, 40.1, 61.9, 39.3, 39.4, 36.7, 49, 65.7]);
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
  });
});

describe('gears', () => {
  const P = (id: number, x: number, y: number, kind: 'fixed' | 'free' = 'free', mass = 0, gear?: number) => ({ id, x, y, kind, mass, ...(gear === undefined ? {} : { gear }) });
  const gear = (id: number, center: number, radius: number, teeth: number, mass: number, locked = false) => ({ id, center, radius, teeth, mass, locked });

  it('a free-spinning gear is a rigid disk: I = ½ m R²', () => {
    const design: Design = { points: [P(0, 0, 0, 'fixed')], links: [], springs: [], gears: [gear(0, 0, 0.3, 20, 2)], meshes: [], rackMeshes: [] };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['ph0']);
    expect(der.M[0][0].evaluate({ mg0: 2, R0: 0.3 })).toBeCloseTo(0.5 * 2 * 0.09, 12);
  });

  it('a bar pinned onto a gear rim makes a physical pendulum about the gear axle', () => {
    // gear (m = 1, R = 0.2) on a fixed axle with a 0.5 kg point mass pinned 0.2 m out: one coordinate φ
    const design: Design = {
      points: [P(0, 0, 0, 'fixed'), P(1, 0.2, 0, 'free', 0.5, 0)],
      links: [],
      springs: [],
      gears: [gear(0, 0, 0.2, 13, 1)],
      meshes: [], rackMeshes: [],
    };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords).toHaveLength(1);
    const I = 0.5 * 1 * 0.04 + 0.5 * 0.04;
    const env = { ph0: 0, mg0: 1, R0: 0.2, mp1: 0.5, u1: 0.2, v1: 0, g: G };
    expect(der.M[0][0].evaluate(env)).toBeCloseTo(I, 12);
    expect(der.f[0].evaluate(env)).toBeCloseTo(-0.5 * G * 0.2, 12); // torque of the weight at φ = 0
  });

  it('meshing gears on fixed axles keep N_a Δφ_a + N_b Δφ_b = 0 and conserve energy', () => {
    const design: Design = {
      points: [P(0, 0, 0, 'fixed'), P(1, 0.5, 0, 'fixed'), P(2, 0, 0.15, 'free', 0.4, 0)],
      links: [],
      springs: [],
      gears: [gear(0, 0, 0.2, 16, 0.5), gear(1, 1, 0.3, 24, 1)],
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
    };
    const { der, sim, maxDrift, maxPhi, scale } = runAndMeasureEnergyDrift(design, 2, [3, 0]);
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-6);
    expect(16 * sim.q[0] + 24 * sim.q[1]).toBeCloseTo(0, 8);
    expect(Math.abs(sim.q[0])).toBeGreaterThan(1e-2);
  });

  it('a planet on a carrier around a locked sun turns relative to the carrier', () => {
    const design: Design = {
      points: [P(0, 0, 0, 'fixed'), P(1, 0.45, 0)],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [],
      gears: [gear(0, 0, 0.3, 30, 1, true), gear(1, 1, 0.15, 15, 0.5)],
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
    };
    const { der, sim, maxPhi, maxDrift, scale } = runAndMeasureEnergyDrift(design, 2, [2, 0]);
    expect(der.coords.map((c) => c.sym)).toEqual(['th0', 'ph1']);
    const carrier = sim.q[0] - der.coords[0].initial;
    // 30(0 − Δθ_c) + 15(Δφ_p − Δθ_c) = 0  ⇒  Δφ_p = 3 Δθ_c
    expect(sim.q[1]).toBeCloseTo(3 * carrier, 8);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-6);
  });

  it('a loop closed through a gear gives coincidence constraints and one DOF (geared five-bar)', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets.find((p) => p.name === 'Geared five-bar linkage')!.build();
    const { der, maxPhi, maxDrift, scale } = runAndMeasureEnergyDrift(design, 2, [1, 0, 0]);
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-4);
  });
});

describe('friction and heating', () => {
  const pendulum = (): Design => ({
    points: [
      { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
      { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
    ],
    links: [{ id: 0, a: 0, b: 1, mass: 3 }],
    springs: [],
    gears: [],
    meshes: [], rackMeshes: [],
  });

  it('a pivot gives the torque −c θ̇ (Rayleigh R = ½ c θ̇²)', () => {
    const sim = new Simulator(new NumericSystem(deriveEquations(pendulum(), { gravity: 0 })));
    sim.qd[0] = 2;
    const a0 = new Float64Array(1);
    const a1 = new Float64Array(1);
    sim.accelerations(sim.q, sim.qd, a0);
    sim.friction = { ...sim.friction, enabled: true, pin: 0.3 };
    sim.accelerations(sim.q, sim.qd, a1);
    const I = (3 * 1) / 3;
    expect(a1[0] - a0[0]).toBeCloseTo((-0.3 * 2) / I, 12);
    expect(sim.power[0]).toBeCloseTo(0.3 * 4, 12); // P = c ω² at joint A
    expect(sim.power[1]).toBe(0); // the free end touches one body only
  });

  it('turns lost mechanical energy into heat: T + V + dissipated is conserved', () => {
    const sim = new Simulator(new NumericSystem(deriveEquations(doublePendulum(), { gravity: G })));
    sim.friction = { ...sim.friction, enabled: true, pin: 0.05 };
    sim.qd.set([2, -1]);
    const e0 = sim.energy();
    for (let i = 0; i < 3000; i++) sim.step(1e-3);
    const e1 = sim.energy();
    expect(sim.dissipated).toBeGreaterThan(0.05);
    expect(Math.abs(e1.T + e1.V + sim.dissipated - e0.T - e0.V)).toBeLessThan(1e-6);
    const heat = sim.heat().joints;
    expect(heat.get(0)!).toBeGreaterThan(0); // pivot
    expect(heat.get(1)!).toBeGreaterThan(0); // elbow
    expect(heat.get(2)!).toBe(0); // free tip
  });

  it('cools by Newton\'s law: ΔT ∝ e^(−t/τ) once friction stops', () => {
    const sim = new Simulator(new NumericSystem(deriveEquations(pendulum(), { gravity: G })));
    sim.friction = { ...sim.friction, enabled: true, pin: 0.2, coolingTime: 2 };
    sim.qd[0] = 3;
    for (let i = 0; i < 500; i++) sim.step(1e-3);
    sim.friction.enabled = false;
    const T0 = sim.jointHeat[0];
    expect(T0).toBeGreaterThan(0);
    for (let i = 0; i < 2000; i++) sim.step(1e-3);
    expect(sim.jointHeat[0] / T0).toBeCloseTo(Math.exp(-1), 9);
  });

  it('heats a gear mesh and a rail', () => {
    const gears: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.5, y: 0, kind: 'fixed', mass: 0 },
      ],
      links: [],
      springs: [],
      gears: [
        { id: 0, center: 0, radius: 0.2, teeth: 16, mass: 1, locked: false },
        { id: 1, center: 1, radius: 0.3, teeth: 24, mass: 1, locked: false },
      ],
      meshes: [{ id: 0, a: 0, b: 1 }], rackMeshes: [],
    };
    const sim = new Simulator(new NumericSystem(deriveEquations(gears, { gravity: 0 })));
    sim.friction = { ...sim.friction, enabled: true, pin: 0, mesh: 2 };
    sim.qd.set([3, -2]);
    const e0 = sim.energy();
    for (let i = 0; i < 1000; i++) sim.step(1e-3);
    const e1 = sim.energy();
    expect(sim.heat().meshes.get(0)!).toBeGreaterThan(0);
    expect(Math.abs(e1.T + e1.V + sim.dissipated - e0.T - e0.V)).toBeLessThan(1e-6);

    const cart: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'lockY', mass: 2 },
        { id: 1, x: 0.5, y: -0.5, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [],
      gears: [],
      meshes: [], rackMeshes: [],
    };
    const s2 = new Simulator(new NumericSystem(deriveEquations(cart, { gravity: G })));
    s2.friction = { ...s2.friction, enabled: true, pin: 0, rail: 1 };
    for (let i = 0; i < 1000; i++) s2.step(1e-3);
    expect(s2.heat().joints.get(0)!).toBeGreaterThan(0);
  });
});

describe('masses placed on their own', () => {
  it('a lone mass is a particle in free fall', () => {
    const design: Design = { points: [{ id: 0, x: 0.3, y: 1, kind: 'free', mass: 1 }], links: [], springs: [], gears: [], meshes: [], rackMeshes: [] };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.map((c) => c.kind)).toEqual(['x', 'y']);
    const sim = new Simulator(new NumericSystem(der));
    const a = new Float64Array(2);
    sim.accelerations(sim.q, sim.qd, a);
    expect(a[0]).toBeCloseTo(0, 12);
    expect(a[1]).toBeCloseTo(-G, 12);
  });
});

describe('racks', () => {
  const gear = (id: number, center: number, radius: number, teeth: number, mass: number, locked = false) => ({ id, center, radius, teeth, mass, locked });

  it('a pinion on a fixed axle drives a rack on rails: Δx = R Δφ', () => {
    // pinion R = 0.3 at the origin with a 1 kg weight pinned 0.2 m out; rack below it on two y-locked sliders
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: -1, y: -0.3, kind: 'lockY', mass: 0 },
        { id: 2, x: 1, y: -0.3, kind: 'lockY', mass: 0 },
        { id: 3, x: 0.2, y: 0, kind: 'free', mass: 1, gear: 0 },
      ],
      links: [{ id: 0, a: 1, b: 2, mass: 2, rack: 1 }],
      springs: [],
      gears: [gear(0, 0, 0.3, 20, 1)],
      meshes: [],
      rackMeshes: [{ id: 0, gear: 0, rack: 0 }],
    };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['ph0', 'x1', 'th0']);
    // y lock on the far slider, and rolling; the rails already hold the rack at distance R
    expect(der.constraints.map((c) => !!c.rack)).toEqual([false, true]);
    expect(der.constraints.some((c) => c.rackDistance)).toBe(false);

    const sim = new Simulator(new NumericSystem(der));
    const acc = new Float64Array(3);
    sim.accelerations(sim.q, sim.qd, acc);
    const I = 0.5 * 1 * 0.09 + 1 * 0.04 + 2 * 0.09; // disk + weight + rack mass reflected through R
    expect(acc[0]).toBeCloseTo((-1 * G * 0.2) / I, 8);
    expect(acc[1]).toBeCloseTo(0.3 * acc[0], 8);

    const { sim: s2, maxDrift, maxPhi, scale } = runAndMeasureEnergyDrift(design, 2);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-6);
    expect(s2.q[1] - -1).toBeCloseTo(0.3 * s2.q[0], 9);
    expect(Math.abs(s2.q[0])).toBeGreaterThan(0.1);
  });

  it('a gear on a free axle rolls down a fixed inclined rack: a = g sin α / (1 + I/mR²)', () => {
    const al = Math.PI / 6;
    const R = 0.2;
    const e = [Math.cos(al), Math.sin(al)];
    const n = [-Math.sin(al), Math.cos(al)];
    const c = [1.2 * e[0] + R * n[0], 1.2 * e[1] + R * n[1]];
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 2 * e[0], y: 2 * e[1], kind: 'fixed', mass: 0 },
        { id: 2, x: c[0], y: c[1], kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1, rack: 1 }],
      springs: [],
      gears: [gear(0, 2, R, 13, 1)],
      meshes: [],
      rackMeshes: [{ id: 0, gear: 0, rack: 0 }],
    };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(der.constraints.filter((k) => k.rackDistance)).toHaveLength(1);
    const { sim, maxPhi, maxDrift, scale } = runAndMeasureEnergyDrift(design, 0.5);
    expect(maxPhi).toBeLessThan(1e-9);
    expect(maxDrift / scale).toBeLessThan(1e-6);
    const pos = sim.jointPositions().get(2)!;
    const along = (pos[0] - c[0]) * e[0] + (pos[1] - c[1]) * e[1];
    expect(along).toBeCloseTo(-0.5 * ((G * Math.sin(al)) / 1.5) * 0.25, 6);
    // rolling down to the left on top of the rack turns the gear anticlockwise: Δξ = −R Δφ
    const phi = sim.q[der.coords.findIndex((k) => k.sym === 'ph0')];
    expect(phi).toBeCloseTo(-along / R, 6);
  });

  it('rack teeth friction heats the mesh', () => {
    const design: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: -1, y: -0.3, kind: 'lockY', mass: 0 },
        { id: 2, x: 1, y: -0.3, kind: 'lockY', mass: 0 },
        { id: 3, x: 0.2, y: 0, kind: 'free', mass: 1, gear: 0 },
      ],
      links: [{ id: 0, a: 1, b: 2, mass: 2, rack: 1 }],
      springs: [],
      gears: [gear(0, 0, 0.3, 20, 1)],
      meshes: [],
      rackMeshes: [{ id: 0, gear: 0, rack: 0 }],
    };
    const sim = new Simulator(new NumericSystem(deriveEquations(design, { gravity: G })));
    sim.friction = { ...sim.friction, enabled: true, pin: 0, rail: 0, mesh: 2 };
    const e0 = sim.energy();
    for (let i = 0; i < 1000; i++) sim.step(1e-3);
    const e = sim.energy();
    expect(sim.heat().racks.get(0)!).toBeGreaterThan(0);
    expect(e.T + e.V + sim.dissipated).toBeCloseTo(e0.T + e0.V, 6);
  });
});

describe('trapezoid steering preset', () => {
  it('turns the inner wheel more, close to true Ackermann', async () => {
    const { presets } = await import('../ui/presets');
    const p = presets.find((x) => x.name.startsWith('Trapezoid'))!;
    expect(p.topView).toBe(true);
    const d = deriveEquations(p.build(), { gravity: 0 });
    expect(d.coords.length - d.constraints.length).toBe(1);
    expect(d.warnings).toEqual([]);
    const sim = new Simulator(new NumericSystem(d));
    // wheel angle from straight ahead: kingpins are joints 0 and 3, wheel fronts 2 and 5
    const angle = (pos: Map<number, [number, number]>, k: number, w: number) => Math.atan2(pos.get(w)![0] - pos.get(k)![0], pos.get(w)![1] - pos.get(k)![1]);
    let worst = 0;
    for (let i = 0; i < 3000; i++) {
      sim.step(1e-3);
      const pos = sim.jointPositions();
      const [a, b] = [Math.abs(angle(pos, 0, 2)), Math.abs(angle(pos, 3, 5))];
      const inner = Math.max(a, b);
      if (inner < 0.05) continue;
      // kingpins 1.6 m apart, wheelbase 1.6 m: cot(outer) = cot(inner) + 1
      const ideal = Math.atan(1 / (1 / Math.tan(inner) + 1));
      worst = Math.max(worst, Math.abs(Math.min(a, b) - ideal));
    }
    expect(worst).toBeLessThan((1 * Math.PI) / 180);
    expect(sim.constraintError()).toBeLessThan(1e-9);
  });
});

describe('fixed angles', () => {
  const run = (d: Design, seconds: number) => {
    const sim = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < seconds * 1000; i++) sim.step(1e-3);
    return sim;
  };
  const angleAt = (pos: Map<number, [number, number]>, c: number, a: number, b: number) => {
    const [pc, pa, pb] = [pos.get(c)!, pos.get(a)!, pos.get(b)!];
    return Math.atan2(pb[1] - pc[1], pb[0] - pc[0]) - Math.atan2(pa[1] - pc[1], pa[0] - pc[0]);
  };

  it('welds a double pendulum into one rigid L, moving like the same L braced by a massless bar', () => {
    const welded = { ...doublePendulum(), angles: [{ id: 0, a: 0, b: 1 }] };
    const d = deriveEquations(welded, { gravity: G });
    expect(d.coords.length - d.constraints.length).toBe(1);
    expect(d.warnings).toEqual([]);
    const braced = doublePendulum();
    braced.links.push({ id: 2, a: 0, b: 2, mass: 1e-6 });
    const [s1, s2] = [run(welded, 2), run(braced, 2)];
    const [p1, p2] = [s1.jointPositions(), s2.jointPositions()];
    for (const id of [1, 2]) {
      expect(p1.get(id)![0]).toBeCloseTo(p2.get(id)![0], 4);
      expect(p1.get(id)![1]).toBeCloseTo(p2.get(id)![1], 4);
    }
    // the corner at joint 1 stays square
    expect(Math.abs(Math.cos(angleAt(p1, 1, 0, 2)))).toBeLessThan(1e-9);
  });

  it('drops a weld the mechanism already holds, with a warning', () => {
    const tri: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.5, y: -0.8, kind: 'free', mass: 0 },
      ],
      links: [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 1, b: 2, mass: 1 },
        { id: 2, a: 2, b: 0, mass: 1 },
      ],
      springs: [], gears: [], meshes: [], rackMeshes: [],
      angles: [{ id: 0, a: 0, b: 2 }],
    };
    const d = deriveEquations(tri, { gravity: G });
    expect(d.coords.length - d.constraints.length).toBe(1);
    expect(d.constraints.some((c) => c.fixedAngle)).toBe(false);
    expect(d.warnings.some((w) => w.includes('already holds that angle'))).toBe(true);
  });

  it('holds the angle at a joint across a loop (five-bar becomes one degree of freedom)', () => {
    const five: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.2, y: 0.6, kind: 'free', mass: 0 },
        { id: 2, x: 0.9, y: 0.9, kind: 'free', mass: 0 },
        { id: 3, x: 1.5, y: 0.5, kind: 'free', mass: 0 },
        { id: 4, x: 1.6, y: 0, kind: 'fixed', mass: 0 },
      ],
      links: [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 1, b: 2, mass: 1 },
        { id: 2, a: 2, b: 3, mass: 1 },
        { id: 3, a: 3, b: 4, mass: 1 },
      ],
      springs: [], gears: [], meshes: [], rackMeshes: [],
      angles: [{ id: 0, a: 1, b: 2 }],
    };
    const d = deriveEquations(five, { gravity: G });
    expect(d.coords.length - d.constraints.length).toBe(1);
    const sim = new Simulator(new NumericSystem(d));
    const before = angleAt(sim.jointPositions(), 2, 1, 3);
    const e0 = sim.energy();
    for (let i = 0; i < 2000; i++) sim.step(1e-3);
    const p = sim.jointPositions();
    expect(Math.hypot(p.get(1)![0] - 0.2, p.get(1)![1] - 0.6)).toBeGreaterThan(0.01); // it moved
    expect(angleAt(p, 2, 1, 3)).toBeCloseTo(before, 9);
    const e1 = sim.energy();
    expect(Math.abs(e1.T + e1.V - e0.T - e0.V)).toBeLessThan(1e-6);
  });
});

describe('joined bodies', () => {
  const sim = (d: Design) => new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
  const total = (s: Simulator) => {
    const e = s.energy();
    return e.T + e.V;
  };

  it('a gear joined to the bar on its axle turns with it, adding its inertia', () => {
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 0, radius: 0.5, teeth: 33, mass: 4, locked: false }],
      joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'gear', id: 0 } }],
    };
    const der = deriveEquations(d, { gravity: G });
    // placed as one body: a single coordinate (the bar's angle) and no constraints at all
    expect(der.coords.map((c) => c.sym)).toEqual(['th0']);
    expect(der.constraints).toHaveLength(0);
    const s = new Simulator(new NumericSystem(der));
    // released horizontal: θ̈ = −(m g L/2) / (m L²/3 + ½ M R²) at the start
    const I = 1 / 3 + 0.5 * 4 * 0.25;
    const h = 1e-3;
    s.step(h);
    const th = s.q[0];
    expect(th).toBeCloseTo(-0.5 * (G * 0.5 / I) * h * h, 9);
    expect(s.gearAngles().get(0)!).toBeCloseTo(th, 12); // the gear turns with the bar
  });

  it('a floating bar joined to a pendulum keeps its place on it', () => {
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.2, y: -0.3, kind: 'free', mass: 0 },
        { id: 3, x: 0.8, y: -0.5, kind: 'free', mass: 0 },
      ],
      links: [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 2, b: 3, mass: 2 },
      ],
      springs: [], gears: [], meshes: [], rackMeshes: [],
      joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 1 } }],
    };
    const s = sim(d);
    const e0 = total(s);
    const dist = (p: Map<number, [number, number]>, i: number, j: number) => Math.hypot(p.get(i)![0] - p.get(j)![0], p.get(i)![1] - p.get(j)![1]);
    const p0 = s.jointPositions();
    for (let i = 0; i < 2000; i++) s.step(1e-3);
    const p = s.jointPositions();
    expect(Math.hypot(p.get(1)![0] - 1, p.get(1)![1])).toBeGreaterThan(0.1); // it swung
    for (const [i, j] of [[0, 2], [0, 3], [1, 2], [1, 3]]) expect(dist(p, i, j)).toBeCloseTo(dist(p0, i, j), 9);
    expect(Math.abs(total(s) - e0)).toBeLessThan(1e-6);
  });

  it('a join inside an already rigid triangle changes nothing', () => {
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.5, y: -0.8, kind: 'free', mass: 0 },
      ],
      links: [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 1, b: 2, mass: 1 },
        { id: 2, a: 2, b: 0, mass: 1 },
      ],
      springs: [], gears: [], meshes: [], rackMeshes: [],
      joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'link', id: 1 } }],
    };
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(der.constraints.some((c) => c.joined !== undefined)).toBe(false);
  });

  it('joins a gear to a rack, so the pair slides and turns as one', () => {
    // a free gear and a rack lying apart, joined: one free rigid body under gravity
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'free', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.5, y: 0.4, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1, rack: 1 }],
      springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 2, radius: 0.2, teeth: 13, mass: 1, locked: false }],
      joins: [{ id: 0, a: { kind: 'gear', id: 0 }, b: { kind: 'link', id: 0 } }],
    };
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(3);
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 500; i++) s.step(1e-3);
    const p = s.jointPositions();
    // free fall: everything drops ½ g t², nothing turns
    for (const [id, y0] of [[0, 0], [1, 0], [2, 0.4]] as const) expect(p.get(id)![1]).toBeCloseTo(y0 - 0.5 * G * 0.25, 6);
  });
});

describe('tension-only springs', () => {
  it('pull nothing while slack, then act as springs once stretched', () => {
    // a 1 kg point hanging on a cable of rest length 0.5 m, starting 0.2 m below its anchor
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0, y: -0.2, kind: 'free', mass: 1 },
      ],
      links: [], gears: [], meshes: [], rackMeshes: [],
      springs: [{ id: 0, a: 0, b: 1, k: 400, rest: 0.5, tensionOnly: true }],
    };
    const der = deriveEquations(d, { gravity: G });
    const s = new Simulator(new NumericSystem(der));
    expect(s.energy().V).toBeCloseTo(1 * G * -0.2, 12); // slack: only gravity, no spring energy
    for (let i = 0; i < 100; i++) s.step(1e-3);
    // free fall for 0.1 s: y = −0.2 − ½ g t²
    expect(s.jointPositions().get(1)![1]).toBeCloseTo(-0.2 - 0.5 * G * 0.01, 9);
    // later it bounces on the cable and keeps its energy
    const e0 = s.energy();
    for (let i = 0; i < 3000; i++) s.step(1e-3);
    const e1 = s.energy();
    expect(Math.abs(e1.T + e1.V - e0.T - e0.V)).toBeLessThan(1e-3);
    expect(s.jointPositions().get(1)![1]).toBeGreaterThan(-0.2 - 0.5 - 2 * G / 400 - 0.1); // held by the cable
  });
});

describe('human leg preset', () => {
  it('swings with two degrees of freedom and the knee stops near full extension', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets.find((p) => p.name.startsWith('Human leg'))!.build();
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(2);
    expect(der.warnings).toEqual([]);
    const s = new Simulator(new NumericSystem(der));
    const dir = (p: Map<number, [number, number]>, a: number, b: number) => Math.atan2(p.get(b)![1] - p.get(a)![1], p.get(b)![0] - p.get(a)![0]);
    const pts = new Map(design.points.map((p) => [p.id, [p.x, p.y] as [number, number]]));
    const flexion = (p: Map<number, [number, number]>) => dir(p, 0, 1) - dir(p, 3, 5);
    // the drawn pose: 40° of knee flexion relative to the femur and tibia bars at full extension
    const offset = flexion(pts) - (40 * Math.PI) / 180;
    let least = Infinity;
    for (let i = 0; i < 3000; i++) {
      expect(s.step(1e-3)).toBe(true);
      least = Math.min(least, flexion(s.jointPositions()) - offset);
    }
    expect((least * 180) / Math.PI).toBeGreaterThan(-10); // at most a few degrees of hyperextension
    expect((least * 180) / Math.PI).toBeLessThan(10); // and it does straighten
    expect(s.constraintError()).toBeLessThan(1e-9);
  });
});

describe('human hand preset', () => {
  it('has 16 degrees of freedom, and its volar plates stop the finger joints bending backwards', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets.find((p) => p.name.startsWith('Human hand'))!.build();
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(16); // wrist + 5 digits × 3 joints
    expect(der.warnings).toEqual([]);
    const s = new Simulator(new NumericSystem(der));
    s.damping = 2; // the preset's tissue damping
    // each finger's PIP joint: the angle between its proximal and middle phalanges (flexion > 0)
    const bonesOf = (start: number) => {
      const out = [];
      let j = start;
      for (let k = 0; k < 3; k++) {
        const l = design.links.find((x) => x.a === j && x.mass > 0.002)!;
        out.push(l);
        j = l.b;
      }
      return out;
    };
    const fingers = design.links.slice(1, 5).map((l) => bonesOf(l.b));
    const dir = (p: Map<number, [number, number]>, l: { a: number; b: number }) => Math.atan2(p.get(l.b)![1] - p.get(l.a)![1], p.get(l.b)![0] - p.get(l.a)![0]);
    const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
    let least = Infinity;
    for (let i = 0; i < 1500; i++) {
      expect(s.step(1e-3)).toBe(true);
      const p = s.jointPositions();
      for (const f of fingers) least = Math.min(least, -wrap(dir(p, f[1]) - dir(p, f[0]))); // fingers curl clockwise
    }
    expect((least * 180) / Math.PI).toBeGreaterThan(-8); // PIP plates allow 0°; a few degrees of give
    expect(s.constraintError()).toBeLessThan(1e-9);
  }, 20000);
});

describe('ground contact', () => {
  const free = (points: Design['points'], extra: Partial<Design> = {}): Design => ({ points, links: [], springs: [], gears: [], meshes: [], rackMeshes: [], ...extra });
  // a straight ground through (0, y) at a slope (rad)
  const run = (d: Design, ground: { y: number; slope: number; restitution: number; friction: number }) => {
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = straightGroundSettings(ground.y, ground.slope, ground.restitution, ground.friction);
    return s;
  };

  it('bounces back to e² of the drop height, and E + W stays constant', () => {
    const s = run(free([{ id: 0, x: 0, y: 1, kind: 'free', mass: 1 }]), { y: 0, slope: 0, restitution: 0.5, friction: 0.5 });
    const E0 = s.energy().T + s.energy().V;
    let [bounced, peak, prev, worst] = [false, 0, 0, 0];
    for (let i = 0; i < 1500; i++) {
      s.step(1e-3);
      const v = s.qd[1];
      if (!bounced && prev < 0 && v > 0) bounced = true;
      if (bounced && prev > 0 && v <= 0 && !peak) peak = s.jointPosition(0)![1];
      prev = v;
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - E0));
    }
    expect(peak).toBeGreaterThan(0.23);
    expect(peak).toBeLessThan(0.26);
    expect(worst).toBeLessThan(5e-3);
  });

  it('rests at the same tiny depth g/ω² whatever the mass', () => {
    for (const mass of [0.005, 5]) {
      const s = run(free([{ id: 0, x: 0, y: 0.05, kind: 'free', mass }]), { y: 0, slope: 0, restitution: 0.2, friction: 0.5 });
      for (let i = 0; i < 2000; i++) expect(s.step(1e-3)).toBe(true);
      expect(-s.jointPosition(0)![1]).toBeCloseTo(G / 250 ** 2, 5);
    }
  });

  it('holds a bar still on a slope with static friction, and lets it slide at g(sin α − μ cos α)', () => {
    const a = (20 * Math.PI) / 180;
    const bar = () => free(
      [
        { id: 0, x: 0, y: 5e-4, kind: 'free', mass: 0 },
        { id: 1, x: 0.5 * Math.cos(a), y: 0.5 * Math.sin(a) + 5e-4, kind: 'free', mass: 0 },
      ],
      { links: [{ id: 0, a: 0, b: 1, mass: 1 }] },
    );
    const along = (s: Simulator) => s.jointPosition(0)![0] / Math.cos(a);
    const held = run(bar(), { y: 0, slope: a, restitution: 0, friction: 0.5 });
    for (let i = 0; i < 1500; i++) held.step(1e-3);
    expect(Math.abs(along(held))).toBeLessThan(2e-4);
    const sliding = run(bar(), { y: 0, slope: a, restitution: 0, friction: 0.2 });
    for (let i = 0; i < 300; i++) sliding.step(1e-3);
    const s1 = along(sliding);
    for (let i = 0; i < 1000; i++) sliding.step(1e-3);
    const accel = (2 * (along(sliding) - s1)) / (1.3 ** 2 - 0.3 ** 2);
    expect(accel).toBeCloseTo(-G * (Math.sin(a) - 0.2 * Math.cos(a)), 2);
  });

  it('rolls a gear down a slope at g sin α / (1 + I/mR²)', () => {
    const b = (15 * Math.PI) / 180;
    const R = 0.2;
    const d = free([{ id: 0, x: -R * Math.sin(b), y: R * Math.cos(b) + 2e-4, kind: 'free', mass: 0 }], { gears: [{ id: 0, center: 0, radius: R, teeth: 13, mass: 2, locked: false }] });
    const s = run(d, { y: 0, slope: b, restitution: 0, friction: 1 });
    const dist = () => Math.hypot(s.jointPosition(0)![0] - d.points[0].x, s.jointPosition(0)![1] - d.points[0].y);
    for (let i = 0; i < 200; i++) s.step(1e-3);
    const d1 = dist();
    for (let i = 0; i < 1000; i++) s.step(1e-3);
    expect((2 * (dist() - d1)) / (1.2 ** 2 - 0.2 ** 2)).toBeCloseTo((G * Math.sin(b)) / 1.5, 2);
  });

  it('the rimless wheel settles into a steady passive gait down its slope', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets.find((p) => p.name.startsWith('Rimless wheel'))!.build();
    const g = design.ground!;
    const s = new Simulator(new NumericSystem(deriveEquations(design, { gravity: G })));
    s.ground = { points: g.points, restitution: g.restitution, friction: g.friction };
    const hubX = () => s.jointPosition(0)![0];
    for (let i = 0; i < 3000; i++) expect(s.step(1e-3)).toBe(true);
    const x3 = hubX();
    for (let i = 0; i < 2000; i++) s.step(1e-3);
    const x5 = hubX();
    for (let i = 0; i < 2000; i++) s.step(1e-3);
    const x7 = hubX();
    expect(x5 - x3).toBeGreaterThan(1); // still walking downhill, about 1 m/s
    expect(Math.abs((x7 - x5) / (x5 - x3) - 1)).toBeLessThan(0.03); // at a steady pace
  }, 30000);
});

describe('motors', () => {
  const geared = (motor: { torque: number; noLoadRpm: number }, locked = false): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }],
    links: [], springs: [], meshes: [], rackMeshes: [],
    gears: [{ id: 0, center: 0, radius: 0.2, teeth: 13, mass: 2, locked, motor }],
  });
  const I = 0.5 * 2 * 0.2 * 0.2;

  it('a constant torque spins a free gear up at τ/I, and its work becomes kinetic energy', () => {
    const der = deriveEquations(geared({ torque: 0.5, noLoadRpm: 0 }), { gravity: G });
    expect(der.motors).toHaveLength(1);
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 1000; i++) s.step(1e-3);
    expect(s.q[0]).toBeCloseTo(0.5 * (0.5 / I) * 1, 9); // φ = ½ (τ/I) t²
    expect(s.motorWork).toBeCloseTo(s.energy().T, 9);
  });

  it('a motor with a no-load speed settles at that speed', () => {
    const s = new Simulator(new NumericSystem(deriveEquations(geared({ torque: 0.5, noLoadRpm: 120 }), { gravity: G })));
    for (let i = 0; i < 3000; i++) s.step(1e-3); // time constant I/b = 0.04 · 4π / 0.5 ≈ 1 s... let it settle
    for (let i = 0; i < 5000; i++) s.step(1e-3);
    expect((s.qd[0] * 60) / (2 * Math.PI)).toBeCloseTo(120, 1);
  });

  it('warns that a motor on a locked gear cannot turn it', () => {
    const der = deriveEquations(geared({ torque: 1, noLoadRpm: 0 }, true), { gravity: G });
    expect(der.motors).toHaveLength(0);
    expect(der.warnings.some((w) => w.includes('locked'))).toBe(true);
  });

  it('drives the Strandbeest crank round and round, with E + W − W_motor constant', async () => {
    const { presets } = await import('../ui/presets');
    const der = deriveEquations(presets.find((p) => p.name.startsWith('Strandbeest leg driven'))!.build(), { gravity: G });
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    for (let i = 0; i < 4000; i++) expect(s.step(1e-3)).toBe(true);
    expect(s.q[der.motors[0].terms[0].coord] / (2 * Math.PI)).toBeGreaterThan(3); // over three turns in 4 s
    const e = s.energy();
    expect(Math.abs(e.T + e.V + s.dissipated - s.motorWork - E0)).toBeLessThan(1e-6);
  }, 20000);
});

describe('pins and rails on bodies', () => {
  const mk = (points: Design['points'], links: Design['links'], extra: Partial<Design> = {}): Design => ({ points, links, springs: [], gears: [], meshes: [], rackMeshes: [], ...extra });

  it('slides a free mass down a 30° ground rail at g sin 30°', () => {
    const d = mk([{ id: 0, x: 0, y: 0, kind: 'free', mass: 1, slide: { on: null, angle: 30 } }], []);
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.map((c) => c.kind)).toEqual(['s']); // one coordinate along the rail
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 1000; i++) s.step(1e-3);
    const p = s.jointPosition(0)!;
    const along = p[0] * Math.cos(Math.PI / 6) + p[1] * Math.sin(Math.PI / 6);
    expect(along).toBeCloseTo(-0.5 * G * 0.5 * 1, 6); // ½ g sin30° t²
    expect(Math.abs(-p[0] * Math.sin(Math.PI / 6) + p[1] * Math.cos(Math.PI / 6))).toBeLessThan(1e-9); // on the rail
  });

  it('keeps a joint reached through a bar on an angled ground rail', () => {
    // a pendulum whose bob rides a 45° ground rail
    const d = mk(
      [
        { id: 0, x: 0, y: 1, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.6, y: 0.4, kind: 'free', mass: 1, slide: { on: null, angle: 45 } },
      ],
      [{ id: 0, a: 0, b: 1, mass: 0.5 }],
    );
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(0); // locked: one bar, one rail, pinned top
    expect(der.warnings.some((w) => w.includes('locked'))).toBe(true);
  });

  it('pins a joint onto the middle of a bar, where it then pivots', () => {
    // bar 0 hangs from O; bar 1 hangs from a pin P halfway along bar 0
    const d = mk(
      [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.5, y: 0, kind: 'free', mass: 0, pin: { kind: 'link', id: 0 } },
        { id: 3, x: 0.5, y: -0.6, kind: 'free', mass: 0.5 },
      ],
      [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 2, b: 3, mass: 0.3 },
      ],
    );
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(2);
    expect(der.warnings).toEqual([]);
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    for (let i = 0; i < 2000; i++) s.step(1e-3);
    const p = s.jointPositions();
    const [o, a, pin] = [p.get(0)!, p.get(1)!, p.get(2)!];
    expect(Math.hypot(pin[0] - (o[0] + a[0]) / 2, pin[1] - (o[1] + a[1]) / 2)).toBeLessThan(1e-9); // still halfway
    expect(Math.abs(s.energy().T + s.energy().V - E0)).toBeLessThan(1e-6);
  });

  it('slides a crank pin along a slotted lever that turns with it', () => {
    // crank O–P driven round by gravity on its mass; P slides on lever Q–R pivoted at Q
    const d = mk(
      [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 0.2, y: 0.1, kind: 'free', mass: 1, slide: { on: { kind: 'link', id: 1 }, angle: (Math.atan2(0.1 + 0.4, 0.2) * 180) / Math.PI } },
        { id: 2, x: 0, y: -0.4, kind: 'fixed', mass: 0 },
        { id: 3, x: 0.4, y: 0.6, kind: 'free', mass: 0 },
      ],
      [
        { id: 0, a: 0, b: 1, mass: 0.2 },
        { id: 1, a: 2, b: 3, mass: 0.5 },
      ],
    );
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    let moved = 0;
    for (let i = 0; i < 2000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const p = s.jointPositions();
      const [pin, q, r] = [p.get(1)!, p.get(2)!, p.get(3)!];
      // the pin stays on the lever's line
      const cross = ((r[0] - q[0]) * (pin[1] - q[1]) - (r[1] - q[1]) * (pin[0] - q[0])) / Math.hypot(r[0] - q[0], r[1] - q[1]);
      expect(Math.abs(cross)).toBeLessThan(1e-8);
      moved = Math.max(moved, Math.hypot(pin[0] - 0.2, pin[1] - 0.1));
    }
    expect(moved).toBeGreaterThan(0.05);
    expect(Math.abs(s.energy().T + s.energy().V - E0)).toBeLessThan(1e-6);
  });

  it('drops a pin the mechanism already holds, with a warning', () => {
    const d = mk(
      [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0, pin: { kind: 'link', id: 0 } }, // its own bar's end
      ],
      [{ id: 0, a: 0, b: 1, mass: 1 }],
    );
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(der.warnings.some((w) => w.includes('already moves'))).toBe(true);
  });
});

describe('slider ranges', () => {
  it('a slider bounces off its end stop to e² of the drop along the rail', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'free', mass: 1, slide: { on: null, angle: 30, range: { min: -0.3, max: 0.5, restitution: 0.5 } } }],
      links: [], springs: [], gears: [], meshes: [], rackMeshes: [],
    };
    const der = deriveEquations(d, { gravity: G });
    expect(der.sliderStops).toHaveLength(1);
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    let [bounced, peak, prev, worst] = [false, NaN, 0, 0];
    for (let i = 0; i < 2000; i++) {
      s.step(1e-3);
      const v = s.qd[0];
      if (!bounced && prev < 0 && v > 0) bounced = true;
      if (bounced && prev > 0 && v <= 0 && Number.isNaN(peak)) peak = s.q[0];
      prev = v;
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - E0));
    }
    expect(peak + 0.3).toBeGreaterThan(0.068); // e² · 0.3 = 0.075, a little less for gravity during contact
    expect(peak + 0.3).toBeLessThan(0.076);
    expect(s.q[0] + 0.3).toBeCloseTo(-(G * 0.5) / 250 ** 2, 5); // resting g sin 30° / ω² into the stop
    expect(worst).toBeLessThan(1e-3);
  });

  it('stops a bead at the end of a rail on a swinging rod', async () => {
    const { presets } = await import('../ui/presets');
    const design = presets.find((p) => p.name.startsWith('Bead on a swinging rod'))!.build();
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(2);
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    let furthest = 0;
    for (let i = 0; i < 4000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const p = s.jointPositions();
      const [o, a, b] = [p.get(0)!, p.get(1)!, p.get(2)!];
      furthest = Math.max(furthest, ((b[0] - o[0]) * (a[0] - o[0]) + (b[1] - o[1]) * (a[1] - o[1])) / Math.hypot(a[0] - o[0], a[1] - o[1]));
    }
    const stop = Math.hypot(design.points[2].x - design.points[0].x, design.points[2].y - design.points[0].y) + design.points[2].slide!.range!.max!;
    expect(furthest).toBeGreaterThan(stop - 0.01); // it reached the end stop
    expect(furthest).toBeLessThan(stop + 0.03); // and did not go far past it
    const e = s.energy();
    expect(Math.abs(e.T + e.V + s.dissipated - E0)).toBeLessThan(5e-3);
  }, 20000);
});

describe('four-legged Strandbeest preset', () => {
  it('keeps all four cranks in step through the coupling rods as the motor turns them', async () => {
    const { presets } = await import('../ui/presets');
    const der = deriveEquations(presets.find((p) => p.name.startsWith('Strandbeest with four'))!.build(), { gravity: G });
    expect(der.coords.length - der.constraints.length).toBe(1);
    expect(der.warnings).toEqual([]);
    const s = new Simulator(new NumericSystem(der));
    const E0 = s.energy().T + s.energy().V;
    let drift = 0;
    for (let i = 0; i < 5000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const a = [...s.gearAngles().values()];
      for (const x of a) drift = Math.max(drift, Math.abs(x - a[0]));
    }
    expect([...s.gearAngles().values()][3] / (2 * Math.PI)).toBeGreaterThan(2); // through several dead centres
    expect(drift).toBeLessThan(1e-6); // the parallelograms keep every gear at the same angle
    const e = s.energy();
    expect(Math.abs(e.T + e.V + s.dissipated - s.motorWork - E0)).toBeLessThan(1e-2);
  }, 30000);
});

describe('terrain ground', () => {
  it('keeps the ground under the machine: a waypoint stops below the parts, and gives way under them', async () => {
    const { groundCeiling, groundHeight, lowerGroundUnder, straightGround } = await import('./model');
    const g = straightGround(0, 0, -2, 2, 5, 0.3, 0.8); // waypoints at x = −2, −1, 0, 1, 2
    const lows: [number, number][] = [[0.5, 0.4]]; // a joint at (0.5, 0.4)
    // waypoint 2 (x = 0) carries half the ground at x = 0.5: it may rise to 2 × (0.4 − 0.002)
    expect(groundCeiling(g, 2, lows, 0.002)).toBeCloseTo(0.796, 9);
    // move the joint below the ground: the waypoints either side drop until it is above again
    expect(lowerGroundUnder(g, [[0.5, -0.3]], 0.002)).toBe(true);
    expect(groundHeight(g, 0.5)).toBeCloseTo(-0.302, 9);
    expect(g.points[0][1]).toBe(0); // the rest of the ground stays put
  });

  it('a mass slides down into a V-shaped valley and settles at the bottom', () => {
    const s = new Simulator(new NumericSystem(deriveEquations({ points: [{ id: 0, x: -0.6, y: 0.31, kind: 'free', mass: 1 }], links: [], springs: [], gears: [], meshes: [], rackMeshes: [] }, { gravity: G })));
    s.ground = { points: [[-2, 1], [0, 0], [2, 1]], restitution: 0, friction: 0.05 };
    s.damping = 0.5;
    for (let i = 0; i < 8000; i++) expect(s.step(1e-3)).toBe(true);
    const p = s.jointPosition(0)!;
    expect(Math.abs(p[0])).toBeLessThan(0.05); // at the bottom of the valley
    expect(p[1]).toBeGreaterThan(-0.01);
  }, 20000);

  it("a gear rolls on, past a change of slope, at each slope's own acceleration", () => {
    const R = 0.2;
    const b = (15 * Math.PI) / 180;
    // down 15° to x = 1, then level
    const pts: [number, number][] = [[-3, 3 * Math.tan(b)], [1, -1 * Math.tan(b)], [5, -1 * Math.tan(b)]];
    const d: Design = { points: [{ id: 0, x: -R * Math.sin(b), y: R * Math.cos(b) + 2e-4, kind: 'free', mass: 0 }], links: [], springs: [], meshes: [], rackMeshes: [], gears: [{ id: 0, center: 0, radius: R, teeth: 13, mass: 2, locked: false }] };
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = { points: pts, restitution: 0, friction: 1 };
    const x = () => s.jointPosition(0)![0];
    for (let i = 0; i < 200; i++) s.step(1e-3);
    const [x1, t1] = [x(), 0.2];
    for (let i = 0; i < 400; i++) s.step(1e-3);
    const [x2, t2] = [x(), 0.6];
    expect((2 * (x2 - x1)) / (t2 * t2 - t1 * t1) / Math.cos(b)).toBeCloseTo((G * Math.sin(b)) / 1.5, 1); // on the slope
    for (let i = 0; i < 2400; i++) expect(s.step(1e-3)).toBe(true);
    expect(x()).toBeGreaterThan(1.3); // carried on over the level part
    expect(s.jointPosition(0)![1]).toBeCloseTo(-Math.tan(b) + R, 2); // rolling on the level ground
  }, 20000);
});

describe('wheels', () => {
  const wheelAt = (x: number, y: number, R: number, extra: Partial<Design['gears'][number]> = {}): Design => ({
    points: [{ id: 0, x, y, kind: 'free', mass: 0 }],
    links: [], springs: [], meshes: [], rackMeshes: [],
    gears: [{ id: 0, center: 0, radius: R, teeth: 13, mass: 2, locked: false, wheel: true, ...extra }],
  });
  // the true distance from a point to the terrain polyline
  const distance = (pts: [number, number][], [x, y]: [number, number]) => {
    let best = Infinity;
    for (let i = 0; i + 1 < pts.length; i++) {
      const [[x0, y0], [x1, y1]] = [pts[i], pts[i + 1]];
      const [dx, dy] = [x1 - x0, y1 - y0];
      const u = Math.min(1, Math.max(0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)));
      best = Math.min(best, Math.hypot(x - x0 - u * dx, y - y0 - u * dy));
    }
    return best;
  };

  it('cannot mesh: a gear mesh or rack mesh with a wheel is ignored, with a warning', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 0.5, y: 0, kind: 'fixed', mass: 0 }],
      links: [], springs: [], rackMeshes: [],
      gears: [{ id: 0, center: 0, radius: 0.2, teeth: 13, mass: 1, locked: false, wheel: true }, { id: 1, center: 1, radius: 0.3, teeth: 20, mass: 1, locked: false }],
      meshes: [{ id: 0, a: 0, b: 1 }],
    };
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.length).toBe(2); // both turn freely
    expect(der.warnings.some((w) => w.includes('wheel has no teeth'))).toBe(true);
  });

  it('rolls over a crest without sinking into the peak, and E + W stays constant', () => {
    const R = 0.25;
    const b = (12 * Math.PI) / 180;
    // up 12° to a peak at x = 0, then down 12°
    const pts: [number, number][] = [[-4, -4 * Math.tan(b)], [0, 0], [4, -4 * Math.tan(b)]];
    const x0 = -0.5;
    const d = wheelAt(x0 - R * Math.sin(b), x0 * Math.tan(b) + R * Math.cos(b) + 1e-4, R);
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = { points: pts, restitution: 0, friction: 1 };
    // rolling uphill at 1.6 m/s: slow enough to stay on the ground as it pivots over the peak
    s.qd[0] = 1.6 * Math.cos(b);
    s.qd[1] = 1.6 * Math.sin(b);
    s.qd[2] = -1.6 / R;
    const E0 = s.energy().T + s.energy().V;
    let [closest, worst, onPeak] = [Infinity, 0, 0];
    for (let i = 0; i < 1500; i++) {
      expect(s.step(1e-3)).toBe(true);
      const c = s.jointPosition(0)!;
      if (Math.abs(c[0]) < 0.3) closest = Math.min(closest, distance(pts, c));
      if (Math.abs(c[0]) < 0.04 && s.contactCount() > 0) onPeak++;
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - E0));
    }
    expect(s.jointPosition(0)![0]).toBeGreaterThan(0.5); // over the top and rolling down the far side
    expect(onPeak).toBeGreaterThan(10); // it touched the peak as it went over
    expect(closest).toBeGreaterThan(R - 2e-3); // the rim rode over the waypoint, never sinking into it
    expect(worst).toBeLessThan(0.02 * E0);
  }, 20000);

  it('settles at the bottom of a V, resting on both sides at once', () => {
    const R = 0.2;
    const b = (30 * Math.PI) / 180;
    const pts: [number, number][] = [[-2, 2 * Math.tan(b)], [0, 0], [2, 2 * Math.tan(b)]];
    const s = new Simulator(new NumericSystem(deriveEquations(wheelAt(-0.4, 0.4 * Math.tan(b) + R / Math.cos(b) + 0.01, R), { gravity: G })));
    s.ground = { points: pts, restitution: 0, friction: 0.8 };
    s.damping = 1;
    for (let i = 0; i < 6000; i++) expect(s.step(1e-3)).toBe(true);
    const c = s.jointPosition(0)!;
    expect(Math.abs(c[0])).toBeLessThan(2e-3);
    expect(c[1]).toBeCloseTo(R / Math.cos(b), 3); // touching both sides, not sunk into one
    expect(s.contactCount()).toBe(2);
  }, 20000);

  it('the motor car example drives over all its hills, and E + W − W_m stays constant', () => {
    const d = motorCar();
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = { ...d.ground! };
    const E0 = s.energy().T + s.energy().V;
    let worst = 0;
    for (let i = 0; i < 8000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - s.motorWork - E0));
    }
    expect(s.jointPosition(0)![0]).toBeGreaterThan(14); // past the last hill
    expect(worst).toBeLessThan(0.002 * s.motorWork);
  }, 30000);

  it('drives itself along with a motor, its housing joined to the chassis', () => {
    // two wheels on a chassis bar, the rear one driven, the motor housing joined to the chassis
    const R = 0.15;
    const d: Design = {
      points: [{ id: 0, x: 0, y: R + 1e-4, kind: 'free', mass: 0 }, { id: 1, x: 0.6, y: R + 1e-4, kind: 'free', mass: 0 }],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }], springs: [], meshes: [], rackMeshes: [],
      gears: [
        { id: 0, center: 0, radius: R, teeth: 10, mass: 0.5, locked: false, wheel: true, motor: { torque: -0.3, noLoadRpm: 0, mass: 0.3, mounted: false } },
        { id: 1, center: 1, radius: R, teeth: 10, mass: 0.5, locked: false, wheel: true },
      ],
      joins: [{ id: 0, a: { kind: 'motor', id: 0 }, b: { kind: 'link', id: 0 } }],
    };
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = straightGroundSettings(0, 0, 0, 1);
    for (let i = 0; i < 1000; i++) expect(s.step(1e-3)).toBe(true);
    const [a, c] = [s.jointPosition(0)!, s.jointPosition(1)!];
    expect(a[0]).toBeGreaterThan(0.1); // a clockwise torque drives the car to the right
    expect(Math.abs(c[1] - a[1])).toBeLessThan(0.05); // the reaction tips the chassis back only a little
  }, 20000);
});

describe('terrain peaks against bars', () => {
  it('holds up a bar lying across a peak, though both its ends are in the air', () => {
    // a bar 1 m long dropped flat onto a sharp peak at x = 0
    const d: Design = {
      points: [
        { id: 0, x: -0.5, y: 0.3, kind: 'free', mass: 0 },
        { id: 1, x: 0.5, y: 0.3, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }], springs: [], gears: [], meshes: [], rackMeshes: [],
    };
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    s.ground = { points: [[-3, -1], [0, 0.2], [3, -1]], restitution: 0.1, friction: 0.8 };
    const E0 = s.energy().T + s.energy().V;
    let worst = 0;
    for (let i = 0; i < 400; i++) {
      expect(s.step(1e-3)).toBe(true);
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - E0));
    }
    const p = s.jointPositions();
    const midY = (p.get(0)![1] + p.get(1)![1]) / 2;
    expect(midY).toBeGreaterThan(0.19); // resting on the peak at 0.2, not fallen through it
    expect(worst).toBeLessThan(5e-3);
  });
});

describe('motor speed falloff', () => {
  it('a clockwise motor (τ < 0) settles at its no-load speed turning clockwise, ω = −Ω', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }],
      links: [], springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 0, radius: 0.2, teeth: 13, mass: 1, locked: false, motor: { torque: -0.5, noLoadRpm: 60 } }],
    };
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    for (let i = 0; i < 3000; i++) expect(s.step(1e-3)).toBe(true);
    expect(s.qd[0]).toBeCloseTo(-2 * Math.PI, 3); // 60 rpm clockwise, not running away
  });
});

describe('propellers', () => {
  // a propeller drawn with its blade line horizontal (axis straight up), centre at the origin
  const prop = (extra: Partial<Design['gears'][number]> = {}, kind: 'free' | 'fixed' = 'free', aero: Record<string, number> = {}): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind, mass: 0 }],
    links: [], springs: [], meshes: [], rackMeshes: [],
    gears: [{ id: 0, center: 0, radius: 0.3, teeth: 10, mass: 0.2, locked: false, propeller: { angle: 0, blades: 2, ...aero }, ...extra }],
  });

  it('is a free body with four coordinates, x, y, θ and φ, and the Euler-angle kinetic energy', () => {
    const d = prop();
    const der = deriveEquations(d, { gravity: 0 });
    expect(der.coords.map((c) => c.sym)).toEqual(['x0', 'y0', 'ph0', 'sp0']);
    const s = new Simulator(new NumericSystem(der));
    const { I1, I2, I3 } = propellerInertia(d.gears[0]);
    s.q[3] = 0.7; // φ
    s.qd.set([0.3, -0.2, 1.5, 40]);
    const [phi, th, sp] = [0.7, 1.5, 40];
    const expected = 0.5 * 0.2 * (0.09 + 0.04) + 0.5 * (I1 * sp * sp + I2 * th * th * Math.sin(phi) ** 2 + I3 * th * th * Math.cos(phi) ** 2);
    expect(s.energy().T).toBeCloseTo(expected, 10);
  });

  it('in still air, conserves energy and the yaw momentum (I₂ sin²φ + I₃ cos²φ) θ̇ as the blades spin', () => {
    const d = prop({}, 'free', { rho: 0 });
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: 0 })));
    const { I2, I3 } = propellerInertia(d.gears[0]);
    s.qd.set([0, 0, 0.05, 30]);
    const yaw = () => (I2 * Math.sin(s.q[3]) ** 2 + I3 * Math.cos(s.q[3]) ** 2) * s.qd[2];
    const [E0, L0] = [s.energy().T, yaw()];
    let wobble = 0;
    for (let i = 0; i < 2000; i++) {
      expect(s.step(1e-3)).toBe(true);
      wobble = Math.max(wobble, Math.abs(s.qd[2] - 0.05));
    }
    expect(Math.abs(s.energy().T - E0)).toBeLessThan(1e-6 * E0);
    expect(Math.abs(yaw() - L0)).toBeLessThan(1e-3 * L0); // to RK4's accuracy through the bursts
    expect(wobble).toBeGreaterThan(0.5); // a two-bladed rotor yaws in bursts, when its blades stand out of the plane
  });

  it('on a fixed mount, spins up until motor and air torque balance, with thrust ρD⁴C_T n²', () => {
    const d = prop({ motor: { torque: 0.5, noLoadRpm: 3000, mass: 0.1, mounted: true } }, 'fixed');
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['sp0']); // mounted: the hub cannot move or turn
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 12000; i++) expect(s.step(1e-3)).toBe(true);
    const w = s.qd[0];
    const [rho, D, Om] = [1.225, 0.6, (3000 * 2 * Math.PI) / 60];
    const n = w / (2 * Math.PI);
    expect(0.5 * (1 - w / Om)).toBeCloseTo(rho * D ** 5 * 0.007 * n * n, 4); // motor torque = air torque
    expect(s.thrust.get(0)!).toBeCloseTo(rho * D ** 4 * 0.1 * n * n, 4);
  }, 20000);

  it('flies: a motor-driven propeller climbs, and E − W_m − W_air stays constant', () => {
    const d = prop({ motor: { torque: 1, noLoadRpm: 3000, mass: 0.1, mounted: false } });
    const s = new Simulator(new NumericSystem(deriveEquations(d, { gravity: G })));
    const E0 = s.energy().T + s.energy().V;
    let worst = 0;
    for (let i = 0; i < 3000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V - s.motorWork - s.airWork - E0));
    }
    expect(s.jointPosition(0)![1]).toBeGreaterThan(0.5); // lifted off and climbing
    expect(Math.abs(s.jointPosition(0)![0])).toBeLessThan(1e-9); // straight up its axis
    expect(worst).toBeLessThan(1e-3 * s.motorWork);
  }, 20000);

  it('the twin-rotor drone example lifts off and climbs, with E + W − W_m − W_a constant', () => {
    const d = twinRotor();
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['x0', 'y0', 'th0', 'sp0', 'sp1']); // one rigid frame and two spins
    const s = new Simulator(new NumericSystem(der));
    s.ground = { ...d.ground! };
    const E0 = s.energy().T + s.energy().V;
    let worst = 0;
    for (let i = 0; i < 4000; i++) {
      expect(s.step(1e-3)).toBe(true);
      const e = s.energy();
      worst = Math.max(worst, Math.abs(e.T + e.V + s.dissipated - s.motorWork - s.airWork - E0));
    }
    expect(s.jointPosition(2)![1]).toBeGreaterThan(0.5);
    expect(worst).toBeLessThan(1e-4 * s.motorWork);
  }, 20000);

  it('joined to a pendulum arm through its motor, adds only its spin, and its thrust lifts the arm', () => {
    const d: Design = {
      points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }, { id: 1, x: 1, y: 0, kind: 'free', mass: 0 }],
      links: [{ id: 0, a: 0, b: 1, mass: 0.2 }], springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 1, radius: 0.15, teeth: 10, mass: 0.05, locked: false, propeller: { angle: 0, blades: 3 }, motor: { torque: 0.3, noLoadRpm: 6000, mass: 0.05, mounted: false } }],
      joins: [{ id: 0, a: { kind: 'motor', id: 0 }, b: { kind: 'link', id: 0 } }],
    };
    const der = deriveEquations(d, { gravity: G });
    expect(der.coords.map((c) => c.sym)).toEqual(['th0', 'sp0']);
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 1500; i++) expect(s.step(1e-3)).toBe(true);
    expect(s.q[0]).toBeGreaterThan(1); // swung up past the horizontal by the thrust, its axis turning with the arm
  }, 20000);
});

describe('motor levers', () => {
  const levered = (lever: { scale: number; position: number }, noLoadRpm: number): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }],
    links: [], springs: [], meshes: [], rackMeshes: [],
    gears: [{ id: 0, center: 0, radius: 0.2, teeth: 13, mass: 1, locked: false, motor: { torque: 5, noLoadRpm, lever: { color: '#e04f5f', ...lever } } }],
  });

  it('sets the torque as position × scale, ignoring the motor\'s own torque', () => {
    const s = new Simulator(new NumericSystem(deriveEquations(levered({ scale: 2, position: -0.25 }, 0), { gravity: G })));
    const I = 0.5 * 1 * 0.2 * 0.2;
    s.step(1e-3);
    expect(s.qd[0] / 1e-3).toBeCloseTo(-0.5 / I, 6);
  });

  it('keeps b = scale / Ω as it moves, so the free speed is position × Ω', () => {
    const s = new Simulator(new NumericSystem(deriveEquations(levered({ scale: 1, position: 0.5 }, 60), { gravity: G })));
    for (let i = 0; i < 3000; i++) s.step(1e-3);
    expect(s.qd[0]).toBeCloseTo(0.5 * 2 * Math.PI, 3); // half of 60 rpm
    // moved while running: only τ changes, and the gear settles at the new free speed
    expect(s.setMotorTorque(0, -0.75 * 1)).toBe(true);
    for (let i = 0; i < 3000; i++) s.step(1e-3);
    expect(s.qd[0]).toBeCloseTo(-0.75 * 2 * Math.PI, 3);
  });

  it('an undo snapshot does not share the motor or its lever', () => {
    const d = levered({ scale: 1, position: 0.5 }, 60);
    const c = cloneDesign(d);
    c.gears[0].motor!.lever!.position = -1;
    c.gears[0].motor!.torque = 9;
    expect(d.gears[0].motor!.lever!.position).toBe(0.5);
    expect(d.gears[0].motor!.torque).toBe(5);
  });
});

describe('motor housings', () => {
  const geared = (motor: Record<string, unknown>, extra: Partial<Design> = {}): Design => ({
    points: [{ id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 }],
    links: [], springs: [], meshes: [], rackMeshes: [],
    gears: [{ id: 0, center: 0, radius: 0.2, teeth: 13, mass: 2, locked: false, motor: { torque: 0.5, noLoadRpm: 0, ...motor } }],
    ...extra,
  });

  it('a free housing spins back, so the gear and the housing share no net angular momentum', () => {
    const der = deriveEquations(geared({ mounted: false, mass: 1 }), { gravity: G });
    expect(der.coords.map((c) => c.sym).sort()).toEqual(['ms0', 'ph0']);
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 1000; i++) s.step(1e-3);
    const Ig = 0.5 * 2 * 0.2 ** 2;
    const Is = 0.5 * 1 * (0.6 * 0.2) ** 2;
    const phi = s.gearAngles().get(0)!;
    const psi = s.motorAngles().get(0)!;
    expect(phi).toBeCloseTo(0.5 * (0.5 / Ig), 9); // ½ (τ/I_g) t² with t = 1
    expect(psi).toBeCloseTo(-0.5 * (0.5 / Is), 9); // and the housing the other way, −½ (τ/I_s) t²
    // all the motor's work went into the two spins
    expect(s.motorWork).toBeCloseTo(s.energy().T, 9);
  });

  it('a housing joined to a swinging arm pushes the arm back, keeping the total angular momentum at zero', () => {
    // no gravity: an arm pivoted at O carries the gear's axle at its end, with the motor's housing joined to the arm
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
      ],
      links: [{ id: 0, a: 0, b: 1, mass: 1 }],
      springs: [], meshes: [], rackMeshes: [],
      gears: [{ id: 0, center: 1, radius: 0.2, teeth: 13, mass: 2, locked: false, motor: { torque: 0.5, noLoadRpm: 0, mass: 1 } }],
      joins: [{ id: 0, a: { kind: 'link', id: 0 }, b: { kind: 'motor', id: 0 } }],
    };
    const der = deriveEquations(d, { gravity: 0 });
    expect(der.coords.length - der.constraints.length).toBe(2); // the arm (with the housing) and the gear
    const s = new Simulator(new NumericSystem(der));
    for (let i = 0; i < 1000; i++) expect(s.step(1e-3)).toBe(true);
    const th = s.q[der.coords.findIndex((c) => c.sym === 'th0')];
    const thd = s.qd[der.coords.findIndex((c) => c.sym === 'th0')];
    const phid = s.qd[der.coords.findIndex((c) => c.sym === 'ph0')];
    expect(th).toBeLessThan(0); // the arm turns back against the gear
    // angular momentum about O: arm (m L²/3) + housing and gear masses at the end (m L²) + housing spin (I_s θ̇) + gear spin (I_g φ̇)
    const Ig = 0.5 * 2 * 0.2 ** 2;
    const Is = 0.5 * 1 * (0.6 * 0.2) ** 2;
    const L = (1 / 3 + 1 + 2) * thd + Is * thd + Ig * phid;
    expect(Math.abs(L)).toBeLessThan(1e-9);
    expect(Math.abs(s.energy().T - s.motorWork)).toBeLessThan(1e-9);
  });
});
