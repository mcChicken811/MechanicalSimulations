import { describe, expect, it } from 'vitest';
import { deriveEquations } from './derive';
import type { Design } from './model';
import { NumericSystem, Simulator } from './simulate';

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
    meshes: [],
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
    meshes: [],
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
    meshes: [],
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
        meshes: [],
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
      meshes: [],
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
      meshes: [],
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
      meshes: [],
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
      meshes: [],
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
    const design: Design = { points: [P(0, 0, 0, 'fixed')], links: [], springs: [], gears: [gear(0, 0, 0.3, 20, 2)], meshes: [] };
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
      meshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }],
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
      meshes: [{ id: 0, a: 0, b: 1 }],
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
    meshes: [],
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
      meshes: [{ id: 0, a: 0, b: 1 }],
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
      meshes: [],
    };
    const s2 = new Simulator(new NumericSystem(deriveEquations(cart, { gravity: G })));
    s2.friction = { ...s2.friction, enabled: true, pin: 0, rail: 1 };
    for (let i = 0; i < 1000; i++) s2.step(1e-3);
    expect(s2.heat().joints.get(0)!).toBeGreaterThan(0);
  });
});

describe('masses placed on their own', () => {
  it('a lone mass is a particle in free fall', () => {
    const design: Design = { points: [{ id: 0, x: 0.3, y: 1, kind: 'free', mass: 1 }], links: [], springs: [], gears: [], meshes: [] };
    const der = deriveEquations(design, { gravity: G });
    expect(der.coords.map((c) => c.kind)).toEqual(['x', 'y']);
    const sim = new Simulator(new NumericSystem(der));
    const a = new Float64Array(2);
    sim.accelerations(sim.q, sim.qd, a);
    expect(a[0]).toBeCloseTo(0, 12);
    expect(a[1]).toBeCloseTo(-G, 12);
  });
});
