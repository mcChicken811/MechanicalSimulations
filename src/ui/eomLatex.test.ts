import katex from 'katex';
import { describe, expect, it } from 'vitest';
import { deriveEquations } from '../mechanics/derive';
import { NumericSystem, Simulator } from '../mechanics/simulate';
import { constraintLatex, coordinatesLatex, energyLatex, equationLatex, generalFormLatex, potentialLatex, springLatex } from './eomLatex';
import { presets } from './presets';

describe('presets', () => {
  for (const preset of presets) {
    it(`${preset.name}: derives, renders valid LaTeX and simulates stably`, () => {
      const d = deriveEquations(preset.build(), { gravity: 9.81 });
      const all = [
        coordinatesLatex(d),
        generalFormLatex(d),
        energyLatex('T', d.T),
        potentialLatex(d),
        ...d.springs.map((_, j) => springLatex(d, j)),
        ...d.coords.map((_, i) => equationLatex(d, i)),
        ...d.constraints.map((_, k) => constraintLatex(d, k)),
      ];
      for (const tex of all) expect(() => katex.renderToString(tex, { throwOnError: true, displayMode: true })).not.toThrow();

      const sim = new Simulator(new NumericSystem(d));
      const e0 = sim.energy();
      for (let i = 0; i < 2000; i++) expect(sim.step(1e-3)).toBe(true);
      const e1 = sim.energy();
      const scale = Math.abs(e0.T) + Math.abs(e0.V) + 1;
      // motors add work and losses remove it, so the budget E + W − W_motor is what stays put
      expect(Math.abs(e1.T + e1.V + sim.dissipated - sim.motorWork - e0.T - e0.V) / scale).toBeLessThan(1e-2);
      expect(sim.constraintError()).toBeLessThan(1e-8);
    });
  }
});
