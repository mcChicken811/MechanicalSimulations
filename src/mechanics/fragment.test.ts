import { describe, expect, it } from 'vitest';
import { presets } from '../ui/presets';
import { deriveEquations } from './derive';
import { allParts, emptyParts, extractParts, fragmentCentre, partsInBox, pasteParts } from './fragment';
import type { Design } from './model';

const build = (name: string) => presets.find((p) => p.name.startsWith(name))!.build();
const dof = (d: Design) => {
  const der = deriveEquations(d, { gravity: 9.81 });
  return { dof: der.coords.length - der.constraints.length, constraints: der.constraints.length, warnings: der.warnings };
};

describe('copy and paste', () => {
  it("a pasted PID controller measures the pasted copy, or loses its sensor if that was left behind", () => {
    const d = build('Robot-arm joint');
    const pasted = pasteParts(d, extractParts(d, allParts(d)), 3, 0);
    const copy = d.gears.find((g) => pasted.gears.has(g.id))!;
    const sensor = copy.motor!.pid!.sensor!;
    expect(pasted.links.has(sensor.body.id)).toBe(true);
    expect(copy.motor!.pid!.color).not.toBe(d.gears[0].motor!.pid!.color);
    // the gear and motor alone: the arm it measured stays behind
    const alone = pasteParts(d, extractParts(d, { ...emptyParts(), gears: new Set([0]) }), 0, 3);
    expect(d.gears.find((g) => alone.gears.has(g.id))!.motor!.pid!.sensor).toBeNull();
  });

  it('a pasted slider keeps its end stops', () => {
    const d: Design = { points: [{ id: 0, x: 0, y: 0, kind: 'free', mass: 1, slide: { on: null, angle: 90, range: { min: -0.1, max: 0.2, restitution: 0.3 } } }], links: [], springs: [], gears: [], meshes: [], rackMeshes: [] };
    pasteParts(d, extractParts(d, allParts(d)), 1, 0);
    expect(d.points[1].slide?.range).toEqual({ min: -0.1, max: 0.2, restitution: 0.3 });
    expect(d.points[1].slide?.range).not.toBe(d.points[0].slide?.range);
  });

  it('a pasted motor and lever are copies of their own, the lever in a new colour', () => {
    const d = build('Motor car');
    d.gears[0].motor!.lever = { color: '#e04f5f', scale: 2, position: 0.5 };
    const pasted = pasteParts(d, extractParts(d, allParts(d)), 3, 0);
    const copy = d.gears.find((g) => pasted.gears.has(g.id) && g.motor)!;
    expect(copy.motor).not.toBe(d.gears[0].motor);
    expect(copy.motor!.lever).toEqual({ color: copy.motor!.lever!.color, scale: 2, position: 0.5 });
    expect(copy.motor!.lever!.color).not.toBe('#e04f5f');
    copy.motor!.lever!.position = -1;
    expect(d.gears[0].motor!.lever!.position).toBe(0.5); // the original is untouched
  });

  it('a pasted copy of a whole mechanism keeps all its constraints', () => {
    // gears, gear pins and a mesh; add a fixed angle and a join so every kind is covered
    const d = build("Watt's sun");
    d.angles = [{ id: 0, a: 1, b: 3 }];
    d.joins = [{ id: 0, a: { kind: 'gear', id: 1 }, b: { kind: 'link', id: 2 } }];
    const before = dof(d);
    const frag = extractParts(d, allParts(d));
    const pasted = pasteParts(d, frag, 3, 0);
    expect(pasted.points.size).toBe(frag.points.length);
    // ids are fresh, and every reference points into the copy
    expect(new Set(d.points.map((p) => p.id)).size).toBe(d.points.length);
    expect(new Set(d.links.map((l) => l.id)).size).toBe(d.links.length);
    for (const l of d.links.filter((x) => pasted.links.has(x.id))) expect(pasted.points.has(l.a) && pasted.points.has(l.b)).toBe(true);
    for (const p of d.points.filter((x) => pasted.points.has(x.id) && x.gear !== undefined)) expect(pasted.gears.has(p.gear!)).toBe(true);
    expect(d.meshes).toHaveLength(2);
    expect(d.angles).toHaveLength(2);
    expect(d.joins).toHaveLength(2);
    // two independent copies: twice the constraints and degrees of freedom
    const after = dof(d);
    expect(after.constraints).toBe(2 * before.constraints);
    expect(after.dof).toBe(2 * before.dof);
  });

  it('leaves behind constraints to parts that were not copied', () => {
    const d = build("Watt's sun");
    // the planet gear's pin joint (4) without the planet gear itself, plus the rod to it
    const s = emptyParts();
    s.links.add(1).add(2); // rod and rod strut
    const frag = extractParts(d, s);
    expect(frag.gears).toHaveLength(0);
    expect(frag.meshes).toHaveLength(0);
    expect(frag.points.every((p) => p.gear === undefined)).toBe(true);
  });

  it('selects what lies inside a box, and pastes centred where asked', () => {
    const d = build('Double pendulum');
    const xs = d.points.map((p) => p.x);
    const ys = d.points.map((p) => p.y);
    const parts = partsInBox(d, [Math.min(...xs) - 0.01, Math.min(...ys) - 0.01], [Math.max(...xs) + 0.01, Math.max(...ys) + 0.01]);
    expect(parts.points.size).toBe(d.points.length);
    expect(parts.links.size).toBe(d.links.length);
    // a box around one joint only takes no bars
    const one = partsInBox(d, [d.points[0].x - 0.01, d.points[0].y - 0.01], [d.points[0].x + 0.01, d.points[0].y + 0.01]);
    expect([one.points.size, one.links.size]).toEqual([1, 0]);
    const frag = extractParts(d, parts);
    const c = fragmentCentre(frag);
    const pasted = pasteParts(d, frag, 5 - c[0], 2 - c[1]);
    const copy = extractParts(d, pasted);
    const c2 = fragmentCentre(copy);
    expect(c2[0]).toBeCloseTo(5, 12);
    expect(c2[1]).toBeCloseTo(2, 12);
  });

  it('moves pins and rails over to the pasted bodies, and drops them for bodies left behind', () => {
    const d: Design = {
      points: [
        { id: 0, x: 0, y: 0, kind: 'fixed', mass: 0 },
        { id: 1, x: 1, y: 0, kind: 'free', mass: 0 },
        { id: 2, x: 0.5, y: 0, kind: 'free', mass: 0, pin: { kind: 'link', id: 0 } },
        { id: 3, x: 0.5, y: -0.5, kind: 'free', mass: 0, slide: { on: { kind: 'link', id: 0 }, angle: 0 } },
        { id: 4, x: 0.5, y: -1, kind: 'free', mass: 1, slide: { on: null, angle: 30 } },
      ],
      links: [
        { id: 0, a: 0, b: 1, mass: 1 },
        { id: 1, a: 2, b: 3, mass: 1 },
        { id: 2, a: 3, b: 4, mass: 1 },
      ],
      springs: [], gears: [], meshes: [], rackMeshes: [],
    };
    const pasted = pasteParts(d, extractParts(d, allParts(d)), 3, 0);
    const copy = d.points.filter((p) => pasted.points.has(p.id));
    const bar0copy = d.links.find((l) => pasted.links.has(l.id) && l.a === copy[0].id)!;
    expect(copy[2].pin).toEqual({ kind: 'link', id: bar0copy.id });
    expect(copy[3].slide).toEqual({ on: { kind: 'link', id: bar0copy.id }, angle: 0 });
    expect(copy[4].slide).toEqual({ on: null, angle: 30 }); // a ground rail stays a ground rail
    // copying only bars 1 and 2 leaves bar 0 behind: the pin and the rail on it go
    const s = emptyParts();
    s.links.add(1).add(2);
    const part = extractParts(d, s);
    expect(part.points.find((p) => p.id === 2)!.pin).toBeUndefined();
    expect(part.points.find((p) => p.id === 3)!.slide).toBeUndefined();
    expect(part.points.find((p) => p.id === 4)!.slide).toEqual({ on: null, angle: 30 });
  });
});
