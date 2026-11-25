/**
 * Groups of parts: selecting them, copying them out of a design, and pasting them back in.
 * A copied fragment keeps every constraint among its own parts (ground pins and rails, joints
 * pinned on its gears, gear and rack meshes, fixed angles, joined bodies); constraints to
 * parts that were not copied are left behind.
 */
import { nextId, type DBodyRef, type Design } from './model';

/** A set of parts of a design, by id. */
export interface PartSet {
  points: Set<number>;
  links: Set<number>;
  springs: Set<number>;
  gears: Set<number>;
}

export const emptyParts = (): PartSet => ({ points: new Set(), links: new Set(), springs: new Set(), gears: new Set() });

export const partCount = (s: PartSet) => s.points.size + s.links.size + s.springs.size + s.gears.size;

/** Every part of the design. */
export function allParts(d: Design): PartSet {
  return {
    points: new Set(d.points.map((p) => p.id)),
    links: new Set(d.links.map((l) => l.id)),
    springs: new Set(d.springs.map((s) => s.id)),
    gears: new Set(d.gears.map((g) => g.id)),
  };
}

/**
 * The parts inside a rectangle: joints inside it, bars, racks and springs with both ends
 * inside, and gears whose centre is inside (with the joints pinned on them).
 */
export function partsInBox(d: Design, [x0, y0]: [number, number], [x1, y1]: [number, number]): PartSet {
  const [lx, hx, ly, hy] = [Math.min(x0, x1), Math.max(x0, x1), Math.min(y0, y1), Math.max(y0, y1)];
  const s = emptyParts();
  for (const p of d.points) if (p.x >= lx && p.x <= hx && p.y >= ly && p.y <= hy) s.points.add(p.id);
  for (const l of d.links) if (s.points.has(l.a) && s.points.has(l.b)) s.links.add(l.id);
  for (const sp of d.springs) if (s.points.has(sp.a) && s.points.has(sp.b)) s.springs.add(sp.id);
  for (const g of d.gears) if (s.points.has(g.center)) s.gears.add(g.id);
  return closeParts(d, s);
}

/**
 * Add what the chosen parts cannot exist without: the end joints of bars and springs, and the
 * centre and pinned joints of gears.
 */
export function closeParts(d: Design, s: PartSet): PartSet {
  const out: PartSet = { points: new Set(s.points), links: new Set(s.links), springs: new Set(s.springs), gears: new Set(s.gears) };
  for (const l of d.links) if (out.links.has(l.id)) out.points.add(l.a).add(l.b);
  for (const sp of d.springs) if (out.springs.has(sp.id)) out.points.add(sp.a).add(sp.b);
  for (const g of d.gears) if (out.gears.has(g.id)) out.points.add(g.center);
  for (const p of d.points) if (p.gear !== undefined && out.gears.has(p.gear)) out.points.add(p.id);
  return out;
}

const hasBody = (s: PartSet, r: DBodyRef) => (r.kind === 'gear' ? s.gears.has(r.id) : s.links.has(r.id));

/** A copy of the chosen parts and the constraints among them, as a design of its own. */
export function extractParts(d: Design, chosen: PartSet): Design {
  const s = closeParts(d, chosen);
  return {
    points: d.points
      .filter((p) => s.points.has(p.id))
      .map((p) => {
        const q = { ...p };
        if (q.gear !== undefined && !s.gears.has(q.gear)) delete q.gear; // its gear stays behind
        return q;
      }),
    links: d.links.filter((l) => s.links.has(l.id)).map((l) => ({ ...l })),
    springs: d.springs.filter((sp) => s.springs.has(sp.id)).map((sp) => ({ ...sp })),
    gears: d.gears.filter((g) => s.gears.has(g.id)).map((g) => ({ ...g })),
    meshes: d.meshes.filter((m) => s.gears.has(m.a) && s.gears.has(m.b)).map((m) => ({ ...m })),
    rackMeshes: d.rackMeshes.filter((m) => s.gears.has(m.gear) && s.links.has(m.rack)).map((m) => ({ ...m })),
    angles: (d.angles ?? []).filter((a) => s.links.has(a.a) && s.links.has(a.b)).map((a) => ({ ...a })),
    joins: (d.joins ?? []).filter((j) => hasBody(s, j.a) && hasBody(s, j.b)).map((j) => ({ id: j.id, a: { ...j.a }, b: { ...j.b } })),
  };
}

/** Centre of a fragment's bounding box. */
export function fragmentCentre(f: Design): [number, number] {
  if (!f.points.length) return [0, 0];
  const xs = f.points.map((p) => p.x);
  const ys = f.points.map((p) => p.y);
  return [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2];
}

/**
 * Add a copy of a fragment to the design, shifted by (dx, dy), with fresh ids so it never
 * clashes with existing parts. Returns the new parts.
 */
export function pasteParts(d: Design, f: Design, dx: number, dy: number): PartSet {
  const renumber = <T extends { id: number }>(items: readonly T[], existing: readonly { id: number }[]) => {
    let next = nextId(existing);
    return new Map(items.map((it) => [it.id, next++]));
  };
  const P = renumber(f.points, d.points);
  const L = renumber(f.links, d.links);
  const S = renumber(f.springs, d.springs);
  const G = renumber(f.gears, d.gears);
  const M = renumber(f.meshes, d.meshes);
  const RM = renumber(f.rackMeshes, d.rackMeshes);
  const A = renumber(f.angles ?? [], d.angles ?? []);
  const J = renumber(f.joins ?? [], d.joins ?? []);
  const body = (r: DBodyRef): DBodyRef => ({ kind: r.kind, id: (r.kind === 'gear' ? G : L).get(r.id)! });

  for (const p of f.points) {
    const q = { ...p, id: P.get(p.id)!, x: p.x + dx, y: p.y + dy };
    if (p.gear !== undefined) q.gear = G.get(p.gear)!;
    d.points.push(q);
  }
  for (const l of f.links) d.links.push({ ...l, id: L.get(l.id)!, a: P.get(l.a)!, b: P.get(l.b)! });
  for (const sp of f.springs) d.springs.push({ ...sp, id: S.get(sp.id)!, a: P.get(sp.a)!, b: P.get(sp.b)! });
  for (const g of f.gears) d.gears.push({ ...g, id: G.get(g.id)!, center: P.get(g.center)! });
  for (const m of f.meshes) d.meshes.push({ id: M.get(m.id)!, a: G.get(m.a)!, b: G.get(m.b)! });
  for (const m of f.rackMeshes) d.rackMeshes.push({ id: RM.get(m.id)!, gear: G.get(m.gear)!, rack: L.get(m.rack)! });
  for (const a of f.angles ?? []) (d.angles ??= []).push({ id: A.get(a.id)!, a: L.get(a.a)!, b: L.get(a.b)! });
  for (const j of f.joins ?? []) (d.joins ??= []).push({ id: J.get(j.id)!, a: body(j.a), b: body(j.b) });

  return { points: new Set(P.values()), links: new Set(L.values()), springs: new Set(S.values()), gears: new Set(G.values()) };
}
