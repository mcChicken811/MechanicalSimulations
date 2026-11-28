import { describe, expect, it } from 'vitest';
import { Backdrop, DEFAULT_ANIM, SCENES, type BackgroundState } from './background';

/** A backdrop showing `kind` at a still hour, with the weather set. */
function at(kind: string, hour: number, anim: Partial<BackgroundState['anim']> = {}) {
  const b = new Backdrop();
  b.set({ kind, base: 'none', layers: [], opacity: 1, anim: { ...DEFAULT_ANIM, dayLength: 0, weather: 'clear', hour, ...anim } });
  return b;
}

describe('background sounds', () => {
  it('blueprint, graph paper and plain have none', () => {
    for (const kind of ['blueprint', 'paper', 'none']) expect(at(kind, 12).soundscape).toBe(null);
    expect(SCENES.filter((s) => s.sounds).map((s) => s.id).sort()).toEqual(['city', 'mountains', 'night', 'sky', 'sunset']);
  });

  it('birds sing by day, crickets and owls by night', () => {
    const noon = at('sky', 12).soundscape!;
    const night = at('sky', 0).soundscape!;
    expect(noon.birds).toBeGreaterThan(0.5);
    expect(noon.crickets).toBe(0);
    expect(night.birds).toBe(0);
    expect(night.crickets).toBeGreaterThan(0.5);
    expect(night.owls).toBeGreaterThan(0);
    // loudest at dawn
    expect(at('sky', 7).soundscape!.birds).toBeGreaterThanOrEqual(at('sky', 14).soundscape!.birds);
  });

  it('the city hums with traffic, quieter at night; the mountains have their stream', () => {
    expect(at('city', 12).soundscape!.traffic).toBeGreaterThan(at('city', 1).soundscape!.traffic);
    expect(at('city', 1).soundscape!.traffic).toBeGreaterThan(0);
    expect(at('mountains', 12).soundscape!.stream).toBeGreaterThan(0);
    expect(at('sky', 12).soundscape!.traffic).toBe(0);
  });

  it('the rain hushes the birds and is heard itself', () => {
    const b = at('sky', 12, { weather: 'storm' });
    for (let i = 0; i < 1200; i++) b.weather.step(0.05, 'storm');
    const s = b.soundscape!;
    expect(s.birds).toBe(0);
    expect(s.rain).toBeGreaterThan(0.9);
    expect(s.wind).toBeGreaterThan(0.5);
  });

  it('none when still, and no birdsong without the birds', () => {
    expect(at('sky', 12, { on: false }).soundscape).toBe(null);
    expect(at('sky', 12, { birds: false }).soundscape!.birds).toBe(0);
  });

  it('a custom background sounds like its base scene', () => {
    const b = new Backdrop();
    b.set({ kind: 'custom', base: 'city', layers: [], opacity: 1, anim: { ...DEFAULT_ANIM, dayLength: 0, hour: 12 } });
    expect(b.soundscape!.traffic).toBeGreaterThan(0);
    b.set({ kind: 'custom', base: 'blueprint', layers: [], opacity: 1, anim: { ...DEFAULT_ANIM } });
    expect(b.soundscape).toBe(null);
  });
});
