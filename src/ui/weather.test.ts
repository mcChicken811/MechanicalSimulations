import { describe, expect, it } from 'vitest';
import { isWeatherMode, Weather, type WeatherKind } from './weather';

/** Run the weather for `secs` s in steps of dt, calling `each` after every step. */
function run(w: Weather, secs: number, mode: Parameters<Weather['step']>[1], each?: () => void, dt = 0.05) {
  for (let t = 0; t < secs; t += dt) {
    w.step(dt, mode);
    each?.();
  }
}

describe('weather', () => {
  it('a set weather eases to its levels and stays', () => {
    const w = new Weather();
    run(w, 60, 'storm');
    expect(w.kind).toBe('storm');
    expect(w.now.cover).toBeCloseTo(1, 2);
    expect(w.now.rain).toBeCloseTo(1, 2);
    expect(w.now.hail).toBeCloseTo(0, 3);
    run(w, 60, 'hail');
    expect(w.kind).toBe('hail');
    expect(w.now.hail).toBeCloseTo(1, 2);
    expect(w.now.rain).toBeLessThan(0.3);
  });

  it('eases rather than switching', () => {
    const w = new Weather();
    run(w, 1, 'rain');
    expect(w.now.rain).toBeGreaterThan(0);
    expect(w.now.rain).toBeLessThan(0.2);
  });

  it('changing weather: clear spells broken by weather of every kind, each for a while', () => {
    const w = new Weather();
    const spells: { kind: WeatherKind; secs: number }[] = [];
    let last = w.kind;
    let since = 0;
    run(w, 40000, 'random', () => {
      since += 0.05;
      if (w.kind !== last) {
        spells.push({ kind: last, secs: since });
        [last, since] = [w.kind, 0];
      }
    }, 0.05);
    const kinds = new Set(spells.map((s) => s.kind));
    for (const k of ['clear', 'cloudy', 'rain', 'storm', 'thunder', 'hail', 'snow'] as WeatherKind[]) expect(kinds.has(k)).toBe(true);
    // every spell lasts its while (no flicker), and none for ever
    for (const s of spells.slice(1)) {
      expect(s.secs).toBeGreaterThan(19);
      expect(s.secs).toBeLessThan(181);
    }
    // it is clear for much of the time
    const clear = spells.filter((s) => s.kind === 'clear').reduce((a, s) => a + s.secs, 0);
    const all = spells.reduce((a, s) => a + s.secs, 0);
    expect(clear / all).toBeGreaterThan(0.35);
  });

  it('lightning strikes only in a thunderstorm, its thunder after it, the nearer the louder', () => {
    const w = new Weather();
    const thunder: [number, number][] = [];
    w.onThunder = (delay, loud) => thunder.push([delay, loud]);
    run(w, 120, 'storm');
    expect(thunder.length).toBe(0);
    let flashed = 0;
    run(w, 120, 'thunder', () => (flashed = Math.max(flashed, w.flash)));
    expect(thunder.length).toBeGreaterThan(5);
    expect(flashed).toBeGreaterThan(0.3);
    for (const [delay, loud] of thunder) {
      expect(delay).toBeGreaterThan(0);
      expect(loud).toBeGreaterThan(0.3);
      expect(loud).toBeLessThanOrEqual(1);
    }
    const near = thunder.reduce((a, b) => (b[0] < a[0] ? b : a));
    const far = thunder.reduce((a, b) => (b[0] > a[0] ? b : a));
    expect(near[1]).toBeGreaterThan(far[1]);
  });

  it('clears at once for a new background', () => {
    const w = new Weather();
    run(w, 60, 'thunder');
    w.clear();
    expect(w.active).toBe(false);
    expect(w.flash).toBe(0);
  });

  it('a weather started by hand lasts its time, then changing weather clears', () => {
    const w = new Weather();
    run(w, 5, 'random');
    w.trigger('hail', 30);
    run(w, 29, 'random');
    expect(w.kind).toBe('hail');
    expect(w.now.hail).toBeGreaterThan(0.9);
    run(w, 2, 'random');
    expect(w.triggered).toBe(null);
    expect(w.kind).toBe('clear');
    run(w, 30, 'random');
    expect(w.now.hail).toBeLessThan(0.01);
  });

  it('a weather started by hand over a set weather goes back to it; until stopped lasts until stopped', () => {
    const w = new Weather();
    run(w, 5, 'cloudy');
    w.trigger('rain', 10);
    run(w, 11, 'cloudy');
    expect(w.kind).toBe('cloudy');
    w.trigger('storm', Infinity);
    run(w, 1000, 'cloudy');
    expect(w.kind).toBe('storm');
    w.endTrigger();
    run(w, 1, 'cloudy');
    expect(w.kind).toBe('cloudy');
    // choosing a weather ends one started by hand
    w.trigger('thunder', Infinity);
    run(w, 1, 'random');
    expect(w.triggered).toBe(null);
  });

  it('a thunderstorm started by hand strikes soon; lightning strikes on demand', () => {
    const w = new Weather();
    let n = 0;
    w.onThunder = () => n++;
    w.strikeNow();
    expect(n).toBe(1);
    expect(w.bolt).not.toBe(null);
    run(w, 2, 'random');
    w.trigger('thunder', 60);
    run(w, 25, 'random');
    expect(n).toBeGreaterThan(1);
  });

  it('snow settles while it snows and melts after, faster in the rain', () => {
    const w = new Weather();
    run(w, 200, 'snow');
    expect(w.now.snow).toBeGreaterThan(0.99);
    expect(w.settled).toBeGreaterThan(0.9);
    const lying = w.settled;
    run(w, 30, 'clear');
    expect(w.settled).toBeLessThan(lying);
    expect(w.settled).toBeGreaterThan(0.5);
    const a = new Weather();
    const b = new Weather();
    run(a, 200, 'snow');
    run(b, 200, 'snow');
    run(a, 40, 'clear');
    run(b, 40, 'rain');
    expect(b.settled).toBeLessThan(a.settled);
    run(a, 400, 'clear');
    expect(a.settled).toBe(0);
    expect(a.active).toBe(false);
  });

  it('knows its modes', () => {
    expect(isWeatherMode('random')).toBe(true);
    expect(isWeatherMode('hail')).toBe(true);
    expect(isWeatherMode('snow')).toBe(true);
    expect(isWeatherMode('fog')).toBe(false);
    expect(isWeatherMode(3)).toBe(false);
  });
});
