import { describe, expect, it } from 'vitest';
import { inkStressColor, letter, parseColor, reink } from './inkstyle';
import { inkFor, inkOn, inkStyleOf, NO_BACKGROUND, type BackgroundState } from './background';

const BLUE: [number, number, number, number] = [124, 154, 195, 1];
const PAPER: [number, number, number, number] = [250, 247, 239, 1];
const rgb = (c: string) => parseColor(c).slice(0, 3);
const light = (c: string) => {
  const [r, g, b] = parseColor(c);
  return (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255;
};

describe('drawing styles', () => {
  it('reads colours', () => {
    expect(parseColor('#1f2125')).toEqual([31, 33, 37, 1]);
    expect(parseColor('#fff')).toEqual([255, 255, 255, 1]);
    expect(parseColor('rgba(10, 20, 30, 0.5)')).toEqual([10, 20, 30, 0.5]);
    const [r, g, b] = parseColor('hsl(120, 100%, 50%)');
    expect([Math.round(r), Math.round(g), Math.round(b)]).toEqual([0, 255, 0]);
  });

  it('a blueprint: dark ink becomes white lines, white halos the blue paper', () => {
    expect(light(reink('#1f2125', 'line', 'blueprint', BLUE).color)).toBeGreaterThan(0.9);
    const halo = rgb(reink('rgba(255,255,255,0.92)', 'line', 'blueprint', BLUE).color);
    halo.forEach((v, i) => expect(Math.abs(v - BLUE[i])).toBeLessThan(10));
    // bodies are a faint wash of the paper, not solid
    const body = light(reink('#3b6ea8', 'body', 'blueprint', BLUE).color);
    expect(body).toBeLessThan(0.8);
    expect(body).toBeGreaterThan(light('rgb(124,154,195)') - 0.01);
  });

  it('pencil: dark ink becomes graphite, light fills the paper, bodies hatched', () => {
    expect(light(reink('#1f2125', 'line', 'sketch', PAPER).color)).toBeLessThan(0.2);
    expect(light(reink('#ffffff', 'body', 'sketch', PAPER).color)).toBeGreaterThan(0.9);
    const bar = reink('#3b6ea8', 'body', 'sketch', PAPER);
    expect(bar.hatch).toBeGreaterThan(0);
    expect(light(bar.color)).toBeGreaterThan(0.6);
  });

  it('highlights keep their colour; strong colours keep a hint of their hue', () => {
    expect(reink('#e07b39', 'body', 'sketch', PAPER, true).color).toBe('#e07b39');
    const [r, , b] = parseColor(reink('#e07b39', 'line', 'sketch', PAPER).color);
    expect(r).toBeGreaterThan(b); // still orange-ish
  });

  it('internal forces: blue to white on a blueprint, grey to black in pencil', () => {
    expect(rgb(inkStressColor(1, 'blueprint'))).toEqual([255, 255, 255]);
    expect(light(inkStressColor(0, 'blueprint'))).toBeLessThan(0.4);
    expect(light(inkStressColor(0, 'sketch'))).toBeGreaterThan(0.7);
    expect(light(inkStressColor(1, 'sketch'))).toBeLessThan(0.1);
    expect(light(inkStressColor(0.5, 'sketch'))).toBeLessThan(light(inkStressColor(0.2, 'sketch')));
  });

  it('lettering: the style’s font at the size', () => {
    expect(letter('600 12px system-ui, sans-serif', 'blueprint')).toMatch(/^600 12px .*Condensed/);
    expect(letter('italic 600 10px system-ui, sans-serif', 'sketch')).toMatch(/^italic 600 12px .*cursive$/);
  });

  it('which background draws the machine how, and on what paper', () => {
    const bg = (kind: string, extra: Partial<BackgroundState> = {}): BackgroundState => ({ ...NO_BACKGROUND, kind, ...extra });
    expect(inkStyleOf(bg('blueprint'))).toBe('blueprint');
    expect(inkStyleOf(bg('paper'))).toBe('sketch');
    expect(inkStyleOf(bg('none'))).toBe('sketch');
    expect(inkStyleOf(bg('mountains'))).toBe(null);
    // on by default for the blueprint and graph paper, off for plain paper
    expect(inkOn(bg('blueprint'))).toBe(true);
    expect(inkOn(bg('paper'))).toBe(true);
    expect(inkOn(bg('none'))).toBe(false);
    expect(inkOn(bg('none', { style: { none: true } }))).toBe(true);
    expect(inkFor(bg('blueprint', { style: { blueprint: false } }), '#f6f6f3')).toBe(null);
    // the paper as it shows: the blueprint's blue at its strength over the page
    const ink = inkFor(bg('blueprint', { opacity: 1 }), '#f6f6f3')!;
    expect(ink.style).toBe('blueprint');
    expect(rgb(ink.paper)).toEqual([42, 93, 156]);
    expect(inkFor(bg('custom', { base: 'paper' }), '#f6f6f3')!.style).toBe('sketch');
  });
});
