/**
 * The examples gallery: a panel in the middle of the screen with every example as a card, its
 * drawing above its name, in groups (and a box to filter them by name or description). Choosing
 * one shows it on the right: a bigger drawing to drag (any button) and zoom (the wheel), with its
 * labels shown or not (a box in its corner, remembered), its name and what it is, what it is made
 * of, and Open. Double-clicking a card, or Enter, opens it as well; Esc or a
 * click outside closes the gallery.
 */
import type { Design } from '../mechanics/model';
import type { Preset } from './presets';
import { drawScene, toWorld, type Scene, type Vec, type View } from './render';

/** A design drawn still, as drawn (no labels unless asked; the scale bar where it says). */
function sceneOf(d: Design, labels: boolean, scaleBar: Scene['scaleBar']): Scene {
  const at = new Map(d.points.map((p) => [p.id, [p.x, p.y] as Vec]));
  return { design: d, pos: (id) => at.get(id) ?? [0, 0], selection: null, hover: null, preview: null, gearAngle: () => 0, grab: null, simulating: false, labels, scaleBar, ground: d.ground ? d.ground.points : null };
}

/** The view that fits a design's parts (joints, and gears to their rims) into w × h px with a margin. */
function fitDesign(d: Design, w: number, h: number, margin = 14): View {
  const xs: number[] = [];
  const ys: number[] = [];
  for (const p of d.points) {
    xs.push(p.x);
    ys.push(p.y);
  }
  for (const g of d.gears) {
    const c = d.points.find((p) => p.id === g.center);
    if (!c) continue;
    xs.push(c.x - g.radius, c.x + g.radius);
    ys.push(c.y - g.radius, c.y + g.radius);
  }
  if (!xs.length) return { scale: 100, ox: w / 2, oy: h / 2 };
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  const scale = Math.min((w - 2 * margin) / Math.max(x1 - x0, 1e-3), (h - 2 * margin) / Math.max(y1 - y0, 1e-3), 2000);
  return { scale, ox: w / 2 - ((x0 + x1) / 2) * scale, oy: h / 2 + ((y0 + y1) / 2) * scale };
}

/** Draw a design on a canvas at its CSS size (sharp on high-density screens). */
function paint(cv: HTMLCanvasElement, d: Design, view: View, labels: boolean, scaleBar: Scene['scaleBar']) {
  const dpr = window.devicePixelRatio || 1;
  const [w, h] = [cv.clientWidth, cv.clientHeight];
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) [cv.width, cv.height] = [Math.round(w * dpr), Math.round(h * dpr)];
  const ctx = cv.getContext('2d')!;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  drawScene(ctx, w, h, view, sceneOf(d, labels, scaleBar));
}

/** What a design is made of, in words: "11 bars · 8 joints · 1 gear with a motor". */
function makeup(d: Design): string {
  const n = (k: number, one: string, many = `${one}s`) => (k ? `${k} ${k === 1 ? one : many}` : '');
  const bars = d.links.filter((l) => !l.rack).length;
  const wheels = d.gears.filter((g) => g.wheel).length;
  const props = d.gears.filter((g) => g.propeller).length;
  const motors = d.gears.filter((g) => g.motor).length;
  return [
    n(bars, 'bar'),
    n(d.links.length - bars, 'rack'),
    n(d.points.length, 'joint'),
    n(d.gears.length - wheels - props, 'gear'),
    n(wheels, 'wheel'),
    n(props, 'propeller'),
    n(d.springs.length, 'spring'),
    n(motors, 'motor'),
    n((d.joins ?? []).length, 'join'),
    d.ground ? 'ground contact' : '',
  ]
    .filter(Boolean)
    .join(' · ');
}

const esc = (s: string) => s.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);

export class Gallery {
  private readonly backdrop: HTMLDivElement;
  private readonly grid: HTMLDivElement;
  private readonly detail: HTMLElement;
  private readonly preview: HTMLCanvasElement;
  private readonly filter: HTMLInputElement;
  private designs: Design[] | null = null;
  private chosen = -1;
  private view: View = { scale: 100, ox: 0, oy: 0 };
  private drag: { x: number; y: number; ox: number; oy: number } | null = null;
  /** Whether the bigger drawing shows the joints' and parts' names (remembered). */
  private labels = localStorage.getItem('linkage-lab:gallery-labels') !== 'off';

  /** `open(i)` is asked to open example i (it may ask about unsaved work first); the gallery closes. */
  constructor(private readonly presets: Preset[], private readonly onOpen: (i: number) => void) {
    this.backdrop = document.createElement('div');
    this.backdrop.id = 'gallery-backdrop';
    this.backdrop.hidden = true;
    this.backdrop.innerHTML = `
      <div id="gallery" class="panel" role="dialog" aria-label="Examples">
        <header><h2>Examples</h2><input type="search" class="g-filter" placeholder="Filter by name or description…" spellcheck="false"><button class="g-close" title="Close (Esc)">×</button></header>
        <div class="g-main">
          <div class="g-grid"></div>
          <aside class="g-detail" hidden>
            <div class="g-view">
              <canvas class="g-preview" title="Drag to move, scroll to zoom, double-click to fit"></canvas>
              <label class="g-labels"><input type="checkbox"> Labels</label>
            </div>
            <h3 class="g-name"></h3>
            <p class="g-group"></p>
            <p class="g-desc"></p>
            <p class="g-makeup"></p>
            <button class="primary g-open">Open</button>
          </aside>
        </div>
      </div>`;
    document.body.append(this.backdrop);
    this.grid = this.backdrop.querySelector('.g-grid')!;
    this.detail = this.backdrop.querySelector('.g-detail')!;
    this.preview = this.backdrop.querySelector('.g-preview')!;
    this.filter = this.backdrop.querySelector('.g-filter')!;
    this.backdrop.addEventListener('pointerdown', (e) => {
      if (e.target === this.backdrop) this.close(); // a click outside the panel
    });
    this.backdrop.querySelector('.g-close')!.addEventListener('click', () => this.close());
    this.backdrop.querySelector('.g-open')!.addEventListener('click', () => this.openChosen());
    this.filter.addEventListener('input', () => this.applyFilter());
    this.backdrop.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        this.close();
      } else if (e.key === 'Enter' && this.chosen >= 0 && !(e.target as HTMLElement).closest('button')) {
        e.preventDefault();
        this.openChosen();
      }
    });
    // the preview: drag to move it, the wheel zooms about the mouse, double-click fits it again
    this.preview.addEventListener('pointerdown', (e) => {
      this.preview.setPointerCapture(e.pointerId);
      this.drag = { x: e.clientX, y: e.clientY, ox: this.view.ox, oy: this.view.oy };
    });
    this.preview.addEventListener('pointermove', (e) => {
      if (!this.drag) return;
      this.view = { ...this.view, ox: this.drag.ox + e.clientX - this.drag.x, oy: this.drag.oy + e.clientY - this.drag.y };
      this.paintPreview();
    });
    const endDrag = () => (this.drag = null);
    this.preview.addEventListener('pointerup', endDrag);
    this.preview.addEventListener('pointercancel', endDrag);
    this.preview.addEventListener(
      'wheel',
      (e) => {
        e.preventDefault();
        const f = Math.exp(-e.deltaY * 0.0015);
        const [wx, wy] = toWorld(this.view, [e.offsetX, e.offsetY]);
        const scale = Math.min(Math.max(this.view.scale * f, 5), 20000);
        this.view = { scale, ox: e.offsetX - wx * scale, oy: e.offsetY + wy * scale };
        this.paintPreview();
      },
      { passive: false },
    );
    this.preview.addEventListener('dblclick', () => this.fitPreview());
    // a right-drag pans too: no browser menu (Safari opens it on the press, ending the drag)
    this.preview.addEventListener('contextmenu', (e) => e.preventDefault());
    const box = this.backdrop.querySelector<HTMLInputElement>('.g-labels input')!;
    box.checked = this.labels;
    box.addEventListener('change', () => {
      this.labels = box.checked;
      localStorage.setItem('linkage-lab:gallery-labels', this.labels ? 'on' : 'off');
      this.paintPreview();
    });
    window.addEventListener('resize', () => {
      if (!this.backdrop.hidden && this.chosen >= 0) this.fitPreview();
    });
  }

  get isOpen() {
    return !this.backdrop.hidden;
  }

  open() {
    this.backdrop.hidden = false;
    if (!this.designs) this.build();
    this.filter.value = '';
    this.applyFilter();
    // the drawings are sized by the layout, so drawn once it is there
    requestAnimationFrame(() => this.paintCards());
    if (this.chosen >= 0) requestAnimationFrame(() => this.fitPreview());
    this.filter.focus();
  }

  close() {
    this.backdrop.hidden = true;
    this.drag = null;
  }

  /** The cards, in their groups (each example built once). */
  private build() {
    this.designs = this.presets.map((p) => p.build());
    const groups = [...new Set(this.presets.map((p) => p.group))];
    this.grid.innerHTML = groups
      .map(
        (g) => `<section class="g-section" data-group="${esc(g)}"><h3>${esc(g)}</h3><div class="g-cards">${this.presets
          .map((p, i) => (p.group === g ? `<button class="g-card" data-i="${i}" title="${esc(p.name)}"><canvas></canvas><span>${esc(p.name)}</span></button>` : ''))
          .join('')}</div></section>`,
      )
      .join('');
    this.grid.querySelectorAll<HTMLButtonElement>('.g-card').forEach((card) => {
      const i = Number(card.dataset.i);
      card.addEventListener('click', () => this.choose(i));
      card.addEventListener('dblclick', () => {
        this.choose(i);
        this.openChosen();
      });
    });
  }

  private paintCards() {
    this.grid.querySelectorAll<HTMLButtonElement>('.g-card').forEach((card) => {
      const cv = card.querySelector('canvas')!;
      if (cv.dataset.drawn === String(cv.clientWidth) || !cv.clientWidth) return;
      const d = this.designs![Number(card.dataset.i)];
      paint(cv, d, fitDesign(d, cv.clientWidth, cv.clientHeight, 8), false, 'none'); // a card: just the mechanism
      cv.dataset.drawn = String(cv.clientWidth);
    });
  }

  /** Show only the examples whose name, group or description has every word typed. */
  private applyFilter() {
    const words = this.filter.value.toLowerCase().split(/\s+/).filter(Boolean);
    this.grid.querySelectorAll<HTMLElement>('.g-section').forEach((sec) => {
      let any = false;
      sec.querySelectorAll<HTMLElement>('.g-card').forEach((card) => {
        const p = this.presets[Number(card.dataset.i)];
        const text = `${p.name} ${p.group} ${p.description ?? ''}`.toLowerCase();
        const show = words.every((w) => text.includes(w));
        card.hidden = !show;
        any ||= show;
      });
      sec.hidden = !any;
    });
    requestAnimationFrame(() => this.paintCards());
  }

  private choose(i: number) {
    this.chosen = i;
    this.grid.querySelectorAll<HTMLElement>('.g-card').forEach((c) => c.classList.toggle('on', Number(c.dataset.i) === i));
    const p = this.presets[i];
    const d = this.designs![i];
    this.detail.hidden = false;
    this.detail.querySelector('.g-name')!.textContent = p.name;
    this.detail.querySelector('.g-group')!.textContent = `${p.group}${p.topView ? ' · seen from above (no gravity)' : ''}${p.follow ? ' · the view follows it' : ''}`;
    this.detail.querySelector('.g-desc')!.textContent = p.description ?? '';
    this.detail.querySelector('.g-makeup')!.textContent = makeup(d);
    requestAnimationFrame(() => this.fitPreview());
  }

  private fitPreview() {
    if (this.chosen < 0) return;
    this.view = fitDesign(this.designs![this.chosen], this.preview.clientWidth, this.preview.clientHeight, 20);
    this.paintPreview();
  }

  private paintPreview() {
    if (this.chosen >= 0) paint(this.preview, this.designs![this.chosen], this.view, this.labels, 'left'); // clear of the labels box
  }

  private openChosen() {
    if (this.chosen < 0) return;
    const i = this.chosen;
    this.close();
    this.onOpen(i);
  }
}
