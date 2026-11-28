/**
 * The gallery: a panel in the middle of the screen with the user's own projects (saved in the
 * browser) above the examples, each as a card with its drawing above its name, in groups (and a
 * box to filter them by name or description). Choosing one shows it on the right: a bigger
 * drawing to drag (any button) and zoom (the wheel), with its labels shown or not (a box in its
 * corner, remembered), its name and description (editable for one's own projects), what it is
 * made of, and the buttons: Open (the whole project, or the example) and Import (just its
 * drawing, added to the one there). Double-clicking a card, or Enter, opens it as well; Esc or a
 * click outside closes the gallery.
 *
 * The first card of one's own projects saves the current project there.
 */
import type { Design } from '../mechanics/model';
import { drawScene, toWorld, type Scene, type Vec, type View } from './render';

/** Something in the gallery: an example, or a project of the user's (`editable`). */
export interface GalleryItem {
  key: string;
  group: string;
  name: string;
  description: string;
  /** A line under the name: the group, when it was saved, a note such as "seen from above". */
  notes: string;
  design: () => Design;
  editable?: boolean;
}

export interface GalleryHooks {
  /** Everything to show, own projects first (asked again whenever they change). */
  items: () => GalleryItem[];
  /** Open it (the gallery closes first; the app may ask about unsaved work). */
  open: (key: string) => void;
  /** Add its drawing to the one there (the gallery closes). */
  import: (key: string) => void;
  /** Save the current project as one of the user's; its key, or null if it could not be. */
  saveCurrent: () => string | null;
  /** Save the current project over one of the user's. */
  replace: (key: string) => void;
  rename: (key: string, name: string, description: string) => void;
  /** Delete one of the user's (after asking); whether it was. */
  remove: (key: string) => Promise<boolean>;
}

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

/** What a design is made of, in words: "11 bars · 8 joints · 1 gear · 1 motor". */
function makeup(d: Design): string {
  const n = (k: number, one: string, many = `${one}s`) => (k ? `${k} ${k === 1 ? one : many}` : '');
  const bars = d.links.filter((l) => !l.rack).length;
  const wheels = d.gears.filter((g) => g.wheel).length;
  const props = d.gears.filter((g) => g.propeller).length;
  return (
    [
      n(bars, 'bar'),
      n(d.links.length - bars, 'rack'),
      n(d.points.length, 'joint'),
      n(d.gears.length - wheels - props, 'gear'),
      n(wheels, 'wheel'),
      n(props, 'propeller'),
      n(d.springs.length, 'spring'),
      n(d.gears.filter((g) => g.motor).length, 'motor'),
      n((d.joins ?? []).length, 'join'),
      d.ground ? 'ground contact' : '',
    ]
      .filter(Boolean)
      .join(' · ') || 'empty'
  );
}

const esc = (s: string) => s.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);

/** The heading of the user's own projects. */
export const MY_PROJECTS = 'My projects';

export class Gallery {
  private readonly backdrop: HTMLDivElement;
  private readonly grid: HTMLDivElement;
  private readonly detail: HTMLElement;
  private readonly preview: HTMLCanvasElement;
  private readonly filter: HTMLInputElement;
  private items: GalleryItem[] = [];
  private readonly designs = new Map<string, Design>();
  private chosen: string | null = null;
  private view: View = { scale: 100, ox: 0, oy: 0 };
  private drag: { x: number; y: number; ox: number; oy: number } | null = null;
  /** Whether the bigger drawing shows the joints' and parts' names (remembered). */
  private labels = localStorage.getItem('linkage-lab:gallery-labels') !== 'off';

  constructor(private readonly hooks: GalleryHooks) {
    this.backdrop = document.createElement('div');
    this.backdrop.id = 'gallery-backdrop';
    this.backdrop.hidden = true;
    this.backdrop.innerHTML = `
      <div id="gallery" class="panel" role="dialog" aria-label="Gallery">
        <header><h2>Gallery</h2><input type="search" class="g-filter" placeholder="Filter by name or description…" spellcheck="false"><button class="g-close" title="Close (Esc)">×</button></header>
        <div class="g-main">
          <div class="g-grid"></div>
          <aside class="g-detail empty">
            <p class="g-placeholder">Choose a project or an example to see it here.</p>
            <div class="g-view">
              <canvas class="g-preview" title="Drag to move, scroll to zoom, double-click to fit"></canvas>
              <label class="g-labels"><input type="checkbox"> Labels</label>
            </div>
            <h3 class="g-name"></h3>
            <input class="g-name-edit" type="text" spellcheck="false" placeholder="Name" hidden>
            <p class="g-notes"></p>
            <p class="g-desc"></p>
            <textarea class="g-desc-edit" rows="4" placeholder="Describe it: what it is, what to look for…" hidden></textarea>
            <p class="g-makeup"></p>
            <div class="g-actions">
              <button class="primary g-open"></button>
              <button class="g-import" title="Add its drawing to yours: its outline follows the mouse, click to place it (its settings are left out)">Import</button>
              <span class="g-own">
                <button class="g-replace" title="Save the current project over this one">Update</button>
                <button class="g-delete" title="Delete this project from the gallery">Delete</button>
              </span>
            </div>
          </aside>
        </div>
      </div>`;
    document.body.append(this.backdrop);
    this.grid = this.backdrop.querySelector('.g-grid')!;
    this.detail = this.backdrop.querySelector('.g-detail')!;
    this.preview = this.backdrop.querySelector('.g-preview')!;
    this.filter = this.backdrop.querySelector('.g-filter')!;
    const q = <T extends Element>(sel: string) => this.backdrop.querySelector<T>(sel)!;
    this.backdrop.addEventListener('pointerdown', (e) => {
      if (e.target === this.backdrop) this.close(); // a click outside the panel
    });
    q('.g-close').addEventListener('click', () => this.close());
    q('.g-open').addEventListener('click', () => this.act('open'));
    q('.g-import').addEventListener('click', () => this.act('import'));
    q('.g-replace').addEventListener('click', () => {
      if (!this.chosen) return;
      this.hooks.replace(this.chosen);
      this.designs.delete(this.chosen);
      this.refresh(this.chosen);
    });
    q('.g-delete').addEventListener('click', async () => {
      if (!this.chosen || !(await this.hooks.remove(this.chosen))) return;
      this.designs.delete(this.chosen);
      this.chosen = null;
      this.detail.classList.add('empty');
      this.refresh();
    });
    // one's own project's name and description, edited in place
    const nameEdit = q<HTMLInputElement>('.g-name-edit');
    const descEdit = q<HTMLTextAreaElement>('.g-desc-edit');
    const saveText = () => {
      const it = this.item(this.chosen);
      if (!it?.editable) return;
      const name = nameEdit.value.trim() || 'Untitled project';
      this.hooks.rename(it.key, name, descEdit.value.trim());
      const card = this.grid.querySelector(`.g-card[data-key="${CSS.escape(it.key)}"]`);
      if (card) {
        card.querySelector('span')!.textContent = name;
        card.setAttribute('title', name);
      }
      this.items = this.hooks.items();
    };
    nameEdit.addEventListener('change', saveText);
    descEdit.addEventListener('change', saveText);
    for (const el of [nameEdit, descEdit])
      el.addEventListener('keydown', (e) => {
        const k = e as KeyboardEvent;
        if (k.key === 'Enter' && (el === nameEdit || k.metaKey || k.ctrlKey)) {
          e.preventDefault();
          el.blur(); // done: saved by its change
        }
      });
    this.filter.addEventListener('input', () => this.applyFilter());
    // its keys, wherever the focus is while it is open (a click on its text leaves the focus on the page)
    window.addEventListener('keydown', (e) => {
      if (this.backdrop.hidden || document.querySelector('.dialog-backdrop')) return;
      const t = e.target as HTMLElement;
      if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        if (t.matches('.g-name-edit, .g-desc-edit')) t.blur();
        else this.close();
      } else if (e.key === 'Enter' && this.chosen && !t.closest('button, textarea, input')) {
        e.preventDefault();
        this.act('open');
      }
    }, { capture: true });
    // the preview: drag to move it (any button), the wheel zooms about the mouse, double-click fits it again
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
    const box = q<HTMLInputElement>('.g-labels input');
    box.checked = this.labels;
    box.addEventListener('change', () => {
      this.labels = box.checked;
      localStorage.setItem('linkage-lab:gallery-labels', this.labels ? 'on' : 'off');
      this.paintPreview();
    });
    window.addEventListener('resize', () => {
      if (!this.backdrop.hidden && this.chosen) this.fitPreview();
    });
  }

  get isOpen() {
    return !this.backdrop.hidden;
  }

  open() {
    this.backdrop.hidden = false;
    this.filter.value = '';
    this.refresh(this.chosen);
    this.filter.focus();
  }

  close() {
    this.backdrop.hidden = true;
    this.drag = null;
  }

  private item(key: string | null) {
    return key ? this.items.find((x) => x.key === key) : undefined;
  }

  private designOf(it: GalleryItem): Design {
    let d = this.designs.get(it.key);
    if (!d) this.designs.set(it.key, (d = it.design()));
    return d;
  }

  /** Build the cards again (own projects first, after the card that saves the current one), and show `choose`. */
  private refresh(choose: string | null = null) {
    this.items = this.hooks.items();
    const groups = [MY_PROJECTS, ...new Set(this.items.filter((x) => x.group !== MY_PROJECTS).map((x) => x.group))];
    const card = (it: GalleryItem) => `<button class="g-card" data-key="${esc(it.key)}" title="${esc(it.name)}"><canvas></canvas><span>${esc(it.name)}</span></button>`;
    this.grid.innerHTML = groups
      .map((g) => {
        const own = g === MY_PROJECTS;
        const cards = this.items.filter((x) => x.group === g).map(card).join('');
        const save = own ? `<button class="g-card g-save" title="Save the current drawing, its settings and view here, in this browser"><span class="g-plus">＋</span><span>Save current project</span></button>` : '';
        return `<section class="g-section${own ? ' g-own-section' : ''}" data-group="${esc(g)}"><h3>${esc(g)}</h3><div class="g-cards">${save}${cards}</div></section>`;
      })
      .join('');
    this.grid.querySelectorAll<HTMLButtonElement>('.g-card[data-key]').forEach((c) => {
      const key = c.dataset.key!;
      c.addEventListener('click', () => this.choose(key));
      c.addEventListener('dblclick', () => {
        this.choose(key);
        this.act('open');
      });
    });
    this.grid.querySelector('.g-save')!.addEventListener('click', () => {
      const key = this.hooks.saveCurrent();
      if (!key) return;
      this.refresh(key);
      // straight to naming it
      requestAnimationFrame(() => {
        const name = this.backdrop.querySelector<HTMLInputElement>('.g-name-edit')!;
        name.focus();
        name.select();
      });
    });
    this.applyFilter();
    if (choose && this.item(choose)) this.choose(choose);
    else {
      this.chosen = null;
      this.detail.classList.add('empty'); // (it stays in place: the cards must not move under a double-click)
    }
  }

  private paintCards() {
    this.grid.querySelectorAll<HTMLButtonElement>('.g-card[data-key]').forEach((card) => {
      const cv = card.querySelector('canvas')!;
      if (cv.dataset.drawn === String(cv.clientWidth) || !cv.clientWidth) return;
      const it = this.item(card.dataset.key!);
      if (!it) return;
      const d = this.designOf(it);
      paint(cv, d, fitDesign(d, cv.clientWidth, cv.clientHeight, 8), false, 'none'); // a card: just the mechanism
      cv.dataset.drawn = String(cv.clientWidth);
    });
  }

  /** Show only what has every word typed in its name, group or description (the save card always). */
  private applyFilter() {
    const words = this.filter.value.toLowerCase().split(/\s+/).filter(Boolean);
    this.grid.querySelectorAll<HTMLElement>('.g-section').forEach((sec) => {
      let any = sec.classList.contains('g-own-section');
      sec.querySelectorAll<HTMLElement>('.g-card[data-key]').forEach((card) => {
        const it = this.item(card.dataset.key!);
        const text = `${it?.name} ${it?.group} ${it?.description}`.toLowerCase();
        const show = words.every((w) => text.includes(w));
        card.hidden = !show;
        any ||= show;
      });
      sec.hidden = !any;
    });
    requestAnimationFrame(() => this.paintCards());
  }

  private choose(key: string) {
    const it = this.item(key);
    if (!it) return;
    this.chosen = key;
    this.grid.querySelectorAll<HTMLElement>('.g-card').forEach((c) => c.classList.toggle('on', c.dataset.key === key));
    const d = this.designOf(it);
    const q = <T extends Element>(sel: string) => this.detail.querySelector<T>(sel)!;
    this.detail.classList.remove('empty');
    q<HTMLElement>('.g-name').hidden = !!it.editable;
    q<HTMLElement>('.g-desc').hidden = !!it.editable;
    q<HTMLInputElement>('.g-name-edit').hidden = !it.editable;
    q<HTMLTextAreaElement>('.g-desc-edit').hidden = !it.editable;
    q<HTMLElement>('.g-own').hidden = !it.editable;
    q('.g-name').textContent = it.name;
    q('.g-desc').textContent = it.description;
    q<HTMLInputElement>('.g-name-edit').value = it.name;
    q<HTMLTextAreaElement>('.g-desc-edit').value = it.description;
    q('.g-notes').textContent = it.notes;
    q('.g-makeup').textContent = makeup(d);
    const open = q<HTMLButtonElement>('.g-open');
    open.textContent = it.editable ? 'Open project' : 'Open';
    open.title = it.editable ? 'Open the whole project: its drawing, settings and view, in place of yours' : 'Open this example in place of your drawing';
    requestAnimationFrame(() => this.fitPreview());
  }

  private fitPreview() {
    const it = this.item(this.chosen);
    if (!it) return;
    this.view = fitDesign(this.designOf(it), this.preview.clientWidth, this.preview.clientHeight, 20);
    this.paintPreview();
  }

  private paintPreview() {
    const it = this.item(this.chosen);
    if (it) paint(this.preview, this.designOf(it), this.view, this.labels, 'left'); // clear of the labels box
  }

  private act(what: 'open' | 'import') {
    const key = this.chosen;
    if (!key) return;
    this.close();
    if (what === 'open') this.hooks.open(key);
    else this.hooks.import(key);
  }
}
