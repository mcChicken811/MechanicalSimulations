/**
 * Background edit mode: building a custom background from a scene and images at distances.
 *
 * A panel at the top lists the base scene and the image layers, each with its distance from the
 * viewer (1: moves with the drawing; 2: half as far as the view pans; ∞: stays put), a button to
 * fill the screen with it, its order and delete; "Add image…" places a new one across the middle
 * of the view. On the drawing, drag an image to move it and its corner handle to resize it (its
 * shape kept); right-drag pans the view as ever, so the depths can be seen working. Changing a
 * layer's distance keeps it where it looks now. Esc or Done leaves the mode; Delete removes the
 * chosen image.
 */
import { imageFileToDataUrl, layerRect, placeLayer, SCENES, type BackgroundState, type ImageLayer } from './background';
import type { Vec, View } from './render';

export interface EditorHooks {
  view: () => View;
  /** The drawing's size (CSS px). */
  size: () => [number, number];
  state: () => BackgroundState;
  /** Show (and keep, unless `passing`) a changed background. */
  apply: (s: BackgroundState, passing?: boolean) => void;
  notify: (text: string, kind?: 'info' | 'warn' | 'error') => void;
  /** The mode ended. */
  closed: () => void;
}

const HANDLE = 12; // px: the resize handle at an image's bottom-right corner
const esc = (s: string) => s.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);

export class BackgroundEditor {
  readonly panel: HTMLDivElement;
  active = false;
  private selected: string | null = null;
  private drag: { mode: 'move' | 'size'; id: string; at: Vec; r0: { x: number; y: number; w: number }; moved: boolean } | null = null;

  constructor(private readonly hooks: EditorHooks) {
    this.panel = document.createElement('div');
    this.panel.id = 'bg-editor';
    this.panel.className = 'panel';
    this.panel.hidden = true;
    document.body.append(this.panel);
  }

  open() {
    const s = this.hooks.state();
    if (s.kind !== 'custom') this.hooks.apply({ ...s, kind: 'custom', base: s.kind !== 'none' ? s.kind : s.base }); // build on the scene shown
    this.active = true;
    this.panel.hidden = false;
    this.render();
  }

  close() {
    if (!this.active) return;
    this.active = false;
    this.panel.hidden = true;
    this.drag = null;
    this.hooks.closed();
  }

  private change(f: (s: BackgroundState) => void, passing = false) {
    const s = structuredClone(this.hooks.state());
    f(s);
    this.hooks.apply(s, passing);
  }

  private layer(s: BackgroundState, id: string | null) {
    return s.layers.find((l) => l.id === id);
  }

  /** The panel: the base scene, the layers (nearest first), add an image, done. */
  private render() {
    const s = this.hooks.state();
    const opts = [['none', 'None (plain)'], ...SCENES.map((x) => [x.id, `${x.name}${x.layers > 1 ? ` (${x.layers} layers deep)` : ''}`])];
    this.panel.innerHTML = `
      <header><b>Background editor</b><span class="muted">Drag an image to move it, its corner to resize it. Right-drag to pan and see the depths.</span><button class="primary be-done">Done</button></header>
      <div class="be-row"><label>Base scene <select class="be-base">${opts.map(([id, name]) => `<option value="${id}" ${id === s.base ? 'selected' : ''}>${esc(name)}</option>`).join('')}</select></label><button class="be-add">Add image…</button></div>
      <div class="be-layers">${
        s.layers.length
          ? s.layers
              .map(
                (l, i) => `<div class="be-layer${l.id === this.selected ? ' on' : ''}" data-id="${l.id}">
            <img src="${l.image}" alt="">
            <span class="be-name" title="${esc(l.name)}">${esc(l.name)}</span>
            <label title="Its distance from the viewer: 1 moves with the drawing, 2 half as far, and so on">Distance <input type="number" class="be-depth" min="1" step="0.5" value="${l.depth ?? ''}" ${l.depth === null ? 'disabled' : ''}></label>
            <label title="Infinitely far: it stays put on the screen"><input type="checkbox" class="be-far" ${l.depth === null ? 'checked' : ''}> ∞</label>
            <button class="be-fill" title="Fill the screen with it (and make it infinitely far)">Fill</button>
            <button class="be-up" title="Draw it over the one below (among layers at the same distance)" ${i === 0 ? 'disabled' : ''}>▲</button>
            <button class="be-down" title="Draw it under the one below" ${i === s.layers.length - 1 ? 'disabled' : ''}>▼</button>
            <button class="be-del" title="Remove it (Delete)">×</button>
          </div>`,
              )
              .join('')
          : '<p class="muted be-empty">No images yet: add one (PNG, JPEG, SVG…), then drag it into place and set how far away it is.</p>'
      }</div>`;
    const q = <T extends Element>(sel: string) => this.panel.querySelector<T>(sel)!;
    q('.be-done').addEventListener('click', () => this.close());
    q<HTMLSelectElement>('.be-base').addEventListener('change', (e) => {
      this.change((st) => (st.base = (e.target as HTMLSelectElement).value));
    });
    q('.be-add').addEventListener('click', () => void this.addImage());
    this.panel.querySelectorAll<HTMLElement>('.be-layer').forEach((row) => {
      const id = row.dataset.id!;
      row.addEventListener('pointerdown', (e) => {
        if ((e.target as HTMLElement).closest('input, button')) return;
        this.selected = id;
        this.render();
      });
      const [w, h] = this.hooks.size();
      // a new distance: it stays where it looks now
      const setDepth = (depth: number | null) =>
        this.change((st) => {
          const l = this.layer(st, id)!;
          const r = layerRect(l, this.hooks.view(), w, h);
          l.depth = depth;
          placeLayer(l, r, this.hooks.view(), w, h);
        });
      row.querySelector<HTMLInputElement>('.be-depth')!.addEventListener('change', (e) => {
        const v = parseFloat((e.target as HTMLInputElement).value);
        if (Number.isFinite(v) && v >= 1) setDepth(v);
        this.render();
      });
      row.querySelector<HTMLInputElement>('.be-far')!.addEventListener('change', (e) => {
        setDepth((e.target as HTMLInputElement).checked ? null : 2);
        this.render();
      });
      row.querySelector('.be-fill')!.addEventListener('click', () => {
        this.change((st) => {
          const l = this.layer(st, id)!;
          const cover = Math.max(w, h / l.aspect);
          l.depth = null;
          placeLayer(l, { x: (w - cover) / 2, y: (h - cover * l.aspect) / 2, w: cover }, this.hooks.view(), w, h);
        });
        this.render();
      });
      const move = (d: number) => {
        this.change((st) => {
          const i = st.layers.findIndex((l) => l.id === id);
          const j = i + d;
          if (j < 0 || j >= st.layers.length) return;
          [st.layers[i], st.layers[j]] = [st.layers[j], st.layers[i]];
        });
        this.render();
      };
      row.querySelector('.be-up')!.addEventListener('click', () => move(-1));
      row.querySelector('.be-down')!.addEventListener('click', () => move(1));
      row.querySelector('.be-del')!.addEventListener('click', () => this.remove(id));
    });
  }

  private remove(id: string) {
    this.change((st) => (st.layers = st.layers.filter((l) => l.id !== id)));
    if (this.selected === id) this.selected = null;
    this.render();
  }

  /** Ask for an image and add it, across the middle of the view, moving with the drawing (distance 1). */
  private async addImage() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'image/*,.svg';
    const file = await new Promise<File | null>((res) => {
      input.addEventListener('change', () => res(input.files?.[0] ?? null));
      input.addEventListener('cancel', () => res(null));
      input.click();
    });
    if (!file) return;
    try {
      const { url, aspect } = await imageFileToDataUrl(file);
      const [w, h] = this.hooks.size();
      const iw = Math.min(w * 0.6, (h * 0.6) / aspect);
      const l: ImageLayer = { id: `${Date.now().toString(36)}${Math.random().toString(36).slice(2, 6)}`, name: file.name, image: url, aspect, depth: 1, ref: { cx: 0, cy: 0, s: 1 }, rect: { x: 0, y: 0, w: 1 } };
      placeLayer(l, { x: (w - iw) / 2, y: (h - iw * aspect) / 2, w: iw }, this.hooks.view(), w, h);
      this.change((st) => st.layers.unshift(l)); // nearest of its distance: on top
      this.selected = l.id;
      this.render();
    } catch (e) {
      this.hooks.notify(`${file.name}: ${e instanceof Error ? e.message : String(e)}`, 'error');
    }
  }

  /** Which layer's handle or body is at screen point sp (the nearest drawn on top first). */
  private hit(sp: Vec): { id: string; mode: 'move' | 'size' } | null {
    const s = this.hooks.state();
    const [w, h] = this.hooks.size();
    const v = this.hooks.view();
    const sel = this.layer(s, this.selected);
    if (sel) {
      const r = layerRect(sel, v, w, h);
      if (Math.abs(sp[0] - (r.x + r.w)) <= HANDLE && Math.abs(sp[1] - (r.y + r.h)) <= HANDLE) return { id: sel.id, mode: 'size' };
    }
    // nearest first: smaller distance, then earlier in the list
    const order = s.layers.map((l, k) => ({ l, k })).sort((a, b) => (a.l.depth ?? Infinity) - (b.l.depth ?? Infinity) || a.k - b.k);
    for (const { l } of order) {
      const r = layerRect(l, v, w, h);
      if (sp[0] >= r.x && sp[0] <= r.x + r.w && sp[1] >= r.y && sp[1] <= r.y + r.h) return { id: l.id, mode: 'move' };
    }
    return null;
  }

  /** A left press on the drawing while editing: choose an image, and start moving or resizing it. */
  pointerDown(sp: Vec) {
    const hit = this.hit(sp);
    this.selected = hit?.id ?? null;
    if (hit) {
      const l = this.layer(this.hooks.state(), hit.id)!;
      const [w, h] = this.hooks.size();
      const r = layerRect(l, this.hooks.view(), w, h);
      this.drag = { mode: hit.mode, id: hit.id, at: sp, r0: { x: r.x, y: r.y, w: r.w }, moved: false };
    }
    this.render();
  }

  /** The mouse moved: carry on a move or resize (shown at once, kept when let go). Whether it was one. */
  pointerMove(sp: Vec): boolean {
    const d = this.drag;
    if (!d) return false;
    const [dx, dy] = [sp[0] - d.at[0], sp[1] - d.at[1]];
    const [w, h] = this.hooks.size();
    d.moved = true;
    this.change((st) => {
      const l = this.layer(st, d.id);
      if (!l) return;
      const r = d.mode === 'move' ? { x: d.r0.x + dx, y: d.r0.y + dy, w: d.r0.w } : { x: d.r0.x, y: d.r0.y, w: Math.max(12, d.r0.w + Math.max(dx, dy / l.aspect)) };
      placeLayer(l, r, this.hooks.view(), w, h);
    }, true);
    return true;
  }

  pointerUp() {
    const d = this.drag;
    this.drag = null;
    if (d?.moved) this.change(() => {}); // keep where it was put
  }

  /** Keys while editing: Esc leaves, Delete removes the chosen image. Whether it was taken. */
  key(e: KeyboardEvent): boolean {
    if (e.key === 'Escape') {
      this.close();
      return true;
    }
    if ((e.key === 'Delete' || e.key === 'Backspace') && this.selected) {
      this.remove(this.selected);
      return true;
    }
    return false;
  }

  /** Over everything: each image's outline, the chosen one's firm with its resize handle. */
  drawOverlay(ctx: CanvasRenderingContext2D, w: number, h: number, v: View) {
    if (!this.active) return;
    const s = this.hooks.state();
    ctx.save();
    for (const l of s.layers) {
      const r = layerRect(l, v, w, h);
      const on = l.id === this.selected;
      ctx.strokeStyle = on ? '#e07b39' : 'rgba(59,110,168,0.7)';
      ctx.lineWidth = on ? 2 : 1;
      ctx.setLineDash(on ? [] : [5, 4]);
      ctx.strokeRect(r.x, r.y, r.w, r.h);
      ctx.setLineDash([]);
      ctx.font = '600 11px system-ui, sans-serif';
      ctx.fillStyle = on ? '#e07b39' : 'rgba(59,110,168,0.9)';
      ctx.fillText(`${l.name} · ${l.depth === null ? '∞' : `distance ${l.depth}`}`, r.x + 4, r.y - 5);
      if (on) {
        ctx.fillStyle = '#fff';
        ctx.fillRect(r.x + r.w - HANDLE / 2, r.y + r.h - HANDLE / 2, HANDLE, HANDLE);
        ctx.strokeRect(r.x + r.w - HANDLE / 2, r.y + r.h - HANDLE / 2, HANDLE, HANDLE);
      }
    }
    ctx.restore();
  }
}
