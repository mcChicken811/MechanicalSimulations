/**
 * A search list of commands, opened at the mouse (a right-click on the drawing): its search box
 * has the keyboard at once, so typing narrows the list; ↑ ↓ move, Enter runs, Esc closes, and a
 * click runs an entry. Every word typed must appear in an entry (its name, group or keywords);
 * entries whose name starts with the words, or has them at word starts, come first.
 *
 * Three kinds of entry:
 *  - an action (a tool, Play, an example): runs, and the list closes;
 *  - a toggle (a checkbox, a switch's option): flips, and the list stays open, redrawn as things
 *    now are (ticking one may bring others: Friction brings its sliders);
 *  - a value (a slider, a number box): edited right in its row, a slider or number box of its own
 *    (dragged sideways like every number box); Enter on it gives that control the keyboard.
 */
export interface Command {
  label: string;
  /** Where it is from, shown dimmed and searched too: 'Tool', 'Example', 'Setting', … */
  group: string;
  /** A shortcut key, or the current value of a setting, shown on the right. */
  hint?: string;
  /** More words to find it by (its tooltip). */
  keywords?: string;
  /** A check mark: a setting that is on, the tool in use. */
  on?: boolean;
  /** A toggle: running it keeps the list open (else it closes). */
  toggle?: boolean;
  run?: () => void;
  /** A value edited in its row: `set` with done false while a slider moves, true when it is let go. */
  value?: { type: 'range' | 'number'; min: string; max: string; step: string; get: () => string; set: (v: string, done: boolean) => void; shown?: () => string; unit?: string };
}

let rowIds = 0;

export class Palette {
  readonly el: HTMLDivElement;
  private readonly input: HTMLInputElement;
  private readonly list: HTMLDivElement;
  private source: () => Command[] = () => [];
  private items: Command[] = [];
  /** The press that last closed it (so that press does nothing else, as a click outside a menu). */
  private dismissal: Event | null = null;
  private shown: Command[] = [];
  private active = 0;

  constructor() {
    this.el = document.createElement('div');
    this.el.id = 'palette';
    this.el.className = 'panel';
    this.el.hidden = true;
    this.el.innerHTML = `<input type="text" placeholder="Search tools, settings, examples…" spellcheck="false" autocomplete="off" /><div class="p-list" role="listbox"></div>`;
    document.body.append(this.el);
    this.input = this.el.querySelector('input')!;
    this.list = this.el.querySelector('.p-list')!;
    this.input.addEventListener('input', () => {
      this.active = 0;
      this.render();
    });
    this.input.addEventListener('keydown', (e) => {
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        const n = this.shown.length;
        if (n) this.active = (this.active + (e.key === 'ArrowDown' ? 1 : n - 1)) % n;
        this.highlight();
      } else if (e.key === 'Enter') {
        e.preventDefault();
        this.runAt(this.active);
      } else if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        this.close();
      }
    });
    this.list.addEventListener('pointermove', (e) => {
      const row = (e.target as HTMLElement).closest<HTMLElement>('.p-item');
      if (row && Number(row.dataset.i) !== this.active) {
        this.active = Number(row.dataset.i);
        this.highlight(false);
      }
    });
    this.list.addEventListener('click', (e) => {
      const t = e.target as HTMLElement;
      if (t.closest('input')) return; // working a row's own slider or number box
      const row = t.closest<HTMLElement>('.p-item');
      if (row) this.runAt(Number(row.dataset.i));
    });
    // a press anywhere else closes it
    window.addEventListener('pointerdown', (e) => {
      if (!this.el.hidden && !this.el.contains(e.target as Node)) {
        this.dismissal = e;
        this.close();
      }
    }, { capture: true });
  }

  get isOpen() {
    return !this.el.hidden;
  }

  /** Whether this press closed the list (and should do nothing more). */
  dismissedBy(e: Event) {
    return this.dismissal === e;
  }

  /** Open at (x, y) (viewport px) with the commands `source` gives (asked again after a toggle), the search box empty and ready for typing. */
  open(x: number, y: number, source: () => Command[]) {
    this.source = source;
    this.items = source();
    this.input.value = '';
    this.active = 0;
    this.el.hidden = false;
    this.render();
    // beside the mouse, inside the window
    const { width, height } = this.el.getBoundingClientRect();
    this.el.style.left = `${Math.max(8, Math.min(x, window.innerWidth - width - 8))}px`;
    this.el.style.top = `${Math.max(8, Math.min(y, window.innerHeight - height - 8))}px`;
    this.input.focus();
  }

  close() {
    if (this.el.hidden) return;
    this.el.hidden = true;
    (document.activeElement as HTMLElement | null)?.blur();
  }

  private runAt(i: number) {
    const c = this.shown[i];
    if (!c) return;
    if (c.value) {
      // a value: its own control takes the keyboard
      const ctl = this.list.querySelector<HTMLInputElement>(`.p-item[data-i="${i}"] input`);
      ctl?.focus();
      if (ctl?.type === 'number') ctl.select();
      return;
    }
    if (c.toggle) {
      // flip it, and stay: the list as things now are, the same entry chosen
      c.run?.();
      const label = c.label;
      this.items = this.source();
      this.render(false);
      const k = this.shown.findIndex((x) => x.label === label);
      if (k >= 0) this.active = k;
      this.highlight();
      this.input.focus();
      return;
    }
    this.close();
    c.run?.();
  }

  /** The commands matching the search, best first. */
  private match(): Command[] {
    const words = this.input.value.toLowerCase().split(/\s+/).filter(Boolean);
    if (!words.length) return this.items;
    const scored: { c: Command; s: number; k: number }[] = [];
    this.items.forEach((c, k) => {
      const label = c.label.toLowerCase();
      const all = `${label} ${c.group.toLowerCase()} ${(c.keywords ?? '').toLowerCase()}`;
      let s = 0;
      for (const w of words) {
        if (!all.includes(w)) return;
        s += label.startsWith(w) ? 4 : new RegExp(`(^|[^a-z0-9])${w.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}`).test(label) ? 3 : label.includes(w) ? 2 : c.group.toLowerCase().includes(w) ? 1 : 0.5;
      }
      scored.push({ c, s, k });
    });
    return scored.sort((a, b) => b.s - a.s || a.k - b.k).map((x) => x.c);
  }

  private render(resetScroll = true) {
    const scroll = this.list.scrollTop;
    this.shown = this.match();
    const esc = (s: string) => s.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);
    const ids: string[] = [];
    this.list.innerHTML = this.shown.length
      ? this.shown
          .map((c, i) => {
            const head = `<span class="p-check">${c.on ? '✓' : ''}</span><span class="p-label">${esc(c.label)}</span>`;
            if (!c.value) return `<div class="p-item" role="option" data-i="${i}">${head}<span class="p-group">${esc(c.group)}</span>${c.hint ? `<kbd>${esc(c.hint)}</kbd>` : ''}</div>`;
            const v = c.value;
            const id = `pal-v-${++rowIds}`; // number boxes are dragged by id (ui/scrub.ts)
            ids.push(id);
            const ctl = `<input id="${id}" type="${v.type}" min="${esc(v.min)}" max="${esc(v.max)}" step="${esc(v.step)}" value="${esc(v.get())}">`;
            return `<div class="p-item p-value" role="option" data-i="${i}">${head}${ctl}<span class="p-shown">${esc(v.shown?.() ?? v.unit ?? '')}</span></div>`;
          })
          .join('')
      : '<div class="p-none">Nothing matches</div>';
    // a row's own control drives the setting, as the side panel's does
    this.shown.forEach((c, i) => {
      if (!c.value) return;
      const v = c.value;
      const ctl = this.list.querySelector<HTMLInputElement>(`.p-item[data-i="${i}"] input`)!;
      const shown = this.list.querySelector<HTMLElement>(`.p-item[data-i="${i}"] .p-shown`)!;
      const update = (done: boolean) => {
        v.set(ctl.value, done);
        if (v.shown) shown.textContent = v.shown();
      };
      if (v.type === 'range') ctl.addEventListener('input', () => update(false));
      ctl.addEventListener('change', () => update(true));
      ctl.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') {
          e.preventDefault();
          e.stopPropagation();
          this.close();
        } else if (e.key === 'Enter') {
          // done typing: set it, and back to searching
          e.preventDefault();
          update(true);
          this.input.focus();
        } else if ((e.key === 'ArrowDown' || e.key === 'ArrowUp') && v.type === 'range') {
          // up and down move through the list from a slider too (left and right move it)
          e.preventDefault();
          this.input.focus();
          this.input.dispatchEvent(new KeyboardEvent('keydown', { key: e.key }));
        }
      });
    });
    if (!resetScroll) this.list.scrollTop = scroll;
    this.highlight(resetScroll);
  }

  private highlight(scroll = true) {
    this.list.querySelectorAll<HTMLElement>('.p-item').forEach((row, i) => {
      row.classList.toggle('on', i === this.active);
      if (i === this.active && scroll) row.scrollIntoView({ block: 'nearest' });
    });
  }
}
