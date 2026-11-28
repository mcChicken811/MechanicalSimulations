/**
 * A panel that slides away off its edge of the screen when the mouse has not been over it for a
 * while (3 s), and slides back when the mouse comes to its part of that edge.
 *
 *  - It stays while in use: the mouse over it, a press that began on it still held (a slider or
 *    number box being dragged), or the keyboard in one of its boxes or menus.
 *  - Hidden, a thin tab at the edge marks where it went; the mouse within `reach` px of the edge,
 *    in the panel's part of it, brings it back (as does `show()`, for news: a new message, a part
 *    picked to inspect), and it goes again 3 s after the mouse leaves (or never comes over it).
 *
 * Its part of the edge: 'start' runs from the top (or left) down to the panel's far side, or 40 %
 * of the screen if that is more; 'end' the same from the bottom (or right); 'span' just alongside
 * the panel. Two panels on one edge (the equations at the top right, the inspector at the bottom
 * right) so each have their own.
 */
export interface AutoHide {
  /** Bring it back now, and away again 3 s after (unless the mouse comes over it). */
  show(): void;
  readonly hidden: boolean;
}

type Side = 'left' | 'right' | 'top' | 'bottom';

export function autoHide(el: HTMLElement, side: Side, opts: { delay?: number; reach?: number; part?: 'start' | 'end' | 'span' } = {}): AutoHide {
  const delay = opts.delay ?? 3000;
  const reach = opts.reach ?? 24;
  const vertical = side === 'left' || side === 'right'; // the edge runs up and down
  const part = opts.part ?? (vertical ? 'start' : 'span');
  el.classList.add('slides');
  const tab = document.createElement('div');
  tab.className = `edge-tab ${side}`;
  tab.title = 'Move the mouse here to bring the panel back';
  document.body.append(tab);

  let hidden = false;
  let over = false;
  let pressed = false;
  let timer = 0;
  let pointer: [number, number] | null = null;

  /** Where the panel is when shown (its rectangle less the slide it may be part-way through). */
  const home = () => {
    const r = el.getBoundingClientRect();
    const t = getComputedStyle(el).transform;
    const m = t && t !== 'none' ? new DOMMatrixReadOnly(t) : null;
    const [dx, dy] = m ? [m.m41, m.m42] : [0, 0];
    return { left: r.left - dx, right: r.right - dx, top: r.top - dy, bottom: r.bottom - dy };
  };
  /** Whether (x, y) is at the panel's part of its edge. */
  const inZone = (x: number, y: number) => {
    const [W, H] = [window.innerWidth, window.innerHeight];
    const atEdge = side === 'left' ? x <= reach : side === 'right' ? x >= W - reach : side === 'top' ? y <= reach : y >= H - reach;
    if (!atEdge) return false;
    const r = home();
    const [along, lo, hi, size] = vertical ? [y, r.top, r.bottom, H] : [x, r.left, r.right, W];
    if (part === 'start') return along <= Math.max(hi + 24, size * 0.4);
    if (part === 'end') return along >= Math.min(lo - 24, size * 0.6);
    return along >= lo - 40 && along <= hi + 40;
  };
  /** The tab where it went: at the top or bottom of a side edge, or under or over it on the top or bottom edge. */
  const placeTab = () => {
    const r = home();
    tab.style.top = tab.style.bottom = tab.style.left = tab.style.right = '';
    if (vertical) {
      if (part === 'end') [tab.style.top, tab.style.bottom] = ['auto', '12px'];
      else tab.style.top = `${part === 'span' ? Math.max(12, r.top) : 12}px`;
    } else tab.style.left = `${(r.left + r.right) / 2 - 36}px`;
  };
  const busy = () => {
    const a = document.activeElement;
    const typing = !!a && el.contains(a) && a.matches('input:not([type="checkbox"]):not([type="range"]), select, textarea');
    return over || pressed || typing || (!!pointer && inZone(...pointer)) || document.body.classList.contains('scrubbing');
  };
  const arm = () => {
    clearTimeout(timer);
    timer = window.setTimeout(() => {
      if (busy()) arm();
      else hide();
    }, delay);
  };
  const hide = () => {
    if (hidden) return;
    placeTab();
    hidden = true;
    el.classList.add(`away-${side}`);
    tab.classList.add('shown');
  };
  const show = () => {
    if (hidden) {
      hidden = false;
      el.classList.remove(`away-${side}`);
      tab.classList.remove('shown');
    }
    if (!over) arm();
  };

  el.addEventListener('pointerenter', () => {
    over = true;
    clearTimeout(timer);
  });
  el.addEventListener('pointerleave', () => {
    over = false;
    arm();
  });
  el.addEventListener('pointerdown', () => (pressed = true));
  window.addEventListener('pointerup', () => {
    if (!pressed) return;
    pressed = false;
    if (!over) arm();
  });
  el.addEventListener('focusout', () => arm());
  tab.addEventListener('pointerenter', show);
  window.addEventListener('pointermove', (e) => {
    pointer = [e.clientX, e.clientY];
    if (hidden && inZone(e.clientX, e.clientY)) show();
  });
  document.addEventListener('pointerleave', () => (pointer = null));
  arm(); // at first: there, then away unless the mouse comes to it

  return {
    show,
    get hidden() {
      return hidden;
    },
  };
}
