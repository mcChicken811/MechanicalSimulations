/**
 * A panel that slides away off its edge of the screen when the mouse has not been over it for a
 * while (3 s), and slides back when the mouse comes to the top part of that edge.
 *
 *  - It stays while in use: the mouse over it, a press that began on it still held (a slider or
 *    number box being dragged), or the keyboard in one of its boxes or menus.
 *  - Hidden, a thin tab at the edge marks where it went; the mouse within `reach` px of the edge,
 *    anywhere from the top down to the panel's own height (at least 40 % of the screen), brings
 *    it back, and it goes again 3 s after the mouse leaves (or never comes over it).
 */
export interface AutoHide {
  /** Bring it back now (as the mouse at the edge does). */
  show(): void;
  readonly hidden: boolean;
}

export function autoHide(el: HTMLElement, side: 'left' | 'right', opts: { delay?: number; reach?: number } = {}): AutoHide {
  const delay = opts.delay ?? 3000;
  const reach = opts.reach ?? 24;
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

  /** The part of the edge that brings it back: from the top down to its height, or 40 % of the screen. */
  const inZone = (x: number, y: number) => {
    const edge = side === 'left' ? x <= reach : x >= window.innerWidth - reach;
    const bottom = Math.max(el.getBoundingClientRect().height + 24, window.innerHeight * 0.4);
    return edge && y <= bottom;
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
    arm();
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
