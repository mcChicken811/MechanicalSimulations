/**
 * Number boxes you can drag, as in Blender: press on a number box and drag sideways to change
 * its value continuously; a click without a drag opens it for typing as usual.
 *  - by default the value moves in steps of 0.01, 0.01 per pixel;
 *  - holding Alt (Option on a Mac) shows the third decimal and moves 10× slower (0.001 per pixel);
 *  - holding Shift rounds to one decimal and moves 10× faster (0.1 per pixel).
 * A whole-number box (step 1) moves by whole numbers: 1 per 10 px, per 100 px with Alt, per px
 * with Shift. The box's min and max hold. Esc during a drag puts the value back.
 *
 * Each change is reported as the box's own 'change' event, so whatever the page does on a typed
 * change happens as you drag. The page may rebuild the box meanwhile (an inspector re-rendering):
 * the drag carries on with the new box of the same id.
 */
const START_PX = 3; // a press becomes a drag after this much sideways movement

interface Scrub {
  id: string;
  root: HTMLElement;
  startX: number;
  lastX: number;
  start: string;
  raw: number;
  integer: boolean;
  dragging: boolean;
  pending: number | null;
}

export interface ScrubHooks {
  /** A drag starts (one undo step for the whole drag). */
  begin?: () => void;
  /** It ends: `changed` if the value moved. */
  end?: (changed: boolean) => void;
}

/** The precision (decimal places) and change per pixel for the modifiers held. */
function mode(e: { altKey: boolean; shiftKey: boolean }, integer: boolean): { digits: number; perPx: number } {
  if (integer) return { digits: 0, perPx: e.shiftKey ? 1 : e.altKey ? 0.01 : 0.1 };
  return e.shiftKey ? { digits: 1, perPx: 0.1 } : e.altKey ? { digits: 3, perPx: 0.001 } : { digits: 2, perPx: 0.01 };
}

/** Make every number box under `root` draggable. */
export function enableScrubbing(root: HTMLElement, hooks: ScrubHooks = {}) {
  let s: Scrub | null = null;
  const box = () => (s ? (s.root.querySelector<HTMLInputElement>(`#${CSS.escape(s.id)}`) ?? null) : null);

  const show = (e: { altKey: boolean; shiftKey: boolean }) => {
    const el = box();
    if (!s || !el) return;
    const { digits } = mode(e, s.integer);
    const step = 10 ** -digits;
    let v = Math.round(s.raw / step) * step;
    const [min, max] = [parseFloat(el.min), parseFloat(el.max)];
    if (Number.isFinite(min)) v = Math.max(min, v);
    if (Number.isFinite(max)) v = Math.min(max, v);
    const text = v.toFixed(digits);
    if (el.value === text) return;
    el.value = text;
    // report it once a frame at most (a change may rebuild the inspector or the simulation)
    if (s.pending === null)
      s.pending = requestAnimationFrame(() => {
        if (!s) return;
        s.pending = null;
        box()?.dispatchEvent(new Event('change', { bubbles: true }));
      });
  };

  root.addEventListener('pointerdown', (e) => {
    const el = e.target as HTMLElement;
    if (!(el instanceof HTMLInputElement) || el.type !== 'number' || el.disabled || !el.id) return;
    if (document.activeElement === el || e.button !== 0) return; // typing in it: leave the text alone
    e.preventDefault(); // no focus, no spinner: decide on release whether it was a click
    const step = parseFloat(el.step);
    s = {
      id: el.id,
      root,
      startX: e.clientX,
      lastX: e.clientX,
      start: el.value,
      raw: parseFloat(el.value) || 0,
      integer: Number.isFinite(step) && step >= 1 && Number.isInteger(step),
      dragging: false,
      pending: null,
    };
  });

  window.addEventListener('pointermove', (e) => {
    if (!s) return;
    if (!s.dragging) {
      if (Math.abs(e.clientX - s.startX) < START_PX) return;
      s.dragging = true; // counted from where it was pressed: the whole movement
      document.body.classList.add('scrubbing');
      hooks.begin?.();
    }
    s.raw += (e.clientX - s.lastX) * mode(e, s.integer).perPx;
    s.lastX = e.clientX;
    show(e);
  });

  const finish = (e: PointerEvent | null) => {
    if (!s) return;
    const done = s;
    if (!done.dragging) {
      // a click: type into it
      s = null;
      const el = done.root.querySelector<HTMLInputElement>(`#${CSS.escape(done.id)}`);
      el?.focus();
      el?.select();
      return;
    }
    if (done.pending !== null) cancelAnimationFrame(done.pending);
    const el = box();
    const changed = !!el && el.value !== done.start;
    if (el) el.dispatchEvent(new Event('change', { bubbles: true })); // the final value
    s = null;
    document.body.classList.remove('scrubbing');
    hooks.end?.(changed);
    void e;
  };
  window.addEventListener('pointerup', finish);
  window.addEventListener('pointercancel', finish);

  // modifiers pressed or let go mid-drag: the value re-rounds at once (Alt shows the third decimal)
  for (const ev of ['keydown', 'keyup'] as const)
    window.addEventListener(ev, (e) => {
      if (!s?.dragging) return;
      if (e.key === 'Escape') {
        // put it back
        const el = box();
        if (s.pending !== null) cancelAnimationFrame(s.pending);
        if (el) {
          el.value = s.start;
          el.dispatchEvent(new Event('change', { bubbles: true }));
        }
        s = null;
        document.body.classList.remove('scrubbing');
        hooks.end?.(false);
        e.stopPropagation();
        return;
      }
      if (e.key === 'Alt' || e.key === 'Shift') show(e);
    }, { capture: true });
}
