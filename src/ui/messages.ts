/**
 * The message bar at the top of the screen, and the one way the app talks to the user there.
 *
 *  - instruct(text): the standing instruction for what is going on (the tool in use, a pick half
 *    done, running or paused). Shown whenever no notice is.
 *  - notify(text, kind): a passing notice (copied, a bar broke, a warning, an error), shown for
 *    long enough to read (longer for longer ones), then the instruction again.
 *
 * Like the side panels, the bar slides away (up) when the mouse has not been over it for 3 s and
 * comes back when the mouse goes to the top edge above it; here it also comes back by itself
 * for anything new: a notice, or an instruction that changed. Hovering it keeps it.
 */
import { autoHide, type AutoHide } from './autohide';

export type MessageKind = 'info' | 'warn' | 'error';

export class Messages {
  private readonly bar: HTMLDivElement;
  private readonly panel: AutoHide;
  private instruction = '';
  private notice: { text: string; kind: MessageKind } | null = null;
  private timer = 0;

  /** `host` spans the top of the drawing; the bar sits centred in it. */
  constructor(host: HTMLElement) {
    this.bar = document.createElement('div');
    this.bar.className = 'msg panel';
    this.bar.setAttribute('role', 'status');
    this.bar.setAttribute('aria-live', 'polite');
    host.append(this.bar);
    this.panel = autoHide(this.bar, 'top');
  }

  /** Set the standing instruction; a different one brings the bar out. */
  instruct(text: string) {
    if (text === this.instruction) return;
    this.instruction = text;
    if (!this.notice) this.render();
  }

  /** Show a passing notice (over the instruction), long enough to read; it brings the bar out. */
  notify(text: string, kind: MessageKind = 'info') {
    this.notice = { text, kind };
    clearTimeout(this.timer);
    const ms = Math.min(9000, Math.max(kind === 'error' ? 5000 : 1800, text.length * 45));
    this.timer = window.setTimeout(() => {
      this.notice = null;
      this.render();
    }, ms);
    this.render();
  }

  /** What the bar says now. */
  get text() {
    return this.notice?.text ?? this.instruction;
  }

  private render() {
    const text = this.text;
    if (this.bar.textContent === text && this.bar.dataset.kind === (this.notice?.kind ?? 'info')) return;
    this.bar.textContent = text;
    this.bar.dataset.kind = this.notice?.kind ?? 'info';
    this.bar.classList.toggle('empty', !text);
    if (text) this.panel.show(); // news: out it comes
  }
}
