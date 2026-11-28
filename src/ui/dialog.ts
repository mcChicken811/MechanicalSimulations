/**
 * A question in the middle of the screen, answered with one of its buttons: a title, what it is
 * about, the buttons, and optionally "Don't show this again" (for this session, or ever). Esc or a
 * click outside it answers 'cancel'. Resolves with the button chosen and how long not to ask.
 */
export interface DialogAnswer {
  choice: string;
  /** Not to ask again: for this session, ever, or (unticked) null. */
  remember: 'session' | 'forever' | null;
}

export function askDialog(opts: { title: string; body: string; buttons: { id: string; label: string; primary?: boolean }[]; remember?: boolean }): Promise<DialogAnswer> {
  return new Promise((resolve) => {
    const back = document.createElement('div');
    back.className = 'dialog-backdrop';
    const esc = (s: string) => s.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);
    back.innerHTML = `
      <div class="dialog panel" role="alertdialog" aria-modal="true">
        <h3>${esc(opts.title)}</h3>
        <p>${esc(opts.body)}</p>
        ${opts.remember ? `<label class="d-remember"><input type="checkbox"> Don't show this again <select><option value="session">for this session</option><option value="forever">ever</option></select></label>` : ''}
        <div class="d-buttons">${opts.buttons.map((b) => `<button data-id="${esc(b.id)}" class="${b.primary ? 'primary' : ''}">${esc(b.label)}</button>`).join('')}</div>
      </div>`;
    document.body.append(back);
    const done = (choice: string) => {
      const box = back.querySelector<HTMLInputElement>('.d-remember input');
      const how = back.querySelector<HTMLSelectElement>('.d-remember select');
      back.remove();
      resolve({ choice, remember: box?.checked ? (how!.value as 'session' | 'forever') : null });
    };
    back.querySelectorAll<HTMLButtonElement>('.d-buttons button').forEach((b) => b.addEventListener('click', () => done(b.dataset.id!)));
    back.addEventListener('pointerdown', (e) => {
      if (e.target === back) done('cancel');
    });
    back.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        e.stopPropagation();
        done('cancel');
      }
    });
    (back.querySelector<HTMLButtonElement>('.d-buttons button.primary') ?? back.querySelector<HTMLButtonElement>('.d-buttons button'))?.focus();
  });
}
