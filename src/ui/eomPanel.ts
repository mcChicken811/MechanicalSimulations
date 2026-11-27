import katex from 'katex';
import { gearLabel, linkLabel, pointLabel } from '../mechanics/model';
import type { Derivation } from '../mechanics/derive';
import { constraintLatex, coordinatesLatex, energyLatex, equationLatex, generalFormLatex, potentialLatex, springLatex } from './eomLatex';

const katexHtml = (s: string, display = true) => katex.renderToString(s, { displayMode: display, throwOnError: false, strict: 'ignore' });

const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

/**
 * LaTeX longer than this is not typeset up front. A big mechanism (the 30-link chain) has
 * equations of 17 kB and a kinetic energy of 170 kB, which typeset to tens of megabytes of HTML
 * and freeze the page for seconds. Such an equation shows a placeholder that typesets on demand.
 */
const MAX_TEX = 4000;

function tex(s: string, display = true): string {
  if (s.length <= MAX_TEX) return katexHtml(s, display);
  return `<div class="eq-long" data-tex="${esc(s)}"><span>Too long to show here (${Math.round(s.length / 1000)} kB of LaTeX).</span> <button class="show-tex">Show</button></div>`;
}

/** Let the Show buttons of long equations typeset them. Call once for the panel element. */
export function enableLongEquations(el: HTMLElement) {
  el.addEventListener('click', (e) => {
    const button = (e.target as HTMLElement).closest('.show-tex');
    const box = button?.closest<HTMLElement>('.eq-long');
    if (!box) return;
    box.outerHTML = katexHtml(box.dataset.tex ?? '');
  });
}

/** The ground, as the equations panel describes it (slope in degrees). */
export interface GroundNote {
  y: number;
  slope: number;
  restitution: number;
  friction: number;
  zeta: number;
  omega: number;
}

export function renderEomPanel(el: HTMLElement, d: Derivation | null, error: string | null, friction = false, ground: GroundNote | null = null) {
  const steps = eomPanelSteps(d, error, friction, ground);
  for (;;) {
    const r = steps.next();
    if (r.done) {
      el.innerHTML = r.value;
      return;
    }
  }
}

/**
 * Typeset the panel as a generator: it yields its progress (0 → 1) between equations, so the
 * app can typeset a big mechanism a slice at a time, and returns the panel's HTML.
 */
export function* eomPanelSteps(d: Derivation | null, error: string | null, friction = false, ground: GroundNote | null = null): Generator<number, string, void> {
  if (error) return `<p class="eom-error">Could not derive the equations: ${esc(error)}</p>`;
  if (!d || d.coords.length === 0)
    return `<p class="eom-empty">Draw a bar to start. The equations of motion are derived automatically from the geometry.</p>`;
  // the equations, constraints and springs are the slow part; the rest counts as two items
  const total = d.coords.length + d.constraints.length + d.springs.length + 2;
  let done = 0;
  const step = () => ++done / total;
  const dof = d.coords.length - d.constraints.length;
  const parts: string[] = [];

  parts.push(`<section><h3>Generalized coordinates <span class="pill">${dof} DOF</span></h3>${tex(coordinatesLatex(d))}`);
  if (d.constraints.length)
    parts.push(`<p class="note">${d.coords.length} coordinates − ${d.constraints.length} constraint${d.constraints.length > 1 ? 's' : ''}</p>`);
  parts.push('</section>');

  parts.push(`<section><h3>Equations of motion</h3>${tex(generalFormLatex(d, friction))}`);
  for (const [i, c] of d.coords.entries()) {
    parts.push(`<div class="eq"><div class="eq-label">${tex(c.latex, false)}</div>${tex(equationLatex(d, i))}</div>`);
    yield step();
  }
  parts.push(
    `<p class="note">${tex('Q_{q_i}', false)} is the generalized force from your mouse pull${ground ? ' and from ground contact' : ''}, ${tex('Q = J^{\\mathsf T} F', false)}` +
      (d.constraints.length ? `, and ${tex('\\lambda_k', false)} are Lagrange multipliers.` : '.') +
      (d.motors.length
        ? ` ${tex('\\tau_{\\mathrm{g}}', false)} is the torque of the motor on gear g, reacting on the ground` +
          (d.motors.some((m) => m.b) ? `; ${tex('b_{\\mathrm{g}} = \\tau_{\\mathrm{g}}/\\Omega_{\\mathrm{g}}', false)} makes it fall to zero at the motor's no-load speed ${tex('\\Omega_{\\mathrm{g}}', false)}.` : '.')
        : '') +
      `</p></section>`,
  );

  if (friction) {
    const racks = new Set(d.meshes.filter((m) => m.kind === 'rack').map((m) => m.b));
    const bodies = (j: Derivation['joints'][number]) => [
      ...j.bars.map((b) => `${racks.has(b.link) ? 'rack' : 'bar'} ${linkLabel(b.link)}`),
      ...j.gears.map((g) => `g${gearLabel(g)}`),
      ...(j.ground ? [j.rail !== null ? 'slider' : 'ground'] : []),
    ];
    const pinJoints = d.joints.filter((j) => bodies(j).length >= 2);
    const rails = d.joints.filter((j) => j.rail !== null);
    parts.push('<section><h3>Friction</h3>');
    parts.push(
      tex(
        `R = \\tfrac{1}{2} c_{p} \\sum_{\\text{joints}} \\sum_{i<k} (\\omega_i - \\omega_k)^2` +
          (rails.length ? ` + \\tfrac{1}{2} c_{r} \\sum_{\\text{rails}} v^2` : '') +
          (d.meshes.length ? ` + \\tfrac{1}{2} c_{g} \\sum_{\\text{meshes}} v_{\\text{pitch}}^2` : ''),
      ),
    );
    parts.push(
      `<p class="note">Rayleigh dissipation function: ${tex('\\omega_i', false)} are the rotation rates of the bodies meeting at a joint. Friction removes power ${tex('P = 2R', false)}, which heats each joint: ${tex('C\\,\\dot{\\Delta T} = P - \\tfrac{C}{\\tau}\\Delta T', false)} (Newton cooling).</p>`,
    );
    const rows = [
      ...pinJoints.map((j) => `<li>joint ${pointLabel(j.id)}: ${esc(bodies(j).join(' ↔ '))}</li>`),
      ...rails.map((j) => `<li>joint ${pointLabel(j.id)}: slides on its rail</li>`),
      ...d.meshes.map((m) => `<li>teeth of g${gearLabel(m.a)} and ${m.kind === 'rack' ? `rack ${linkLabel(m.b)}` : `g${gearLabel(m.b)}`}</li>`),
    ];
    parts.push(rows.length ? `<ul class="note">${rows.join('')}</ul>` : '<p class="note">No joint joins two bodies yet.</p>');
    parts.push('</section>');
  }

  if (ground) {
    const t = Math.tan((ground.slope * Math.PI) / 180);
    parts.push('<section><h3>Ground contact</h3>');
    parts.push(tex(`Q_c = \\sum_{\\text{touching } p} J_p^{\\mathsf T}\\left(F_n\\,\\mathbf n + F_t\\,\\mathbf t\\right),\\quad y_{\\text{ground}} = ${ground.y.toFixed(3)}${t ? ` ${t > 0 ? '+' : '-'} ${Math.abs(t).toFixed(3)}\\,x` : ''}`));
    parts.push(tex('F_n = \\max\\left(0,\\ m_p\\omega^2\\delta_p + 2\\zeta\\, m_p\\omega\\,\\dot\\delta_p\\right),\\qquad |F_t| \\le \\mu F_n'));
    parts.push(
      `<p class="note">Every joint, and the lowest point of every gear rim, is checked against the ground (a bar cannot cross it before one of its ends does). ` +
        `A point sunk by ${tex('\\delta_p', false)} is pushed out by a spring-damper scaled by the effective mass ${tex('m_p', false)} it pushes against, ` +
        `so every contact responds at ${tex(`\\omega = ${ground.omega}\\ \\mathrm{rad/s}`, false)} whatever it carries. ` +
        `${tex(`\\zeta = ${ground.zeta.toFixed(3)}`, false)} gives the restitution ${tex(`e = ${ground.restitution.toFixed(2)}`, false)}; ` +
        `friction holds a point where it landed until ${tex(`|F_t|`, false)} reaches ${tex(`\\mu F_n`, false)}, ${tex(`\\mu = ${ground.friction.toFixed(2)}`, false)}, and then lets it slide. ` +
        `These forces are evaluated numerically every step; they are not part of the symbolic equations above.</p>`,
    );
    parts.push('</section>');
  }

  if (d.springs.length) {
    parts.push('<section><h3>Springs</h3>');
    parts.push(
      `<p class="note">Springs add no constraint. Each adds ${tex('V_j = \\tfrac{1}{2} k_j (\\ell_j - \\ell_{0,j})^2', false)} to the potential energy, ` +
        `so equation ${tex('q_i', false)} gains ${tex('-\\partial V_j/\\partial q_i = -k_j\\left(1 - \\ell_{0,j}/\\ell_j\\right)\\tfrac{1}{2}\\,\\partial(\\ell_j^2)/\\partial q_i', false)}.` +
        (d.springs.some((s) => s.tensionOnly)
          ? ` A tension-only spring (a cable or ligament) acts only while stretched: its terms carry ${tex('\\max(0,\\,\\cdot)', false)}, and it is slack when ${tex('\\ell_j < \\ell_{0,j}', false)}.`
          : '') +
        `</p>`,
    );
    for (let j = 0; j < d.springs.length; j++) {
      parts.push(`<div class="eq">${tex(springLatex(d, j))}</div>`);
      yield step();
    }
    parts.push('</section>');
  }

  if (d.constraints.length) {
    parts.push('<section><h3>Constraints</h3>');
    for (const [k, c] of d.constraints.entries()) {
      parts.push(`<div class="eq">${tex(constraintLatex(d, k))}<div class="reason">${esc(c.reason)}</div></div>`);
      yield step();
    }
    parts.push('</section>');
  }

  parts.push(
    `<details><summary>Energies</summary>${tex(energyLatex('T', d.T))}${tex(potentialLatex(d))}<p class="note">V up to an additive constant.</p></details>`,
  );
  yield step();

  parts.push('<details><summary>Parameters</summary><table class="params">');
  for (const p of d.params) parts.push(`<tr><td>${tex(p.latex, false)}</td><td>${Number.isInteger(p.value) ? p.value : p.value.toFixed(3)} ${esc(p.unit)}</td></tr>`);
  parts.push('</table></details>');

  if (d.warnings.length) parts.push(`<ul class="warnings">${d.warnings.map((w) => `<li>${esc(w)}</li>`).join('')}</ul>`);

  yield 1;
  return parts.join('');
}
