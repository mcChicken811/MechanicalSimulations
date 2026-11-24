import katex from 'katex';
import { gearLabel, linkLabel, pointLabel } from '../mechanics/model';
import type { Derivation } from '../mechanics/derive';
import { constraintLatex, coordinatesLatex, energyLatex, equationLatex, generalFormLatex, potentialLatex, springLatex } from './eomLatex';

const tex = (s: string, display = true) => katex.renderToString(s, { displayMode: display, throwOnError: false, strict: 'ignore' });

const esc = (s: string) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]!);

export function renderEomPanel(el: HTMLElement, d: Derivation | null, error: string | null, friction = false) {
  if (error) {
    el.innerHTML = `<p class="eom-error">Could not derive the equations: ${esc(error)}</p>`;
    return;
  }
  if (!d || d.coords.length === 0) {
    el.innerHTML = `<p class="eom-empty">Draw a bar to start. The equations of motion are derived automatically from the geometry.</p>`;
    return;
  }
  const dof = d.coords.length - d.constraints.length;
  const parts: string[] = [];

  parts.push(`<section><h3>Generalized coordinates <span class="pill">${dof} DOF</span></h3>${tex(coordinatesLatex(d))}`);
  if (d.constraints.length)
    parts.push(`<p class="note">${d.coords.length} coordinates − ${d.constraints.length} constraint${d.constraints.length > 1 ? 's' : ''}</p>`);
  parts.push('</section>');

  parts.push(`<section><h3>Equations of motion</h3>${tex(generalFormLatex(d, friction))}`);
  d.coords.forEach((c, i) => {
    parts.push(`<div class="eq"><div class="eq-label">${tex(c.latex, false)}</div>${tex(equationLatex(d, i))}</div>`);
  });
  parts.push(
    `<p class="note">${tex('Q_{q_i}', false)} is the generalized force from your mouse pull, ${tex('Q = J^{\\mathsf T} F', false)}` +
      (d.constraints.length ? `, and ${tex('\\lambda_k', false)} are Lagrange multipliers.` : '.') +
      `</p></section>`,
  );

  if (friction) {
    const bodies = (j: Derivation['joints'][number]) => [
      ...j.bars.map((b) => `bar ${linkLabel(b.link)}`),
      ...j.gears.map((g) => `g${gearLabel(g)}`),
      ...(j.ground ? [j.rail ? 'slider' : 'ground'] : []),
    ];
    const pinJoints = d.joints.filter((j) => bodies(j).length >= 2);
    const rails = d.joints.filter((j) => j.rail);
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
      ...d.meshes.map((m) => `<li>teeth of g${gearLabel(m.a)} and g${gearLabel(m.b)}</li>`),
    ];
    parts.push(rows.length ? `<ul class="note">${rows.join('')}</ul>` : '<p class="note">No joint joins two bodies yet.</p>');
    parts.push('</section>');
  }

  if (d.springs.length) {
    parts.push('<section><h3>Springs</h3>');
    parts.push(
      `<p class="note">Springs add no constraint. Each adds ${tex('V_j = \\tfrac{1}{2} k_j (\\ell_j - \\ell_{0,j})^2', false)} to the potential energy, ` +
        `so equation ${tex('q_i', false)} gains ${tex('-\\partial V_j/\\partial q_i = -k_j\\left(1 - \\ell_{0,j}/\\ell_j\\right)\\tfrac{1}{2}\\,\\partial(\\ell_j^2)/\\partial q_i', false)}.</p>`,
    );
    d.springs.forEach((_, j) => parts.push(`<div class="eq">${tex(springLatex(d, j))}</div>`));
    parts.push('</section>');
  }

  if (d.constraints.length) {
    parts.push('<section><h3>Constraints</h3>');
    d.constraints.forEach((c, k) => parts.push(`<div class="eq">${tex(constraintLatex(d, k))}<div class="reason">${esc(c.reason)}</div></div>`));
    parts.push('</section>');
  }

  parts.push(
    `<details><summary>Energies</summary>${tex(energyLatex('T', d.T))}${tex(potentialLatex(d))}<p class="note">V up to an additive constant.</p></details>`,
  );

  parts.push('<details><summary>Parameters</summary><table class="params">');
  for (const p of d.params) parts.push(`<tr><td>${tex(p.latex, false)}</td><td>${Number.isInteger(p.value) ? p.value : p.value.toFixed(3)} ${esc(p.unit)}</td></tr>`);
  parts.push('</table></details>');

  if (d.warnings.length) parts.push(`<ul class="warnings">${d.warnings.map((w) => `<li>${esc(w)}</li>`).join('')}</ul>`);

  el.innerHTML = parts.join('');
}
