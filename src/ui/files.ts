/**
 * Saving and opening files: designs (the drawing) and projects (the drawing and every setting).
 *
 * Both are JSON with a `format` tag:
 *   { "format": "linkage-lab-design",  "version": 1, "design": { … } }
 *   { "format": "linkage-lab-project", "version": 1, "design": { … }, "controls": { … }, … }
 * A bare design (as kept in the browser) opens as a design too. Saving asks where (the browser's
 * save dialog, where it has one) or downloads the file; opening asks for a file.
 */
export const DESIGN_FORMAT = 'linkage-lab-design';
export const PROJECT_FORMAT = 'linkage-lab-project';

/** A project: the design, the side panel's settings by control id, the beam colouring, the view. */
export interface ProjectFile {
  format: typeof PROJECT_FORMAT;
  version: 1;
  savedAt: string;
  design: unknown;
  controls: Record<string, boolean | string>;
  beamColour?: unknown;
  view?: { ox: number; oy: number; scale: number };
  /** The background: a scene, or an image (as a data URL) and where it lies (ui/background.ts). */
  background?: unknown;
}

/** What a file holds: a design (from a design file, a project, or bare), and the project if it is one. */
export function readFile(data: unknown): { design: unknown; project: ProjectFile | null } | null {
  if (!data || typeof data !== 'object') return null;
  const o = data as Record<string, unknown>;
  if (o.format === PROJECT_FORMAT && o.design) return { design: o.design, project: o as unknown as ProjectFile };
  if (o.format === DESIGN_FORMAT && o.design) return { design: o.design, project: null };
  if (Array.isArray(o.points) && Array.isArray(o.links)) return { design: o, project: null }; // a bare design
  return null;
}

type SavePicker = (opts: { suggestedName: string; types: { description: string; accept: Record<string, string[]> }[] }) => Promise<{ createWritable(): Promise<{ write(d: Blob): Promise<void>; close(): Promise<void> }> }>;

/**
 * Save `obj` as a JSON file named like `name`: in the browser's save dialog where it has one
 * (choose the folder and name), else as a download. Whether it was saved (false: cancelled).
 */
export async function saveJson(obj: unknown, name: string, description: string): Promise<boolean> {
  const blob = new Blob([JSON.stringify(obj, null, 1)], { type: 'application/json' });
  const picker = (window as unknown as { showSaveFilePicker?: SavePicker }).showSaveFilePicker;
  if (picker) {
    try {
      const handle = await picker({ suggestedName: name, types: [{ description, accept: { 'application/json': ['.json'] } }] });
      const w = await handle.createWritable();
      await w.write(blob);
      await w.close();
      return true;
    } catch (e) {
      if ((e as Error).name === 'AbortError') return false; // cancelled
      // no access (a file:// page, a sandbox): download it instead
    }
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 10000);
  return true;
}

/** Ask for a JSON file and read it: its name and contents, or null if cancelled; throws if it is not JSON. */
export function pickJson(): Promise<{ name: string; data: unknown } | null> {
  return new Promise((resolve, reject) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json,application/json';
    input.addEventListener('cancel', () => resolve(null));
    input.addEventListener('change', async () => {
      const file = input.files?.[0];
      if (!file) return resolve(null);
      try {
        resolve({ name: file.name, data: JSON.parse(await file.text()) });
      } catch {
        reject(new Error(`${file.name} is not a JSON file`));
      }
    });
    input.click();
  });
}

/** A file name from a title: letters, digits and dashes. */
export function fileName(title: string, kind: 'design' | 'project'): string {
  const base = title.trim().replace(/[^\p{L}\p{N}]+/gu, '-').replace(/^-|-$/g, '') || 'linkage';
  return `${base}.${kind}.json`;
}
