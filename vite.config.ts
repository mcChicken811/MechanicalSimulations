import type { Plugin } from 'vite';
import { viteSingleFile } from 'vite-plugin-singlefile';
import { defineConfig } from 'vitest/config';

/**
 * KaTeX's stylesheet lists every font as woff2, woff and ttf. Every current browser
 * reads woff2, so drop the other two before the fonts are inlined into the page.
 */
function katexWoff2Only(): Plugin {
  return {
    name: 'katex-woff2-only',
    enforce: 'pre',
    transform(code, id) {
      if (!/katex(\.min)?\.css/.test(id)) return null;
      return code.replace(/,\s*url\([^)]*\.woff\)\s*format\("woff"\)|,\s*url\([^)]*\.ttf\)\s*format\("truetype"\)/g, '');
    },
  };
}

// The whole app (index.html included) lives in src/.
// `npm run build` produces one self-contained dist/index.html (scripts, styles and
// fonts inlined), so it can be opened straight from disk without a web server.
export default defineConfig({
  root: 'src',
  cacheDir: '../node_modules/.vite',
  plugins: [katexWoff2Only(), viteSingleFile()],
  build: {
    outDir: '../dist',
    emptyOutDir: true,
    assetsInlineLimit: Number.MAX_SAFE_INTEGER,
  },
  test: { include: ['**/*.test.ts'] },
});
