/**
 * Solve A x = b in place by Gaussian elimination with partial pivoting.
 * A is n×n row-major and is destroyed; the solution overwrites b.
 * Returns false if the matrix is numerically singular.
 */
export function solveInPlace(A: Float64Array, b: Float64Array, n: number): boolean {
  for (let k = 0; k < n; k++) {
    let piv = k;
    let max = Math.abs(A[k * n + k]);
    for (let i = k + 1; i < n; i++) {
      const v = Math.abs(A[i * n + k]);
      if (v > max) {
        max = v;
        piv = i;
      }
    }
    if (max < 1e-14) return false;
    if (piv !== k) {
      for (let j = 0; j < n; j++) {
        const t = A[k * n + j];
        A[k * n + j] = A[piv * n + j];
        A[piv * n + j] = t;
      }
      const t = b[k];
      b[k] = b[piv];
      b[piv] = t;
    }
    const akk = A[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const factor = A[i * n + k] / akk;
      if (factor === 0) continue;
      for (let j = k; j < n; j++) A[i * n + j] -= factor * A[k * n + j];
      b[i] -= factor * b[k];
    }
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i];
    for (let j = i + 1; j < n; j++) s -= A[i * n + j] * b[j];
    b[i] = s / A[i * n + i];
  }
  return true;
}
