// bench/oracle.mjs — INDEPENDENT re-implementation used to cross-check the
// certificate. It deliberately shares no code and no constant with src/core.mjs:
// forward kinematics uses 4x4 homogeneous matrices, the segment-segment distance
// solves the 2x2 least-squares system by Cramer's rule with three clamped
// refinement passes, and the sampler is uniform + golden-section refinement.
// Nothing here imports from src/core.mjs.
const RAD = Math.PI / 180;

function mul4(A, B) {
  const C = new Array(16).fill(0);
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += A[r * 4 + k] * B[k * 4 + c];
    C[r * 4 + c] = s;
  }
  return C;
}
function apply4(M, p) {
  return [
    M[0] * p[0] + M[1] * p[1] + M[2] * p[2] + M[3],
    M[4] * p[0] + M[5] * p[1] + M[6] * p[2] + M[7],
    M[8] * p[0] + M[9] * p[1] + M[10] * p[2] + M[11],
  ];
}
function rot4(axis, ang) {
  const n = Math.hypot(axis[0], axis[1], axis[2]);
  const x = axis[0] / n, y = axis[1] / n, z = axis[2] / n;
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ];
}
function trans4(p) { return [1, 0, 0, p[0], 0, 1, 0, p[1], 0, 0, 1, p[2], 0, 0, 0, 1]; }

export function oracleFrames(spec, q) {
  const frames = [];
  let M = trans4([0, 0, 0]);
  for (let i = 0; i < spec.joints.length; i++) {
    const j = spec.joints[i];
    M = mul4(M, trans4(j.offset));
    M = mul4(M, rot4(j.axis, q[i]));
    frames.push(M);
  }
  return frames;
}

// Distance between segments by enumerating the exact candidate set: the
// unconstrained least-squares solution plus every boundary projection. The
// optimum lies in this set, so no iterative refinement is needed.
export function segDist(segA, segB) {
  const u = [segA.b[0] - segA.a[0], segA.b[1] - segA.a[1], segA.b[2] - segA.a[2]];
  const v = [segB.b[0] - segB.a[0], segB.b[1] - segB.a[1], segB.b[2] - segB.a[2]];
  const w = [segA.a[0] - segB.a[0], segA.a[1] - segB.a[1], segA.a[2] - segB.a[2]];
  const uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
  const vv = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  const uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const uw = u[0] * w[0] + u[1] * w[1] + u[2] * w[2];
  const vw = v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
  const cl = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);
  const cands = [];
  const det = uu * vv - uv * uv;
  if (uu > 1e-30 && vv > 1e-30 && det > 1e-30) {
    cands.push([(uv * vw - vv * uw) / det, (uu * vw - uv * uw) / det]);
  }
  if (uu > 1e-30) {
    cands.push([cl(-uw / uu), 0], [cl((uv - uw) / uu), 1]);
  } else cands.push([0, 0], [0, 1]);
  if (vv > 1e-30) {
    cands.push([0, cl(vw / vv)], [1, cl((uv + vw) / vv)]);
  } else cands.push([0, 0], [1, 0]);
  for (const [s0, t0] of [[0, 0], [0, 1], [1, 0], [1, 1]]) cands.push([s0, t0]);
  let best = Infinity;
  for (const raw of cands) {
    const s = cl(raw[0]), t = cl(raw[1]);
    const pa = [segA.a[0] + u[0] * s, segA.a[1] + u[1] * s, segA.a[2] + u[2] * s];
    const pb = [segB.a[0] + v[0] * t, segB.a[1] + v[1] * t, segB.a[2] + v[2] * t];
    const d = Math.hypot(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]);
    if (d < best) best = d;
  }
  return best;
}

export function oracleAngles(spec, t) {
  const ts = spec.motion.times, ks = spec.motion.keys;
  let idx = 0;
  while (idx < ts.length - 2 && ts[idx + 1] <= t) idx++;
  const u = (t - ts[idx]) / (ts[idx + 1] - ts[idx]);
  return ks[idx].map((v, j) => v + (ks[idx + 1][j] - v) * u);
}

export function oracleGap(spec, t, i, k) {
  const q = oracleAngles(spec, t);
  const F = oracleFrames(spec, q);
  const capA = spec.joints[i].capsule, capB = spec.joints[k].capsule;
  const segA = { a: apply4(F[i], capA.a), b: apply4(F[i], capA.b) };
  const segB = { a: apply4(F[k], capB.a), b: apply4(F[k], capB.b) };
  return segDist(segA, segB) - capA.radius - capB.radius;
}

// Uniform sampling + golden-section refinement around every sampled local min.
export function oracleScan(spec, samples = 1200) {
  const ts = spec.motion.times;
  const t0 = ts[0], t1 = ts[ts.length - 1];
  const n = spec.joints.length;
  const minima = [];
  let worst = Infinity, worstAt = null;
  for (let i = 0; i < n - 2; i++) {
    for (let k = i + 2; k < n; k++) {
      let best = Infinity, bestT = t0;
      const vals = [];
      for (let s = 0; s <= samples; s++) {
        const t = t0 + (t1 - t0) * (s / samples);
        const g = oracleGap(spec, t, i, k);
        vals.push({ t, g });
        if (g < best) { best = g; bestT = t; }
      }
      // refine the 3 lowest samples with golden-section over +-1 sample spacing
      const sorted = vals.slice().sort((a, b) => a.g - b.g).slice(0, 3);
      const h = (t1 - t0) / samples;
      for (const cand of sorted) {
        let a = Math.max(t0, cand.t - h), b = Math.min(t1, cand.t + h);
        const gr = (Math.sqrt(5) - 1) / 2;
        let c = b - gr * (b - a), d = a + gr * (b - a);
        let gc = oracleGap(spec, c, i, k), gd = oracleGap(spec, d, i, k);
        for (let it = 0; it < 40; it++) {
          if (gc < gd) { b = d; d = c; gd = gc; c = b - gr * (b - a); gc = oracleGap(spec, c, i, k); }
          else { a = c; c = d; gc = gd; d = a + gr * (b - a); gd = oracleGap(spec, d, i, k); }
        }
        const tt = 0.5 * (a + b), gg = oracleGap(spec, tt, i, k);
        if (gg < best) { best = gg; bestT = tt; }
      }
      minima.push({ i, k, minGap: best, at: bestT });
      if (best < worst) { worst = best; worstAt = { i, k, t: bestT }; }
    }
  }
  return { pairs: minima.length, minima, worst, worstAt, samples };
}

export { RAD };
