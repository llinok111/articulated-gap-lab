// src/core.mjs — articulated-chain continuous-time self-collision certificate.
// Pure ES module: no three.js, no DOM, no globals, no I/O.
//
// Geometry: a serial chain of rigid links. Link i is a capsule whose endpoints
// live in the frame of joint i. Joint i rotates its descendants about `axis`
// through its origin. Motion: piecewise-linear joint-space keyframes.
//
// Certificate: for a pair of links (i,k) the capsule gap g(t) is 1-Lipschitz in
// time with a bound L = v_i + v_k, where v_i is a rigorous upper bound on the
// speed of any point of link i:
//
//     v_i <= sum_{j<=i} |qdot_j| * D[j][i]
//
// and D[j][i] is the (time-invariant) distance from joint j's origin to the
// farthest capsule point of link i. Branch and bound over each keyframe
// interval proves g > 0 everywhere, or returns an instant with g < 0.
// No absolute epsilon is used: every length threshold is relative to chain scale.

export const VERSION = '1.0.0';

/* ------------------------------------------------------------------ vec/mat */
const IDENT = [1, 0, 0, 0, 1, 0, 0, 0, 1];

function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add3(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function norm3(a) { return Math.hypot(a[0], a[1], a[2]); }
function scale3(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function unit3(a) {
  const n = norm3(a);
  if (!(n > 0)) throw new Error('zero-length axis');
  return scale3(a, 1 / n);
}
function mul3(A, B) {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
    }
  }
  return C;
}
function matVec3(M, v) {
  return [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
}
export function axisAngle(axis, ang) {
  const [x, y, z] = unit3(axis);
  const c = Math.cos(ang), s = Math.sin(ang), C = 1 - c;
  return [
    c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C,
  ];
}
function clamp01(x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

/* --------------------------------------------------------------- chain build */
export function makeChain(spec) {
  if (!spec || !Array.isArray(spec.joints) || spec.joints.length < 3) {
    throw new Error('chain needs >= 3 joints');
  }
  const joints = spec.joints.map((j, idx) => {
    const radius = Number(j.capsule.radius);
    const a = j.capsule.a.map(Number);
    const b = j.capsule.b.map(Number);
    if (!(radius >= 0) || !Number.isFinite(radius)) throw new Error(`bad radius at ${idx}`);
    if (norm3(sub3(b, a)) === 0 && radius === 0) throw new Error(`degenerate capsule at ${idx}`);
    return {
      axis: unit3(j.axis.map(Number)),
      offset: j.offset.map(Number),
      capsule: { a, b, radius },
    };
  });
  const chain = { name: spec.name || 'chain', joints };
  // Rest-pose transforms and the reach matrix D[j][i].
  const rest = chainTransforms(chain, new Array(joints.length).fill(0));
  const n = joints.length;
  const D = new Array(n);
  for (let j = 0; j < n; j++) D[j] = new Array(n).fill(0);
  const maxReach = [0];
  for (let i = 0; i < n; i++) {
    const T = rest[i];
    const pa = add3(T.t, matVec3(T.R, joints[i].capsule.a));
    const pb = add3(T.t, matVec3(T.R, joints[i].capsule.b));
    const r = joints[i].capsule.radius;
    for (let j = 0; j <= i; j++) {
      D[j][i] = Math.max(norm3(sub3(pa, rest[j].t)), norm3(sub3(pb, rest[j].t))) + r;
    }
    maxReach[0] = Math.max(maxReach[0], D[0][i]);
  }
  chain.reach = D;               // D[j][i]
  chain.scale = Math.max(maxReach[0], 1e-300); // used only for relative thresholds
  chain.rest = rest;
  return chain;
}

// World transforms for joints 0..upTo. Row-major rotation + translation.
export function chainTransforms(chain, q, upTo) {
  const all = chain.joints.length;
  const n = upTo === undefined ? all : Math.min(upTo + 1, all);
  const out = new Array(n);
  let R = IDENT.slice();
  let t = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const j = chain.joints[i];
    t = add3(t, matVec3(R, j.offset));      // joint i origin
    R = mul3(R, axisAngle(j.axis, q[i]));   // link i frame
    out[i] = { R, t };
  }
  return out;
}

export function linkWorldSegment(chain, transforms, i) {
  const T = transforms[i];
  const cap = chain.joints[i].capsule;
  return {
    a: add3(T.t, matVec3(T.R, cap.a)),
    b: add3(T.t, matVec3(T.R, cap.b)),
    radius: cap.radius,
  };
}

/* ------------------------------------------------------- segment-segment gap */
function closestOnSegments(p1, q1, p2, q2, tau2) {
  const d1 = sub3(q1, p1), d2 = sub3(q2, p2), r = sub3(p1, p2);
  const a = dot3(d1, d1), e = dot3(d2, d2), f = dot3(d2, r);
  let s = 0, t = 0;
  if (a <= tau2 && e <= tau2) {
    s = 0; t = 0;
  } else if (a <= tau2) {
    s = 0;
    t = clamp01(f / e);
  } else {
    const c = dot3(d1, r);
    if (e <= tau2) {
      t = 0;
      s = clamp01(-c / a);
    } else {
      const b = dot3(d1, d2);
      const den = a * e - b * b;
      s = den > 0 ? clamp01((b * f - c * e) / den) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp01(-c / a); }
      else if (t > 1) { t = 1; s = clamp01((b - c) / a); }
    }
  }
  return { s, t };
}

export function capsuleGapPoints(segA, segB, scale) {
  const tau = 1e-12 * scale;
  const { s, t } = closestOnSegments(segA.a, segA.b, segB.a, segB.b, tau * tau);
  const pa = add3(segA.a, scale3(sub3(segA.b, segA.a), s));
  const pb = add3(segB.a, scale3(sub3(segB.b, segB.a), t));
  return norm3(sub3(pa, pb)) - (segA.radius + segB.radius);
}

export function gapAt(chain, q, i, k) {
  const T = chainTransforms(chain, q, Math.max(i, k));
  const ga = capsuleGapPoints(linkWorldSegment(chain, T, i), linkWorldSegment(chain, T, k), chain.scale);
  return ga;
}

/* --------------------------------------------------------------- motion model */
export function makeMotion(times, keys, bump) {
  if (times.length !== keys.length || times.length < 2) throw new Error('bad keyframes');
  for (let i = 1; i < times.length; i++) {
    if (!(times[i] > times[i - 1])) throw new Error('times must increase');
  }
  const n = keys[0].length;
  for (const k of keys) if (k.length !== n) throw new Error('key width mismatch');
  return {
    times: Float64Array.from(times),
    keys: keys.map((k) => Float64Array.from(k)),
    n,
    t0: times[0],
    t1: times[times.length - 1],
    bump: bump || null,
    scale: 1,
  };
}

function bumpShape(bump, t) {
  if (!bump) return { w: 0, wd: 0 };
  const { t0, t1, amp } = bump;
  if (t <= t0 || t >= t1) return { w: 0, wd: 0 };
  const u = (t - t0) / (t1 - t0);
  const w = Math.sin(Math.PI * u) ** 2;
  const wd = (Math.PI / (t1 - t0)) * Math.sin(2 * Math.PI * u);
  return { w: amp * w, wd: amp * wd };
}

export function segmentIndexAt(motion, t) {
  const ts = motion.times;
  if (t <= ts[0]) return 0;
  if (t >= ts[ts.length - 1]) return ts.length - 2;
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid;
  }
  return lo;
}

export function anglesAt(motion, t) {
  const idx = segmentIndexAt(motion, t);
  const a = motion.times[idx], b = motion.times[idx + 1];
  const u = (t - a) / (b - a);
  const k0 = motion.keys[idx], k1 = motion.keys[idx + 1];
  const q = new Array(motion.n);
  const ampScale = motion.scale === undefined ? 1 : motion.scale;
  for (let j = 0; j < motion.n; j++) q[j] = (k0[j] + (k1[j] - k0[j]) * u) * ampScale;
  const bm = bumpShape(motion.bump, t);
  if (bm.w !== 0) q[motion.bump.joint] += bm.w;
  return q;
}

export function ratesAt(motion, t) {
  const idx = segmentIndexAt(motion, t);
  const dt = motion.times[idx + 1] - motion.times[idx];
  const k0 = motion.keys[idx], k1 = motion.keys[idx + 1];
  const dq = new Array(motion.n);
  const ampScale = motion.scale === undefined ? 1 : motion.scale;
  for (let j = 0; j < motion.n; j++) dq[j] = ((k1[j] - k0[j]) / dt) * ampScale;
  return dq;
}

// Rigorous speed bound for every link, at time t (rates are piecewise constant).
export function speedBounds(chain, motion, t) {
  const dq = ratesAt(motion, t);
  const n = chain.joints.length;
  const v = new Array(n).fill(0);
  let extra = 0;
  if (motion.bump) {
    const span = motion.bump.t1 - motion.bump.t0;
    extra = Math.abs(motion.bump.amp) * (Math.PI / span);
  }
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let j = 0; j <= i; j++) {
      const rate = Math.abs(dq[j]) + (motion.bump && j === motion.bump.joint ? extra : 0);
      s += rate * chain.reach[j][i];
    }
    v[i] = s;
  }
  return v;
}

/* ------------------------------------------------------------- certificate */
// Certify all non-adjacent link pairs over the whole motion.
// Sound: every returned "clear" interval has a proven positive lower bound.
export function certify(chain, motion, opts = {}) {
  const tolT = opts.tolT ?? 1e-9;
  const maxDepth = opts.maxDepth ?? 26;
  const minPairSep = opts.minPairSep ?? 2;      // skip adjacent links (they share a joint)
  const pairFilter = opts.pairFilter || null;   // (i,k) => bool, for repair search
  const n = chain.joints.length;
  const times = motion.times;
  const violations = [];
  const undecided = [];
  let pairs = 0, intervals = 0, evals = 0;
  let minGap = Infinity;

  const gapAtTime = (i, k, t) => {
    const T = chainTransforms(chain, anglesAt(motion, t), k);
    return capsuleGapPoints(linkWorldSegment(chain, T, i), linkWorldSegment(chain, T, k), chain.scale);
  };

  for (let i = 0; i < n - minPairSep; i++) {
    for (let k = i + minPairSep; k < n; k++) {
      if (pairFilter && !pairFilter(i, k)) continue;
      pairs++;
      const found = [];
      const undec = [];
      for (let seg = 0; seg < times.length - 1; seg++) {
        const a0 = times[seg], b0 = times[seg + 1];
        const v = speedBounds(chain, motion, a0 + 1e-12 * (b0 - a0));
        const L = v[i] + v[k];
        // iterative stack: [a,b,depth]
        const stack = [[a0, b0, 0]];
        while (stack.length) {
          const [a, b, depth] = stack.pop();
          const m = 0.5 * (a + b), half = 0.5 * (b - a);
          const gm = gapAtTime(i, k, m);
          evals++;
          if (gm < minGap) minGap = gm;
          if (gm < 0) { found.push({ t: m, gap: gm, a, b }); continue; }
          if (gm - L * half > 0) { intervals++; continue; }
          if (depth >= maxDepth) { undec.push({ t: m, gap: gm, a, b }); continue; }
          stack.push([m, b, depth + 1], [a, m, depth + 1]);
        }
      }
      if (found.length) violations.push({ i, k, instants: found, undecided: undec.length });
      if (undec.length) undecided.push({ i, k, count: undec.length, t: undec[0].t, gap: undec[0].gap });
    }
  }
  // cluster violating instants per pair into intervals and refine the boundary
  const intervalsFound = [];
  for (const v of violations) {
    const inst = v.instants.slice().sort((x, y) => x.t - y.t);
    const clusters = [];
    for (const it of inst) {
      const last = clusters[clusters.length - 1];
      if (last && it.t - last[last.length - 1].t <= 0.02) last.push(it);
      else clusters.push([it]);
    }
    for (const cl of clusters) {
      let worst = cl[0];
      for (const it of cl) if (it.gap < worst.gap) worst = it;
      const g = (t) => gapAtTime(v.i, v.k, t);
      const t0 = refineBoundary(g, worst.t, -1, motion.t0, motion.t1);
      const t1 = refineBoundary(g, worst.t, +1, motion.t0, motion.t1);
      intervalsFound.push({ i: v.i, k: v.k, t0, t1, minGap: worst.gap });
    }
  }
  intervalsFound.sort((x, y) => (x.i - y.i) || (x.k - y.k) || (x.t0 - y.t0));
  // Each cluster is refined by walking to the boundary of its connected
  // negative component, so different clusters of the same component converge to
  // the same interval: dedupe by overlap instead of bridging positive pockets.
  const mergeTol = 1e-9;
  const merged = [];
  for (const iv of intervalsFound) {
    const prev = merged[merged.length - 1];
    if (prev && prev.i === iv.i && prev.k === iv.k && iv.t0 <= prev.t1 + mergeTol) {
      prev.t1 = Math.max(prev.t1, iv.t1);
      prev.t0 = Math.min(prev.t0, iv.t0);
      prev.minGap = Math.min(prev.minGap, iv.minGap);
    } else merged.push({ ...iv });
  }
  intervalsFound.length = 0;
  intervalsFound.push(...merged);
  return {
    pairs,
    intervalsCertified: intervals,
    undecided,
    violations: intervalsFound,
    minGap,
    evaluations: evals,
    tolT,
    maxDepth,
  };
}

// Walk outward from a violating instant with a growing step until the gap turns
// positive, then bisect that bracket. If the growing step jumps past a small
// positive pocket, the domain edge is checked before declaring a clipped run.
function refineBoundary(gapFn, tBad, dir, tMin, tMax) {
  const span = tMax - tMin;
  let step = 1e-3 * span;
  let lastNeg = tBad;
  let tGood = null;
  for (let k = 0; k < 80; k++) {
    const cand = tBad + dir * step;
    if (cand < tMin || cand > tMax) {
      const edge = dir < 0 ? tMin : tMax;
      if (gapFn(edge) > 0) { tGood = edge; break; }
      return edge;
    }
    if (gapFn(cand) > 0) { tGood = cand; break; }
    lastNeg = cand;
    step *= 1.35;
  }
  if (tGood === null) tGood = dir < 0 ? tMin : tMax;
  let neg = lastNeg, pos = tGood;
  for (let it = 0; it < 80; it++) {
    const mid = 0.5 * (neg + pos);
    if (gapFn(mid) > 0) pos = mid; else neg = mid;
  }
  return 0.5 * (neg + pos);
}

/* ------------------------------------------------------------------ repair */
// Collision is a configuration-space property: a time reparameterization
// t -> k*t maps every configuration to the same configuration, so it cannot
// remove a violation (bench/verify.mjs measures this: 10x slowdown, identical
// violation intervals in normalized time). The repair below shrinks the joint
// motion uniformly: q_s(t) = s * q(t), and bisects the minimal s that is
// certified collision-free. "Minimal" is within this one-parameter family and
// is checked by re-certifying s + 0.005 (must violate again).
export function minimalAmplitudeScale(chain, motion, opts = {}) {
  const iters = opts.iters ?? 22;
  const step = opts.step ?? 0.005;
  const lo0 = opts.minScale ?? 0.02;
  const mk = (s) => ({ ...motion, scale: s });
  const clean = (s) => {
    const r = certify(chain, mk(s), { tolT: 1e-9, maxDepth: 26 });
    return r.violations.length === 0 && r.undecided.length === 0;
  };
  const base = certify(chain, motion, { tolT: 1e-9, maxDepth: 26 });
  if (base.violations.length === 0) {
    return { needed: false, scale: 1, scalePct: 0, certified: summarize(base) };
  }
  if (!clean(lo0)) return { needed: true, found: false, reason: 'not-clean-at-min-scale' };
  if (clean(1 - 1e-9)) return { needed: true, found: false, reason: 'unexpectedly-clean' };
  // invariant: clean(lo) === true, clean(hi) === false, s* = sup{clean}
  let lo = lo0, hi = 1;
  for (let it = 0; it < iters; it++) {
    const mid = 0.5 * (lo + hi);
    if (clean(mid)) lo = mid; else hi = mid;
  }
  const s = lo;
  const repaired = mk(s);
  const full = certify(chain, repaired, { tolT: 1e-9, maxDepth: 26 });
  // minimality witness: s + step must violate again (one step more motion)
  const probe = Math.min(1, s + step);
  const probeCert = probe > s ? certify(chain, mk(probe), { tolT: 1e-9, maxDepth: 26 }) : null;

  let endShift = 0;
  const steps = 200;
  const last = chain.joints.length - 1;
  const span = motion.t1 - motion.t0;
  for (let n = 0; n <= steps; n++) {
    const t = motion.t0 + span * (n / steps);
    const T0 = chainTransforms(chain, anglesAt(motion, t));
    const T1 = chainTransforms(chain, anglesAt(repaired, t));
    const p0 = linkWorldSegment(chain, T0, last).b;
    const p1 = linkWorldSegment(chain, T1, last).b;
    endShift = Math.max(endShift, norm3(sub3(p1, p0)));
  }
  return {
    needed: true,
    found: full.violations.length === 0 && full.undecided.length === 0,
    scale: s,
    scalePct: (1 - s) * 100,
    minimalityProbe: {
      scale: probe,
      violations: probeCert ? probeCert.violations.length : null,
      undecided: probeCert ? probeCert.undecided.length : null,
    },
    endEffectorShift: endShift,
    endEffectorShiftRel: endShift / chain.scale,
    certified: summarize(full),
  };
}

export function summarize(cert) {
  return {
    pairs: cert.pairs,
    violations: cert.violations.length,
    undecided: cert.undecided.length,
    intervalsCertified: cert.intervalsCertified,
    minGap: cert.minGap,
    evaluations: cert.evaluations,
  };
}

/* --------------------------------------------------------------- statistics */
export function chainStats(chain, motion) {
  const q0 = anglesAt(motion, motion.t0);
  const T = chainTransforms(chain, q0);
  let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < chain.joints.length; i++) {
    const s = linkWorldSegment(chain, T, i);
    for (const p of [s.a, s.b]) {
      for (let d = 0; d < 3; d++) { lo[d] = Math.min(lo[d], p[d]); hi[d] = Math.max(hi[d], p[d]); }
    }
  }
  return { chains: 1, links: chain.joints.length, scale: chain.scale, bbox: [lo, hi] };
}
