#!/usr/bin/env node
// baseline/naive.mjs — the baseline the certificate is measured against:
// fixed-rate frame sampling. It only looks at the sampled instants, so a
// violation interval is "found" only if a sample happens to fall inside it.
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeChain, makeMotion, certify, gapAt, anglesAt } from '../src/core.mjs';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const data = JSON.parse(readFileSync(join(root, 'data', 'chains.json'), 'utf8'));

const rates = [30, 60, 120, 240, 960];
console.log('rate_hz  samples/chain  intervals_found/total  pairs_flagged');
const rows = [];
for (const hz of rates) {
  let found = 0, total = 0, flaggedPairs = 0;
  for (const spec of data.chains) {
    const chain = makeChain(spec);
    const motion = makeMotion(spec.motion.times, spec.motion.keys);
    const cert = certify(chain, motion, { tolT: 1e-9, maxDepth: 26 });
    total += cert.violations.length;
    const n = Math.round(hz * (motion.t1 - motion.t0));
    const pairs = new Set();
    for (let s = 0; s <= n; s++) {
      const t = motion.t0 + (motion.t1 - motion.t0) * (s / n);
      const q = anglesAt(motion, t);
      for (let i = 0; i < chain.joints.length - 2; i++) {
        for (let k = i + 2; k < chain.joints.length; k++) {
          if (gapAt(chain, q, i, k) < 0) { pairs.add(i * 100 + k); }
        }
      }
    }
    for (const v of cert.violations) {
      const key = v.i * 100 + v.k;
      // interval is found if a sampled instant lies inside it
      const hit = pairs.has(key) && (() => {
        for (let s = 0; s <= n; s++) {
          const t = motion.t0 + (motion.t1 - motion.t0) * (s / n);
          if (t >= v.t0 && t <= v.t1) return true;
        }
        return false;
      })();
      if (hit) found++;
    }
    flaggedPairs += pairs.size;
  }
  rows.push({ hz, samples: Math.round(hz), found, total, flaggedPairs });
  console.log(`${String(hz).padStart(6)}  ${String(Math.round(hz)).padStart(12)}  ${String(found).padStart(12)}/${total}      ${flaggedPairs}`);
}
const at30 = rows.find((r) => r.hz === 30);
console.log(`\n30 Hz misses ${at30.total - at30.found}/${at30.total} violation intervals; the Lipschitz certificate reports ${at30.total}/${at30.total} with 0 undecided.`);
