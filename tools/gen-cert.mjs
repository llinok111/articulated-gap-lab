#!/usr/bin/env node
// tools/gen-cert.mjs — run the certificate + repair on data/chains.json and
// write the exact numbers the browser replays (browser numbers == node numbers).
import { readFileSync, writeFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeChain, makeMotion, certify, minimalAmplitudeScale, summarize } from '../src/core.mjs';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const data = JSON.parse(readFileSync(join(root, 'data', 'chains.json'), 'utf8'));

const out = { source: 'tools/gen-cert.mjs', chains: [], totals: { pairs: 0, violations: 0, undecided: 0, intervalsCertified: 0 } };
for (const spec of data.chains) {
  const chain = makeChain(spec);
  const motion = makeMotion(spec.motion.times, spec.motion.keys);
  const cert = certify(chain, motion, { tolT: 1e-9, maxDepth: 26 });
  const repair = cert.violations.length ? minimalAmplitudeScale(chain, motion) : { needed: false };
  out.chains.push({
    name: spec.name,
    links: chain.joints.length,
    scale: chain.scale,
    certificate: {
      ...summarize(cert),
      violationCount: cert.violations.length,
      violations: cert.violations.map((v) => ({
        i: v.i, k: v.k,
        t0: +v.t0.toFixed(9), t1: +v.t1.toFixed(9), minGap: v.minGap,
      })),
    },
    repair: repair.needed === false ? { needed: false } : {
      needed: true,
      found: repair.found,
      scale: repair.scale,
      scalePct: repair.scalePct,
      endEffectorShiftRel: repair.endEffectorShiftRel,
      minimalityProbe: repair.minimalityProbe,
      certified: repair.certified,
    },
  });
  out.totals.pairs += cert.pairs;
  out.totals.violations += cert.violations.length;
  out.totals.undecided += cert.undecided.length;
  out.totals.intervalsCertified += cert.intervalsCertified;
}
writeFileSync(join(root, 'data', 'cert.json'), JSON.stringify(out, null, 1) + '\n');
console.log(`cert: ${out.totals.pairs} pairs, ${out.totals.violations} violation intervals, ${out.totals.undecided} undecided`);
