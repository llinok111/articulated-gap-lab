#!/usr/bin/env node
// bench/bench.mjs — real timings: 1 warmup + 9 measured reps, median/p95/stddev,
// plus a separately recorded cold start (first certify call after import).
import { readFileSync, writeFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeChain, makeMotion, certify, minimalAmplitudeScale } from '../src/core.mjs';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const data = JSON.parse(readFileSync(join(root, 'data', 'chains.json'), 'utf8'));
const built = data.chains.map((spec) => ({ spec, chain: makeChain(spec), motion: makeMotion(spec.motion.times, spec.motion.keys) }));

function runCorpus() {
  let pairs = 0, viol = 0, undec = 0;
  for (const b of built) {
    const c = certify(b.chain, b.motion, { tolT: 1e-9, maxDepth: 26 });
    pairs += c.pairs; viol += c.violations.length; undec += c.undecided.length;
  }
  return { pairs, viol, undec };
}

// cold start: very first call, no warmup
let t = performance.now();
const cold = runCorpus();
const coldStartMs = performance.now() - t;

// warmup
runCorpus();

const reps = 9;
const samples = [];
const perChain = {};
for (let r = 0; r < reps; r++) {
  const t0 = performance.now();
  for (const b of built) {
    const a = performance.now();
    certify(b.chain, b.motion, { tolT: 1e-9, maxDepth: 26 });
    (perChain[b.spec.name] ||= []).push(performance.now() - a);
  }
  samples.push(performance.now() - t0);
}
samples.sort((a, b) => a - b);
const median = samples[Math.floor(samples.length / 2)];
const mean = samples.reduce((a, b) => a + b, 0) / samples.length;
const sd = Math.sqrt(samples.reduce((a, b) => a + (b - mean) ** 2, 0) / samples.length);
const p95 = samples[Math.min(samples.length - 1, Math.ceil(0.95 * samples.length) - 1)];

// repair timing (the one chain that needs it), measured once honestly
const b = built.find((x) => x.spec.name === 'fold-b');
const tr = performance.now();
const repair = minimalAmplitudeScale(b.chain, b.motion);
const repairMs = performance.now() - tr;

const perChainMedian = {};
for (const [name, arr] of Object.entries(perChain)) {
  arr.sort((a, b2) => a - b2);
  perChainMedian[name] = +arr[Math.floor(arr.length / 2)].toFixed(3);
}

const out = {
  generatedAt: new Date().toISOString(),
  node: process.version,
  corpus: {
    chains: built.length,
    linksPerChain: built[0].chain.joints.length,
    pairs: cold.pairs,
    violationIntervals: cold.viol,
    undecided: cold.undec,
  },
  certify: {
    samples: samples.length,
    coldStartMs: +coldStartMs.toFixed(3),
    minMs: +samples[0].toFixed(3),
    medianMs: +median.toFixed(3),
    p95Ms: +p95.toFixed(3),
    meanMs: +mean.toFixed(3),
    stddevMs: +sd.toFixed(3),
    relStdPct: +((100 * sd) / mean).toFixed(2),
  },
  perChainMedianMs: perChainMedian,
  repair: {
    chain: 'fold-b',
    ms: +repairMs.toFixed(3),
    scale: repair.scale,
    scalePct: +repair.scalePct.toFixed(3),
    certifiedViolations: repair.certified.violations,
    certifiedUndecided: repair.certified.undecided,
    endEffectorShiftRel: +repair.endEffectorShiftRel.toFixed(5),
  },
};
writeFileSync(join(root, 'bench', 'result.json'), JSON.stringify(out, null, 2) + '\n');
console.log(JSON.stringify(out, null, 2));
