#!/usr/bin/env node
// tools/gen-sample.mjs — deterministic articulated-chain corpus.
// Same seed => byte-identical data/chains.json (verified by bench/verify.mjs).
import { mkdirSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const root = join(here, '..');

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const LINKS = 12;
const KEYS = 9;

// collide: a fold-back sweep whose joint angles reach ~pi, so non-adjacent
// links on the same chain cross. clean: a small wiggle that never folds.
const PLANS = [
  { name: 'fold-a', amp: 1.26, freq: 0.5, phase: 0.0, collide: true },
  { name: 'fold-b', amp: 1.32, freq: 0.5, phase: 0.9, collide: true },
  { name: 'fold-c', amp: 1.30, freq: 0.5, phase: 1.7, collide: true },
  { name: 'fold-d', amp: 1.24, freq: 0.75, phase: 2.4, collide: true },
  { name: 'wave-clean-1', amp: 0.22, freq: 1.0, phase: 0.3, collide: false },
  { name: 'wave-clean-2', amp: 0.30, freq: 0.5, phase: 1.1, collide: false },
];

function makeChain(plan, rnd) {
  const tilt = plan.name.charCodeAt(0) % 2 === 0 ? 1 : -1;
  const joints = [];
  for (let i = 0; i < LINKS; i++) {
    const zx = 0.18 * Math.sin(1.7 * i + plan.phase) * tilt;
    const zy = 0.12 * Math.cos(1.1 * i + plan.phase);
    joints.push({
      axis: [zx, zy, 1],
      offset: i === 0 ? [0, 0, 0] : [0.45, 0.02 * Math.sin(i + plan.phase), 0.01 * Math.cos(i)],
      capsule: {
        a: [0, 0, 0],
        b: [0.40 + 0.004 * ((i * 7) % 3), 0, 0],
        radius: 0.085 + 0.004 * ((i * 5) % 3),
      },
    });
  }
  const times = [];
  for (let k = 0; k < KEYS; k++) times.push(+(k / (KEYS - 1)).toFixed(6));
  const keys = [];
  for (let k = 0; k < KEYS; k++) {
    const row = [];
    for (let j = 0; j < LINKS; j++) {
      const u = k / (KEYS - 1);
      const ph = 2 * Math.PI * plan.freq * u + plan.phase + (j * Math.PI) / 5;
      let q = plan.amp * Math.sin(ph);
      // deterministic jitter keeps the corpus off exact symmetry
      q += 0.05 * (rnd() - 0.5);
      if (j === 0) q *= 0.35;
      row.push(+q.toFixed(9));
    }
    keys.push(row);
  }
  return { name: plan.name, joints, motion: { times, keys } };
}

const rnd = mulberry32(0x5EEDB2026);
const chains = PLANS.map((p) => makeChain(p, rnd));
const out = {
  generator: 'tools/gen-sample.mjs',
  seed: '0x5EEDB2026',
  version: 1,
  linksPerChain: LINKS,
  keys: KEYS,
  chains,
};

mkdirSync(join(root, 'data'), { recursive: true });
const json = JSON.stringify(out, null, 1) + '\n';
writeFileSync(join(root, 'data', 'chains.json'), json);
console.log(`wrote data/chains.json: ${chains.length} chains x ${LINKS} links, ${json.length} bytes`);
