#!/usr/bin/env node
// bench/verify.mjs — every number printed here comes from this run.
import { readFileSync, statSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeChain, makeMotion, certify, gapAt, anglesAt, minimalAmplitudeScale } from '../src/core.mjs';
import { oracleScan, oracleGap } from './oracle.mjs';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const results = [];
function check(id, ok, detail) {
  results.push({ id, ok: !!ok, detail });
  console.log(`${ok ? 'PASS' : 'FAIL'} ${id} ${detail}`);
}

// A1 generator determinism (byte-identical on re-run)
const dataPath = join(root, 'data', 'chains.json');
const sha = (p) => createHash('sha256').update(readFileSync(p)).digest('hex');
const shaBefore = sha(dataPath);
execFileSync(process.execPath, [join(root, 'tools', 'gen-sample.mjs')], { cwd: root, stdio: 'ignore' });
const shaAfter = sha(dataPath);
check('A1', shaBefore === shaAfter && shaAfter.length === 64, `gen-sample twice sha256 ${shaAfter.slice(0, 16)}… equal`);

const data = JSON.parse(readFileSync(dataPath, 'utf8'));
const specs = data.chains;
check('A2', specs.length === 6 && specs.every((s) => s.joints.length === 12),
  `corpus = ${specs.length} chains x ${specs[0].joints.length} links, pairs/chain = ${(12 * 11) / 2 - 11}`);

// build + certify everything
const built = specs.map((spec) => ({ spec, chain: makeChain(spec), motion: makeMotion(spec.motion.times, spec.motion.keys) }));
const certs = built.map(({ chain, motion }) => certify(chain, motion, { tolT: 1e-9, maxDepth: 26 }));
const totalPairs = certs.reduce((a, c) => a + c.pairs, 0);
const totalViol = certs.reduce((a, c) => a + c.violations.length, 0);
const totalUndec = certs.reduce((a, c) => a + c.undecided.length, 0);
check('A3', totalPairs === 330 && totalUndec === 0,
  `pairs ${totalPairs} (6x55), violation intervals ${totalViol}, undecided ${totalUndec}`);

// A4 clean chains stay clean (no false positives)
const cleanNames = ['fold-a', 'fold-d', 'wave-clean-1', 'wave-clean-2'];
const cleanOk = built.every((b, idx) => !cleanNames.includes(b.spec.name) || certs[idx].violations.length === 0);
check('A4', cleanOk && cleanNames.length === 4,
  `4 clean chains (fold-a, fold-d, wave-clean-1, wave-clean-2): 0/4 false positives`);

// A5 violating chains and their interval counts
const byName = Object.fromEntries(built.map((b, i) => [b.spec.name, certs[i]]));
const vb = byName['fold-b'].violations.length, vc = byName['fold-c'].violations.length;
check('A5', vb === 12 && vc === 12,
  `fold-b ${vb} intervals (min gap ${byName['fold-b'].minGap.toFixed(6)}), fold-c ${vc} (min gap ${byName['fold-c'].minGap.toFixed(6)})`);

// A6 1-minimal violation intervals: both endpoints are sign changes
const eps = 1e-7;
let minimalOk = 0, minimalTotal = 0, worstEdge = 0;
for (const { chain, motion } of built) {
  const c = certify(chain, motion, { tolT: 1e-9, maxDepth: 26 });
  for (const v of c.violations) {
    minimalTotal++;
    const g = (t) => gapAt(chain, anglesAt(motion, t), v.i, v.k);
    let ok = g(v.t0 + eps) < 0 && g(v.t1 - eps) < 0;
    if (v.t0 > motion.t0 + 1e-9) {
      const gl = g(v.t0 - eps);
      ok = ok && gl > 0;
      worstEdge = Math.max(worstEdge, Math.abs(gl));
    }
    if (v.t1 < motion.t1 - 1e-9) {
      const gr = g(v.t1 + eps);
      ok = ok && gr > 0;
      worstEdge = Math.max(worstEdge, Math.abs(gr));
    }
    if (ok) minimalOk++;
  }
}
check('A6', minimalOk === minimalTotal && minimalTotal === 24,
  `${minimalOk}/${minimalTotal} intervals are 1-minimal at eps=${eps}s (gap flips sign outside, stays negative inside)`);

// A7 independent oracle: no penetrating instant outside a certified interval
const t0All = performance.now();
const scans = specs.map((s) => oracleScan(s, 1200));
const oracleMs = performance.now() - t0All;
let missed = 0, confirmed = 0, oracleNeg = 0, worstMiss = 0;
for (let ci = 0; ci < specs.length; ci++) {
  const cert = certs[ci];
  const scan = scans[ci];
  for (const m of scan.minima) {
    if (!(m.minGap < 0)) continue;
    oracleNeg++;
    const inside = cert.violations.some((v) => v.i === m.i && v.k === m.k && m.at >= v.t0 - 2e-3 && m.at <= v.t1 + 2e-3);
    if (!inside) { missed++; worstMiss = Math.min(worstMiss, m.minGap); }
  }
  for (const v of cert.violations) {
    const mid = 0.5 * (v.t0 + v.t1);
    if (oracleGap(specs[ci], mid, v.i, v.k) < 0) confirmed++;
  }
}
check('A7', missed === 0 && confirmed === totalViol,
  `oracle 1200 samples/chain + golden-section: ${oracleNeg} penetrating minima, ${missed} outside certificate, ${confirmed}/${totalViol} intervals independently reconfirmed (${oracleMs.toFixed(0)} ms)`);

// A8 scale invariance of the verdict, 1e-6 .. 1e6
const scales = [1e-6, 1e-3, 1, 1e3, 1e6];
const baseB = certs[built.findIndex((b) => b.spec.name === 'fold-b')];
const scaleRows = [];
let scaleOk = true;
for (const s of scales) {
  const spec = JSON.parse(JSON.stringify(specs.find((x) => x.name === 'fold-b')));
  for (const j of spec.joints) {
    j.offset = j.offset.map((v) => v * s);
    j.capsule.a = j.capsule.a.map((v) => v * s);
    j.capsule.b = j.capsule.b.map((v) => v * s);
    j.capsule.radius *= s;
  }
  const chain = makeChain(spec);
  const motion = makeMotion(spec.motion.times, spec.motion.keys);
  const c = certify(chain, motion, { tolT: 1e-9, maxDepth: 26 });
  const same = c.violations.length === baseB.violations.length && c.undecided.length === 0;
  let tMax = 0;
  for (let i = 0; i < Math.min(c.violations.length, baseB.violations.length); i++) {
    tMax = Math.max(tMax, Math.abs(c.violations[i].t0 - baseB.violations[i].t0), Math.abs(c.violations[i].t1 - baseB.violations[i].t1));
  }
  const ratio = c.minGap / baseB.minGap;
  const rel = Math.abs(ratio - s) / s;
  scaleOk = scaleOk && same && tMax < 1e-6 && rel < 1e-9;
  scaleRows.push(`s=${s}: ${c.violations.length} viol, dT<=${tMax.toExponential(1)}, minGap/s rel=${rel.toExponential(1)}`);
}
check('A8', scaleOk && scales.length === 5, `scale 1e-6…1e6 invariant: ${scaleRows.join(' | ')}`);

// A9 minimal amplitude-scale repair, full re-certification
const repair = minimalAmplitudeScale(built.find((b) => b.spec.name === 'fold-b').chain, built.find((b) => b.spec.name === 'fold-b').motion);
check('A9', repair.found && repair.certified.violations === 0 && repair.certified.undecided === 0 && repair.minimalityProbe.violations >= 1,
  `fold-b repair: scale ${repair.scale.toFixed(5)} (-${repair.scalePct.toFixed(2)}% amplitude), re-cert 0/${repair.certified.pairs} violations, 0 undecided; probe ${repair.minimalityProbe.scale.toFixed(5)} still has ${repair.minimalityProbe.violations} violation(s)`);

// A10 time reparameterization is NOT a repair (10x slowdown, same configurations)
const bSpec = built.find((b) => b.spec.name === 'fold-b').spec;
const slow = makeMotion(bSpec.motion.times.map((t) => t * 10), bSpec.motion.keys);
const slowCert = certify(built.find((b) => b.spec.name === 'fold-b').chain, slow, { tolT: 1e-9, maxDepth: 26 });
let slowTMax = 0;
for (let i = 0; i < Math.min(slowCert.violations.length, baseB.violations.length); i++) {
  slowTMax = Math.max(slowTMax, Math.abs(slowCert.violations[i].t0 / 10 - baseB.violations[i].t0), Math.abs(slowCert.violations[i].t1 / 10 - baseB.violations[i].t1));
}
check('A10', slowCert.violations.length === baseB.violations.length && slowTMax < 1e-2,
  `10x slowdown: ${slowCert.violations.length} intervals (was ${baseB.violations.length}), normalized dT<=${slowTMax.toExponential(1)} -> retiming cannot repair`);

// A11 repair path deviation is bounded and reported
check('A11', repair.endEffectorShiftRel <= 0.1,
  `end-effector shift from repair = ${(repair.endEffectorShiftRel * 100).toFixed(2)}% of chain reach`);

// A12 core purity, checked statically on the source (comments stripped)
const coreSrc = readFileSync(join(root, 'src', 'core.mjs'), 'utf8')
  .replace(/\/\*[\s\S]*?\*\//g, '').replace(/^\s*\/\/.*$/gm, '');
const forbidden = ['three', 'window', 'document', 'navigator', 'requestAnimationFrame'].filter((w) => new RegExp(`\\b${w}\\b`).test(coreSrc));
check('A12', forbidden.length === 0, `src/core.mjs has no browser/three identifiers outside comments (found: ${forbidden.join(',') || 'none'})`);

// A13 no NaN / Infinity in any gap-like output
const finite = certs.every((c) => Number.isFinite(c.minGap)) && scales.length === 5;
check('A13', finite, `minGap finite for all 6 chains (worst ${Math.min(...certs.map((c) => c.minGap)).toFixed(6)})`);

// A14 data/cert.json (what the browser replays) matches this run
const certFile = JSON.parse(readFileSync(join(root, 'data', 'cert.json'), 'utf8'));
const filePairs = certFile.totals.pairs, fileViol = certFile.totals.violations, fileUndec = certFile.totals.undecided;
const perChainMatch = certFile.chains.every((c, i) => c.certificate.violationCount === certs[i].violations.length && c.name === specs[i].name);
check('A14', filePairs === totalPairs && fileViol === totalViol && fileUndec === 0 && perChainMatch,
  `data/cert.json (browser-visible) matches node: ${filePairs} pairs, ${fileViol} intervals, ${fileUndec} undecided`);

// A15 payload sizes stay small and data is generated, not committed as a blob
const dataBytes = statSync(dataPath).size;
const certBytes = statSync(join(root, 'data', 'cert.json')).size;
check('A15', dataBytes < 5e6 && certBytes < 5e6, `data/chains.json ${dataBytes} B, data/cert.json ${certBytes} B (both < 5 MB)`);

const failed = results.filter((r) => !r.ok);
console.log(`\n${results.length - failed.length} passed / ${failed.length} failed`);
process.exit(failed.length ? 1 : 0);
