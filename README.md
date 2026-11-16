# articulated-gap-lab

**Continuous-time self-collision certificate for articulated chains: 330 link pairs across six 12-link 6-DOF chains are proven non-intersecting over the whole motion in 117 ms (9-rep median, 330 pairs), with 24 violating intervals localized to 1e-7 s, 0 undecided — the baseline (30 Hz frame sampling, `baseline/naive.mjs`) misses 4 of those 24 intervals and can never prove the gaps between samples.**

Upstream: none

This is not a frame-sampled demo. `src/core.mjs` implements a Lipschitz
branch-and-bound over continuous time: for every link pair it derives a rigorous
upper bound on point speed from the joint rates and the chain's reach matrix, and
either proves `gap(t) > 0` on an interval or returns an instant with `gap(t) < 0`.
Every proof decision is reproducible in node; the browser runs the same module.

## Run it

```bash
node tools/gen-sample.mjs && node tools/gen-cert.mjs   # data/chains.json, data/cert.json
node bench/verify.mjs                                  # 15 assertions, exit 0
node bench/bench.mjs                                   # writes bench/result.json
node baseline/naive.mjs                                # the frame-sampling baseline
python3 -m http.server 4173                             # then open http://localhost:4173
```

No install step, no `node_modules`, no bundler. Three.js r0.170.0 is loaded from
a pinned CDN importmap in `index.html`; the page needs the static server only
because it fetches `data/*.json`.

## Evidence (this run, AMD64 4-core VM, Node v24.19.0)

| what | number |
|---|---|
| corpus | 6 chains × 12 links, 330 non-adjacent link pairs, 9 keyframes each |
| certified pairs | 330/330, **0 undecided** |
| violation intervals | 24 (fold-b: 12, fold-c: 12; 4 clean chains: 0 false positives) |
| interval precision | all 24 are 1-minimal at ε = 1e-7 s (gap flips sign outside, stays negative inside) |
| independent oracle | 1200 uniform samples/chain + golden-section refinement: 24/24 intervals reconfirmed, 0 penetrations outside the certificate |
| scale invariance | scale 1e-6 … 1e6: same 12 intervals, dT ≤ 2.2e-16 s, minGap/s relative error ≤ 4.3e-15 |
| timing | cold start 207.6 ms; 9 reps: median **117.3 ms**, p95 122.4 ms, stddev 2.9 ms (2.47 %) |
| per chain | fold-a 17.6 ms, fold-b 31.6 ms, fold-c 36.1 ms, fold-d 26.1 ms, clean chains 2.3–3.0 ms |
| repair | fold-b: motion amplitude ×0.95817 (−4.18 %), re-certified 0/55 violations, 0 undecided; probe at ×0.96317 violates again (2 intervals) |
| repair cost | end-effector shift 5.02 % of chain reach; repair search 494 ms |
| baseline | 30 Hz sampling finds 20/24 intervals; 60 Hz and above find 24/24 **at the sampled instants only** — sampling cannot certify between samples |
| generator | `rm -rf data && node tools/gen-sample.mjs` twice → identical sha256 (A1) |

### Why slowdown is not a repair
Collision is a configuration-space property. Stretching the same motion 10×
leaves every violation interval unchanged in normalized time (A10: 12 intervals,
normalized dT ≤ 4.4e-16 s), measured in `bench/verify.mjs`. The repair therefore
shrinks joint motion uniformly (`minimalAmplitudeScale`) and re-certifies the
whole motion rather than retiming it.

## 本轮创新

**B 档：把「铰接多刚体链的自碰撞」从逐帧采样推进到连续时间的可复算证书，并把修复定义为可证伪的最小运动幅值收缩。**
Ledger has `toi-cert` (moving triangle-pair TOI), `skinint-cert-lab` (single
skinned mesh, continuous self-intersection) and `camera-retime`, but no
multi-rigid-body chain where the speed bound comes from the articulated reach
matrix `D[j][i]` (distance from joint *j*'s origin to link *i*'s farthest capsule
point — time-invariant for a serial chain). That bound is what makes 330 pairs
provable in 117 ms instead of sampling. The negative result (10× slowdown changes
nothing) and the minimality probe (`s`, `s+0.005` re-certified) are in the same
run, so the claim "minimal" is falsifiable rather than asserted.

## Known limitations (disclosed, not hidden)

- **Scope**: only *within-chain, non-adjacent* pairs. Adjacent links share a joint
  and are excluded by construction (11 per chain); inter-chain collisions are out
  of scope. 330 = 6 × (66 − 11) is exactly this scope.
- **Absolute thresholds**: the only length threshold is the segment-degeneracy
  epsilon `tau = 1e-12 * chain.scale`, where `scale` is the rest-pose reach. There
  is no fixed absolute epsilon anywhere. Time tolerance is `tolT = 1e-9 s`
  (absolute, but time is not scaled in this tool). The certificate uses float64
  without directed rounding, so bounds are conservative up to ordinary rounding
  (~1e-16 relative).
- **Valid scale range**: verified at 1e-6 … 1e6 with geometry (offsets, capsule
  endpoints, radii) scaled together. Below 1e-6, `tau` and float64 ULP approach
  the gaps; above 1e6, absolute time tolerances are unchanged but length terms
  remain relative.
- **Tangency**: a grazing contact (`gap → 0` without crossing) makes the
  branch-and-bound subdivide until `maxDepth`; such pairs are reported in
  `undecided` as a first-class output, never silently dropped. This corpus has 0.
- **"Minimal" repair**: minimal within one fixed family — a single uniform
  amplitude scale for the whole motion. It is not a global joint-space optimum;
  the witness is that `s + 0.005` violates again.
- **Oracle strength**: the oracle is a separate implementation (4×4 FK, exact
  segment-distance candidate enumeration) sampled at 1200 instants per chain plus
  golden-section refinement. It can refute a certificate but cannot by itself
  prove the absence of a sub-sample hit; the proof is the Lipschitz argument.

## Layout

```
src/core.mjs          pure ES module: FK, reach matrix, Lipschitz certificate, repair
main.js               three.js rendering + interaction only
index.html            importmap (three r0.170.0) + control panel
tools/gen-sample.mjs  deterministic corpus (seed 0x5EEDB2026)
tools/gen-cert.mjs    runs the certificate and writes the numbers the page replays
bench/oracle.mjs      independent cross-check (shares no code or constant)
bench/verify.mjs      A1–A15 assertions
bench/bench.mjs       1 warmup + 9 reps + cold start -> bench/result.json
baseline/naive.mjs    fixed-rate frame sampling baseline
```
