// main.js — rendering + interaction only. All geometry and gap numbers come
// from src/core.mjs, the same module bench/verify.mjs calls in node.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { makeChain, makeMotion, gapAt, anglesAt, chainTransforms } from './src/core.mjs';

const stage = document.getElementById('stage');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
stage.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.position.set(4, 3, 5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(2.4, 0, 0);

scene.add(new THREE.HemisphereLight(0xbcd6ff, 0x101820, 2.0));
const key = new THREE.DirectionalLight(0xffffff, 2.2);
key.position.set(3, 5, 4);
scene.add(key);
const grid = new THREE.GridHelper(12, 24, 0x1e2b3a, 0x16202c);
grid.position.y = -1.2;
scene.add(grid);

const state = {
  data: null, cert: null, chain: null, motion: null, group: new THREE.Group(),
  t: 0, playing: false, linkMats: [], violationPairs: [],
};
scene.add(state.group);

const CAP_GEO = new THREE.CylinderGeometry(1, 1, 1, 14, 1, false);
const SPH_GEO = new THREE.SphereGeometry(1, 14, 10);
const matClear = () => new THREE.MeshStandardMaterial({ color: 0x6f8fae, metalness: 0.3, roughness: 0.55 });
const matHit = () => new THREE.MeshStandardMaterial({ color: 0xff5d52, emissive: 0x5a0f0b, metalness: 0.2, roughness: 0.4 });

function disposeGroup() {
  // CAP_GEO/SPH_GEO are shared across links: only per-link materials are owned.
  for (const c of state.group.children) { c.material?.dispose?.(); }
  state.group.clear();
  state.linkMats = [];
}

function buildChainMeshes() {
  disposeGroup();
  const n = state.chain.joints.length;
  for (let i = 0; i < n; i++) {
    const cap = state.chain.joints[i].capsule;
    const len = Math.hypot(cap.b[0] - cap.a[0], cap.b[1] - cap.a[1], cap.b[2] - cap.a[2]);
    const mat = matClear();
    const cyl = new THREE.Mesh(CAP_GEO, mat);
    cyl.userData = { i, kind: 'cyl', len, r: cap.radius };
    const s1 = new THREE.Mesh(SPH_GEO, mat);
    const s2 = new THREE.Mesh(SPH_GEO, mat);
    s1.userData = { i, kind: 'sph', local: cap.a, r: cap.radius };
    s2.userData = { i, kind: 'sph', local: cap.b, r: cap.radius };
    state.group.add(cyl, s1, s2);
    state.linkMats[i] = mat;
  }
}

const _up = new THREE.Vector3(0, 1, 0);
function applyPose(t) {
  const q = anglesAt(state.motion, t);
  const T = chainTransforms(state.chain, q);
  for (const obj of state.group.children) {
    const i = obj.userData.i;
    const M = T[i];
    const rot = new THREE.Matrix4().set(M.R[0], M.R[1], M.R[2], M.t[0], M.R[3], M.R[4], M.R[5], M.t[1], M.R[6], M.R[7], M.R[8], M.t[2], 0, 0, 0, 1);
    if (obj.userData.kind === 'sph') {
      const p = obj.userData.local;
      obj.position.set(M.R[0] * p[0] + M.R[1] * p[1] + M.R[2] * p[2] + M.t[0],
        M.R[3] * p[0] + M.R[4] * p[1] + M.R[5] * p[2] + M.t[1],
        M.R[6] * p[0] + M.R[7] * p[1] + M.R[8] * p[2] + M.t[2]);
      obj.quaternion.identity();
      obj.scale.setScalar(obj.userData.r);
    } else {
      const cap = state.chain.joints[i].capsule;
      const mid = [0, 1, 2].map((d) => (cap.a[d] + cap.b[d]) / 2);
      const w = [0, 1, 2].map((d) => M.R[d] * mid[0] + M.R[3 + d] * mid[1] + M.R[6 + d] * mid[2] + M.t[d]);
      obj.position.set(w[0], w[1], w[2]);
      const dir = new THREE.Vector3(
        M.R[0] * (cap.b[0] - cap.a[0]) + M.R[1] * (cap.b[1] - cap.a[1]) + M.R[2] * (cap.b[2] - cap.a[2]),
        M.R[3] * (cap.b[0] - cap.a[0]) + M.R[4] * (cap.b[1] - cap.a[1]) + M.R[5] * (cap.b[2] - cap.a[2]),
        M.R[6] * (cap.b[0] - cap.a[0]) + M.R[7] * (cap.b[1] - cap.a[1]) + M.R[8] * (cap.b[2] - cap.a[2]),
      );
      const len = dir.length() || 1e-9;
      obj.scale.set(obj.userData.r, len, obj.userData.r);
      obj.quaternion.setFromUnitVectors(_up, dir.clone().normalize());
    }
    obj.matrixAutoUpdate = true;
  }
  return T;
}

function liveStats(t) {
  const q = anglesAt(state.motion, t);
  let best = Infinity, bestPair = null;
  const n = state.chain.joints.length;
  for (let i = 0; i < n - 2; i++) {
    for (let k = i + 2; k < n; k++) {
      const g = gapAt(state.chain, q, i, k);
      if (g < best) { best = g; bestPair = [i, k]; }
    }
  }
  return { best, bestPair };
}

function inViolation(i, k, t) {
  return state.violationPairs.some((v) => v.i === i && v.k === k && t >= v.t0 && t <= v.t1);
}

function refresh() {
  const t = state.t;
  applyPose(t);
  const { best, bestPair } = liveStats(t);
  const hit = bestPair ? inViolation(bestPair[0], bestPair[1], t) : false;
  // highlight links that participate in an active certified violation
  const active = new Set();
  for (const v of state.violationPairs) if (t >= v.t0 && t <= v.t1) { active.add(v.i); active.add(v.k); }
  for (let i = 0; i < state.linkMats.length; i++) {
    const mat = state.linkMats[i];
    if (active.has(i)) { mat.color.setHex(0xff5d52); mat.emissive.setHex(0x4a0d0a); }
    else { mat.color.setHex(0x6f8fae); mat.emissive.setHex(0x000000); }
  }
  document.getElementById('live').textContent = best.toFixed(5);
  document.getElementById('live').className = best < 0 ? 'bad' : 'ok';
  document.getElementById('pair').textContent = bestPair ? `(${bestPair[0]}, ${bestPair[1]})` : '–';
  document.getElementById('rel').textContent = (best / state.chain.scale).toExponential(3);
  const inv = document.getElementById('inv');
  inv.textContent = hit ? 'yes' : 'no';
  inv.className = hit ? 'bad' : 'ok';
  document.getElementById('tval').textContent = `= ${t.toFixed(4)} s`;
}

function loadChain(name) {
  const spec = state.data.chains.find((c) => c.name === name);
  const cert = state.cert.chains.find((c) => c.name === name);
  state.chain = makeChain(spec);
  state.motion = makeMotion(spec.motion.times, spec.motion.keys);
  state.violationPairs = cert.certificate.violations;
  buildChainMeshes();
  state.t = 0;
  document.getElementById('cPairs').textContent = String(cert.certificate.pairs);
  document.getElementById('cViol').textContent = String(cert.certificate.violationCount);
  document.getElementById('cUndec').textContent = String(cert.certificate.undecided);
  const rep = cert.repair;
  document.getElementById('cRepair').textContent = rep && rep.found
    ? `${(rep.scale * 100).toFixed(2)}% of motion (−${rep.scalePct.toFixed(2)}%)`
    : 'not needed / not found';
  document.getElementById('cShift').textContent = rep && rep.found
    ? `${(rep.endEffectorShiftRel * 100).toFixed(2)}% of reach` : '–';
  const R = state.chain.scale;
  controls.target.set(R * 0.55, 0, 0);
  camera.position.set(R * 0.9, R * 0.55, R * 1.15);
  camera.near = Math.max(1e-4, R / 500);
  camera.far = R * 40;
  camera.updateProjectionMatrix();
  controls.update();
  refresh();
}

const slider = document.getElementById('time');
slider.addEventListener('input', () => {
  state.t = state.motion.t0 + (state.motion.t1 - state.motion.t0) * (Number(slider.value) / 1000);
  refresh();
});
document.getElementById('play').addEventListener('click', (e) => {
  state.playing = !state.playing;
  e.target.textContent = state.playing ? 'pause' : 'play';
});
document.getElementById('jump').addEventListener('click', () => {
  const vs = state.violationPairs;
  if (!vs.length) return;
  const nxt = vs.find((v) => v.t0 > state.t + 1e-6) || vs[0];
  state.t = 0.5 * (nxt.t0 + nxt.t1);
  slider.value = String(Math.round(1000 * (state.t - state.motion.t0) / (state.motion.t1 - state.motion.t0)));
  refresh();
});
document.getElementById('chain').addEventListener('change', (e) => loadChain(e.target.value));

function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

let last = performance.now();
function loop(now) {
  const dt = Math.min(0.05, (now - last) / 1000);
  last = now;
  if (state.playing) {
    const span = state.motion.t1 - state.motion.t0;
    state.t += dt * 0.35;
    if (state.t > state.motion.t1) state.t = state.motion.t0;
    slider.value = String(Math.round(1000 * (state.t - state.motion.t0) / span));
    refresh();
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}

const [data, cert] = await Promise.all([
  fetch('./data/chains.json').then((r) => r.json()),
  fetch('./data/cert.json').then((r) => r.json()),
]);
state.data = data;
state.cert = cert;
const sel = document.getElementById('chain');
for (const c of data.chains) {
  const o = document.createElement('option');
  o.value = c.name;
  o.textContent = `${c.name} · ${c.joints.length} links`;
  sel.appendChild(o);
}
resize();
loadChain(data.chains[0].name);
requestAnimationFrame(loop);
