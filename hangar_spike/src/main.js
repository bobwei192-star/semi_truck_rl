import '@fontsource/rajdhani/500.css';
import '@fontsource/rajdhani/600.css';
import '@fontsource/rajdhani/700.css';
import './style.css';

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

import { buildTruck } from './truck.js';
import { FakeParkingSim, SCENE_ORIGIN } from './fakeSim.js';
import { SCENE } from './sceneConfig.js';
import { LiveSim } from './liveSim.js';
import { createAudioEngine } from './audio.js';

const audio = createAudioEngine();
if (import.meta.env.DEV) window.__audio = audio;

/* ---------------- sim mode ---------------- */

const params = new URLSearchParams(location.search);
const useFake = params.get('demo') === '1';
const wsOverride = params.get('ws');
const wsUrl = wsOverride
  || `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;

let sim;
if (useFake) {
  sim = new FakeParkingSim();
} else {
  sim = new LiveSim({
    url: wsUrl,
    onPhase: handleSimPhase,
    onToast: toast,
  });
  // 初始摆位在 Step 4 (buildTruck 之后) 完成
}

/* ---------------- renderer / scene ---------------- */

const canvas = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x060a0e);
scene.fog = new THREE.FogExp2(0x060a0e, 0.014);
scene.environmentIntensity = 0.7;

const HOME_CAM = new THREE.Vector3(16, 10, -42);
const HOME_TARGET = new THREE.Vector3(-2, 2.0, -17);
const FOLLOW_OFFSET = new THREE.Vector3(18, 8, -18);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 400);
camera.position.copy(HOME_CAM);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.copy(HOME_TARGET);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.minDistance = 6;
controls.maxDistance = 80;
controls.maxPolarAngle = Math.PI * 0.495;
controls.autoRotate = true;
controls.autoRotateSpeed = 0.35;
controls.update();

let follow = false;
controls.addEventListener('start', () => {
  controls.autoRotate = false;
  follow = false;
});

const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

/* ---------------- lights ---------------- */

const hemi = new THREE.HemisphereLight(0x9db7cc, 0x070a0d, 0.55);
scene.add(hemi);

const key = new THREE.SpotLight(0xffd6a0, 900, 90, 0.62, 0.65, 1.7);
key.position.set(11, 17, 7);
key.target.position.set(0, 0.8, -2);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.bias = -0.0003;
key.shadow.normalBias = 0.02;
scene.add(key, key.target);

const rim = new THREE.SpotLight(0x6ea8c9, 650, 90, 0.7, 0.75, 1.8);
rim.position.set(-16, 12, 16);
rim.target.position.set(0, 1.4, 0);
scene.add(rim, rim.target);

const fill = new THREE.PointLight(0xffb45e, 55, 45, 2);
fill.position.set(-5, 7, -18);
scene.add(fill);

const startLight = new THREE.SpotLight(0xffd0a0, 520, 70, 0.6, 0.7, 1.8);
startLight.position.set(7, 13, -38);
startLight.target.position.set(-2, 1.2, -18);
scene.add(startLight, startLight.target);

/* ---------------- hangar ---------------- */

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(180, 180),
  new THREE.MeshStandardMaterial({ color: 0x10161c, roughness: 0.72, metalness: 0.28 })
);
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);

const grid = new THREE.GridHelper(180, 90, 0x26323c, 0x141b21);
grid.position.y = 0.012;
grid.material.transparent = true;
grid.material.opacity = 0.42;
scene.add(grid);

const wallMat = new THREE.MeshStandardMaterial({ color: 0x0c1116, roughness: 0.9, metalness: 0.36 });
const beamMat = new THREE.MeshStandardMaterial({ color: 0x1b242c, roughness: 0.6, metalness: 0.7 });

function hangarBox(w, h, d, x, y, z, mat = wallMat) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  scene.add(m);
  return m;
}

hangarBox(120, 24, 1.2, 0, 12, -42);
hangarBox(1.2, 24, 96, -42, 12, -2);
hangarBox(1.2, 24, 96, 42, 12, -2);

for (let z = -38; z <= 32; z += 10) {
  hangarBox(84, 0.35, 0.35, 0, 13.5, z, beamMat);
  hangarBox(84, 0.18, 0.18, 0, 13.0, z, beamMat);
  const strip = new THREE.Mesh(
    new THREE.BoxGeometry(62, 0.06, 0.14),
    new THREE.MeshStandardMaterial({
      color: 0x2b1b09, emissive: 0xffb45e, emissiveIntensity: 2.4, roughness: 0.5,
    })
  );
  strip.position.set(0, 12.85, z);
  scene.add(strip);
}

for (let x = -36; x <= 36; x += 12) {
  hangarBox(0.3, 24, 0.3, x, 6, -42, beamMat);
}

// faint service lights along the back wall
for (let x = -32; x <= 32; x += 16) {
  const lamp = new THREE.PointLight(0x9fc4dd, 14, 26, 2);
  lamp.position.set(x, 8, -37);
  scene.add(lamp);
}

// a couple of crates for depth
hangarBox(3.2, 2.2, 2.4, -22, 1.1, -18, beamMat);
hangarBox(2.2, 1.4, 2.2, -25, 0.7, -15, beamMat);
hangarBox(4.4, 1.0, 2.0, 24, 0.5, 8, beamMat);

/* ---------------- road markings ---------------- */

const S = SCENE_ORIGIN;

function toScene(x, y, h = 0.03) {
  return new THREE.Vector3(x - S.x, h, y - S.y);
}

function line(points, color, opacity = 1, dashed = false) {
  const geo = new THREE.BufferGeometry().setFromPoints(points);
  const mat = dashed
    ? new THREE.LineDashedMaterial({ color, transparent: true, opacity, dashSize: 0.9, gapSize: 0.7 })
    : new THREE.LineBasicMaterial({ color, transparent: true, opacity });
  const l = new THREE.Line(geo, mat);
  if (dashed) l.computeLineDistances();
  scene.add(l);
  return l;
}

line([toScene(SCENE.road.left, SCENE.road.yStart), toScene(SCENE.road.left, SCENE.road.yEnd)], 0xd8dde2, 0.55);
line([toScene(SCENE.road.right, SCENE.road.yStart), toScene(SCENE.road.right, SCENE.road.yEnd)], 0xd8dde2, 0.55);
line([toScene(SCENE.road.center, SCENE.road.yStart), toScene(SCENE.road.center, SCENE.road.yEnd)], 0xd8dde2, 0.3, true);

// target dock: SCENE.slot
const slot = [
  toScene(SCENE.slot.left, SCENE.slot.top), toScene(SCENE.slot.right, SCENE.slot.top),
  toScene(SCENE.slot.right, SCENE.slot.bottom), toScene(SCENE.slot.left, SCENE.slot.bottom),
  toScene(SCENE.slot.left, SCENE.slot.top),
];
line(slot, 0xe39a2d, 0.95);

for (const [x, y] of [
  [SCENE.slot.left, SCENE.slot.top], [SCENE.slot.right, SCENE.slot.top],
  [SCENE.slot.right, SCENE.slot.bottom], [SCENE.slot.left, SCENE.slot.bottom],
]) {
  const c = toScene(x, y);
  line([new THREE.Vector3(c.x - 0.9, 0.04, c.z), new THREE.Vector3(c.x + 0.9, 0.04, c.z)], 0xf7c766, 1);
  line([new THREE.Vector3(c.x, 0.04, c.z - 0.9), new THREE.Vector3(c.x, 0.04, c.z + 0.9)], 0xf7c766, 1);
}

function floorLabel(text, x, y, w = 5, h = 2.5) {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 256;
  const g = c.getContext('2d');
  g.clearRect(0, 0, 512, 256);
  g.font = '700 120px Rajdhani, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillStyle = 'rgba(227,154,45,0.75)';
  g.fillText(text, 256, 132);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const m = new THREE.Mesh(
    new THREE.PlaneGeometry(w, h),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, opacity: 0.7, depthWrite: false })
  );
  m.rotation.set(-Math.PI / 2, 0, Math.PI);
  m.position.set(x - S.x, 0.035, y - S.y);
  scene.add(m);
}

floorLabel(
  'DOCK 01',
  (SCENE.slot.left + SCENE.slot.right) / 2,
  Math.min(SCENE.slot.top, SCENE.slot.bottom) - 3.5,
  4.6, 2.3
);

// start pad
const pad = new THREE.Mesh(
  new THREE.RingGeometry(3.4, 3.55, 64),
  new THREE.MeshBasicMaterial({ color: 0xe39a2d, transparent: true, opacity: 0.5, side: THREE.DoubleSide })
);
pad.rotation.x = -Math.PI / 2;
pad.position.set(SCENE.start.x - S.x, 0.025, SCENE.start.y - S.y);
scene.add(pad);

/* ---------------- truck ---------------- */

const truck = buildTruck();
scene.add(truck.group);
if (useFake) {
  truck.setState(sim.state);
} else {
  truck.setState({ ...SCENE.start, steer: 0, speed: 0 }, 0);
}

/* ---------------- post processing ---------------- */

const composer = new EffectComposer(renderer);
composer.addPass(new RenderPass(scene, camera));
const bloom = new UnrealBloomPass(new THREE.Vector2(window.innerWidth, window.innerHeight), 0.42, 0.55, 0.82);
composer.addPass(bloom);
composer.addPass(new OutputPass());

/* ---------------- UI ---------------- */

const el = {
  artAngle: document.getElementById('artAngle'),
  steerAngle: document.getElementById('steerAngle'),
  speed: document.getElementById('speed'),
  progressPct: document.getElementById('progressPct'),
  progressBar: document.getElementById('progressBar'),
  stateText: document.getElementById('stateText'),
  stateLed: document.getElementById('stateLed'),
  simPill: document.getElementById('simPill'),
  simPillText: document.getElementById('simPillText'),
  startBtn: document.getElementById('startBtn'),
  resetBtn: document.getElementById('resetBtn'),
  toast: document.getElementById('toast'),
  mmStatus: document.getElementById('mmStatus'),
  controllerList: document.getElementById('controllerList'),
  latError: document.getElementById('latError'),
  psiError: document.getElementById('psiError'),
  doneSummary: document.getElementById('doneSummary'),
};

let phase = 'idle';
let toastTimer = 0;

function toast(msg) {
  el.toast.textContent = msg;
  el.toast.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.toast.classList.remove('show'), 2600);
}

function setButtonLabel(text, glyph) {
  el.startBtn.textContent = text;
  const s = document.createElement('span');
  s.textContent = glyph;
  el.startBtn.appendChild(s);
}

function setPhase(next) {
  phase = next;
  const live = !useFake;
  const label = live ? 'LIVE · CASCADE' : 'FAKE DATA';
  if (phase === 'running') {
    el.simPill.classList.add('live');
    el.simPill.classList.remove('done', 'paused', 'offline');
    el.simPillText.textContent = live ? `RUNNING · ${label}` : 'DEMO RUNNING · FAKE DATA';
    el.startBtn.disabled = true;
    el.startBtn.classList.add('running');
    setButtonLabel('演示中… ', '■');
    el.mmStatus.textContent = 'REVERSING';
    el.stateLed.className = 'led on';
    el.stateText.textContent = '倒车中 · REVERSING';
  } else if (phase === 'done') {
    el.simPill.classList.remove('live');
    el.simPill.classList.add('done');
    el.simPill.classList.remove('paused', 'offline');
    el.simPillText.textContent = `DOCKED · ${label}`;
    el.startBtn.disabled = false;
    el.startBtn.classList.remove('running');
    setButtonLabel('再次演示 ', '▶');
    el.mmStatus.textContent = 'DOCKED';
    el.stateLed.className = 'led ok';
    el.stateText.textContent = '已入库 · DOCKED';
  } else if (phase === 'paused') {
    el.simPill.classList.remove('live', 'done', 'offline');
    el.simPill.classList.add('paused');
    el.simPillText.textContent = `PAUSED · ${label}`;
    el.startBtn.disabled = false;
    el.startBtn.classList.remove('running');
    setButtonLabel('继续 ', '▶');
    el.mmStatus.textContent = 'PAUSED';
    el.stateLed.className = 'led';
    el.stateText.textContent = '已暂停 · PAUSED';
  } else if (phase === 'failed') {
    el.simPill.classList.remove('live', 'done', 'paused', 'offline');
    el.simPillText.textContent = `FAILED · ${label}`;
    el.startBtn.disabled = false;
    el.startBtn.classList.remove('running');
    setButtonLabel('重新开始 ', '▶');
    el.mmStatus.textContent = 'FAILED';
    el.stateLed.className = 'led';
    el.stateText.textContent = '入库失败 · FAILED';
  } else if (phase === 'offline') {
    el.simPill.classList.remove('live', 'done', 'paused');
    el.simPill.classList.add('offline');
    el.simPillText.textContent = 'RECONNECTING · CASCADE';
    el.startBtn.disabled = true;
    el.startBtn.classList.remove('running');
    setButtonLabel('等待后端… ', '▶');
    el.mmStatus.textContent = 'OFFLINE';
    el.stateText.textContent = '后端未连接';
  } else {  // ready / idle
    el.simPill.classList.remove('live', 'done', 'paused', 'offline');
    el.simPillText.textContent = live ? `STANDBY · ${label}` : `SIM OFFLINE · ${label}`;
    el.startBtn.disabled = live && !sim.connected;
    el.startBtn.classList.remove('running');
    setButtonLabel('开始演示 ', '▶');
    el.mmStatus.textContent = 'STANDBY';
    el.stateLed.className = 'led';
    el.stateText.textContent = '待命 · STANDBY';
    if (live) el.stateText.textContent = '待命 · 等待开始';
  }
}

function handleSimPhase(phase) {
  setPhase(phase);
  if (phase === 'ready' || phase === 'offline') {
    el.doneSummary.textContent = '';
    el.doneSummary.className = 'done-summary';
  }
  if (phase === 'done' && sim.doneInfo) {
    const d = sim.doneInfo;
    el.doneSummary.className = 'done-summary ok';
    el.doneSummary.textContent =
      `入库成功 · ${d.steps} 步 · ${d.simTime.toFixed(1)} s · ` +
      `违规 ${d.violationSteps} · 最大横向误差 ${d.maxLateralError.toFixed(2)} m`;
    toast('入库成功 · 真实串级控制轨迹');
  } else if (phase === 'failed' && sim.doneInfo) {
    const d = sim.doneInfo;
    el.doneSummary.className = 'done-summary bad';
    el.doneSummary.textContent =
      `入库失败 · ${d.steps} 步 · 违规 ${d.violationSteps} 步`;
    toast('入库失败 · 详见汇总');
  } else if (phase === 'offline') {
    toast('后端未连接；离线可加 ?demo=1 看假数据');
  }
}

function startDemo() {
  if (useFake) {
    if (sim.running) return;
    if (sim.done) sim.reset();
  } else {
    if (!sim.connected || sim.running) return;
    if (phase === 'done' || phase === 'failed') sim.reset();
  }
  audio.unlock();
  sim.start();
  follow = true;
  controls.autoRotate = false;
  if (useFake) {
    setPhase('running');
    toast('假数据演示开始 · 轨迹由前端生成');
  }
}

function resetDemo() {
  sim.reset();
  if (useFake) {
    truck.setState(sim.state);
    follow = false;
    controls.autoRotate = true;
    camera.position.copy(HOME_CAM);
    controls.target.copy(HOME_TARGET);
    controls.update();
    setPhase('idle');
  }
  el.doneSummary.textContent = '';
  el.doneSummary.className = 'done-summary';
  toast('已重置');
}

el.startBtn.addEventListener('click', startDemo);
el.resetBtn.addEventListener('click', resetDemo);
document.getElementById('settingsBtn').addEventListener('click', () => toast('设置入口占位'));

window.addEventListener('keydown', (e) => {
  if (e.code === 'Space') {
    e.preventDefault();
    if (useFake) {
      if (phase === 'running') resetDemo();   // fake 保持原语义
      else startDemo();
    } else if (phase === 'running') sim.pause();
    else if (phase === 'paused') sim.resume();
    else if (phase === 'ready' || phase === 'done' || phase === 'failed') startDemo();
  } else if (e.key === 'r' || e.key === 'R') {
    resetDemo();
  } else if (e.key === 'm' || e.key === 'M') {
    toast(`音效：${audio.toggleMute() ? '已静音' : '开启'}`);
  }
});

document.querySelectorAll('.rail-item').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.rail-item').forEach((b) => b.classList.remove('active'));
    btn.classList.add('active');
    document.querySelectorAll('.panel-view').forEach((v) => {
      v.classList.toggle('active', v.dataset.view === btn.dataset.tab);
    });
  });
});

document.querySelectorAll('.controller:not(:disabled)').forEach((btn) => {
  btn.addEventListener('click', () => toast('1a 仅接入 Cascade · tail-in 真实数据'));
});
document.querySelectorAll('.scenario:not(:disabled)').forEach((btn) => {
  btn.addEventListener('click', () => toast('场景：倒车入库'));
});

/* ---------------- HUD / minimap ---------------- */

let hudTimer = 0;

function updateHud(state, dt) {
  hudTimer += dt;
  if (hudTimer < 0.08) return;
  hudTimer = 0;

  const live = !useFake && sim.latestState;
  if (live) {
    const p = sim.latestState.pose;
    el.artAngle.textContent = THREE.MathUtils.radToDeg(p.g).toFixed(1);
    el.steerAngle.textContent = THREE.MathUtils.radToDeg(p.steer).toFixed(1);
    el.speed.textContent = p.speed.toFixed(1);
    const pct = Math.round(sim.latestState.progress * 100);
    el.progressPct.textContent = pct;
    el.progressBar.style.width = `${pct}%`;
    el.latError.textContent = sim.latestState.error.lateral.toFixed(2);
    el.psiError.textContent = THREE.MathUtils.radToDeg(sim.latestState.error.psi).toFixed(1);
    return;
  }

  el.artAngle.textContent = THREE.MathUtils.radToDeg(state.g).toFixed(1);
  el.steerAngle.textContent = THREE.MathUtils.radToDeg(state.steer).toFixed(1);
  el.speed.textContent = state.speed.toFixed(1);
  const pct = Math.round(state.progress * 100);
  el.progressPct.textContent = pct;
  el.progressBar.style.width = `${pct}%`;
  el.latError.textContent = '—';
  el.psiError.textContent = '—';

  if (phase === 'running' && sim.done) {
    setPhase('done');
    el.doneSummary.className = 'done-summary ok';
    el.doneSummary.textContent = '假数据演示完成 · 轨迹由前端生成';
    toast('演示完成 · 假数据轨迹');
  }
}

const mm = document.getElementById('minimap');
const MMS = 240;
mm.width = MMS * 2;
mm.height = MMS * 2;
const ctx = mm.getContext('2d');
ctx.scale(2, 2);

const MM = { cx: 120, cy: 120, scale: 3.3, x0: 56, y0: 35 };
const mmX = (x) => MM.cx + (x - MM.x0) * MM.scale;
const mmY = (y) => MM.cy - (y - MM.y0) * MM.scale;

function drawVehicle(state, elapsed) {
  const s = MM.scale;
  const hx = state.x - 1.5 * Math.cos(state.h);
  const hy = state.y - 1.5 * Math.sin(state.h);
  const tx = hx - 6.5 * Math.cos(state.h + state.g);
  const ty = hy - 6.5 * Math.sin(state.h + state.g);

  const rect = (cx, cy, len, wid, heading, fill, stroke) => {
    ctx.save();
    ctx.translate(mmX(cx), mmY(cy));
    ctx.rotate(-heading);
    ctx.fillStyle = fill;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1;
    ctx.fillRect((-len * s) / 2, (-wid * s) / 2, len * s, wid * s);
    ctx.strokeRect((-len * s) / 2, (-wid * s) / 2, len * s, wid * s);
    ctx.restore();
  };

  rect(tx, ty, 15, 2.8, state.h + state.g, 'rgba(150,162,172,0.9)', 'rgba(220,228,235,0.95)');
  rect(state.x, state.y, 5, 3, state.h, 'rgba(124,29,29,0.95)', 'rgba(247,199,102,0.95)');

  // heading tick on the cab
  ctx.save();
  ctx.translate(mmX(state.x), mmY(state.y));
  ctx.rotate(-state.h);
  ctx.fillStyle = 'rgba(247,199,102,0.95)';
  ctx.beginPath();
  ctx.moveTo(5 * s * 0.5 + 4, 0);
  ctx.lineTo(5 * s * 0.5 - 1, -3);
  ctx.lineTo(5 * s * 0.5 - 1, 3);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  const targetX = mmX(SCENE.goal.x);
  const targetY = mmY(SCENE.goal.y);
  const pulse = 5 + Math.sin(elapsed * 3) * 1.8;
  ctx.strokeStyle = 'rgba(227,154,45,0.75)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(targetX, targetY, pulse, 0, Math.PI * 2);
  ctx.stroke();

  const cgX = mmX(SCENE.controlGoal.x);
  const cgY = mmY(SCENE.controlGoal.y);
  ctx.strokeStyle = 'rgba(127,180,201,0.7)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cgX, cgY - 4); ctx.lineTo(cgX + 4, cgY);
  ctx.lineTo(cgX, cgY + 4); ctx.lineTo(cgX - 4, cgY);
  ctx.closePath(); ctx.stroke();
}

function drawMinimap(state, elapsed) {
  ctx.clearRect(0, 0, MMS, MMS);

  const bg = ctx.createRadialGradient(120, 120, 8, 120, 120, 170);
  bg.addColorStop(0, '#0d151b');
  bg.addColorStop(1, '#06090c');
  ctx.fillStyle = bg;
  ctx.fillRect(0, 0, MMS, MMS);

  ctx.strokeStyle = 'rgba(58,72,84,0.22)';
  ctx.lineWidth = 0.5;
  for (let i = 20; i < MMS; i += 20) {
    ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, MMS); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(MMS, i); ctx.stroke();
  }

  // road
  const left = mmX(SCENE.road.left);
  const right = mmX(SCENE.road.right);
  const top = mmY(SCENE.road.yEnd);
  const bottom = mmY(SCENE.road.yStart);
  ctx.fillStyle = 'rgba(30,42,52,0.5)';
  ctx.fillRect(left, top, right - left, bottom - top);
  ctx.strokeStyle = 'rgba(190,202,211,0.55)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(left, top); ctx.lineTo(left, bottom); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(right, top); ctx.lineTo(right, bottom); ctx.stroke();

  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = 'rgba(190,202,211,0.28)';
  ctx.beginPath(); ctx.moveTo(mmX(SCENE.road.center), top); ctx.lineTo(mmX(SCENE.road.center), bottom); ctx.stroke();
  ctx.setLineDash([]);

  // dock slot
  const sx = mmX(SCENE.slot.left);
  const sy = mmY(SCENE.slot.bottom);
  const sw = mmX(SCENE.slot.right) - sx;
  const sh = mmY(SCENE.slot.top) - sy;
  ctx.fillStyle = 'rgba(227,154,45,0.08)';
  ctx.fillRect(sx, sy, sw, sh);
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = 'rgba(227,154,45,0.85)';
  ctx.strokeRect(sx, sy, sw, sh);
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(247,199,102,0.7)';
  ctx.font = '600 8px Rajdhani, sans-serif';
  ctx.fillText('DOCK 01', sx + 3, sy + 10);

  // planned path (fake: 71 采样点; live: 220 点参考线, Task 5 接入)
  const pts = sim.getPathPoints();
  ctx.beginPath();
  pts.forEach(([px, py], i) => {
    if (i === 0) ctx.moveTo(mmX(px), mmY(py));
    else ctx.lineTo(mmX(px), mmY(py));
  });
  ctx.setLineDash([3, 4]);
  ctx.strokeStyle = 'rgba(127,180,201,0.42)';
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.setLineDash([]);

  drawVehicle(state, elapsed);

  // scale bar + north
  ctx.strokeStyle = 'rgba(190,202,211,0.55)';
  ctx.beginPath();
  ctx.moveTo(12, 226);
  ctx.lineTo(12 + 5 * MM.scale, 226);
  ctx.stroke();
  ctx.fillStyle = 'rgba(131,144,156,0.8)';
  ctx.font = '500 7px Rajdhani, sans-serif';
  ctx.fillText('5 m', 12, 220);

  ctx.fillStyle = 'rgba(247,199,102,0.8)';
  ctx.font = '700 9px Rajdhani, sans-serif';
  ctx.fillText('N', 222, 16);
  ctx.beginPath();
  ctx.moveTo(225, 20);
  ctx.lineTo(225, 30);
  ctx.strokeStyle = 'rgba(247,199,102,0.7)';
  ctx.stroke();
}

/* ---------------- resize / loop ---------------- */

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  composer.setSize(window.innerWidth, window.innerHeight);
}

window.addEventListener('resize', onResize);

const timer = new THREE.Timer();
timer.connect(document);
let elapsed = 0;

function rigCenter(state) {
  const hx = state.x - 1.5 * Math.cos(state.h);
  const hy = state.y - 1.5 * Math.sin(state.h);
  const tx = hx - 6.5 * Math.cos(state.h + state.g);
  const ty = hy - 6.5 * Math.sin(state.h + state.g);
  return new THREE.Vector3(
    (state.x + tx) / 2 - SCENE_ORIGIN.x,
    1.8,
    (state.y + ty) / 2 - SCENE_ORIGIN.y
  );
}

function frame(timestamp) {
  requestAnimationFrame(frame);
  timer.update(timestamp);
  const dt = Math.min(timer.getDelta(), 0.05);
  elapsed += dt;

  const state = sim.update(dt);
  truck.setState(state, dt);
  audio.update(state, phase, performance.now());

  if (follow) {
    const center = rigCenter(state);
    const alpha = 1 - Math.exp(-2.4 * dt);
    controls.target.lerp(center, alpha);
    camera.position.lerp(center.clone().add(FOLLOW_OFFSET), alpha);
  }

  controls.update();
  updateHud(state, dt);
  drawMinimap(state, elapsed);
  composer.render();
}

function applyPose(p) {
  const state = sim.seek(p);
  truck.setState(state, 0);
  follow = false;
  controls.autoRotate = false;
  const center = rigCenter(state);
  controls.target.copy(center);
  camera.position.copy(center.clone().add(FOLLOW_OFFSET));
  controls.update();
  setPhase(p >= 1 ? 'done' : 'running');
  updateHud(state, 1);
}

if (useFake && params.has('pose')) {
  applyPose(THREE.MathUtils.clamp(Number(params.get('pose')) || 0, 0, 1));
} else {
  setPhase(useFake ? 'idle' : 'offline');
}
frame();

if (useFake) {
  setTimeout(startDemo, 700);   // fake 自动开始 (spec §10.4.1)
}
