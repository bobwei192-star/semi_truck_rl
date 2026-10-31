import * as THREE from 'three';
import { SCENE_ORIGIN } from './sceneConfig.js';

const mats = {
  paint: new THREE.MeshStandardMaterial({ color: 0x7c1d1d, metalness: 0.62, roughness: 0.34 }),
  paintDark: new THREE.MeshStandardMaterial({ color: 0x3f0f0f, metalness: 0.5, roughness: 0.45 }),
  alu: new THREE.MeshStandardMaterial({ color: 0x9fa9b2, metalness: 0.88, roughness: 0.3 }),
  steel: new THREE.MeshStandardMaterial({ color: 0x2a323a, metalness: 0.8, roughness: 0.48 }),
  darkMetal: new THREE.MeshStandardMaterial({ color: 0x131920, metalness: 0.75, roughness: 0.55 }),
  rubber: new THREE.MeshStandardMaterial({ color: 0x0a0b0c, metalness: 0.05, roughness: 0.95 }),
  chrome: new THREE.MeshStandardMaterial({ color: 0xd9dee3, metalness: 1.0, roughness: 0.16 }),
  glass: new THREE.MeshPhysicalMaterial({
    color: 0x0c1a22, metalness: 0.0, roughness: 0.12,
    transparent: true, opacity: 0.78, envMapIntensity: 1.4,
  }),
  amber: new THREE.MeshStandardMaterial({
    color: 0x2a1500, emissive: 0xff9a2a, emissiveIntensity: 3.2, roughness: 0.4,
  }),
  red: new THREE.MeshStandardMaterial({
    color: 0x2a0503, emissive: 0xff2d1a, emissiveIntensity: 2.6, roughness: 0.4,
  }),
  white: new THREE.MeshStandardMaterial({
    color: 0x1a1c1e, emissive: 0xfff2d0, emissiveIntensity: 2.2, roughness: 0.4,
  }),
};

function box(parent, w, h, d, mat, x, y, z) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

function cyl(parent, rt, rb, h, mat, x, y, z, rx = 0, rz = 0, seg = 18) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(rt, rb, h, seg), mat);
  m.position.set(x, y, z);
  m.rotation.x = rx;
  m.rotation.z = rz;
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}

function decalTexture() {
  const c = document.createElement('canvas');
  c.width = 1024;
  c.height = 192;
  const g = c.getContext('2d');
  g.clearRect(0, 0, c.width, c.height);
  g.font = '700 88px Rajdhani, sans-serif';
  g.fillStyle = 'rgba(20, 24, 28, 0.92)';
  g.textBaseline = 'middle';
  g.fillText('SEMI / TRAILER', 30, 82);
  g.font = '600 34px Rajdhani, sans-serif';
  g.fillStyle = 'rgba(120, 38, 30, 0.95)';
  g.fillText('REVERSE DOCK  ·  UNIT 01', 34, 150);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

export function buildTruck() {
  const root = new THREE.Group();
  const tractor = new THREE.Group();
  const trailerPivot = new THREE.Group();
  trailerPivot.position.set(-1.5, 0, 0);

  const steeringHubs = [];
  const spinParts = [];

  function addWheel(parent, x, z, { steer = false, radius = 0.54, width = 0.42 } = {}) {
    const hub = new THREE.Group();
    hub.position.set(x, radius, z);
    if (steer) steeringHubs.push(hub);

    const spin = new THREE.Group();
    hub.add(spin);
    spinParts.push(spin);

    const tireGeo = new THREE.CylinderGeometry(radius, radius, width, 24, 1);
    tireGeo.rotateX(Math.PI / 2);
    const tire = new THREE.Mesh(tireGeo, mats.rubber);
    tire.castShadow = true;
    spin.add(tire);

    const rimGeo = new THREE.CylinderGeometry(radius * 0.6, radius * 0.6, width + 0.03, 18, 1);
    rimGeo.rotateX(Math.PI / 2);
    const rim = new THREE.Mesh(rimGeo, mats.chrome);
    spin.add(rim);

    for (let i = 0; i < 4; i++) {
      const spoke = box(spin, radius * 0.82, 0.06, width + 0.05, mats.steel, 0, 0, 0);
      spoke.rotation.z = (i * Math.PI) / 4;
    }

    parent.add(hub);
    return hub;
  }

  /* ---------------- tractor ---------------- */

  box(tractor, 5.2, 0.42, 2.3, mats.steel, 0, 0.95, 0);
  box(tractor, 2.6, 2.5, 2.5, mats.paint, 0.95, 2.3, 0);
  box(tractor, 1.75, 0.5, 2.3, mats.paintDark, 1.0, 3.58, 0);
  box(tractor, 0.08, 1.15, 2.1, mats.glass, 2.27, 2.55, 0);
  box(tractor, 1.2, 0.8, 0.07, mats.glass, 1.05, 2.62, 1.27);
  box(tractor, 1.2, 0.8, 0.07, mats.glass, 1.05, 2.62, -1.27);
  box(tractor, 0.12, 0.72, 1.72, mats.darkMetal, 2.3, 1.48, 0);
  for (let i = 0; i < 4; i++) {
    box(tractor, 0.06, 0.06, 1.5, mats.chrome, 2.37, 1.22 + i * 0.16, 0);
  }
  box(tractor, 0.34, 0.34, 2.44, mats.steel, 2.44, 0.78, 0);
  box(tractor, 0.08, 0.2, 0.36, mats.white, 2.51, 1.12, 0.86);
  box(tractor, 0.08, 0.2, 0.36, mats.white, 2.51, 1.12, -0.86);
  box(tractor, 1.15, 0.14, 1.38, mats.steel, -1.5, 1.26, 0);
  box(tractor, 0.55, 0.55, 2.44, mats.paintDark, 2.38, 0.72, 0);
  box(tractor, 1.2, 0.12, 0.55, mats.darkMetal, -1.2, 1.28, 1.08);
  box(tractor, 1.2, 0.12, 0.55, mats.darkMetal, -1.2, 1.28, -1.08);

  for (const side of [-1, 1]) {
    cyl(tractor, 0.3, 0.3, 1.6, mats.chrome, -0.1, 0.95, side * 1.22, 0, Math.PI / 2, 16);
    cyl(tractor, 0.09, 0.09, 1.9, mats.darkMetal, -0.35, 3.0, side * 1.15, 0, 0, 12);
    box(tractor, 0.5, 0.12, 0.08, mats.steel, 1.95, 2.95, side * 1.42);
    box(tractor, 0.1, 0.34, 0.24, mats.darkMetal, 1.72, 3.08, side * 1.55);
    box(tractor, 0.62, 0.12, 0.4, mats.paintDark, 0.9, 3.88, side * 0.95);
  }

  for (let i = -2; i <= 2; i++) {
    box(tractor, 0.13, 0.08, 0.14, mats.amber, 2.22, 3.52, i * 0.38);
  }

  addWheel(tractor, 1.65, 1.07, { steer: true });
  addWheel(tractor, 1.65, -1.07, { steer: true });
  for (const x of [-0.65, -1.75]) {
    for (const z of [0.78, 1.09, -0.78, -1.09]) {
      addWheel(tractor, x, z, { radius: 0.52, width: 0.36 });
    }
  }

  /* ---------------- trailer ---------------- */

  box(trailerPivot, 14.8, 0.28, 2.5, mats.steel, -6.7, 1.0, 0);
  box(trailerPivot, 14.4, 2.6, 2.78, mats.alu, -6.7, 2.45, 0);
  box(trailerPivot, 14.55, 0.18, 2.9, mats.steel, -6.7, 3.82, 0);
  box(trailerPivot, 13.5, 0.55, 2.88, mats.paintDark, -6.7, 0.76, 0);
  box(trailerPivot, 0.55, 0.5, 2.55, mats.steel, 0.0, 1.55, 0);

  for (let i = 0; i < 11; i++) {
    const x = -1.25 - i * 1.18;
    box(trailerPivot, 0.07, 2.55, 0.05, mats.steel, x, 2.45, 1.4);
    box(trailerPivot, 0.07, 2.55, 0.05, mats.steel, x, 2.45, -1.4);
  }

  box(trailerPivot, 0.1, 2.6, 2.8, mats.steel, -13.9, 2.45, 0);
  box(trailerPivot, 0.08, 2.35, 1.3, mats.alu, -13.96, 2.45, 0.68);
  box(trailerPivot, 0.08, 2.35, 1.3, mats.alu, -13.96, 2.45, -0.68);
  box(trailerPivot, 0.2, 0.2, 2.6, mats.steel, -14.02, 0.72, 0);

  for (const x of [-10.2, -11.45]) {
    for (const z of [0.72, 1.03, -0.72, -1.03]) {
      addWheel(trailerPivot, x, z, { radius: 0.52, width: 0.36 });
    }
  }

  for (const side of [-1, 1]) {
    box(trailerPivot, 0.06, 0.7, 0.55, mats.rubber, -12.2, 0.38, side * 1.02);
    box(trailerPivot, 0.16, 1.2, 0.16, mats.steel, -2.8, 0.6, side * 0.82);
    box(trailerPivot, 0.5, 0.1, 0.3, mats.steel, -2.8, 0.05, side * 0.82);
  }

  for (const x of [-2.2, -5.2, -8.2, -11.2]) {
    box(trailerPivot, 0.18, 0.08, 0.08, mats.amber, x, 1.02, 1.46);
    box(trailerPivot, 0.18, 0.08, 0.08, mats.amber, x, 1.02, -1.46);
  }
  for (const z of [-1.0, 1.0]) {
    box(trailerPivot, 0.06, 0.14, 0.3, mats.red, -14.05, 1.05, z);
  }

  const decal = new THREE.MeshBasicMaterial({
    map: decalTexture(), transparent: true, opacity: 0.88, depthWrite: false,
  });
  for (const side of [1, -1]) {
    const p = new THREE.Mesh(new THREE.PlaneGeometry(6.4, 1.2), decal);
    p.position.set(-6.4, 2.6, side * 1.405);
    if (side < 0) p.rotation.y = Math.PI;
    trailerPivot.add(p);
  }

  root.add(tractor, trailerPivot);
  root.traverse((o) => {
    if (o.isMesh) {
      o.castShadow = true;
      o.receiveShadow = true;
    }
  });

  let spin = 0;

  function setState(s, dt = 0.016) {
    root.position.set(s.x - SCENE_ORIGIN.x, 0, s.y - SCENE_ORIGIN.y);
    root.rotation.y = -s.h;
    trailerPivot.rotation.y = -s.g;
    for (const hub of steeringHubs) hub.rotation.y = -s.steer;

    spin -= (s.speed * dt) / 0.54;
    for (const part of spinParts) part.rotation.z = spin * 0.35;
  }

  return { group: root, setState };
}
