import * as THREE from 'three';
import { SCENE_ORIGIN, SCENE } from './sceneConfig.js';

// Spike-only: a hand-tuned path, not the real controller output.
export { SCENE_ORIGIN };
export const TRACK = {
  start: { x: SCENE.start.x, y: SCENE.start.y },
  target: { x: SCENE.goal.x, y: SCENE.goal.y },
};

const curve = new THREE.CatmullRomCurve3(
  [
    new THREE.Vector3(56.25, 0, 9.91),
    new THREE.Vector3(54.2, 0, 20.0),
    new THREE.Vector3(54.8, 0, 29.0),
    new THREE.Vector3(55.8, 0, 36.5),
    new THREE.Vector3(57.0, 0, 43.0),
    new THREE.Vector3(58.25, 0, 53.56),
  ],
  false,
  'catmullrom',
  0.2
);

const L = 3.0;              // tractor wheelbase (m)
const V = -2.5;             // reverse speed, accelerated for the spike (m/s)
const MAX_STEER = THREE.MathUtils.degToRad(35);
// Choreographed swing, not integrated trailer dynamics: it peaks mid-manoeuvre
// and returns to zero so the trailer ends square in the dock.
const SWING = THREE.MathUtils.degToRad(16);

function wrap(a) {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

function headingAt(u) {
  const t = curve.getTangentAt(u);
  // The cab faces opposite the direction of travel while reversing.
  return wrap(Math.atan2(t.z, t.x) + Math.PI);
}

export class FakeParkingSim {
  constructor() {
    this.length = curve.getLength();
    this.reset();
  }

  get path() {
    return curve;
  }

  reset() {
    this.progress = 0;
    this.running = false;
    this.done = false;
    this.h = headingAt(0);
    const p = curve.getPointAt(0);
    this.state = {
      x: p.x, y: p.z, h: this.h, g: 0, steer: 0,
      speed: 0, progress: 0, done: false,
    };
    return this.state;
  }

  start() {
    if (this.done) this.reset();
    this.running = true;
  }

  pause() { this.running = false; }

  resume() { if (!this.done) this.running = true; }

  getPathPoints(n = 71) {
    const pts = [];
    for (let i = 0; i < n; i++) {
      const p = curve.getPointAt(i / (n - 1));
      pts.push([p.x, p.z]);
    }
    return pts;
  }

  // Dev helper for screenshots: freeze the rig at a given point on the path.
  seek(progress) {
    this.progress = THREE.MathUtils.clamp(progress, 0, 1);
    this.running = false;
    this.done = this.progress >= 1;
    this.h = headingAt(this.progress);
    const p = curve.getPointAt(this.progress);
    this.state = {
      x: p.x,
      y: p.z,
      h: this.h,
      g: SWING * Math.sin(Math.PI * this.progress),
      steer: 0,
      speed: 0,
      progress: this.progress,
      done: this.done,
    };
    return this.state;
  }

  update(dt) {
    if (!this.running) return this.state;

    const step = (Math.abs(V) * dt) / this.length;
    const prevH = this.h;
    this.progress = Math.min(1, this.progress + step);

    const p = curve.getPointAt(this.progress);
    const h = headingAt(this.progress);
    const dh = wrap(h - prevH);
    const ds = Math.max(Math.abs(V) * dt, 1e-4);
    // dh = v*tan(steer)/L * dt with v = -ds/dt while reversing.
    const steer = THREE.MathUtils.clamp(
      Math.atan(-(dh / ds) * L),
      -MAX_STEER,
      MAX_STEER
    );
    const g = SWING * Math.sin(Math.PI * this.progress);
    this.h = h;

    if (this.progress >= 1) {
      this.running = false;
      this.done = true;
    }

    this.state = {
      x: p.x,
      y: p.z,
      h,
      g,
      steer,
      speed: this.running ? V : 0,
      progress: this.progress,
      done: this.done,
    };
    return this.state;
  }
}
