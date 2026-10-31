// 实时仿真客户端 — 连接 demo.server 的 /ws, 只负责收发与显示插值 (spec §10.3)。
// 不是仿真器: update(dt) 不积分车辆, 只做指数平滑。
import { SCENE } from './sceneConfig.js';

const PROTOCOL = 1;
const RETRY_DELAYS = [1000, 2000, 4000, 5000];   // 重连退避 (spec §10.3)

function wrapAngle(a) {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

export class LiveSim {
  constructor({ url, onPhase, onToast }) {
    this._url = url;
    this._onPhase = onPhase || (() => {});
    this._onToast = onToast || (() => {});
    this._ws = null;
    this._connected = false;
    this._retry = 0;
    this._stopped = false;
    this._latest = null;       // 服务端最新 state
    this._doneInfo = null;
    this._scenario = null;
    this._pathPoints = [];
    this._seq = -1;
    this._phase = 'offline';   // ready/running/paused/done/failed/offline
    this._disp = { ...SCENE.start, steer: 0, speed: 0, progress: 0 };
    this.connect();
  }

  // ---------------- 连接 ----------------

  connect() {
    if (this._stopped) return;
    const ws = new WebSocket(this._url);
    this._ws = ws;
    ws.onopen = () => { this._retry = 0; this._connected = true; };
    ws.onclose = () => {
      this._connected = false;
      this._setPhase('offline');
      if (!this._stopped) {
        const delay = RETRY_DELAYS[Math.min(this._retry, RETRY_DELAYS.length - 1)];
        this._retry += 1;
        setTimeout(() => this.connect(), delay);
      }
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => this._handle(JSON.parse(ev.data));
  }

  _setPhase(phase) {
    if (phase !== this._phase) {
      this._phase = phase;
      this._onPhase(phase);
    }
  }

  _send(cmd) {
    if (this._connected && this._ws) {
      this._ws.send(JSON.stringify({ type: 'cmd', cmd }));
    }
  }

  // ---------------- 命令 ----------------

  start() { this._send('start'); }
  pause() { this._send('pause'); }
  resume() { this._send('resume'); }
  reset() { this._send('reset'); }

  // ---------------- 消息 ----------------

  _handle(msg) {
    if (msg.protocol !== PROTOCOL) {
      this._onToast(`协议版本不匹配: ${msg.protocol}`);
    }
    switch (msg.type) {
      case 'hello':
        this._scenario = msg.scenario;
        this._pathPoints = msg.scenario.referencePath || [];
        this._applyState(msg.state, true);
        break;
      case 'state':
        if (msg.seq < this._seq && msg.seq !== 0) return;  // 丢弃旧 seq
        this._applyState(msg, msg.seq === 0 || this._seq < 0);
        break;
      case 'done':
        this._doneInfo = msg;
        this._phase = msg.success ? 'done' : 'failed';
        this._onPhase(this._phase);
        break;
      case 'error':
        this._onToast(`后端错误: ${msg.message}`);
        break;
    }
  }

  _applyState(state, snap = false) {
    const prevSeq = this._seq;
    this._seq = state.seq;
    this._latest = state;
    if (snap || prevSeq < 0 || state.seq === 0) {
      // 首帧 / 重置: 立即吸附, 避免从库位滑回起点
      const p = state.pose;
      this._disp = { x: p.x, y: p.y, h: p.h, g: p.g,
                     steer: p.steer, speed: p.speed, progress: state.progress };
    }
    this._setPhase(state.phase);
  }

  // ---------------- 显示插值 ----------------

  update(dt) {
    const d = this._disp;
    const latest = this._latest;
    if (!latest) return d;
    const p = latest.pose;
    const a = 1 - Math.exp(-dt * 12);            // spec §10.3 指数平滑
    d.x += (p.x - d.x) * a;
    d.y += (p.y - d.y) * a;
    d.h += wrapAngle(p.h - d.h) * a;
    d.g += wrapAngle(p.g - d.g) * a;
    d.steer += wrapAngle(p.steer - d.steer) * a;
    d.speed = p.speed;
    d.progress = latest.progress;
    return d;
  }

  // ---------------- getter ----------------

  get phase() { return this._phase; }
  get running() { return this._phase === 'running'; }
  get done() { return this._phase === 'done'; }
  get connected() { return this._connected; }
  get latestState() { return this._latest; }
  get pathPoints() { return this._pathPoints; }
  getPathPoints() { return this._pathPoints; }
  get scenario() { return this._scenario; }
  get doneInfo() { return this._doneInfo; }
}
