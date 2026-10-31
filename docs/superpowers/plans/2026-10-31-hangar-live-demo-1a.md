# 半挂车 3D 实时演示台 · 里程碑 1a 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `another_solve/parking_three_lines.py` 的真实「规划 + 串级控制」轨迹经 FastAPI + WebSocket 接入 `hangar_spike` 的 Three.js 机库前端，替换 fake 编排轨迹；保留 `?demo=1` 假数据回退。

**Architecture:** 单 Python 进程（FastAPI + uvicorn，127.0.0.1:8000）持有全局 `ReverseDockSession`（`run_once()` 的逐步骤镜像，直接 import 已验证文件，不搬代码）；`asyncio` ticker 以 20 Hz × `--speed` 推进并广播协议 v1 消息；前端 `LiveSim` 订阅 `/ws`（Vite 代理），只做显示插值，HUD 用服务端最新值。

**Tech Stack:** Python 3.14 + FastAPI + uvicorn + websockets + httpx（后端/测试）；Vite 8 + Three.js 0.186（前端，已有）；pygame-ce + numpy（已有 .venv）。

**Spec:** `docs/superpowers/specs/2026-10-28-hangar-live-demo-design.md`（本计划严格从 spec 出发；spec §15 的 8 个默认选择已全部获批）。

## Global Constraints

- **不改控制算法、不改停车判定、不改成功条件**；不改 `another_solve/parking_three_lines.py`、`vehicle/`、`config/` 的任何文件。
- `parking_three_lines.py` 的导入副作用（pygame 横幅、~3s 启动）只通过 `PYGAME_HIDE_SUPPORT_PROMPT=1` 缓解。
- WebSocket 协议版本 `protocol: 1`；`DT = 0.05 s` 恒定，`--speed` 只改墙钟倍率。
- 单全局 session、单操作员；最后一个客户端断开时 running 自动 paused。
- UI 不得把串级控制器标成 MPC；`?demo=1` 的所有 live 标识必须是 `FAKE DATA`。
- 测试基线（本机 2026-10-31 复验）：带噪实际轨迹 866 步 / 成功 / 0 违规 / 最大横向误差 0.4702 m / 终点控制点 (58.1124, 58.61)；理想轨迹 867 步 / 终点控制点 (58.0034, 58.6586) / 最大横向误差 0.5836 m；参考路径 220 点。
- 后端测试用 `.venv/bin/python`（仓库约定，见 Makefile）；前端验证用 `npm run build`；浏览器人工验收由用户执行（§Task 7）。
- 每个 Task 末尾提交一次 commit（沿用仓库最近提交风格）；若用户要求不提交，跳过 commit 步骤即可。

## 文件结构

```text
demo/                          # 新增（仓库根，与 another_solve/ 平级）
  __init__.py                  # 空
  session.py                   # ReverseDockSession：run_once() 逐步镜像
  server.py                    # FastAPI app、/health、/ws、ticker、CLI
tests/
  test_demo_session.py         # 新增：确定性回归（纯 python，不依赖 pytest）
  test_demo_protocol_smoke.py  # 新增：真实子进程 WS 协议冒烟（asyncio + websockets + httpx）
requirements-demo.txt          # 新增
Makefile                       # 修改：demo-install / demo-backend / demo-frontend
hangar_spike/
  vite.config.js               # 新增：/ws 代理
  src/sceneConfig.js           # 新增：场景常量镜像（JS 单一来源）
  src/liveSim.js               # 新增：WS 客户端 + 显示插值 + 重连
  src/fakeSim.js               # 修改：常量来自 sceneConfig，端点对齐真实场景，+pause/resume
  src/truck.js                 # 修改：SCENE_ORIGIN import 改向 sceneConfig
  src/main.js                  # 修改：live/fake 选择、phase 机、HUD 误差、小地图、按键
  index.html                   # 修改：参数/控制器卡/readout 网格/汇总行/禁用态
  src/style.css                # 修改：少量 phase/禁用/done-summary 样式
README.md                      # 修改：本地启动说明
```

---

### Task 1: 前端场景常量对齐（sceneConfig + fakeSim/truck/main 常量 + index.html 文案）

**Files:**
- Create: `hangar_spike/src/sceneConfig.js`
- Modify: `hangar_spike/src/fakeSim.js`
- Modify: `hangar_spike/src/truck.js`（1 行 import）
- Modify: `hangar_spike/src/main.js`（import、FOLLOW_OFFSET、minimap 常量）
- Modify: `hangar_spike/index.html`（参数/卡片文案）

**Interfaces:**
- Produces: `sceneConfig.js` 导出 `SCENE_ORIGIN = {x: 56, y: 35}` 和 `SCENE`（见下），供 Task 5 的 `liveSim.js`/`main.js` 使用；`FakeParkingSim` 新增 `pause()`/`resume()`/`getPathPoints()`，Task 5 的 main.js 依赖这三个方法。

- [ ] **Step 1: 新建 `hangar_spike/src/sceneConfig.js`**

```js
// 场景常量镜像 — JS 侧单一来源。
// 与 Python 侧手工同步，来源: config/scene_config.py / vehicle/truckTrailerOnAxle.py
// (spec §10.2 / §4.5)。修改 Python 常量时必须同步改这里。
export const SCENE_ORIGIN = { x: 56, y: 35 };

export const SCENE = {
  start: { x: 56.25, y: 9.91, h: -Math.PI / 2, g: 0 },
  goal: { x: 58.25, y: 53.56 },        // 牵引车参考点目标 (hitch)
  controlGoal: { x: 58.0, y: 58.61 },  // 虚拟控制点停车线 (goal y + 11/2 - 0.45)
  road: { left: 50, center: 56, right: 60, yStart: 0, yEnd: 70 },
  slot: { left: 56, right: 60, top: 50.5, bottom: 69 },
  progressY: { from: 15.41, to: 58.61 },  // 控制点 y 的进度区间 (path 首末端)
  vehicle: {
    truckWheelbase: 3.0, trailerWheelbase: 11.0,
    truckLength: 5.0, truckWidth: 3.0,
    trailerLength: 15.0, trailerWidth: 2.8,
    maxSteeringDeg: 35, maxArticulationDeg: 90, jackknifeDeg: 85,
    reverseSpeedMps: -1.0,
  },
};
```

- [ ] **Step 2: 修改 `fakeSim.js`**

具体改动（保持编排逻辑/2.5 m/s 加速/SWING 不变，仅对齐常量与端点）：

1. 文件头 import 改为：
```js
import * as THREE from 'three';
import { SCENE_ORIGIN, SCENE } from './sceneConfig.js';

// Spike-only: a hand-tuned path, not the real controller output.
export { SCENE_ORIGIN };
export const TRACK = {
  start: { x: SCENE.start.x, y: SCENE.start.y },
  target: { x: SCENE.goal.x, y: SCENE.goal.y },
};
```
（删除本地 `SCENE_ORIGIN`/`TRACK` 定义，向后兼容 re-export，main.js 旧 import 不断。）

2. 曲线控制点首末对齐真实场景，中间形状保持：
```js
const curve = new THREE.CatmullRomCurve3(
  [
    new THREE.Vector3(56.25, 0, 9.91),
    new THREE.Vector3(54.2, 0, 20.0),
    new THREE.Vector3(54.8, 0, 29.0),
    new THREE.Vector3(55.8, 0, 36.5),
    new THREE.Vector3(57.0, 0, 43.0),
    new THREE.Vector3(58.25, 0, 53.56),
  ],
  false, 'catmullrom', 0.2);
```
（即首点 `(54.0,0,10.0)`→`(56.25,0,9.91)`，末点 `(58.25,0,50.56)`→`(58.25,0,53.56)`，中间 4 点不动。）

3. `FakeParkingSim` 新增两个方法（插在 `start()` 之后）：
```js
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
```

- [ ] **Step 3: 修改 `truck.js`（1 行）**

```js
// 旧
import { SCENE_ORIGIN } from './fakeSim.js';
// 新
import { SCENE_ORIGIN } from './sceneConfig.js';
```

- [ ] **Step 4: 修改 `main.js` 常量与 minimap**

1. import 行改为：
```js
import { FakeParkingSim, SCENE_ORIGIN } from './fakeSim.js';
import { SCENE } from './sceneConfig.js';
```
（`TRACK` 不再从 fakeSim 导入。）

2. `const FOLLOW_OFFSET = new THREE.Vector3(18, 8, -25);` → `new THREE.Vector3(18, 8, -18);`（spec §10.4.10：起点 rig 中心 z≈-21，z=-25 会穿出机库后墙。）

3. `drawMinimap` 内道路/库位常量替换：
```js
  // road
  const left = mmX(SCENE.road.left);
  const right = mmX(SCENE.road.right);
  const top = mmY(SCENE.road.yEnd);
  const bottom = mmY(SCENE.road.yStart);
```
```js
  ctx.beginPath(); ctx.moveTo(mmX(SCENE.road.center), top); ctx.lineTo(mmX(SCENE.road.center), bottom); ctx.stroke();
```
```js
  // dock slot
  const sx = mmX(SCENE.slot.left);
  const sy = mmY(SCENE.slot.bottom);
  const sw = mmX(SCENE.slot.right) - sx;
  const sh = mmY(SCENE.slot.top) - sy;
```
（`TRACK.target` 在 `drawVehicle` 中的两处引用改为 `SCENE.goal`。）

4. minimap 参考线路径改为通用折线（Task 5 的 live 模式复用）：
```js
  // planned path (fake: 71 采样点; live: 220 点参考线, Task 5 接入)
  const pts = sim.getPathPoints();
  ctx.beginPath();
  pts.forEach(([px, py], i) => {
    if (i === 0) ctx.moveTo(mmX(px), mmY(py));
    else ctx.lineTo(mmX(px), mmY(py));
  });
```

- [ ] **Step 5: 修改 `index.html` 文案（spec §10.4.4/6、§4.4.8）**

1. brand small：`REVERSE HANGAR · SPIKE` → `REVERSE HANGAR · LIVE 1a`。
2. 「车辆参数」block 中 `铰接角上限 ±85.0°` → `±90.0°`，并在该 block 末尾加：
```html
        <div class="stat"><span>场景道路</span><b>X 50–60 <i>m</i></b></div>
        <div class="stat"><span>库位</span><b>56–60 × 50.5–69 <i>m</i></b></div>
        <div class="stat"><span>起点 / 目标</span><b>(56.25, 9.91) / (58.25, 53.56)</b></div>
        <p class="panel-note">铰接角物理上限 ±90°；防折叠告警阈值 85°。目标为牵引车参考点（铰点）位置。</p>
```
3. 控制器 list（`#controllerList`）三个按钮替换为：
```html
          <button class="controller active" data-controller="Cascade · Tail-in">
            <span class="c-name">Cascade · Tail-in</span><span class="c-note">串级 · 真实数据</span>
          </button>
          <button class="controller" data-controller="MPC · Paper" disabled>
            <span class="c-name">MPC · Paper</span><span class="c-note">1b · 未接入</span>
          </button>
          <button class="controller" data-controller="RL · DDPG+HER" disabled>
            <span class="c-name">RL · DDPG + HER</span><span class="c-note">未接入</span>
          </button>
```
（controllers 面板里的三个 `list-row` 同步：`RL · DDPG + HER` 行去掉 `active`，`MPC · Paper` 行加 `active` 之外的标注 `<b>MODEL · 1b</b>`。）
4. 场景卡（底部 `#scenarioList`）：后三个 button 加 `disabled` 属性；scenes 面板 `panel-note` 改为「当前仅 01 已接入真实数据；其它场景占位。」
5. readout-grid 追加两格（在「进度」之后）：
```html
          <div class="readout"><em>横向误差 e</em><b><span id="latError">—</span><i>m</i></b></div>
          <div class="readout"><em>航向误差 ψ</em><b><span id="psiError">—</span><i>°</i></b></div>
```
6. `state-line` div 之后加汇总行：
```html
        <div class="done-summary" id="doneSummary"></div>
```

- [ ] **Step 6: `style.css` 追加少量样式（文件末尾）**

```css
/* live demo 1a: phase / disabled / summary */
.pill.paused { border-color: rgba(131,144,156,0.5); color: var(--muted); }
.pill.offline { border-color: rgba(199,88,79,0.55); color: #e08076; }
.controller:disabled, .scenario:disabled { opacity: 0.42; cursor: not-allowed; }
.done-summary { margin-top: 8px; font-size: 11px; line-height: 1.6; color: var(--muted); letter-spacing: 0.04em; }
.done-summary.ok { color: var(--good); }
.done-summary.bad { color: #e08076; }
```

- [ ] **Step 7: 构建验证**

Run: `cd hangar_spike && npm run build`
Expected: PASS（无 import 错误；`sim.getPathPoints` 在 main.js 已可用，fake 路径不变）。

- [ ] **Step 8: Commit**

```bash
git add hangar_spike/src/sceneConfig.js hangar_spike/src/fakeSim.js hangar_spike/src/truck.js hangar_spike/src/main.js hangar_spike/index.html hangar_spike/src/style.css
git commit -m "1a: 场景常量镜像 sceneConfig，前端常量/文案对齐真实源码"
```

---

### Task 2: 后端依赖与 Makefile 目标

**Files:**
- Create: `requirements-demo.txt`
- Modify: `Makefile`

**Interfaces:**
- Produces: 可安装的依赖集（fastapi/uvicorn/websockets/httpx）；`make demo-install` / `make demo-backend` / `make demo-frontend` 目标，Task 3/4/6 使用。

- [ ] **Step 1: 新建 `requirements-demo.txt`**

```text
fastapi>=0.115
uvicorn>=0.30
websockets>=13
httpx>=0.27
```
（spec §9.3：不装 `uvicorn[standard]`，避免 uvloop/httptools 在 Python 3.14 的构建风险。）

- [ ] **Step 2: Makefile 追加（放在 `.PHONY` 行和 `test` 目标旁，`.PHONY` 列表加 `demo-install demo-backend demo-frontend`）**

```make
demo-install:
	$(PY) -m pip install -r requirements-demo.txt

demo-backend:
	$(PY) -m demo.server --host 127.0.0.1 --port 8000

demo-frontend:
	cd hangar_spike && npm run dev
```

- [ ] **Step 3: 安装并冒烟导入**

Run: `make demo-install && .venv/bin/python -c "import fastapi, uvicorn, websockets, httpx; print('demo deps ok')"`
Expected: 安装成功，打印 `demo deps ok`。

- [ ] **Step 4: Commit**

```bash
git add requirements-demo.txt Makefile
git commit -m "1a: demo 后端依赖 (fastapi/uvicorn/websockets/httpx) 与 make 目标"
```

---

### Task 3: `demo/session.py` — `run_once()` 逐步镜像 + 确定性回归测试

**Files:**
- Create: `demo/__init__.py`（空文件）
- Create: `demo/session.py`
- Test: `tests/test_demo_session.py`

**Interfaces:**
- Consumes: `another_solve/parking_three_lines` 的 `plan_path`, `run_once`, `ParkingController`, `corner_violations`, `corners_in_slot`, `TruckTrailerOnAxle`, `DT`, `V_REVERSE`, `STEER_RATE`, `MAX_STEPS`, `NOISE_STD`, `ACTUAL_SEED`, `GOAL_Y`, `TRAILER_LEN`, `START_X`, `START_Y`, `INIT_HEADING_RAD`。
- Produces（Task 4 的 server.py 依赖这些精确签名）:
  - `ReverseDockSession(seed=ACTUAL_SEED, noise_std=NOISE_STD)`；`reset() -> None`、`start()`、`pause()`、`resume()`、`step() -> dict | None`（执行一步并返回 8.4 state dict；非 running 返回 None）、`state_message() -> dict`、`done_message() -> dict`、`hello_message() -> dict`。
  - 属性：`phase: str`（`ready/running/paused/done/failed`）、`seq: int`、`path: np.ndarray`（220×2）。
  - state dict 键（spec §8.4）：`type='state'`, `protocol=1`, `seq`, `t`, `phase`, `done`, `success`, `pose{x,y,h,g,steer}`, `trailerCenter[x,y]`, `controlPoint[x,y]`, `progress`, `error{lateral,psi}`, `violationSteps`。

- [ ] **Step 1: 新建 `demo/__init__.py`（空）**

- [ ] **Step 2: 写失败的测试 `tests/test_demo_session.py`**

```python
# -*- coding: utf-8 -*-
"""ReverseDockSession 确定性回归 (spec §12.1)。

镜像硬指标: session 逐步 step() 的结果必须与 parking_three_lines.run_once()
逐帧一致 (1e-9)。运行: .venv/bin/python tests/test_demo_session.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import another_solve.parking_three_lines as P
from config.scene_config import START_X, START_Y, INIT_HEADING_RAD
from demo.session import ReverseDockSession

TOL = 1e-9


def _run_session(sess):
    """逐步推进到终态, 返回收集的 (x,y,h,g,s_deg) 列表。"""
    got = []
    while sess.phase == 'running':
        msg = sess.step()
        assert msg is not None, 'running 状态下 step() 必须返回 state'
        p = msg['pose']
        got.append((p['x'], p['y'], p['h'], p['g'], np.rad2deg(p['steer'])))
    return np.array(got)


def _assert_states_equal(got, expected_states):
    exp = np.array(expected_states, dtype=float)
    assert len(got) == len(exp), f'步数不一致: {len(got)} vs {len(exp)}'
    assert np.allclose(got, exp, atol=TOL, rtol=0), '逐步结果与 run_once 不一致'


def test_actual_trajectory_matches_run_once():
    expected = P.run_once(P.plan_path(), noise_std=1.0, seed=2)
    sess = ReverseDockSession(seed=2, noise_std=1.0)
    sess.start()
    got = _run_session(sess)
    _assert_states_equal(got, expected['states'])
    assert sess.phase == 'done'
    assert sess.seq == 866
    assert expected['success'] is True
    assert sess.done_message()['success'] is True
    assert sess.done_message()['steps'] == 866
    assert sess.done_message()['violationSteps'] == 0
    assert abs(sess.done_message()['maxLateralError'] - 0.4702) < 1e-3


def test_ideal_trajectory_matches_run_once():
    expected = P.run_once(P.plan_path(), noise_std=0.0)
    sess = ReverseDockSession(seed=2, noise_std=0.0)
    sess.start()
    got = _run_session(sess)
    _assert_states_equal(got, expected['states'])
    assert sess.phase == 'done' and sess.seq == 867
    d = sess.done_message()
    assert abs(d['finalControlPoint'][0] - 58.0034) < 1e-3
    assert abs(d['finalControlPoint'][1] - 58.6586) < 1e-3
    assert abs(d['maxLateralError'] - 0.5836) < 1e-3


def test_initial_pose_is_noisy_default():
    rng = np.random.default_rng(2)
    x0 = START_X + rng.normal(0, 0.15)
    h0 = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5))
    sess = ReverseDockSession(seed=2, noise_std=1.0)
    msg = sess.state_message()
    assert msg['seq'] == 0 and msg['t'] == 0.0 and msg['phase'] == 'ready'
    assert msg['progress'] == 0.0
    assert abs(msg['pose']['x'] - x0) < TOL
    assert abs(msg['pose']['y'] - START_Y) < TOL
    assert abs(msg['pose']['h'] - h0) < TOL
    assert msg['pose']['g'] == 0.0 and msg['pose']['steer'] == 0.0
    assert msg['pose']['speed'] == 0.0


def test_reset_is_deterministic():
    def run_n(n):
        s = ReverseDockSession(seed=2, noise_std=1.0)
        s.start()
        out = []
        for _ in range(n):
            m = s.step()
            p = m['pose']
            out.append((p['x'], p['y'], p['h'], p['g'], p['steer']))
        return np.array(out), s.seq

    a, seq_a = run_n(100)
    b, seq_b = run_n(100)
    assert seq_a == seq_b == 100
    assert np.array_equal(a, b)


def test_start_pause_resume_semantics():
    s = ReverseDockSession(seed=2, noise_std=1.0)
    assert s.phase == 'ready'
    assert s.step() is None                      # ready 不推进
    s.start()
    assert s.phase == 'running'
    s.pause()
    assert s.phase == 'paused' and s.step() is None
    s.resume()
    assert s.phase == 'running'
    s.step(); s.step()
    seq_before = s.seq
    # done 后 start 必须先 reset 再 running
    s._phase = 'done'
    s.start()
    assert s.phase == 'running' and s.seq == 0   # reset 后 seq 重计


def test_max_steps_failure_path():
    # 把停车线推到不可能到达的位置, 触发 2500 步上限
    sess = ReverseDockSession(seed=0, noise_std=0.0)
    sess._tc_goal_y = 1e9
    sess.start()
    while sess.phase == 'running':
        sess.step()
    assert sess.phase == 'failed'
    assert sess.seq == 2500
    assert sess.done_message()['success'] is False


if __name__ == '__main__':
    for name, fn in sorted({k: v for k, v in list(globals().items())
                            if k.startswith('test_') and callable(v)}.items()):
        fn()
        print(f'PASS {name}')
    print('ALL PASS')
```

- [ ] **Step 3: 运行测试确认失败**

Run: `cd /home/tong/semi_truck_rl && .venv/bin/python tests/test_demo_session.py`
Expected: FAIL —— `ModuleNotFoundError: No module named 'demo.session'`。

- [ ] **Step 4: 实现 `demo/session.py`**

```python
# -*- coding: utf-8 -*-
"""倒车入库实时 session — parking_three_lines.run_once() 的逐步骤镜像 (spec §9.1)。

直接复用已验证文件, 不搬代码、不改增益、不改停车判定。
"""
import os

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

import numpy as np

from another_solve.parking_three_lines import (
    ACTUAL_SEED, DT, GOAL_Y, MAX_STEPS, NOISE_STD, STEER_RATE, TRAILER_LEN,
    V_REVERSE, ParkingController, corner_violations, corners_in_slot, plan_path,
)
from config.scene_config import INIT_HEADING_RAD, START_X, START_Y
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle

PROTOCOL = 1
CONTROL_POINT_BACK = TRAILER_LEN / 2              # 5.5 m: 牵引车参考点 -> 虚拟控制点
HITCH_BACK = 1.5                                  # 牵引车参考点 -> 铰接点 (模型几何)
TRAILER_CENTER_BACK = 6.5                         # 铰接点 -> 挂车几何中心 (模型几何)


class ReverseDockSession:
    """单操作员倒车入库 session。phase: ready/running/paused/done/failed。"""

    def __init__(self, seed=ACTUAL_SEED, noise_std=NOISE_STD):
        self._seed = seed
        self._noise_std = noise_std
        self.path = plan_path()                   # 220 点, 无随机性, 只算一次
        self._tc_goal_y = GOAL_Y + TRAILER_LEN / 2 - 0.45   # 58.61, 与 plan_path/run_once 一致
        self.reset()

    # ---------------- 生命周期 ----------------

    def reset(self):
        rng = np.random.default_rng(self._seed)
        truck = TruckTrailerOnAxle()
        if self._noise_std:
            x0 = START_X + rng.normal(0, 0.15)    # 与 run_once 完全一致的取值与顺序
            h0 = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5))
        else:
            x0, h0 = START_X, INIT_HEADING_RAD
        truck.set_state([x0, START_Y, h0, 0.0])   # g=0
        truck.v_mps = V_REVERSE
        self._rng = rng
        self._truck = truck
        self._controller = ParkingController(self.path)
        self._prev_applied = 0.0                  # 执行器上一拍实际转角 (度)
        self._phase = 'ready'
        self._seq = 0
        self._steps = 0
        self._violation_steps = 0
        self._max_lat_err = 0.0
        self._last_e = 0.0
        self._last_psi = 0.0
        self._success = None

    @property
    def phase(self):
        return self._phase

    @property
    def seq(self):
        return self._seq

    # ---------------- 命令 (spec §8.7) ----------------

    def start(self):
        if self._phase in ('done', 'failed'):
            self.reset()
        if self._phase in ('ready', 'paused'):
            self._phase = 'running'

    def pause(self):
        if self._phase == 'running':
            self._phase = 'paused'

    def resume(self):
        if self._phase == 'paused':
            self._phase = 'running'

    # ---------------- 步进 (镜像 run_once 的一次循环) ----------------

    def step(self):
        if self._phase != 'running':
            return None
        truck = self._truck
        swa_deg, e, psi = self._controller.compute(truck)
        if self._noise_std:
            swa_deg += self._rng.normal(0, self._noise_std)
        swa_deg = float(np.clip(swa_deg, -35, 35))
        swa_deg = float(np.clip(swa_deg,
                                self._prev_applied - np.rad2deg(STEER_RATE),
                                self._prev_applied + np.rad2deg(STEER_RATE)))
        self._prev_applied = swa_deg
        truck.set_steering(swa_deg)
        truck.drive(DT)

        self._steps += 1
        self._seq += 1
        self._last_e, self._last_psi = float(e), float(psi)
        self._max_lat_err = max(self._max_lat_err, abs(float(e)))

        if corner_violations(truck.get_corner_positions()):
            self._violation_steps += 1

        cp = self._control_point()
        if cp[1] >= self._tc_goal_y:                       # 到达停车线
            self._success = (self._violation_steps == 0
                             and corners_in_slot(truck.get_corner_positions()))
            self._phase = 'done' if self._success else 'failed'
        elif self._steps >= MAX_STEPS:
            self._success = False
            self._phase = 'failed'
        return self.state_message()

    # ---------------- 消息构造 (spec §8) ----------------

    def _control_point(self):
        th = self._truck.h_rad + self._truck.g_rad
        return (self._truck.x_m - CONTROL_POINT_BACK * np.cos(th),
                self._truck.y_m - CONTROL_POINT_BACK * np.sin(th))

    def _trailer_center(self):
        h, g = self._truck.h_rad, self._truck.g_rad
        th = h + g
        return (self._truck.x_m - HITCH_BACK * np.cos(h) - TRAILER_CENTER_BACK * np.cos(th),
                self._truck.y_m - HITCH_BACK * np.sin(h) - TRAILER_CENTER_BACK * np.sin(th))

    def state_message(self):
        cp = self._control_point()
        tc = self._trailer_center()
        progress = float(np.clip((cp[1] - self.path[0, 1]) / (self._tc_goal_y - self.path[0, 1]),
                                 0.0, 1.0))
        return {
            'type': 'state', 'protocol': PROTOCOL,
            'seq': self._seq, 't': round(self._seq * DT, 6),
            'phase': self._phase,
            'done': self._phase in ('done', 'failed'),
            'success': self._success,
            'pose': {
                'x': float(self._truck.x_m), 'y': float(self._truck.y_m),
                'h': float(self._truck.h_rad), 'g': float(self._truck.g_rad),
                'steer': float(np.deg2rad(self._truck.s_deg)),
                'speed': V_REVERSE if self._phase == 'running' else 0.0,
            },
            'trailerCenter': [float(tc[0]), float(tc[1])],
            'controlPoint': [float(cp[0]), float(cp[1])],
            'progress': progress,
            'error': {'lateral': self._last_e, 'psi': self._last_psi},
            'violationSteps': self._violation_steps,
        }

    def done_message(self):
        msg = self.state_message()
        return {
            'type': 'done', 'protocol': PROTOCOL,
            'success': bool(self._success),
            'steps': self._steps,
            'simTime': round(self._steps * DT, 3),
            'violationSteps': self._violation_steps,
            'maxLateralError': round(self._max_lat_err, 4),
            'finalPose': msg['pose'],
            'finalControlPoint': msg['controlPoint'],
            'finalTrailerCenter': msg['trailerCenter'],
        }

    def hello_message(self):
        return {
            'type': 'hello', 'protocol': PROTOCOL,
            'scenario': {
                'id': 'reverse_dock', 'name': '倒车入库',
                'controller': {
                    'id': 'cascade', 'name': 'Cascade · tail-in',
                    'source': 'another_solve/parking_three_lines.py', 'real': True,
                },
                'dt': DT, 'noiseStdDeg': self._noise_std, 'seed': self._seed,
                'maxSteps': MAX_STEPS,
                'estimatedSteps': 866, 'estimatedStepsNoiseFree': 867,
                'estimatedDurationSec': 43.3,
                'start': {'x': 56.25, 'y': 9.91, 'h': -np.pi / 2, 'g': 0.0},
                'goal': {'x': 58.25, 'y': 53.56},
                'goalMeaning': 'tractor reference point (hitch) at GOAL_X/GOAL_Y',
                'controlGoal': {'x': 58.0, 'y': 58.61},
                'controlGoalMeaning': 'virtual control point stop y = GOAL_Y + 11/2 - 0.45',
                'road': {'left': 50.0, 'center': 56.0, 'right': 60.0,
                         'yStart': 0.0, 'yEnd': 70.0},
                'slot': {'left': 56.0, 'right': 60.0, 'top': 50.5, 'bottom': 69.0},
                'vehicle': {
                    'truckWheelbase': 3.0, 'trailerWheelbase': 11.0,
                    'truckLength': 5.0, 'truckWidth': 3.0,
                    'trailerLength': 15.0, 'trailerWidth': 2.8,
                    'maxSteeringDeg': 35.0, 'maxSteeringRateDegPerSec': 60.0,
                    'maxArticulationDeg': 90.0, 'jackknifeDeg': 85.0,
                    'reverseSpeedMps': -1.0,
                },
                'referencePath': [[float(px), float(py)] for px, py in self.path[:, :2]],
                'referencePathMeaning':
                    'virtual control point = root - 5.5 m along trailer heading',
            },
            'state': self.state_message(),
        }
```

注意：`step()` 中噪声抽签顺序必须与 `run_once` 完全一致——`compute` 之后立刻抽 `rng.normal(0, noise_std)`，`reset()` 中先抽 `x0` 再抽 `h0`（顺序错了逐帧测试会在 1e-9 下立刻失败）。常量直接 import `config.scene_config`（与 `run_once` 同一来源，import 不影响 RNG 序列）。

- [ ] **Step 5: 运行测试确认通过**

Run: `cd /home/tong/semi_truck_rl && .venv/bin/python tests/test_demo_session.py`
Expected: 6 个测试全部 PASS（实际 6 个：`test_actual...`, `test_ideal...`, `test_initial...`, `test_max_steps...`, `test_reset...`, `test_start...`）。

- [ ] **Step 6: 跑既有基线确认未破坏**

Run: `cd /home/tong/semi_truck_rl && .venv/bin/python tests/test_parking_baseline.py`
Expected: PASS（本任务未改任何已验证文件）。

- [ ] **Step 7: Commit**

```bash
git add demo/__init__.py demo/session.py tests/test_demo_session.py
git commit -m "1a: ReverseDockSession 逐步镜像 run_once + 确定性回归测试"
```

---

### Task 4: `demo/server.py` — FastAPI + WebSocket + 协议冒烟测试

**Files:**
- Create: `demo/server.py`
- Test: `tests/test_demo_protocol_smoke.py`

**Interfaces:**
- Consumes: Task 3 的 `ReverseDockSession`（构造参数 `seed`, `noise_std`；方法 `step/start/pause/resume/reset/state_message/done_message/hello_message`；属性 `phase/seq`）。
- Produces: `app`（FastAPI 实例，lifespan 内创建 session 与 ticker）；CLI `python -m demo.server --host --port --speed --seed --noise --log-level`；`/health` 返回 `{ok, phase, clients, step}`。

- [ ] **Step 1: 写失败的测试 `tests/test_demo_protocol_smoke.py`**

```python
# -*- coding: utf-8 -*-
"""WS 协议 v1 冒烟 — 拉起真实 demo.server 子进程走完整协议 (spec §12.1)。

运行: .venv/bin/python tests/test_demo_protocol_smoke.py
"""
import asyncio
import json
import os
import subprocess
import sys
import time

import httpx
import websockets

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = sys.executable


def _start_server(port, speed):
    env = dict(os.environ, PYGAME_HIDE_SUPPORT_PROMPT='1')
    return subprocess.Popen(
        [PY, '-m', 'demo.server', '--host', '127.0.0.1', '--port', str(port),
         '--speed', str(speed)],
        cwd=ROOT, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)


def _wait_health(port, timeout=20.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = httpx.get(f'http://127.0.0.1:{port}/health', timeout=1.0)
            if r.status_code == 200 and r.json().get('ok'):
                return r.json()
        except httpx.TransportError:
            time.sleep(0.2)
    raise RuntimeError(f'server on {port} did not become healthy')


async def _recv_until(ws, pred, timeout=10.0):
    async def _loop():
        while True:
            msg = json.loads(await ws.recv())
            if pred(msg):
                return msg
    return await asyncio.wait_for(_loop(), timeout)


async def _interactive_check(port):
    async with websockets.connect(f'ws://127.0.0.1:{port}/ws') as ws:
        hello = json.loads(await ws.recv())
        assert hello['type'] == 'hello' and hello['protocol'] == 1
        assert len(hello['scenario']['referencePath']) == 220
        assert hello['scenario']['vehicle']['truckWheelbase'] == 3.0
        assert hello['state']['phase'] == 'ready'

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'reset'}))
        st = await _recv_until(ws, lambda m: m['type'] == 'state' and m['seq'] == 0)
        assert st['phase'] == 'ready' and st['progress'] == 0.0

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'start'}))
        st = await _recv_until(ws, lambda m: m['type'] == 'state' and m['phase'] == 'running')
        assert st['speed'] == -1.0 and st['seq'] >= 1

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'foo'}))
        err = await _recv_until(ws, lambda m: m['type'] == 'error')
        assert err['code'] == 'bad_command'

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'pause'}))
        st = await _recv_until(ws, lambda m: m['type'] == 'state' and m['phase'] == 'paused')
        assert st['seq'] >= 1

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'resume'}))
        st = await _recv_until(ws, lambda m: m['type'] == 'state' and m['phase'] == 'running')

        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'ping', 't': 123.4}))
        pong = await _recv_until(ws, lambda m: m['type'] == 'pong')
        assert pong['t'] == 123.4

        await ws.send('{bad json')
        err = await _recv_until(ws, lambda m: m['type'] == 'error')
        assert err['code'] == 'bad_json'

        await ws.send(json.dumps({'type': 'nope'}))
        err = await _recv_until(ws, lambda m: m['type'] == 'error')
        assert err['code'] == 'unknown_type'
    print('PASS interactive protocol checks')


async def _full_run_check(port):
    async with websockets.connect(f'ws://127.0.0.1:{port}/ws') as ws:
        hello = json.loads(await ws.recv())
        assert hello['state']['phase'] == 'ready'
        await ws.send(json.dumps({'type': 'cmd', 'cmd': 'start'}))
        done = await _recv_until(
            ws, lambda m: m['type'] == 'done', timeout=120.0)
        assert done['success'] is True and done['steps'] == 866
        assert done['violationSteps'] == 0
        assert abs(done['simTime'] - 43.3) < 0.05
    print('PASS full run to done (866 steps, success)')


def main():
    # 场景 A: 交互命令 (speed=1, 20 msg/s, 便于断言)
    pa = _start_server(8765, 1)
    try:
        _wait_health(8765)
        asyncio.run(_interactive_check(8765))
    finally:
        pa.terminate(); pa.wait()

    # 场景 B: 高速全 run (speed=1000, 几秒跑完 866 步)
    pb = _start_server(8766, 1000)
    try:
        _wait_health(8766)
        asyncio.run(_full_run_check(8766))
    finally:
        pb.terminate(); pb.wait()
    print('ALL PASS')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd /home/tong/semi_truck_rl && .venv/bin/python tests/test_demo_protocol_smoke.py`
Expected: FAIL —— `demo.server` 模块不存在（subprocess 起不来，`_wait_health` 超时）。

- [ ] **Step 3: 实现 `demo/server.py`**

```python
# -*- coding: utf-8 -*-
"""半挂车倒车入库 3D 演示后端 (spec §9.2)。

启动: .venv/bin/python -m demo.server --host 127.0.0.1 --port 8000 [--speed 1.0]
"""
import argparse
import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from another_solve.parking_three_lines import ACTUAL_SEED, DT, NOISE_STD
from demo.session import PROTOCOL, ReverseDockSession

log = logging.getLogger('demo.server')

MAX_CATCHUP_STEPS = 4        # 一次醒来最多补几步, 宁可慢不跳帧 (spec §9.2)
SEND_TIMEOUT = 0.25          # 广播发送超时, 超时客户端判死剔除


def create_app(speed=1.0, seed=ACTUAL_SEED, noise=NOISE_STD):
    session = ReverseDockSession(seed=seed, noise_std=noise)
    clients = set()
    state = {'speed': speed}

    async def broadcast(msg):
        dead = []
        for ws in list(clients):
            try:
                await asyncio.wait_for(ws.send_json(msg), SEND_TIMEOUT)
            except Exception:
                dead.append(ws)
        for ws in dead:
            clients.discard(ws)

    async def ticker():
        interval = DT / state['speed']
        accum = 0.0
        last = asyncio.get_running_loop().time()
        while True:
            await asyncio.sleep(min(interval, 0.05))
            now = asyncio.get_running_loop().time()
            accum += now - last
            last = now
            if accum > interval * MAX_CATCHUP_STEPS:
                accum = interval * MAX_CATCHUP_STEPS   # 追帧上限
            if session.phase == 'running':
                while accum >= interval:
                    accum -= interval
                    try:
                        msg = session.step()
                    except Exception as exc:           # 单步异常: 回 error, phase=failed
                        log.exception('step failed')
                        session._success = False
                        session._phase = 'failed'
                        await broadcast({'type': 'error', 'protocol': PROTOCOL,
                                         'code': 'internal', 'message': str(exc)})
                        msg = session.state_message()
                    if msg is not None:
                        await broadcast(msg)
                    if session.phase in ('done', 'failed'):
                        await broadcast(session.done_message())
                        break
            else:
                accum = min(accum, interval)

    @asynccontextmanager
    async def lifespan(app):
        task = asyncio.create_task(ticker())
        yield
        task.cancel()

    app = FastAPI(lifespan=lifespan)

    @app.get('/health')
    async def health():
        return {'ok': True, 'phase': session.phase,
                'clients': len(clients), 'step': session.seq}

    @app.websocket('/ws')
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        clients.add(ws)
        await ws.send_json(session.hello_message())
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    await ws.send_json({'type': 'error', 'protocol': PROTOCOL,
                                        'code': 'bad_json',
                                        'message': 'message is not valid JSON'})
                    continue
                if not isinstance(msg, dict) or msg.get('type') != 'cmd':
                    await ws.send_json({'type': 'error', 'protocol': PROTOCOL,
                                        'code': 'unknown_type',
                                        'message': 'expected {"type":"cmd",...}'})
                    continue
                cmd = msg.get('cmd')
                if cmd == 'ping':
                    await ws.send_json({'type': 'pong', 'protocol': PROTOCOL,
                                        't': msg.get('t')})
                elif cmd in ('start', 'pause', 'resume', 'reset'):
                    getattr(session, cmd)()
                    await ws.send_json(session.state_message())
                else:
                    await ws.send_json({'type': 'error', 'protocol': PROTOCOL,
                                        'code': 'bad_command',
                                        'message': f'unknown cmd: {cmd}'})
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(ws)
            if not clients and session.phase == 'running':
                session.pause()          # 无人观看自动暂停 (spec §8.8)

    return app


def main():
    parser = argparse.ArgumentParser(description='semi truck reverse dock demo server')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--seed', type=int, default=ACTUAL_SEED)
    parser.add_argument('--noise', type=float, default=NOISE_STD)
    parser.add_argument('--log-level', default='info')
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level.upper(),
                        format='%(asctime)s %(levelname)s %(name)s %(message)s')
    app = create_app(speed=args.speed, seed=args.seed, noise=args.noise)
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level)


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd /home/tong/semi_truck_rl && .venv/bin/python tests/test_demo_protocol_smoke.py`
Expected: `PASS interactive protocol checks`、`PASS full run to done`、`ALL PASS`。

- [ ] **Step 5: 手动起服务再验一次 `/health`（可选但推荐）**

Run: `.venv/bin/python -m demo.server --port 8000 &` 然后 `curl -s http://127.0.0.1:8000/health`，kill 掉。
Expected: `{"ok": true, "phase": "ready", "clients": 0, "step": 0}`。

- [ ] **Step 6: Commit**

```bash
git add demo/server.py tests/test_demo_protocol_smoke.py
git commit -m "1a: FastAPI/WS 演示后端 + 协议冒烟测试"
```

---

### Task 5: `liveSim.js` + main.js 接入（live/fake 选择、phase 机、HUD、小地图、按键）

**Files:**
- Create: `hangar_spike/src/liveSim.js`
- Modify: `hangar_spike/src/main.js`
- Modify: `hangar_spike/index.html`（Task 1 已加元素；本任务确认 id 齐全）
- Modify: `hangar_spike/src/style.css`（Task 1 已加；本任务无需再改）

**Interfaces:**
- Consumes: Task 1 的 `SCENE`/`SCENE_ORIGIN`、Task 3/4 的协议 v1（`hello/state/done/error/pong`）、Task 1 给 `FakeParkingSim` 加的 `pause()/resume()/getPathPoints()`。
- Produces: `LiveSim` 类（构造 `{url, onPhase, onToast}`；方法 `connect/start/pause/resume/reset/update(dt)`；getter `phase/running/done/connected/latestState/pathPoints/scenario/doneInfo`；`update(dt)` 返回 `{x,y,h,g,steer,speed,progress}` 显示态）。main.js 是唯一消费者。

- [ ] **Step 1: 新建 `hangar_spike/src/liveSim.js`**

```js
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
    this._snap = true;
    this.connect();
  }

  // ---------------- 连接 ----------------

  connect() {
    if (this._stopped) return;
    const ws = new WebSocket(this._url);
    this._ws = ws;
    ws.onopen = () => { this._retry = 0; this._connected = true; };
    ws.onclose = () => {
      const was = this._connected;
      this._connected = false;
      if (was) this._onPhase('offline');
      else this._setPhase('offline');
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
      this._snap = false;
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
```

- [ ] **Step 2: main.js — live/fake 选择与 UI 状态机**

1. 顶部 import 追加 `import { LiveSim } from './liveSim.js';`。
2. 在 `const audio = ...` 之后、renderer 之前插入模式选择：

```js
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
```

（注意：LiveSim 的 WS 事件全是异步触发，回调执行时 `el`/`truck` 均已初始化；本块只创建 sim，初始摆位见 Step 4。）

3. 替换 `setPhase` 为支持 live 的多态版本（fake 行为与现状逐字等价）：

```js
function setPhase(next) {
  phase = next;
  const live = !useFake;
  const label = live ? 'LIVE · CASCADE' : 'FAKE DATA';
  if (phase === 'running') {
    el.simPill.classList.add('live');
    el.simPill.classList.remove('done', 'paused', 'offline');
    el.simPillText.textContent = `RUNNING · ${label}`;
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
```

4. `startDemo` / `resetDemo` / 按键 / 控制器与场景点击（fake 保持原语义，live 走命令）：

```js
function startDemo() {
  if (useFake) {
    if (sim.running) return;
    if (sim.done) sim.reset();
  } else if (!sim.connected || sim.running) {
    return;
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
```

（注意：live 模式下 phase 由服务端消息经 `handleSimPhase` 驱动，本地不直接 `setPhase`。fake 模式保持 `setPhase` 立即切换。）

按键：

```js
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
```

控制器卡点击 handler 改为（disabled 按钮天然不触发 click，这里防重复 active）：

```js
document.querySelectorAll('.controller:not(:disabled)').forEach((btn) => {
  btn.addEventListener('click', () => toast('1a 仅接入 Cascade · tail-in 真实数据'));
});
document.querySelectorAll('.scenario:not(:disabled)').forEach((btn) => {
  btn.addEventListener('click', () => toast('场景：倒车入库'));
});
```

5. `el` 对象追加三个引用：

```js
  latError: document.getElementById('latError'),
  psiError: document.getElementById('psiError'),
  doneSummary: document.getElementById('doneSummary'),
```

- [ ] **Step 3: main.js — HUD 与小地图**

1. `updateHud` 重写（live 用 `latestState` 显示服务端数值，含误差；fake 行为不变）：

```js
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
```

2. `drawVehicle` 内目标环：`TRACK.target` → `SCENE.goal`（Task 1 已改）；在其后追加控制点小菱形：

```js
  const cgX = mmX(SCENE.controlGoal.x);
  const cgY = mmY(SCENE.controlGoal.y);
  ctx.strokeStyle = 'rgba(127,180,201,0.7)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cgX, cgY - 4); ctx.lineTo(cgX + 4, cgY);
  ctx.lineTo(cgX, cgY + 4); ctx.lineTo(cgX - 4, cgY);
  ctx.closePath(); ctx.stroke();
```

3. `drawMinimap` 中的参考线（Task 1 已改为 `sim.getPathPoints()` 折线）在 live 模式自动画 220 点参考线，无需再改。

- [ ] **Step 4: main.js — 初始化顺序与入口**

当前 `main.js` 中 `sim` 在 import 处即 `new FakeParkingSim()`（模块级）。改为：

1. 删除模块级 `const sim = new FakeParkingSim();`，把 sim 创建放进 Step 2 的模式选择块。
2. `buildTruck()` 调用移到 sim 创建之后，并在 live 模式立即摆位：

```js
const truck = buildTruck();
if (useFake) {
  truck.setState(sim.state);
} else {
  truck.setState({ ...SCENE.start, steer: 0, speed: 0 }, 0);
}
```

（原代码中 `truck.setState(sim.state)` 处替换为上面这段。）

3. 入口段（文件尾部）改为：

```js
if (useFake && params.has('pose')) {
  applyPose(THREE.MathUtils.clamp(Number(params.get('pose')) || 0, 0, 1));
} else {
  setPhase(useFake ? 'idle' : 'offline');
}
frame();

if (useFake) {
  setTimeout(startDemo, 700);   // fake 自动开始 (spec §10.4.1)
}
```

（`applyPose` 保持仅 fake 可用。live 初始 pill 为 `RECONNECTING`，连上后 hello 驱动 `setPhase('ready')`。）

- [ ] **Step 5: 构建验证**

Run: `cd hangar_spike && npm run build`
Expected: PASS。

- [ ] **Step 6: Commit**

```bash
git add hangar_spike/src/liveSim.js hangar_spike/src/main.js
git commit -m "1a: liveSim WS 客户端 + main.js live/fake 双模式接入"
```

---

### Task 6: Vite 代理、本地联调、README、总验收

**Files:**
- Create: `hangar_spike/vite.config.js`
- Modify: `README.md`

**Interfaces:**
- Consumes: Task 1–5 全部产物。
- Produces: 可复现的双进程启动路径；`make demo-backend` / `make demo-frontend`（Task 2）。

- [ ] **Step 1: 新建 `hangar_spike/vite.config.js`**

```js
import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    proxy: {
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true, changeOrigin: true },
    },
  },
});
```

- [ ] **Step 2: 端到端联调（无浏览器环境下能做的全部自动化验证）**

终端 1 起后端，终端 2 用 vite dev 起前端并用 curl 验证代理与页面：

```bash
cd /home/tong/semi_truck_rl
.venv/bin/python -m demo.server --host 127.0.0.1 --port 8000 &
cd hangar_spike && npx vite --port 5173 &
sleep 3
curl -s http://127.0.0.1:5173/ | grep -q 'src/main.js' && echo 'dev server ok'
curl -s http://127.0.0.1:8000/health
# WS 经代理: 用 python websockets 连 ws://127.0.0.1:5173/ws 收 hello
.venv/bin/python - <<'EOF'
import asyncio, json, websockets
async def go():
    async with websockets.connect('ws://127.0.0.1:5173/ws') as ws:
        hello = json.loads(await ws.recv())
        assert hello['type'] == 'hello' and len(hello['scenario']['referencePath']) == 220
        print('vite /ws proxy ok')
asyncio.run(go())
EOF
kill %1 %2
```

Expected: `dev server ok`、`/health` 返回 ready、打印 `vite /ws proxy ok`。

- [ ] **Step 3: 回归全部测试**

Run:
```bash
cd /home/tong/semi_truck_rl
.venv/bin/python tests/test_parking_baseline.py
.venv/bin/python tests/test_demo_session.py
.venv/bin/python tests/test_demo_protocol_smoke.py
cd hangar_spike && npm run build
```
Expected: 全部 PASS / 构建通过。

- [ ] **Step 4: README 追加启动说明**

在 `README.md` 末尾追加：

```markdown

## 3D 实时演示台 (hangar live demo 1a)

需要两个终端:

```bash
make demo-install     # 首次: 装 fastapi/uvicorn/websockets/httpx
make demo-backend     # 终端 1: FastAPI + WS 后端, 127.0.0.1:8000
make demo-frontend    # 终端 2: Vite dev server
```

浏览器打开 `http://localhost:5173/` 看真实串级控制倒车入库(约 43.3 s);
`http://localhost:5173/?demo=1` 为前端假数据离线演示; `?ws=ws://...` 可直连后端。
Space 暂停/继续, R 重置。后端测试: `.venv/bin/python tests/test_demo_session.py`、
`tests/test_demo_protocol_smoke.py`。
```

- [ ] **Step 5: Commit**

```bash
git add hangar_spike/vite.config.js README.md
git commit -m "1a: vite /ws 代理 + 启动文档"
```

- [ ] **Step 6: 浏览器人工验收（用户在浏览器执行，spec §12.3）**

交付说明中附此清单，并如实列出未验证项：
1. 后端 + 前端启动，页面无 console 错误。
2. 初始车在 `(56.25, 9.91)`，道路/库位/起点/目标与 spec §4.5 一致。
3. 点「开始演示」：pill 变 `RUNNING · LIVE · CASCADE`，车按真实轨迹倒车，HUD 的铰接角/方向盘/车速/进度/横向误差/航向误差持续更新。
4. 小地图参考线（220 点）、目标环（58.25, 53.56）、控制点菱形（58, 58.61）与实际一致；入库后显示 `DOCKED`，汇总 866 步 / 成功 / 0 违规。
5. Space 暂停/继续，R 重置；重置后回到初始位姿、0% 进度。
6. 杀掉后端：pill 变 `RECONNECTING · CASCADE`，页面不崩；重启后端后恢复，状态与服务端一致。
7. `?demo=1` 在后端关闭时仍能播放，标识为 FAKE DATA。
8. 3D 相机在起点不被后墙遮挡；如遮挡，按 spec §10.4.10 调 `HOME_CAM`/偏移。

---

## 未验证项（交付说明必须如实列出，spec §13）

- 浏览器视觉效果（相机遮挡、布局）未经本环境确认——需用户走 Task 6 Step 6 清单。
- 显示插值 ~80 ms 滞后：3D 位置略滞后于 HUD 数值（HUD 用 latest，3D 用 display，spec §10.3）。
- 3D 挂车网格是风格化近似（中心 -6.7、14.4 m），与模型精确几何（-6.5、15 m）差 0.2–0.5 m；小地图/控制用模型值。
- `parking_three_lines` 导入 pygame 约 3 s，后端启动稍慢（已设 `PYGAME_HIDE_SUPPORT_PROMPT`）。
- `?pose=` 参数仅 fake 模式可用（live 由服务端状态驱动）。
