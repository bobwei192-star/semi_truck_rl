# 半挂车 3D 实时演示台 — 里程碑 1 设计

- 状态：待确认（确认后再进入实现计划）
- 日期：2026-10-28
- 范围：单场景「倒车入库」，真实车模 + 已验证串级控制器 + 浏览器 3D 前端
- 关键决定：**双轨，先串级后 MPC**；里程碑 1a 使用串级控制器，论文 MPC 修复放到 1b，不阻塞 1a

---

## 1. 意图与为什么

仓库里已经有三样东西：

1. `vehicle/TruckTrailerOnAxle`：真实半挂车运动学模型；
2. `another_solve/parking_three_lines.py`：一条已验证能成功倒车入库的「规划 + 串级控制 + 实际执行」管线；
3. `hangar_spike/`：一个已经确认视觉方向的 Vite + Three.js 机库风格 3D 前端，但它现在播放的是 `fakeSim.js` 编排出来的假轨迹。

里程碑 1 要把第 2 条的真实轨迹接到第 3 条上，让浏览器里看到的是 Python 实时算出来的车、挂车、转向和执行轨迹，而不是前端编排。前端只负责显示和交互，不改仿真算法。

这件事的价值不只是「好看」：它建立一条可复用的数据通路（车模 → 控制 → WebSocket → 3D），后面修好 MPC、接 RL、加侧方位场景时，都复用同一条通路。

---

## 2. 范围

### 2.1 里程碑 1a（本设计）

- 浏览器默认连接本地 Python 后端，显示真实数据。
- 场景只有「倒车入库」，控制器只有已验证的 `Cascade · tail-in`。
- 保留 `?demo=1` 假数据回退，用于断网演示和视觉对比。
- 保留 spike 的机库、切角金属面板、琥珀主按钮、3D 车+挂车、跟随相机、小地图、右侧参数/HUD。
- 修正场景里与源码不一致的常量与文案（道路、库位、起点、目标、铰接角上限等）。
- 后端可本地启动；前端 `npm run dev`；浏览器能看到真实轨迹。
- 增加确定性回归测试，保证实时步进结果与 `run_once()` 完全一致。

### 2.2 里程碑 1b（后续，不在本设计实现）

- 修论文 MPC：状态映射、`phi_1` 符号、Δδ 累加、倒车参考朝向、路径规划。
- 修好并通过入库验证后，再在 UI 里增加控制器切换。
- 1b 失败不影响 1a 交付；UI 不得把串级控制器标成 MPC。

### 2.3 非目标

- RL 接入（`env/truck_parking_env.py` 目前缺失）。
- 侧方位、S 弯、窄路会车等多场景。
- 多控制器对比、训练监控、回放列表。
- 3D 拖拽车辆、重新规划倒车（2D 窗口已有，3D 先不做）。
- C++/Qt、原生客户端、重做 UI。
- 多用户并发控制。

### 2.4 反目标

- 把假数据或前端编排的轨迹标成「实时 MPC」。
- 为了让 3D 好看而改控制算法、改停车判定、改成功条件。
- 用 `docs/doc_requirements_docking.md` 的旧参数覆盖当前源码。
- 用插值平滑掩盖碰撞或越界；插值只影响显示，不影响服务端真值。

---

## 3. 事实来源与已验证证据

### 3.1 事实来源优先级

1. 当前源码：`config/scene_config.py`、`vehicle/truckTrailerOnAxle.py`、`another_solve/parking_three_lines.py`。
2. 当前基线：`tests/parking_baseline.json`、`tests/test_parking_baseline.py`。
3. spike 前端：`hangar_spike/src/*`（视觉参考，不是物理真值）。
4. 旧文档：`docs/doc_requirements_docking.md` **已过时**，只作历史参考。

### 3.2 已验证的真实管线

`another_solve/parking_three_lines.py` 的默认 `plan_path()` + `run_once()` 已在当前环境验证：

| 指标 | 理想（无噪） | 实际（`NOISE_STD=1.0`, `ACTUAL_SEED=2`） |
|---|---|---|
| 步数 | 867 | 866 |
| 仿真时长 | 43.35 s | 43.30 s |
| 成功 | True | True |
| 角点违规步数 | 0 | 0 |
| 终点牵引车参考点 | (58.0035, 53.1586) | (58.1136, 53.1100) |
| 终点控制点 | (58.0034, 58.6586) | (58.1124, 58.6100) |
| 终点航向 h | -89.9956° | -89.9914° |
| 终点铰接角 g | -0.0036° | 0.0045° |
| 最大横向误差 \|e\| | 0.5836 m | 0.4702 m |
| 转向角范围 | — | -21.85° … +21.43° |

默认规划线：220 个点，`(56.25, 15.41)` → `(58.00, 58.61)`。默认起点 `(56.25, 9.91)`，航向 `-90°`。实际带噪起点为 `x=56.2786`、`h=-90.2614°`（seed 2）。

这些数值是 1a 的验收基准。

### 3.3 尚未验证的事

- 浏览器视觉基线在当前环境没有重新截图确认（本环境无浏览器）；用户此前已确认 spike 视觉方向。
- `fastapi`/`uvicorn` 只做了 `pip install --dry-run` 依赖解析验证，尚未安装和实际启动。
- 跟随相机在起点是否被机库后墙遮挡，只做了几何推断，未在浏览器确认。
- 3D 挂车网格是风格化近似，不是碰撞/控制使用的精确几何。
- 论文 MPC 目前不可用；原因见 §4.4，1a 不依赖它。

---

## 4. 坐标、符号、单位审计

### 4.1 状态定义

`parking_three_lines.py` 和 `TruckTrailerOnAxle` 使用：

```text
state = [x, y, h, g]
```

- `x, y`：牵引车几何中心（米）。`TruckTrailerOnAxle` 的 `get_corner_positions()` 和 `hangar_spike/src/truck.js` 都把 5 m × 3 m 的牵引车矩形中心放在 `(x, y)`。
- `h`：牵引车航向（弧度，归一化到 [-π, π]）。默认起点 `h = -π/2`，车头朝 -y。
- `g`：铰接角（弧度），**挂车航向 = h + g**。模型把 `g` 限制在 ±90°。
- 倒车速度 `v = -1.0 m/s`；`v < 0` 时沿车头反方向运动，所以 `y` 增大、朝库位方向走。
- 转向角 `s_deg`：前轮转角（度），模型内部限幅 ±35°，执行层再按 60°/s × 0.05 s = 3°/步 限速。

### 4.2 3D 场景映射

spike 的映射：

```text
scene.x = sim.x - 56
scene.z = sim.y - 35
root.position = (scene.x, 0, scene.z)
root.rotation.y = -h
trailerPivot.rotation.y = -g
steeringHubs.rotation.y = -steer
```

这套映射与 `truck.js` 现有实现一致：

- 牵引车根节点在 `(x, y)`；
- 铰接点（trailer pivot）在根节点局部 `x = -1.5`；
- 挂车几何中心在铰接点后方 `-6.5 m`，长度 15 m，宽 2.8 m（模型值）；
- 小地图使用同一套 `-1.5` / `-6.5` / 5×3 / 15×2.8 几何。

### 4.3 三个容易混淆的「中心」

这是本次审计最重要的发现：

| 名称 | 公式（世界坐标） | 用途 |
|---|---|---|
| 牵引车参考点 `(x, y)` | 模型状态本身 | 3D 根节点、车头矩形中心、WS `pose` |
| 挂车几何中心 | `(x, y) - 1.5·e_h - 6.5·e_{h+g}` | 3D 挂车外观、小地图、8 角点碰撞 |
| 控制器虚拟控制点 | `(x, y) - 5.5·e_{h+g}` | `plan_path()` 参考线、`ParkingController` 的 `e/psi`、停车判定 |

其中 `e_h = (cos h, sin h)`，`e_{h+g} = (cos(h+g), sin(h+g))`。

`ParkingController.trailer_center()` 的注释称它为「挂车后轴中心 / 铰点后方 L/2」，但代码实际是 `(x, y) - 5.5·e_{h+g}`，**没有减掉 1.5 m 铰接偏移**，也不是物理挂车后轴。它是一个被这版串级控制器调出来的虚拟控制点。1a 必须按代码真实语义使用它：

- 参考路径 = 虚拟控制点轨迹；
- 进度 = 虚拟控制点沿 y 的进度；
- 停车 = 虚拟控制点 y 到达 58.61；
- UI 文案称它为「控制点」，不能称「挂车中心」。

### 4.4 发现的源码不一致（先列出，1a 不擅自修）

1. **`TruckTrailerOnAxle` 类注释 vs 几何**：类注释说「第五轮是模型参考点」，但 `get_corner_positions()` 把 `(x, y)` 当作牵引车几何中心，铰接点在 `(x, y) - 1.5·e_h`。1a 采用几何实现（也是 `truck.js` 的实现），因为显示和角点碰撞都用它。
2. **`ParkingController.trailer_center()` 注释 vs 实现**：如上，虚拟控制点不是挂车中心。1a 不修改算法，只在契约和 UI 里如实命名。
3. **`another_solve/simulate.py` 的论文状态映射**：把 `phi_1` 写成 `h - g`，而实际挂车航向是 `h + g`；`x_C` 的语义也不匹配。该文件在 1a 不使用。
4. **`another_solve/main.py` 的 MPC 控制量语义**：`MPCController.solve()` 返回 Δδ，但主程序直接当绝对 δ 用，且不调用 `update()` 累加。该文件在 1a 不使用。
5. **论文 MPC 参考/路径**：`PathPlanner.generate_parking_path()` 目前是直线插值，参考朝向按正向切线算，倒车时没有 π 翻转；这些是 1b 的修复项。
6. **旧文档参数过时**：`docs/doc_requirements_docking.md` 写起点 54 / 目标 50.56 / 道路 48..64 / 库位 47.5..66 / 铰接 90 / 防折叠 85；当前源码是起点 56.25 / 目标 53.56 / 道路 50..60 / 库位 50.5..69 / 物理上限 90 / 防折叠告警 85。1a 以源码为准。
7. **spike 常量过时**：`fakeSim.js` 的起点 `(54, 10)`、目标 y `50.56`；`main.js` 的道路 `48..64`、库位 `56.2..63.8 × 47.7..65.8`、跟随相机偏移 `z=-25`。1a 修正这些。
8. **`GOAL_Y` 的语义**：`scene_config.py` 注释称 `(GOAL_X, GOAL_Y)` 为「库位中心」，但库位 y 范围是 50.5..69，中心是 59.75；`53.56` 实际是牵引车参考点（铰点附近）的目标。1a 的 `goal` 标记按「牵引车参考点目标」显示，另给 `controlGoal` 表示虚拟控制点停车目标。
9. **`render_utils.py` 的挂车中心偏移 -6.0**：与模型的 -6.5 不一致；1a 的 3D 前端不使用这个文件，小地图按 -6.5 画。
10. **基线步数**：`tests/parking_baseline.json` 的 867 是理想轨迹；带噪实际轨迹是 866。两者不能混用。1a 的界面显示实际轨迹，回归测试分别校验两条。

### 4.5 场景与车辆参数（1a 唯一口径）

| 项 | 值 | 来源 |
|---|---|---|
| 世界原点映射 | `(x-56, y-35)` | spike + `SCENE_ORIGIN` |
| 道路左/中/右 | 50 / 56 / 60 m | `ROAD_CENTER_X=56`, `LEFT_LANE_WIDTH=6`, `RIGHT_LANE_WIDTH=4` |
| 道路 y 范围 | 0 … 70 m | `ROAD_Y_START/END` |
| 库位 x | 56 … 60 m | 右边车道 |
| 库位 y | 50.5 … 69 m | `PARKING_TOP_Y/BTM_Y` |
| 起点 | `(56.25, 9.91)`, `h=-90°`, `g=0` | `scene_config.py` |
| 牵引车参考点目标 | `(58.25, 53.56)` | `GOAL_X/GOAL_Y` |
| 控制点停车目标 | `(58.00, 58.61)` | `GOAL_Y + 11/2 - 0.45` |
| 车头轴距 / 挂车轴距 | 3.0 / 11.0 m | `TruckTrailerOnAxle` |
| 车头尺寸 | 5.0 × 3.0 m | 3.0 + 2×1.0，宽 3.0 |
| 挂车尺寸 | 15.0 × 2.8 m | 11.0 + 1.0 + 3.0，宽 2.8 |
| 铰接点 | 根节点后 1.5 m | 车头矩形后缘 + 1.0 m margin |
| 挂车中心 | 铰接点后 6.5 m | 模型 `get_corner_positions()` |
| 最大转向 | ±35° | 模型 |
| 转向速率 | 60°/s，即 3°/步 | `MAX_STEERING_RATE` |
| 铰接角物理上限 | ±90° | 模型 |
| 防折叠告警 | 85° | `JACKKNIFE_DEG` |
| 倒车速度 | -1.0 m/s | `V_REVERSE` / `FIXED_SPEED` |
| 仿真步长 | 0.05 s | `DT` |
| 实际轨迹噪声 | 1.0°/步，seed 2 | `NOISE_STD`, `ACTUAL_SEED` |

---

## 5. 控制链决策：双轨，先串级后 MPC

### 5.1 备选

| 方案 | 内容 | 结论 |
|---|---|---|
| A. 直接接论文 MPC | 修 `simulate.py`/`main.py` 的状态映射、Δδ、参考朝向、路径 | 已验证不收敛（300 步跑到 (-11.88, 11.29)，2000 步跑到 (-53.64, 10.01)）；风险高，会阻塞 1a |
| B. 先接已验证串级控制器 | 用 `parking_three_lines.py` 的 `Cascade · tail-in` | **选中**；866 步成功、0 违规、可复现 |
| C. 先接 RL | `env/truck_parking_env.py` 缺失，checkpoint 语义未验证 | 超出范围 |

### 5.2 1a 使用的控制律（不改）

- `ParkingController`：`k_psi=6.0`, `k_lateral=0.01`, `k_steer=6.0`, `g_des_max=5°`。
- 目标铰接角 `g_des = clip(-k_psi·psi + k_lateral·e, ±5°)`；前轮指令 `swa = k_steer·(g_des - g)`。
- 防折叠：`g > 40°` 时满反打 -35°，`g < -40°` 时 +35°。
- 执行层：幅值 ±35°，速率 ±3°/步；带噪实际轨迹每步叠加 `N(0, 1°)`。
- 停车：虚拟控制点 `y >= 58.61`；成功 = 无角点违规且 8 角点全在库位内。

1a 不修改这个控制律，也不把论文 MPC 的代码路径接进来。

---

## 6. 后端接入方案与决策

### 6.1 备选

| 方案 | 结构 | 优点 | 代价 | 结论 |
|---|---|---|---|---|
| A. 同进程 FastAPI + WebSocket | Python 进程内直接持有 session，浏览器经 Vite `/ws` 代理连接 | 无 IPC；固定步长和状态都在一个事件循环里；暂停/重置/重连简单；依赖少 | 后端和 HTTP 框架同进程 | **选中** |
| B. 独立仿真进程 + WebSocket 桥 | 仿真进程和 WS 服务分开，通过队列/管道传状态 | 仿真可独立重启 | 多进程生命周期、序列化、重连状态归属都更复杂；本场景 CPU 很轻，不值得 | 不选 |
| C. 前端直接轮询本地 HTTP | 前端定时 GET 状态，POST 命令 | 实现最直观 | 轮询延迟/抖动；固定步长和暂停语义难做对；不是推送 | 不选 |

### 6.2 决策 A 的具体形态

- 一个 Python 进程：FastAPI + uvicorn，端口默认 `127.0.0.1:8000`。
- 一个 `asyncio` 事件循环，**不用线程**：控制器单步是 NumPy 小计算，远小于 50 ms；线程只会引入锁和竞态。
- 一个全局 `ReverseDockSession`（单操作员）。所有 WS 客户端订阅同一份状态，命令作用于同一 session。多标签页会互相影响，明确作为非目标。
- 固定步长 `DT=0.05`，默认 20 Hz 实时推进；`--speed` 只改变墙钟推进倍率，不改变 `DT` 和轨迹。
- 服务端权威：前端不发 tick，不插值回传；只发命令。
- 重连：session 保留在服务端；最后一个客户端断开时，若正在 running，自动置为 paused，避免无人观看时跑完。重连后 `hello` 带当前状态，用户手动继续。
- 暂停/重置：命令直接改 session phase；暂停不推进、不丢状态；重置回到确定性初始位姿。
- 状态广播：每个仿真步一条 `state`；命令导致 phase 变化时立即补发一条，不等下一 tick。

### 6.3 为什么不是「每连接一个 session」

每连接一个 session 隔离性更好，但浏览器刷新/WS 抖动会丢掉 43 秒的进度。1a 的目标是单机演示台，单操作员；全局 session 的重连体验更好，实现也更少。多客户端隔离留到以后。

---

## 7. 总体架构

```text
浏览器 (Vite dev server, Three.js)
  ├─ truck.js / 机库 / 材质 / 相机 / 小地图 / HUD
  ├─ liveSim.js  ── WebSocket /ws ──┐
  ├─ fakeSim.js (?demo=1，不连后端)  │
  └─ sceneConfig.js（场景常量镜像）  │
                                     │ Vite proxy (ws)
                                     ▼
Python: FastAPI + uvicorn (127.0.0.1:8000)
  ├─ demo/server.py     WS 端点、命令、ticker、广播
  └─ demo/session.py    ReverseDockSession.reset()/step()
        ├─ plan_path() / ParkingController / corner_violations ...
        └─ vehicle/TruckTrailerOnAxle.drive(DT)
```

数据流：

1. 浏览器连 `/ws`，服务端回 `hello`（场景、车辆参数、220 点参考路径、当前状态）。
2. 用户点「开始演示」→ 前端发 `{"type":"cmd","cmd":"start"}`。
3. 服务端 20 Hz 调 `session.step()`，广播 `state`。
4. 前端用最新 `state` 驱动 3D 车、HUD、小地图；显示层做短时平滑。
5. 到达目标 → `state.phase=done` + `done` 汇总；前端显示「已入库」。
6. 重置 → `state.phase=ready`；暂停/继续同理。

---

## 8. WebSocket 协议 v1

### 8.1 连接

- 默认 URL：`ws://<当前页面 host>/ws`（由 Vite 代理到 `ws://127.0.0.1:8000/ws`）。
- 手动覆盖：`?ws=ws://127.0.0.1:8000/ws`。
- 子协议：不使用；消息为 JSON 文本。
- `protocol: 1` 出现在每条消息里；前端不匹配时提示但不断开。

### 8.2 消息总览

| type | 方向 | 频率 | 作用 |
|---|---|---|---|
| `hello` | S→C | 连接时一次 | 协议版本、场景、车辆、参考路径、当前状态 |
| `state` | S→C | 每步 1 条（默认 20 Hz）+ 命令后立即 1 条 | 实时位姿、误差、进度、phase |
| `done` | S→C | 终态一次 | 成功/失败汇总 |
| `error` | S→C | 出错时 | 协议/命令/内部错误 |
| `pong` | S→C | 收到 ping 时 | 延迟探测（可选） |
| `cmd` | C→S | 用户操作时 | start / pause / resume / reset / ping |

### 8.3 `hello`

```json
{
  "type": "hello",
  "protocol": 1,
  "scenario": {
    "id": "reverse_dock",
    "name": "倒车入库",
    "controller": {
      "id": "cascade",
      "name": "Cascade · tail-in",
      "source": "another_solve/parking_three_lines.py",
      "real": true
    },
    "dt": 0.05,
    "noiseStdDeg": 1.0,
    "seed": 2,
    "maxSteps": 2500,
    "estimatedSteps": 866,
    "estimatedStepsNoiseFree": 867,
    "estimatedDurationSec": 43.3,
    "start": { "x": 56.25, "y": 9.91, "h": -1.5707963267948966, "g": 0.0 },
    "goal": { "x": 58.25, "y": 53.56 },
    "goalMeaning": "tractor reference point (hitch) at GOAL_X/GOAL_Y",
    "controlGoal": { "x": 58.0, "y": 58.61 },
    "controlGoalMeaning": "virtual control point stop y = GOAL_Y + 11/2 - 0.45",
    "road": { "left": 50.0, "center": 56.0, "right": 60.0, "yStart": 0.0, "yEnd": 70.0 },
    "slot": { "left": 56.0, "right": 60.0, "top": 50.5, "bottom": 69.0 },
    "vehicle": {
      "truckWheelbase": 3.0,
      "trailerWheelbase": 11.0,
      "truckLength": 5.0,
      "truckWidth": 3.0,
      "trailerLength": 15.0,
      "trailerWidth": 2.8,
      "maxSteeringDeg": 35.0,
      "maxSteeringRateDegPerSec": 60.0,
      "maxArticulationDeg": 90.0,
      "jackknifeDeg": 85.0,
      "reverseSpeedMps": -1.0
    },
    "referencePath": [[56.25, 15.41], [56.25, 15.63], "… 共 220 点 …", [58.0, 58.61]],
    "referencePathMeaning": "virtual control point = root - 5.5 m along trailer heading"
  },
  "state": { "…见 8.4…" }
}
```

`referencePath` 只发一次；1a 的默认起点不重规划。以后支持拖拽重规划时，再加 `type:"path"` 消息。

### 8.4 `state`

```json
{
  "type": "state",
  "protocol": 1,
  "seq": 17,
  "t": 0.85,
  "phase": "running",
  "done": false,
  "success": null,
  "pose": {
    "x": 56.26,
    "y": 10.76,
    "h": -1.5708,
    "g": -0.0021,
    "steer": -0.0524,
    "speed": -1.0
  },
  "trailerCenter": [56.26, 18.76],
  "controlPoint": [56.26, 16.26],
  "progress": 0.02,
  "error": { "lateral": 0.003, "psi": -0.002 },
  "violationSteps": 0
}
```

字段语义：

| 字段 | 单位 | 说明 |
|---|---|---|
| `seq` | 整数 | 每条 `state` 递增；`reset` 后从 0 重计 |
| `t` | 秒 | 仿真时间 = 已执行步数 × 0.05 |
| `phase` | 枚举 | `ready` / `running` / `paused` / `done` / `failed` |
| `done` | bool | `phase in {done, failed}`，兼容前端原有 `state.done` |
| `success` | bool/null | 终态才给值 |
| `pose.x/y` | m | 牵引车几何中心（3D 根节点） |
| `pose.h` | rad | 牵引车航向，[-π, π] |
| `pose.g` | rad | 铰接角，挂车航向 = h+g，±90° 内 |
| `pose.steer` | rad | 已施加的前轮转角（限幅、限速后） |
| `pose.speed` | m/s | running 时为 -1.0，其它 phase 为 0.0 |
| `trailerCenter` | m | 挂车几何中心，`root -1.5·e_h -6.5·e_{h+g}` |
| `controlPoint` | m | 控制器虚拟控制点，`root -5.5·e_{h+g}` |
| `progress` | 0..1 | 控制点 y 相对 15.41→58.61 的进度，clamp |
| `error.lateral` | m | 控制器 `e`（控制点横向偏差） |
| `error.psi` | rad | 控制器 `psi`（挂车航向与局部参考方向之差，已 wrap） |
| `violationSteps` | 整数 | 曾出现至少一个角点越界的步数（不是角点总数） |

`steer` 用弧度，是为了直接兼容 `truck.js` 的 `hubs.rotation.y = -s.steer`；HUD 显示时转成度。

### 8.5 `done`

```json
{
  "type": "done",
  "protocol": 1,
  "success": true,
  "steps": 866,
  "simTime": 43.3,
  "violationSteps": 0,
  "maxLateralError": 0.4702,
  "finalPose": {
    "x": 58.1136,
    "y": 53.11,
    "h": -1.5706,
    "g": 0.00008,
    "steer": 0.0160,
    "speed": 0.0
  },
  "finalControlPoint": [58.1124, 58.61],
  "finalTrailerCenter": [58.1119, 61.11]
}
```

`state.phase=done/failed` 先发，`done` 紧随其后；前端以 `phase` 为准，`done` 用于显示汇总。

### 8.6 `error` 与 `pong`

```json
{ "type": "error", "protocol": 1, "code": "bad_command", "message": "unknown cmd: foo" }
{ "type": "pong", "protocol": 1, "t": 123.4 }
```

错误码：`bad_json`、`unknown_type`、`bad_command`、`internal`。错误不主动断开连接。

### 8.7 客户端命令

```json
{ "type": "cmd", "cmd": "start" }
{ "type": "cmd", "cmd": "pause" }
{ "type": "cmd", "cmd": "resume" }
{ "type": "cmd", "cmd": "reset" }
{ "type": "cmd", "cmd": "ping", "t": 123.4 }
```

语义：

- `start`：ready → running；paused → running；done/failed → 先 reset 再 running；running → no-op。
- `pause`：running → paused；其它 no-op。
- `resume`：paused → running；其它 no-op。
- `reset`：任意 → ready，重建 RNG/车/控制器，清除误差/违规，发 `seq=0` 的初始 `state`。
- `ping`：回 `pong`。

### 8.8 phase 状态机与断线

```text
ready --start--> running --pause--> paused --resume--> running
running --到达控制点目标--> done (success)
running --角点违规/不在库位 / 2500 步--> failed
任意 --reset--> ready
done/failed --start--> ready -> running
```

- 最后一个客户端断开时，running 自动变 paused（不广播，因为没有客户端）。
- 重连只发 `hello`，不自动 start；用户手动继续。
- 服务端内部异常：该步回 `error`，phase=failed，服务端不崩溃。

### 8.9 频率、顺序、精度

- 默认 20 Hz；每个仿真步一条 `state`。`--speed 2` 时墙钟 2×，但仍是每个仿真步一条（40 条/秒）。
- 消息按 `seq` 单调；前端丢弃旧 `seq`。
- JSON 数字用双精度浮点；前端显示时四舍五入，不用来逆向控制。
- `referencePath` 用 `[x, y]` 数组，不用对象，减少体积（约 4–5 KB）。

---

## 9. 后端设计

### 9.1 `demo/session.py`

`ReverseDockSession` 是 `run_once()` 的逐步骤镜像，**不复制、不改写控制算法**：

```python
class ReverseDockSession:
    def __init__(self, seed=ACTUAL_SEED, noise_std=NOISE_STD): ...
    def reset(self) -> None: ...
    def start(self) -> None: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def step(self) -> dict | None: ...
    def snapshot(self, seq: int) -> dict: ...
    @property
    def phase(self) -> str: ...
```

实现要点：

- 直接 `import another_solve.parking_three_lines as P`，复用 `plan_path`、`ParkingController`、`corner_violations`、`corners_in_slot`、`TruckTrailerOnAxle`、所有常量。**不搬代码、不改增益、不改停车判定。**
- `reset()` 严格按 `run_once()` 的顺序建 `np.random.default_rng(seed)`、`TruckTrailerOnAxle()`；有噪声时先抽 `x0 ~ N(0, 0.15)`，再抽 `h0 ~ N(0, 0.5°)`；`g=0`；`v=-1.0`；`prev_applied=0`。
- `step()` 严格按 `run_once()` 的一次循环：
  1. `controller.compute(truck)`；
  2. 有噪声时 `swa_deg += rng.normal(0, noise_std)`；
  3. 幅值 ±35°、速率 ±3°/步；
  4. `set_steering`、`drive(DT)`；
  5. 记录状态、控制点、误差、psi；
  6. `corner_violations(...)`，有则记一步；
  7. 控制点 y ≥ 58.61 → `success = 无违规 and corners_in_slot`，phase done/failed；
  8. 到 2500 步 → phase failed。
- `snapshot()` 计算 `trailerCenter`、`controlPoint`、`progress`、`error`，把 `truck.s_deg` 转弧度后输出。
- `phase` 与 `seq` 由 session 维护；`step()` 在非 running 时返回 `None`。
- 初始 `snapshot()`（未步进）也要能发：`t=0`、`progress=0`、`steer=0`、`speed=0`。

**为什么不抽成纯逻辑模块**：1a 优先低风险，直接复用已验证文件；`parking_three_lines` 导入 pygame 但不创建窗口，启动约 3 秒、会打印 pygame 横幅，可接受。后续如果启动时间成为问题，再做纯逻辑抽取，并用同一回归测试兜底。

### 9.2 `demo/server.py`

```python
app = FastAPI(lifespan=lifespan)
session = ReverseDockSession()
clients: set[WebSocket] = set()

@app.get("/health")
async def health(): ...

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket): ...
```

- `ws_endpoint`：`accept` → 加入 clients → 发 `hello` → 循环 `receive_json` → 分发 `cmd`；`WebSocketDisconnect` 时移除；若 clients 空且 running，置 paused。
- ticker：lifespan 启动一个 `asyncio.Task`；用 `loop.time()` + 累加器，每 `DT / speed` 秒检查；running 时 `session.step()`，广播 `state`；终态额外广播 `done`。
- 追帧上限：一次醒来最多补 4 步，避免系统卡顿后疯狂补帧；宁可慢，不跳帧。
- 广播：对每个 client `await asyncio.wait_for(ws.send_json(msg), 0.25)`；超时/异常则移除该 client，保护主循环。
- 命令处理与 ticker 都在同一事件循环；session 的方法是同步的，单次 step 不会在中间被抢占。命令导致的 phase 变化立即广播一条 state。
- CLI：`--host 127.0.0.1 --port 8000 --speed 1.0 --seed 2 --noise 1.0 --log-level info`。
- `/health` 返回 `{ok: true, phase, clients, step}`，方便本地检查。
- 启动前设置 `PYGAME_HIDE_SUPPORT_PROMPT=1`（在 import `parking_three_lines` 之前），减少横幅；不初始化 pygame display。

### 9.3 依赖

新增 `requirements-demo.txt`（不把 RL/torch 依赖拉进来）：

```text
fastapi>=0.115
uvicorn>=0.30
websockets>=13
httpx>=0.27
```

- `uvicorn` 不装 `[standard]`（避免 uvloop/httptools 在 Python 3.14 的额外构建风险）；显式装 `websockets` 让 uvicorn 的 WS 协议可用。
- `httpx` 供 `fastapi.testclient` 的 WS 冒烟测试。
- 已用 `pip install --dry-run fastapi uvicorn` 验证依赖可在 Python 3.14.4 解析；实际安装/启动在 1a 实现时验证。

---

## 10. 前端设计

### 10.1 模块划分

```text
hangar_spike/
  vite.config.js          # 新增: /ws 代理
  src/
    sceneConfig.js        # 新增: 场景/车辆常量镜像 (JS)
    liveSim.js            # 新增: WebSocket 客户端 + 显示插值 + 重连
    fakeSim.js            # 保留: ?demo=1 的编排轨迹
    main.js               # 修改: 选择 live/fake, 状态机, 场景常量, 小地图, HUD
    truck.js              # 修改: SCENE_ORIGIN 改从 sceneConfig 导入
    style.css             # 修改: 少量 phase/禁用态样式
  index.html              # 修改: 文案/参数/控制器与场景卡真实性
```

### 10.2 `sceneConfig.js`

把场景常量从 `fakeSim.js` / `main.js` 里抽出来，作为 JS 侧的单一镜像：

```js
export const SCENE_ORIGIN = { x: 56, y: 35 };
export const SCENE = {
  start: { x: 56.25, y: 9.91, h: -Math.PI / 2, g: 0 },
  goal: { x: 58.25, y: 53.56 },
  controlGoal: { x: 58.0, y: 58.61 },
  road: { left: 50, center: 56, right: 60, yStart: 0, yEnd: 70 },
  slot: { left: 56, right: 60, top: 50.5, bottom: 69 },
  vehicle: {
    truckWheelbase: 3.0, trailerWheelbase: 11.0,
    truckLength: 5.0, truckWidth: 3.0,
    trailerLength: 15.0, trailerWidth: 2.8,
    maxSteeringDeg: 35, maxArticulationDeg: 90, jackknifeDeg: 85,
    reverseSpeedMps: -1.0,
  },
};
```

注释里写明来源：`config/scene_config.py`、`vehicle/truckTrailerOnAxle.py`。JS 镜像与 Python 常量手工同步；1a 不引入构建期生成。

### 10.3 `liveSim.js`

对 `main.js` 暴露与 `FakeParkingSim` 接近的接口，减少主循环改动：

```js
class LiveSim {
  constructor({ url, onPhase, onToast }) {}
  connect() {}
  start() {}      // send cmd:start
  pause() {}      // send cmd:pause
  resume() {}     // send cmd:resume
  reset() {}      // send cmd:reset
  update(dt) {}   // 只推进显示插值，返回 displayState
  get phase() {}
  get running() {}
  get done() {}
  get connected() {}
  get latestState() {}
  get pathPoints() {}   // hello.referencePath
  get scenario() {}
}
```

行为：

- **连接**：默认 `ws(s)://location.host/ws`；支持 `?ws=...`；断开后 1s → 2s → 4s → 5s 退避重连。
- **hello**：保存 scenario/vehicle/pathPoints；把显示状态 snap 到 `hello.state`；回调 `onPhase`。
- **state**：按 `seq` 丢弃旧消息；更新 `latestState`；首次/重置/seek 时 snap，其它时候平滑。
- **done**：保存汇总，回调 `onPhase('done'|'failed')`，给 toast。
- **error**：toast 显示。
- **断线**：保留最后一帧，`connected=false`，pill 显示 RECONNECTING；不自动切到 fake，避免把假数据当真实数据。
- **重连**：hello 的 phase 可能是 running/paused；按服务端状态恢复，不自动 start。
- **不是仿真器**：`update(dt)` 不积分车辆，只做显示插值。

显示插值：

- `x/y` 指数平滑：`display += (latest - display) * (1 - exp(-dt * 12))`；
- `h/g/steer` 走最短角差再平滑；
- `progress` 直接取 latest（或同样平滑）；
- `speed` 取 latest；
- 收到 `reset` 后 `seq` 回 0，立即 snap，避免从库位滑回起点；
- 插值只影响视觉；HUD 数字可显示 latest，避免「数据也对不上」的质疑。建议 HUD 用 latest，3D 用 display。

### 10.4 `main.js` 修改点

1. 选择模式：
   ```js
   const params = new URLSearchParams(location.search);
   const useFake = params.get('demo') === '1';
   const sim = useFake ? new FakeParkingSim() : new LiveSim({ ... });
   ```
   `?demo=1` 保持自动开始（700 ms），不建立 WS。
2. 初始化 3D 车：live 模式先用 `SCENE.start` 摆好，收到 hello 后 snap 到服务端初始位姿。
3. `setPhase` 扩展为 `ready/running/paused/done/failed/offline`；pill 文案区分 `LIVE · CASCADE`、`FAKE DATA`、`RECONNECTING`。
4. 控制器卡：`Cascade · Tail-in` 激活并标「真实数据」；`MPC · Paper`、`RL · DDPG+HER` 禁用并标「1b / 未接入」。不把串级叫 MPC。
5. 场景卡：倒车入库激活；其它禁用并标「未接入」。
6. 参数行：道路/库位/起点/目标/铰接角文案按 §4.5 修正；「铰接角上限 ±90.0°」，在 note 里写「防折叠告警 85°」。
7. HUD：
   - 保留铰接角、方向盘、车速、进度；
   - 在现有 2 列网格里增加「横向误差 e (m)」和「航向误差 ψ (°)」两格（不新增面板，不改布局结构）；
   - 进度条用 `state.progress`。
8. 小地图：
   - 道路/库位/起点/目标全部来自 `sceneConfig`；
   - 参考线改为 `sim.pathPoints` 折线（live 是 220 点；fake 采样 71 点）；
   - 目标环画在 `SCENE.goal`（牵引车参考点目标）；控制点目标用一个小菱形/细环区分；
   - 车辆矩形用模型几何（-1.5 / -6.5 / 5×3 / 15×2.8）。
9. 按钮/按键：
   - Start：ready → start；paused → resume；done/failed → start（服务端会先 reset）；running → 禁用；
   - Reset：reset；
   - Space：running → pause，paused → resume，其它 → start；
   - R：reset；
   - fake 模式保持原有自动开始和 Space/R 语义，并额外支持 pause/resume。
10. 跟随相机：
    - `FOLLOW_OFFSET` 的 z 从 `-25` 改为 `-18`。原因：起点挂车-牵引车中心在 scene z≈-21，z=-25 会把相机放到机库后墙外侧（墙中心 z=-42，厚 1.2），后墙外表面会挡住画面；z=-18 时相机在 z≈-39，位于墙内侧。
    - `HOME_CAM` 暂不改（spike 已确认）；如果浏览器里初始视角有穿墙/黑面，再把 z 从 -42 调到 -36，并截图确认。
11. 断开/离线：
    - 默认 live 模式断线时 pill 显示 `RECONNECTING · CASCADE`，Start 禁用；
    - toast 提示「后端未连接；离线可加 ?demo=1 看假数据」；
    - 不自动回退 fake。

### 10.5 `fakeSim.js` 保留策略

- 保留 `FakeParkingSim` 的编排逻辑、2.5 m/s 加速和摆动，作为 `?demo=1` 的离线演示与视觉对比。
- 只把起点/目标对齐到真实场景：`start=(56.25, 9.91)`、`target=(58.25, 53.56)`，并相应调整曲线首尾控制点；中间形状保持。
- fake 的 pill、toast、面板明确标 `FAKE DATA`；不能与 live 混淆。
- fake 仍由前端生成，不连后端；`?demo=1` 自动开始。

### 10.6 `truck.js`

- `SCENE_ORIGIN` 改从 `sceneConfig.js` 导入，解除 live 模式对 `fakeSim.js` 的依赖。
- `setState` 的坐标/旋转/铰接/转向映射保持不变。
- 挂车网格保持风格化近似（中心 -6.7、车身 14.4 m），不参与控制/碰撞；小地图和角点判定用模型精确值。若 1a 验收时视觉误差明显，再单独做网格对齐，不在本里程碑混入算法改动。

---

## 11. 目录、依赖、启动

### 11.1 新增/修改文件

```text
demo/
  __init__.py
  session.py
  server.py
tests/
  test_demo_session.py
requirements-demo.txt
hangar_spike/
  vite.config.js
  src/sceneConfig.js
  src/liveSim.js
  src/main.js          (修改)
  src/fakeSim.js       (修改常量/端点)
  src/truck.js         (修改 import)
  index.html           (修改文案/参数/卡片)
  src/style.css        (修改少量状态样式)
Makefile               (新增 demo 目标)
README.md              (可选: 本地启动说明)
```

### 11.2 启动

终端 1（后端）：

```bash
source .venv/bin/activate
pip install -r requirements-demo.txt
make demo-backend
# 等价于 .venv/bin/python -m demo.server --host 127.0.0.1 --port 8000
```

终端 2（前端）：

```bash
make demo-frontend
# 等价于 cd hangar_spike && npm run dev
```

浏览器：

```text
http://localhost:5173/              # live, 真实数据
http://localhost:5173/?demo=1       # fake, 离线演示
http://localhost:5173/?ws=ws://127.0.0.1:8000/ws   # 绕过代理直连
```

`vite.config.js`：

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

`Makefile` 新增：

```make
demo-backend:
	$(PY) -m demo.server --host 127.0.0.1 --port 8000

demo-frontend:
	cd hangar_spike && npm run dev

demo-install:
	$(PY) -m pip install -r requirements-demo.txt
```

不把两个进程塞进一个 `make demo`；本地用两个终端最稳，避免后台进程/端口清理问题。

---

## 12. 测试与验收

### 12.1 后端确定性测试（`tests/test_demo_session.py`）

1. **带噪实际轨迹逐帧一致**：
   - `expected = P.run_once(P.plan_path(), noise_std=1.0, seed=2)`；
   - session 逐步 `step()`，收集 `(x, y, h, g, s_deg)`；
   - 断言步数 866、`success=True`、`violationSteps=0`、与 `expected['states']` 在 `1e-9` 内一致。
2. **理想轨迹逐帧一致**：
   - `noise_std=0.0`，867 步，最终控制点 `(58.0034, 58.6586)` 附近，`max|e|=0.5836`。
3. **初始位姿**：
   - seed 2 初始 `x≈56.2786`, `y=9.91`, `h≈-90.2614°`, `g=0`, `steer=0`。
4. **reset 确定性**：
   - 跑 N 步 → reset → 再跑 N 步，两次状态完全一致。
5. **start 语义**：
   - ready → running；paused → running；done → reset + running。
6. **协议冒烟**（`fastapi.testclient`，测试时关闭 ticker）：
   - 连 `/ws` 收到 `hello`，`protocol=1`、`referencePath` 220 点、`vehicle` 字段齐全；
   - 发 `reset` 收到 `phase=ready`；
   - 发 `start` 收到 `phase=running`；
   - 发非法命令收到 `error`；
   - 发 `pause`/`resume` 收到对应 phase。

### 12.2 前端构建

```bash
cd hangar_spike && npm run build
```

必须通过；当前 spike 构建已在 4.39 s 内通过，改动后重新跑。

### 12.3 浏览器人工验收（必须实际打开页面）

1. 后端 + 前端启动，页面无 console 错误。
2. 初始车在 `(56.25, 9.91)`，道路/库位/起点/目标与 §4.5 一致。
3. 点「开始演示」：pill 变 `LIVE/RUNNING · CASCADE`，车按真实轨迹倒车，HUD 的铰接角/方向盘/车速/进度/误差持续更新。
4. 小地图的参考线、车辆、目标环与实际一致；车辆进入库位后显示 `DOCKED`，汇总步数 866、成功、0 违规。
5. Space 暂停/继续，R 重置；重置后回到初始位姿和 0% 进度。
6. 杀掉后端：pill 变 RECONNECTING，页面不崩；重启后端后能恢复，状态与服务端一致。
7. `?demo=1` 在后端关闭时仍能播放，且所有 live 标识都明确是 `FAKE DATA`。
8. 3D 相机在起点不被后墙遮挡；如仍遮挡，按 §10.4 调整 `HOME_CAM`/偏移并截图确认。

### 12.4 通过标准

- 后端逐步结果与 `run_once()` 完全一致（确定性是硬指标）。
- 浏览器里看到的是真实串级控制轨迹，不是 fake；UI 不把串级标成 MPC。
- `?demo=1` 可用，且明确标注假数据。
- 本地启动命令可复现，`npm run build` 通过。
- 所有「未验证」项在交付说明里如实列出。

---

## 13. 风险与未验证项

| 风险 | 影响 | 处理 |
|---|---|---|
| 1a 全程 43.3 s | 演示节奏偏慢 | 默认 1×，保留 `--speed` 可调；不改变 DT/轨迹 |
| 浏览器视觉基线未在本环境复验 | 相机/布局可能需微调 | 交付时请用户在浏览器确认；提供截图/回退点 |
| 跟随相机起点在后墙外 | 画面被墙挡住 | 已把 `FOLLOW_OFFSET.z` 改为 -18；仍以浏览器为准 |
| 3D 挂车网格与模型差 0.2–0.5 m | 视觉边缘与控制几何不完全一致 | 网格仅外观；小地图/角点用模型值；必要时后续单独对齐 |
| `parking_three_lines` 导入 pygame 约 3 s、打印横幅 | 后端启动稍慢 | 设 `PYGAME_HIDE_SUPPORT_PROMPT`；不改验证过的文件 |
| Python 3.14 依赖 | FastAPI/uvicorn 运行时未装过 | dry-run 已解析；1a 安装后跑 `/health` 和 WS 冒烟 |
| 单全局 session | 多标签页互相影响 | 明确单操作员；多客户端非目标 |
| 显示插值 ~80 ms 滞后 | 3D 与 HUD 数字可能略有差异 | HUD 用 latest；插值仅视觉；文档说明 |
| 论文 MPC 仍不可用 | 1b 不确定 | 1a 不依赖；UI 明确 `MPC · Paper / 1b` |
| 源码注释与实现不一致 | 误读控制点/参考点 | §4.3/§4.4 记录；契约按实现语义命名 |

---

## 14. 实现顺序（确认后执行）

1. **场景常量与文案**：新增 `sceneConfig.js`；修正 `main.js`/`fakeSim.js`/`truck.js` 的道路、库位、起点、目标、铰接角、控制器/场景卡文案；`FOLLOW_OFFSET.z=-18`。
2. **后端 session**：`demo/session.py` 镜像 `run_once()`；先写确定性测试，再实现到测试通过。
3. **后端 server**：`demo/server.py`、`/health`、`/ws`、ticker、命令、重连/暂停；协议冒烟测试。
4. **前端 live 客户端**：`liveSim.js`、`main.js` 的 live/fake 选择、phase/按钮/按键、HUD 误差、小地图参考线。
5. **本地联调**：`requirements-demo.txt`、`vite.config.js`、Makefile；浏览器走一遍验收清单。
6. **收尾**：更新 README 启动说明；整理「未验证」清单；不提交任何未授权的 commit。

每一步都遵守：**不改控制算法、不改停车判定、不改成功条件**。如果实现中发现必须改算法才能跑通，停下来汇报，不擅自改。

---

## 15. 需要确认的点

本设计已经按「双轨，先串级后 MPC」和「同进程 FastAPI + WebSocket」定稿。以下是实现时会采用、但你可以否决的默认选择：

1. 全局单 session、单操作员；最后一个客户端断开时自动暂停。
2. 默认 1× 实时（43.3 s），`--speed` 仅用于演示加速。
3. 3D 显示做短时指数平滑，HUD 用服务端最新值。
4. 右侧 LIVE 网格增加「横向误差 e」「航向误差 ψ」两格；控制器/场景卡改为禁用+真实标注，不重做布局。
5. `?demo=1` 保留编排轨迹，仅把起点/目标对齐真实场景。
6. 跟随相机 z 偏移从 -25 改到 -18；`HOME_CAM` 暂不动。
7. 3D 挂车网格暂不按模型精确重建，只保证控制/小地图/角点用模型值。
8. 后端直接复用 `another_solve/parking_three_lines.py`，不先做纯逻辑抽取。

确认后，下一步按 `superpowers:writing-plans` 写实现计划；未确认前不写实现代码。
