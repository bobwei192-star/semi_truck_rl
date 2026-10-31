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
