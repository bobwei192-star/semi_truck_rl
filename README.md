# semi_truck_rl

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