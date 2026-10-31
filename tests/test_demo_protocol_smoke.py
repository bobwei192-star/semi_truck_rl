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
