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
                elif cmd == 'start':
                    session.start()
                    # 不回显 state: 权威测试要求首个 running state seq>=1,
                    # ticker 在 DT/speed (<=50ms) 内广播 step 后的 state (spec §8.7 只显式要求 reset 立即发 state)
                elif cmd in ('pause', 'resume', 'reset'):
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
