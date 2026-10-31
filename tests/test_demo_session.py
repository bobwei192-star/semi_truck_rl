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
