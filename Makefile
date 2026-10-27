# 半挂车仿真 — 常用命令
# make run        一键拉起倒车入库窗口 (图形化, 默认目标)
# make run-side   拉起侧方位窗口
# make headless   离屏生成两张截图
# make test       运行倒车入库基线测试

PY := .venv/bin/python

.PHONY: run run-side headless test

run: $(PY)
	$(PY) another_solve/parking_three_lines.py

run-side: $(PY)
	$(PY) another_solve/parking_parallel.py

headless: $(PY)
	$(PY) another_solve/parking_three_lines.py --headless
	$(PY) another_solve/parking_parallel.py --headless

test: $(PY)
	$(PY) tests/test_parking_baseline.py

$(PY):
	python3 -m venv .venv
	.venv/bin/pip install -q numpy pygame-ce
