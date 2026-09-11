# nano-astar

[English](README.md) | 中文

[![CI](https://github.com/ygxiuming/nano-astar/actions/workflows/ci.yml/badge.svg)](https://github.com/ygxiuming/nano-astar/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/nano-astar.svg)](https://pypi.org/project/nano-astar/)
[![Python](https://img.shields.io/badge/python-3.9%20–%203.12-blue.svg)](https://www.python.org)

**基于 C++ 内核的占用网格 A* 寻路 —— 实测仅搜索阶段比 networkx 快 14–18 倍，含构图快 52–63 倍，4 连通整数网格最高 464 倍**（精确数字、测试环境与复现脚本见 [BENCH.md](BENCH.md)）。

![A* 迷宫探索动画](https://raw.githubusercontent.com/ygxiuming/nano-astar/main/docs/demo.gif)

## 为什么选择 nano-astar

- **关键路径足够快。** C++17 搜索内核，缓存友好的扁平数组：关闭表每格 1 字节位图，节点链表内嵌 —— 无哈希表、无逐节点内存分配。
- **双开放列表，自动选择。** 4 连通整数网格走**桶队列**（均摊 O(1) 压入/弹出，真 decrease-key）；浮点代价（8 连通 `sqrt(2)` 对角边）自动回退 **4 叉堆**。同样的整数负载下桶队列实测比堆快 **1.3–2.4 倍**（[BENCH.md](BENCH.md)）。
- **真多线程。** 整个搜索期间释放 GIL（`nb::call_guard<nb::gil_scoped_release>`），线程真正并行 —— 实测 **4 线程 2.9 倍加速**（[test_gil_released_under_threads](tests/test_api.py)）。
- **numpy 零拷进零拷出。** 连续的 `uint8` 网格直接传入 C++ 不复制；路径以 `(n, 2)` `int32` 数组返回。
- **仅一个依赖。** 运行时只有 `numpy`。绑定用 [nanobind](https://github.com/wjakob/nanobind)，构建用 scikit-build-core。
- **对照参考实现做过差分测试。** 200 张随机网格与 networkx A* 对比：可达性与最优代价必须完全相等（整数）或误差 < 1e-6（浮点）（[tests/test_correctness.py](tests/test_correctness.py)）。

## 安装与 30 秒上手

```bash
pip install nano-astar
```

源码安装：`pip install .`

```python
import numpy as np
from nano_astar import astar

grid = np.zeros((100, 100), dtype=np.uint8)
grid[20:80, 50] = 1                       # 一堵墙

result = astar(grid, (0, 0), (99, 99), heuristic="octile", diagonal=True)
if result is None:
    print("不可达")
else:
    path, cost = result                   # path: (n, 2) int32，(行, 列)
    print(f"{len(path)} 个格子，代价 {cost:.3f}")

# 动画用探索历史：
path, cost, history = astar(grid, (0, 0), (99, 99), return_history=True)
```

终端演示：`python examples/demo.py`

## Benchmark

30% 障碍率随机网格，预热后取中位数，每次运行都与 networkx 交叉校验代价
（误差 < 1e-6）。完整方法学（重复次数、种子、公平性说明）与复现脚本见
[BENCH.md](BENCH.md)。环境：Python 3.11.15、networkx 3.6.1、
numpy 2.4.6、MSVC 19.51（`/O2`）、Windows 11。

8 连通网格，两侧均用 octile 启发式：

| 网格 | nano-astar | networkx（仅搜索） | networkx（构图+搜索） | 加速（仅搜索） | 加速（含构图） |
|---|---|---|---|---|---|
| 100×100 | 0.66 ms | 9.3 ms | 40.2 ms | 14× | 61× |
| 500×500 | 24.4 ms | 360.0 ms | 1273.4 ms | 15× | 52× |
| 1000×1000 | 115.9 ms | 2030.3 ms | 7304.9 ms | 18× | 63× |

4 连通整数网格，两侧均用 manhattan 启发式 —— 桶队列的主场：

| 网格 | nano-astar | networkx（仅搜索） | networkx（构图+搜索） | 加速（仅搜索） | 加速（含构图） |
|---|---|---|---|---|---|
| 100×100 | 0.07 ms | 3.1 ms | 289.2 ms | 43× | 4037× |
| 500×500 | 4.02 ms | 109.3 ms | 718.4 ms | 27× | 179× |
| 1000×1000 | 7.65 ms | 124.3 ms | 3546.6 ms | 16× | 464× |

![benchmark 图表](https://raw.githubusercontent.com/ygxiuming/nano-astar/main/docs/benchmark.png)

复现：

```bash
python tests/bench_vs_networkx.py   # 重写 BENCH.md + docs/bench_results.json
python tools/make_plots.py          # 重新生成图表
```

## 启发式可视化对比

同一张图、四种内置启发式（4 连通）。蓝色 = 探索过的格子，红色 = 最终路径。
启发式越紧，探索越少 —— 四种返回的最优代价完全相同。

![启发式对比](https://raw.githubusercontent.com/ygxiuming/nano-astar/main/docs/heuristics.png)

## 什么时候不要用 nano-astar

- **通用图。** 带属性的节点、边权重、非网格拓扑都不在范围内 —— 输入就是二值占用网格。请用 networkx、rustworkx 等图库。
- **自定义 Python 启发式。** 没有回调接口：四个内置启发式全部 C++ 内联，这正是速度的来源。需要领域特定启发式请 fork 后在 `src/cpp/heuristics.hpp` 里加。
- **带权地形。** 所有可走格代价相同（正交 1，对角 `sqrt(2)`）。每格不同通行代价的 costmap 需要别的引擎。
- **桶队列帮不上忙的场景。** 桶队列只在 4 连通整数代价网格（`diagonal=False`）启用；`diagonal=True` 时代价是 `sqrt(2)` 的无理数倍，引擎走 4 叉堆 —— 这是设计使然。整数网格上桶队列实测比堆快 1.3–2.4 倍（[BENCH.md](BENCH.md)），8 连通负载放弃的就是这个数。
- **`manhattan` 搭配 `diagonal=True`。** 曼哈顿在 8 连通网格上高估代价、会返回次优路径，nano-astar 会抛 `RuntimeWarning`。8 连通请用默认的 `octile`。

## API

```python
astar(grid, start, goal, heuristic="octile", diagonal=True,
      return_history=False) -> (path, cost) | (path, cost, history) | None
```

| 参数 | 含义 |
|---|---|
| `grid` | 二维数组；`0` = 可走，非零 = 障碍。连续 `uint8` 零拷贝；其他 dtype 经 `grid != 0` 转换一次。 |
| `start`、`goal` | `(行, 列)` 格子索引。 |
| `heuristic` | `"octile"`（默认，8 连通最紧）、`"manhattan"`（4 连通最紧）、`"euclidean"`、`"diagonal"`。全部 C++ 内联，无 Python 回调。 |
| `diagonal` | `True`：8 连通，对角步代价 `sqrt(2)`，禁止切角。`False`：4 连通单位代价 → 整数桶队列引擎。 |
| `return_history` | 额外返回按弹出顺序的关闭格 `(m, 2)` int32 —— 上面的 GIF 就是用它做的。 |

- 不可达时**返回 `None`**（绝不因此抛异常）。
- 起点/终点越界或压在障碍上、网格非二维、启发式名称未知 → **抛 `ValueError`**。
- 代价精确：整数网格返回整数值浮点，8 连通返回 `1`/`sqrt(2)` 项的精确和。
- 线程安全且释放 GIL：GIL 释放期间不触碰任何 Python 对象，多线程并发调用真正并行。

## 开发

```bash
uv venv --python 3.11 .venv
uv pip install --python .venv -e ".[dev]"
pytest tests/                          # 220 个测试，与 networkx 差分对比
python tests/bench_vs_networkx.py      # 重新生成 BENCH.md
python tools/make_gif.py               # 重新生成 docs/demo.gif
```

目录结构：

```
src/cpp/astar.cpp         引擎 + nanobind 绑定
src/cpp/bucket_queue.hpp  环形桶队列（整数代价，O(1) decrease-key）
src/cpp/heap4.hpp         4 叉堆（浮点/通用代价）
src/cpp/heuristics.hpp    octile / manhattan / euclidean / diagonal，内联
src/python/nano_astar/    Python 封装（参数校验、dtype 归一化）
tests/                    差分测试、API 测试、benchmark
tools/                    smoke 自检（纯 C++）、GIF/图表生成器
```

## 许可证

MIT
