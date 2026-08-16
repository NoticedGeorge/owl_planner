# 🦉 OWL-Planner

动态环境下的路径规划演示工程：**感知（聚类/简化）→ 滑动窗口规划（APF + 走廊 + 多项式优化）→
纯追踪执行（Pure Pursuit）**，三线程流水线 + 时间戳管理，带实时指标 HUD 的可视化界面，
支持鼠标点击**用户定点（waypoint）**，并提供 YAML 配置自定义环境（障碍数量、速度范围等）。

| 语言 / 标准 | 依赖 | 协议 |
| --- | --- | --- |
| C++17 / CMake ≥ 3.16 | PCL、Ceres、Eigen3、yaml-cpp | MIT |

## ✨ 特性

- **感知 → 规划 → 执行** 三线程流水线：
  - 感知线程（默认 30 Hz）：世界快照 → 栅格降采样 → DSU 聚类 → PCA 简化建模；
  - 规划线程（事件触发）：只规划机器人前方**滑动窗口**内的路径，输出带时间戳/序号的规划帧；
  - 执行器（主线程，50 Hz）：纯追踪跟踪路径，依据规划帧时间戳判断新鲜度并触发重规划。
- **端到端延迟管理**：感知/规划/执行共用时钟源，HUD 实时显示感知处理耗时、感知→规划延迟、
  规划耗时、端到端延迟。
- **用户定点交互**：GUI 中 Shift+右键单击即可在任意位置设定目标点（含障碍物区域外），最多可预先排队 3 个、依次执行。
- **YAML 配置**：障碍数量、速度范围、半径/高度范围、障碍物区域（`world.bounds`）、定点工作范围（`world.work_range`）、机器人参数等全部可配置，
  无需重新编译；命令行参数可覆盖。
- **安全机制**：常速预测、提前触发、规划结果逐点安全校验、几何应急绕行、速度障碍式躲闪、
  近障减速/硬刹车。
- **可视化**：暗色主题、渐变路径、速度着色障碍、脉冲动画、指标 HUD。

## 🏗️ 架构

```
┌─────────────┐  30 Hz  ┌──────────────┐  事件触发  ┌──────────────┐  50 Hz  ┌───────────┐
│  World      │ ──────▶ │ Perception   │ ────────▶ │  Planner     │ ──────▶ │ Controller │
│  (仿真世界) │ 快照    │ (感知节点)    │ 感知帧     │  (滑动窗口)  │ 规划帧  │ (执行器)   │
└─────────────┘         └──────────────┘            └──────────────┘         └───────────┘
     ▲                       │ 时间戳+序号                     ▲                  │
     └────────── 机器人状态 ◀─┴────────── 重规划请求 ◀──────────┴──────────────────┘
                                     │
                                     ▼
                            ┌────────────────┐
                            │   Visualizer   │  PCL/VTK 可视化（HUD / 渐变路径 / 脉冲动画）
                            │  + 鼠标定点     │  Shift+RMB = 设置终点
                            └────────────────┘
```

## 📁 目录结构

```
OWL-Planner/
├── CMakeLists.txt            # PCL / Ceres / Eigen / yaml-cpp / Threads
├── owl_planner.yaml          # 默认配置文件（环境参数、机器人参数、管线参数）
├── owl_planner_main.cpp      # 程序入口
├── include/
│   ├── common.h              # 共享类型：Obstacle、时间戳帧、PlanRequest、指标
│   ├── Config.h              # 全局配置结构与 YAML/CLI 加载
│   ├── World.h               # 动态障碍仿真世界（参数可配置）
│   ├── Perception.h          # 感知节点
│   ├── Pipeline.h            # 感知+规划线程管线（滑动窗口 + 时间戳）
│   ├── Controller.h          # 纯追踪执行器（重规划决策 + 窗口拼接 + setGoal）
│   ├── Visualizer.h          # PCL 可视化（含鼠标定点）
│   └── Bypass.h / ObstacleProcessor.h / Planner.h
│       / PolynomialTrajectory.h / safeField.h   # 规划算法模块
└── src/                      # 对应实现
```

## 🔧 依赖与构建

- CMake ≥ 3.16，C++17
- [PCL](https://pointclouds.org/) ≥ 1.12（`common` / `io` / `visualization`）
- [Ceres Solver](http://ceres-solver.org/) ≥ 2.0
- Eigen3 ≥ 3.3
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) ≥ 0.6

Ubuntu / Debian 示例：

```bash
sudo apt install cmake g++ libpcl-dev libceres-dev libeigen3-dev libyaml-cpp-dev
```

构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## 🚀 运行

```bash
./build/owl_planner                        # GUI 模式（默认加载 owl_planner.yaml）
./build/owl_planner --config my_env.yaml   # 使用自定义配置文件
./build/owl_planner --headless --seconds 120   # 无界面延迟基准
```

### 命令行参数

| 参数 | 说明 | 对应 YAML |
| --- | --- | --- |
| `--config <file>` | 指定 YAML 配置文件（默认 `owl_planner.yaml`） | — |
| `--headless` | 无 GUI 运行（自动退出，打印指标汇总） | `simulation.headless` |
| `--debug` | 打印每次重规划的诊断信息 | `simulation.debug` |
| `--seconds <s>` | headless 仿真时长 | `simulation.run_seconds` |
| `--dt <s>` | 仿真步长 | `simulation.dt` |
| `--seed <n>` | 随机种子 | `simulation.seed` |
| `--cylinders <n>` | 动态障碍数量 | `world.obstacles` |
| `--obstacle-speed <lo> <hi>` | 障碍速度范围 (m/s) | `world.speed.min/max` |
| `--window <m>` | 滑动窗口长度 | `robot.window_len` |
| `--lookahead <m>` | 纯追踪前瞻距离 | `robot.lookahead` |
| `--robot-speed <m/s>` | 巡航速度 | `robot.speed` |
| `--start <x,y>` | 机器人起始位置 | `robot.start` |
| `--goal <x,y>` | 默认目标点 | `robot.goal` |

命令行参数的优先级高于 YAML 配置。

## ⚙️ YAML 配置说明

配置文件 `owl_planner.yaml` 覆盖仿真、世界、机器人、管线四部分，修改后无需重新编译：

| 配置项 | 默认值 | 说明 |
| --- | --- | --- |
| `simulation.headless` | `false` | 无 GUI 运行 |
| `simulation.run_seconds` | `120.0` | headless 模式时长 (s) |
| `simulation.dt` | `0.02` | 仿真步长 (s)，对应 50 Hz |
| `simulation.seed` | `42` | 随机种子，改变障碍布局 |
| `simulation.debug` | `false` | 重规划诊断输出 |
| `world.bounds.xmin/xmax/ymin/ymax` | `0/20/0/15` | 障碍物区域边界 (m)，障碍物被限制在此区域内 |
| `world.work_range.xmin/xmax/ymin/ymax` | `-20/60/-20/60` | 定点/检测范围 (m)，鼠标终点允许放置的区域 |
| `world.obstacles` | `30` | **动态障碍（圆柱）数量** |
| `world.radius.min/max` | `0.7 / 1.0` | 障碍半径范围 (m) |
| `world.height.min/max` | `1.0 / 2.5` | 障碍高度范围 (m) |
| `world.speed.min/max` | `0.4 / 0.6` | **障碍速度范围 (m/s)** |
| `world.clearance` | `1.0` | 障碍间最小间距 (m) |
| `robot.start` | `[-10, -10]` | 起始位置 (m) |
| `robot.goal` | `[40, 40]` | 默认目标点 (m) |
| `robot.speed` | `2.0` | 巡航速度 (m/s) |
| `robot.safe_radius` | `0.2` | 安全半径 (m) |
| `robot.window_len` | `14.0` | 滑动窗口长度 (m) |
| `robot.lookahead` | `2.5` | 纯追踪前瞻距离 (m) |
| `robot.step_spacing` | `0.05` | 路径采样间距 (m) |
| `robot.plan_expiry_ms` | `250.0` | 规划结果过期阈值 (ms) |
| `pipeline.perception_hz` | `30.0` | 感知节点频率 (Hz) |

示例：改成 60 个障碍、速度 0.2–1.2 m/s、障碍物区域 30×20 m、工作范围 -20…60 m：

```yaml
world:
  bounds: {xmin: 0.0, xmax: 30.0, ymin: 0.0, ymax: 20.0}
  work_range: {xmin: -20.0, xmax: 60.0, ymin: -20.0, ymax: 60.0}
  obstacles: 60
  speed: {min: 0.2, max: 1.2}
```

> 提示：障碍越多 / 速度越快，难度越大。默认参数为调优基线
> （3 个种子 headless 均可达终点、无碰撞）。

## 🖱️ 可视化与用户定点

- **Shift+右键单击**：在工作范围（`world.work_range`）内任意位置设置终点/定点（障碍物区域内外均可）。
  机器人立即转向该点；最多可预先放置 3 个定点，到达当前终点后自动依次执行。
- **左键拖拽**：旋转视角；**中键拖拽**：平移；**右键拖拽/滚轮**：缩放。
- 启动时相机自动框住整个网格，初始缩放即可看到完整工作区。
- 定点允许放置在**工作范围**（`world.work_range`，默认 `[-20,60] x [-20,60]`）内任意位置，
  比障碍物区域（`world.bounds`）更大；障碍物仍限制在中间区域。定点渲染为绿色球体 +
  `WP0/WP1/...` 标签；当前目标以脉冲绿球 `GOAL` 标示。最多可预先排队 3 个定点。
- 机器人碰撞后会安全停下；再次 Shift+右键点击即可重新设定终点并恢复运动。
- 左上 HUD 实时显示：仿真时间、速度、感知频率、规划延迟、重规划次数、
  感知年龄、端到端延迟、窗口长度与状态（FOLLOWING / PLANNING / COLLISION / GOAL）。

## 🧠 算法与延迟优化

**滑动窗口**：重规划只处理机器人前方 `window_len` 米内的参考路径（默认 14 m，
约 280 个点），规划完成后再与锚点之后的旧路径拼接。相比每次重规划整条
71 m 路径（约 1400 个点），规划规模下降一个量级，实测单次规划 **0.005–8 ms**。

**时间戳分配**：
- 感知帧：`PerceptionFrame.stamp` = 数据采集时刻，`seq` 单调递增；
- 规划帧：`PlanFrame.frame.stamp` = 规划完成时刻，并记录
  `perception_seq / perception_stamp`（基于哪一帧感知）与 `plan_ms / perception_age_ms`；
- 执行器按 `frame.seq` 去重消费，按 `plan_age`（默认 >250 ms）判断过期并触发重规划。

Headless 基准（默认参数，3 个随机种子，75 s 仿真）：

```
reached goal : YES（3/3）   collision : NO
replans      : ~180–210
plan latency : 0.005–8 ms（复杂绕行场景偶尔 3–8 ms）
E2E latency  : ~20–50 ms    perception : ~30 Hz
```

## 🛡️ 安全机制

- **常速预测**：感知对障碍按速度外推 1.2 s，规划避让的是障碍“即将到达”的位置；
- **提前触发**：规划器带检测余量（0.45 m），在近距离失效之前就开始绕行；
- **安全校验**：规划结果发布前与膨胀障碍逐点校验，不合格则丢弃并重试；
- **几何应急绕行**：规划连续失败时，按障碍两侧净空生成几何绕行路径；
- **动态避障**：速度障碍式横向躲闪，防止被动态障碍追尾；
- **近障减速 / 硬刹车**：越接近障碍越慢，0.25 m 内停车；
- 仿真障碍对机器人做避让（与障碍间避让同构），避免“静止机器人被障碍碾过”。

## 📄 License

[MIT](LICENSE) © 2026 GeorgeWang
