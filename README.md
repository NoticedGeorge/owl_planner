# 🦉 OWL-Planner

A dynamic path planning demo pipeline: **perception (clustering/simplification) → sliding-window planning (APF + corridor + polynomial optimization) → pure pursuit execution**, with a 3-thread pipeline + timestamp management, a real-time metrics HUD, mouse-driven **user waypoint** interaction, and a YAML config for environment tuning (obstacle count, speed range, etc.).

| Language / Standard | Dependencies | License |
| --- | --- | --- |
| C++17 / CMake ≥ 3.16 | PCL, Ceres, Eigen3, yaml-cpp | MIT |

## ✨ Features

- **Perception → Planning → Execution** 3-thread pipeline:
  - Perception thread (default 30 Hz): world snapshot → grid downsampling → DSU clustering → PCA simplification;
  - Planning thread (event-driven): replans only the **sliding window** ahead of the robot, emitting timestamped/sequenced plan frames;
  - Executor (main thread, 50 Hz): pure-pursuit tracking with plan-freshness checks that trigger replanning.
- **End-to-end latency tracking**: perception/planning/execution share one clock; the HUD shows perception processing time, perception→plan delay, plan time, and end-to-end latency.
- **User waypoint interaction**: left-click in the GUI to set a goal; consecutive clicks queue waypoints that are executed one by one.
- **YAML configuration**: obstacle count, speed range, radius/height ranges, world bounds, robot parameters — all configurable without recompiling; CLI flags override the YAML.
- **Safety mechanisms**: constant-velocity prediction, early replan triggering, pointwise plan safety validation, geometric emergency bypass, velocity-obstacle dodging, proximity slowdown/hard braking.
- **Visualization**: dark theme, gradient path, speed-colored obstacles, pulse animation, metrics HUD.

## 🏗️ Architecture

```
┌─────────────┐  30 Hz  ┌──────────────┐  event    ┌──────────────┐  50 Hz  ┌───────────┐
│  World      │ ──────▶ │ Perception   │ ────────▶ │  Planner     │ ──────▶ │ Controller │
│  (sim world)│ snapshot│ (perception) │ perc frame│ (sliding win)│ plan    │ (executor) │
└─────────────┘         └──────────────┘           └──────────────┘ frame   └───────────┘
     ▲                       │ timestamp+seq              ▲                  │
     └───────── robot state ◀─┴───── replan request ◀─────┴──────────────────┘
                                     │
                                     ▼
                            ┌────────────────┐
                            │   Visualizer   │  PCL/VTK visualization (HUD / gradient path / pulse)
                            │  + waypoints    │  LMB click = set waypoint
                            └────────────────┘
```

## 📁 Directory Layout

```
OWL-Planner/
├── CMakeLists.txt            # PCL / Ceres / Eigen / yaml-cpp / Threads
├── owl_planner.yaml          # default config (world, robot, pipeline params)
├── owl_planner_main.cpp      # entry point
├── README.zh-CN.md           # Chinese version of this README
├── include/
│   ├── common.h              # shared types: Obstacle, timestamped frames, PlanRequest, metrics
│   ├── Config.h              # global config struct + YAML/CLI loading
│   ├── World.h               # dynamic obstacle simulation world (configurable)
│   ├── Perception.h          # perception node
│   ├── Pipeline.h            # perception+planning thread pipeline (sliding window + timestamps)
│   ├── Controller.h          # pure-pursuit executor (replan decisions + window splicing + setGoal)
│   ├── Visualizer.h          # PCL visualization (with mouse waypoints)
│   └── Bypass.h / ObstacleProcessor.h / Planner.h
│       / PolynomialTrajectory.h / safeField.h   # planning algorithm modules
└── src/                      # implementations
```

## 🔧 Dependencies & Build

- CMake ≥ 3.16, C++17
- [PCL](https://pointclouds.org/) ≥ 1.12 (`common` / `io` / `visualization`)
- [Ceres Solver](http://ceres-solver.org/) ≥ 2.0
- Eigen3 ≥ 3.3
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) ≥ 0.6

Ubuntu / Debian:

```bash
sudo apt install cmake g++ libpcl-dev libceres-dev libeigen3-dev libyaml-cpp-dev
```

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## 🚀 Usage

```bash
./build/owl_planner                        # GUI mode (loads ./owl_planner.yaml)
./build/owl_planner --config my_env.yaml   # custom config file
./build/owl_planner --headless --seconds 120   # headless latency benchmark
```

### Command-line options

| Option | Description | YAML key |
| --- | --- | --- |
| `--config <file>` | YAML config file (default `owl_planner.yaml`) | — |
| `--headless` | run without GUI (auto-exit, prints metrics) | `simulation.headless` |
| `--debug` | print per-replan diagnostics | `simulation.debug` |
| `--seconds <s>` | headless sim duration | `simulation.run_seconds` |
| `--dt <s>` | simulation step | `simulation.dt` |
| `--seed <n>` | RNG seed | `simulation.seed` |
| `--cylinders <n>` | number of dynamic obstacles | `world.obstacles` |
| `--obstacle-speed <lo> <hi>` | obstacle speed range (m/s) | `world.speed.min/max` |
| `--window <m>` | sliding window length | `robot.window_len` |
| `--lookahead <m>` | pure-pursuit lookahead | `robot.lookahead` |
| `--robot-speed <m/s>` | cruise speed | `robot.speed` |
| `--start <x,y>` | robot start position | `robot.start` |
| `--goal <x,y>` | default goal position | `robot.goal` |

CLI flags take precedence over the YAML config.

## ⚙️ YAML Configuration

`owl_planner.yaml` covers simulation, world, robot, and pipeline sections. Edit it and re-run — no recompilation needed:

| Key | Default | Description |
| --- | --- | --- |
| `simulation.headless` | `false` | run without GUI |
| `simulation.run_seconds` | `120.0` | headless duration (s) |
| `simulation.dt` | `0.02` | simulation step (s), 50 Hz |
| `simulation.seed` | `42` | RNG seed (changes obstacle layout) |
| `simulation.debug` | `false` | replan diagnostics output |
| `world.bounds.xmin/xmax/ymin/ymax` | `0/20/0/15` | world bounds (m) |
| `world.obstacles` | `30` | **number of dynamic obstacles (cylinders)** |
| `world.radius.min/max` | `0.7 / 1.0` | obstacle radius range (m) |
| `world.height.min/max` | `1.0 / 2.5` | obstacle height range (m) |
| `world.speed.min/max` | `0.4 / 0.6` | **obstacle speed range (m/s)** |
| `world.clearance` | `1.0` | min spacing between obstacles (m) |
| `robot.start` | `[-10, -10]` | start position (m) |
| `robot.goal` | `[40, 40]` | default goal (m) |
| `robot.speed` | `2.0` | cruise speed (m/s) |
| `robot.safe_radius` | `0.2` | safety radius (m) |
| `robot.window_len` | `14.0` | sliding window length (m) |
| `robot.lookahead` | `2.5` | pure-pursuit lookahead (m) |
| `robot.step_spacing` | `0.05` | path sampling step (m) |
| `robot.plan_expiry_ms` | `250.0` | plan expiry threshold (ms) |
| `pipeline.perception_hz` | `30.0` | perception node frequency (Hz) |

Example: 60 obstacles, speed 0.2–1.2 m/s, world 30×20 m:

```yaml
world:
  bounds: {xmin: 0.0, xmax: 30.0, ymin: 0.0, ymax: 20.0}
  obstacles: 60
  speed: {min: 0.2, max: 1.2}
```

> Note: more/faster obstacles make the task harder. The defaults are the tuned baseline
> (3 seeds reach the goal headless with no collisions).

## 🖱️ Visualization & User Waypoints

- **Left-click**: set a user waypoint in the sim area. The robot retargets immediately;
  consecutive clicks append to a queue that is executed in order.
- **Left-drag**: rotate camera; **middle-drag**: pan; **wheel**: zoom.
- Waypoints are clamped to the world bounds (`world.bounds`) and rendered as green spheres
  labeled `WP0/WP1/...`; the current goal is shown as a pulsing green `GOAL`.
- The HUD shows: sim time, speed, perception rate, plan latency, replan count,
  perception age, end-to-end latency, window length, and status
  (FOLLOWING / PLANNING / COLLISION / GOAL).

## 🧠 Algorithm & Latency Optimization

**Sliding window**: replanning only processes the reference path within `window_len`
meters ahead of the robot (default 14 m, ~280 points); the new window is spliced with
the old path tail beyond the anchor. Compared to replanning the full 71 m path
(~1400 points), the planning scale drops by an order of magnitude — measured plan time
**0.005–8 ms**.

**Timestamp management**:
- Perception frame: `PerceptionFrame.stamp` = acquisition time, `seq` monotonic;
- Plan frame: `PlanFrame.frame.stamp` = completion time, plus
  `perception_seq / perception_stamp` (which perception frame it was based on)
  and `plan_ms / perception_age_ms`;
- Executor consumes frames deduplicated by `frame.seq` and triggers replanning when
  `plan_age` exceeds the threshold (default 250 ms).

Headless baseline (default params, 3 random seeds, 75 s):

```
reached goal : YES (3/3)   collision : NO
replans      : ~180–210
plan latency : 0.005–8 ms (complex bypass occasionally 3–8 ms)
E2E latency  : ~20–50 ms   perception : ~30 Hz
```

## 🛡️ Safety Mechanisms

- **Constant-velocity prediction**: obstacles are extrapolated 1.2 s ahead; planning avoids where obstacles *will be*;
- **Early replanning**: planner uses a detection margin (0.45 m) to start bypass before close-range failure;
- **Safety validation**: plan results are checked point-by-point against inflated obstacles and rejected on violation;
- **Geometric emergency bypass**: when planning fails repeatedly, a geometric detour is generated on the clearer side;
- **Dynamic dodging**: velocity-obstacle lateral dodge to avoid being rear-ended by dynamic obstacles;
- **Proximity slowdown / hard braking**: slows down near obstacles, stops within 0.25 m;
- Simulated obstacles avoid the robot (mirroring obstacle-obstacle separation) so a stopped robot is not overrun.

## 📄 License

[MIT](LICENSE) © 2026 GeorgeWang
