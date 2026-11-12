# 恢复运动离线分析

`analyze.py` 读取 `PID_MOTION` / `PID_GATE` 日志，按相同来源、路径版本、控制时间匹配。
`pid_motion_replay` 直接调用生产代码 `buildMotionTransition`，再采样生成用于绘图的中心运动。
它不调用碰撞检测器，不启动 ROS 节点，也不改变导航控制行为。

## 使用

在工作空间根目录增量构建该包后：

```bash
python3 src/pid_tracking_controller/tools/safety_replay/analyze.py start_nav_jazzy.log --list

python3 src/pid_tracking_controller/tools/safety_replay/analyze.py \
  log/2a_runtime_20261112_1617/navigation_snapshot.log \
  --at 16:17:25.972 --at 16:19:23.697 --at 16:19:24.700 \
  --model-config src/pid_tracking_controller/tools/safety_replay/example_model.json \
  --replay install/pid_tracking_controller/lib/pid_tracking_controller/pid_motion_replay \
  --output log/safety_replay_result
```

输出目录必须不存在，避免覆盖已有证据。依赖 Python 3.9+、matplotlib；无需 numpy。
`--line 419` 可精确选 PID_GATE 行，支持多次指定。
`--at` 默认 Asia/Shanghai，容差 1 秒；跨日期日志必须提供完整 ISO 日期时间。
不指定案例时，默认选前 6 条“当前候选通过、恢复目标碰撞”记录。
`--window 5` 控制案例附近时间线范围；默认采样步长 `--step .001` 秒。
支持 launch 前缀及标准 ROS logger 前缀；混合来源时用 `--source` 选择 `--list` 显示的名称。
截断或损坏记录会被跳过并列入报告警告，同时清除恢复目标推断，保留其他完整案例。
不提供模型配置时仅生成日志证据及时间线，不猜测参数。
示例配置是 2026-11-12 分析所用假设，不是自动获取的在线参数，分析其他运行前必须核对。
延迟直接取各记录 `effective_delay`，包含位姿相对当前时间的年龄。

## 输出与解释

- `index.html`：浏览器直接打开，每例四张子图及完整字段。
- `line_*.png`：中心运动、平面速度、角速度、状态事件时间线。
- `line_*_candidate.csv` / `line_*_resume.csv`：时间、位置、角度、速度、阶段。
- `report.json`：记录来源、行号、SHA256、模型假设、恢复目标来源、缺失数据、运动指标。
- `source.log`：本次读取的日志字节快照；实时日志后续增长不影响该报告。

运动阶段：0 初始、1 保持、2 制动、3 延迟、4 加减速过渡。
恢复运动按当前 gate 规则：向保留目标加减速，额外延长至能达到目标，然后制动。
速度是车体坐标下的速度；转动时运动方向随车体改变。
曲线末端 X 表示模型停止点，绝不是碰撞点。

可以通过 `--footprint polygon.json` 叠加车体轮廓示意，格式 `[[x,y], ...]`，至少三个点。
必须提供车体坐标系下、包含实际 padding 的轮廓。此输入只用于示意，工具不验证是否与当时一致。

## 证据边界

恢复目标由日志状态变化推断：在可见放行后首次拒绝时记录候选目标，直到放行、版本或 epoch 变化。
日志被节流，推断不构成每周期状态的完整证明；截断、错配或重置会使目标来源未知。
未知时不绘制恢复曲线，不用当前候选冒充历史恢复目标。
`phase` / `hit_t` 只对应当前候选的线上检查结果，不能解释为恢复运动碰撞时刻。
黄框红框是沿路径风险；时间线使用离散日志点，不把记录间空白当成连续确认。

缺少同时刻 local costmap、精确 pose/footprint 和恢复命中细节的旧记录，无法恢复碰撞位置。
因此本版不绘制猜测障碍，也不输出“离线碰撞通过/失败”。后续精确回放需要另行采集同期输入。

## 验证

遵守项目规则，构建和测试前先检查并停止正在运行的 RBS 导航。

```bash
PID_MOTION_REPLAY=/home/CNF2026987648/s100_docker/install/pid_tracking_controller/lib/pid_tracking_controller/pid_motion_replay \
python3 -m unittest discover -s src/pid_tracking_controller/tools/safety_replay -p 'test_*.py'
```
