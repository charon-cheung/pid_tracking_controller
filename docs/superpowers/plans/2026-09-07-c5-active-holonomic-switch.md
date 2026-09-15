# C5 Active Holonomic Switch — Revised Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Follow the user's AGENTS.md: no commits, branches, worktrees, or unsolicited agent delegation. This document authorizes no execution by itself.

**Goal:** 将已有 C1–C4 组件接入真实控制输出，完成可限速、可确认 Rotation、可验证 Terminal 收敛的三自由度全向控制。

**Architecture:** `PidTrackingController` 负责生命周期、时间连续性和组件编排；速度约束算法仍归 `HolonomicSpeedPlanner`，运动公式仍归 `HolonomicControlLaw`，命令投影与变化率限制仍归 `HolonomicCommandLimiter`。复用现有 Reference、ProgressTracker 和 RotationCompletionTracker，C5 不新增生产组件或控制模式开关。

**Tech Stack:** C++17、ROS 2 Jazzy、Nav2 controller plugin、ament_cmake_gtest；仅为受控时间测试显式声明已有 ROS 基础包 `rcl` 的测试依赖。

**Spec:** 用户提供的 Task C 摘要：`/home/CNF2026987648/.codex/attachments/aa4d6077-763b-4efb-832e-3ad85d94271c/pasted-text.txt`。本文件的“修订设计”是自包含的 C5 增补；优先采用本文件明确修订的 C5/C6 分工和 Rotation 分组约定，其余保留原摘要约束。

## 本轮执行修订（2026-09-09）

本轮一次完成 C5 代码，先验证完整 Shadow 接线（真实输出仍为 Legacy），再将同一个计算 helper 切为 0.20 m/s active。最终没有 Shadow 开关、第二份 Tracker 或两套生产计算链。

与原计划相比，仅修订/收缩：

- 百分比限速基数改为 max_xy_speed；两基数不同的 Controller/lifecycle 测试保护此语义。
- 每次有效 setPlan 安装新路径并清空 Rotation settle，但保持 Planner 参考速度和 Limiter 命令历史。首次、无效时间、gap、异常恢复才 reseed。
- 采用紧凑日志；不改造 Legacy 统计，不保留错误的 active 速度统计。
- Rotation run 分组已有 C4 组件测试，加上本轮 Controller 的反向中间 yaw → 最终 yaw → 单次 ack 测试，不为同一语义重复增加 reference 测试文件。
- 代码完成与实际导航验收分开：C5-D 的真实低速导航、下游命令和 odom 归因仍须单独执行；本轮不启动导航、不修改其他包，也不执行 C6。

## Global Constraints

- 基线：`8f5c4cb`（完成 C4）；2026-09-07 规划时工作树干净。此前六个文件的 `+156/-65` 已在基线内，不重复计入 C5。
- 只修改 `/home/CNF2026987648/s100_docker/src/pid_tracking_controller`。
- 不修改 `nav2_common.yaml`、`webot_nav.py`、FinalApproachController 或其 GoalChecker。
- 不实现动态追加路径、内部避障/碰撞检查、Goal identity、等价 Path 保留、reanchor 或 B2.1。
- 第一版为 P + Feedforward，不增加 I、D、MPC、LQR。
- accepted `current.position` 是位置反馈目标；carrot/preview 仅用于前视、约束和可视化，不推进 progress，也不作为位置 P 目标。
- `path_tangent` 决定平移前馈方向，`target_yaw` 独立决定朝向；横向误差不进入 omega；三点 kappa 仅诊断。
- 保留 Nav2 插件公开接口；不增加全局配置层、manager、factory 或仅用于减少行数的类。
- C6 才删除 Legacy 文件、控制函数和旧参数；C5 可移除它们在 active 编排中的调用，保留直接调用的 Legacy 回归测试。
- 编译、测试前检查 RBS 导航进程；仅在确实运行时执行既有 `webots_ros2_demo/kill_ros.sh --rbs`。
- ROS 命令在同一 shell 中先 source `/opt/ros/jazzy/setup.bash`；需要工作区时再 source `install/setup.bash`。
- 仅增量构建本包，复用现有 build/install，Make 默认 `MAKEFLAGS="-j12"`；不建立临时编译工作区。
- 不修改 README，不检查或修改 LICENSE、CONTRIBUTING.md；不清理无关格式。既有旧文件 uncrustify 失败单独说明。
- 当前请求只编写方案，以下复选框都是未来执行项，不表示实现或验证已经完成。

## 修订设计

### 1. C5 与 C6 的新分工

| 内容 | 批次归属 | 原因 |
| --- | --- | --- |
| 实际 FF yaw rate 的速度可行性补充 | C5-A | 直接连接组件前必须消除约束口径差异 |
| 新增全向参数的声明、加载、校验 | C5-B | active 输出需要确定的配置 |
| setSpeedLimit 的 Nav2 入口接入 | C5-B | Planner 和 Limiter 的内部限速必须有真实来源 |
| 三种模式、真实 dt、reset/reseed、实际 ack、失败处理 | C5-B | 一次形成完整可用的 active 控制链 |
| 三轴实际输出与限制原因、ack 事件的必要诊断 | C5-B，C5-C 验证 | C5 低速验收已经依赖这些观测 |
| Legacy 文件、旧控制函数、旧参数删除 | C6 | 必须等待 C5 低速仿真通过 |
| 非验收必需的诊断整理 | C6 | 不为 C5 引入大规模统计/日志重构 |
| 0.4/0.6/0.8 m/s 分级调试与实车标定 | C7 | C5 仅完成初始低速验证 |

C5-B 同时包含 Translation、RotationPending、Terminal、限速和最小诊断。不能先发布只支持 Translation 的 active 版本，再把 Rotation 的真实 ack 留到后续批次。

### 2. 补齐 yaw 前馈可行性，不修改路径朝向含义

当前 Reference 的 `current.target_yaw_rate_per_meter` 来自单个 translation segment；Planner 的 yaw rate 来自空间窗口平均。窗口平均可能弱化短段尖峰，也可能将相反方向的 yaw 变化抵消。

本方案选择保留现有 target_yaw 插值及真实 segment rate，并在 Planner 内补充直接约束：

```text
omega_available = max_omega - yaw_feedback_reserve

当前 FF 速度上限：
  yaw_ff_speed_limit = omega_available / abs(current_segment_yaw_rate)
  rate 接近零时取 +infinity

未来每个 Translation segment：
  v_i = omega_available / abs(segment_yaw_rate_i)
  d_i = max(0, segment.translation_s_begin - current_s)
  propagated_i = sqrt(v_i * v_i + 2 * a_ref_down * d_i)

yaw_speed_limit = min(原窗口约束, 所有 propagated_i)
```

遍历只到首个未完成 Rotation boundary 或 terminal；不把 Rotation 的零长度 yaw 变化当作 translation rate。

保留原窗口算法用于未来 yaw/tangent 趋势约束。新增检查在现有扫描内完成，避免创建第二份路径、第二套插值或新的 rate 组件。

时间斜坡之后，最终 `reference_velocity` 还必须满足当前 `yaw_ff_speed_limit`，与已有 external/body-axis/XY 硬上限一起取 min。这修订了“reference 始终满足减速度斜坡”的绝对表述：新路径或状态跳变导致当前硬约束突然收紧时，reference 可以立即降低；最终命令仍由 Limiter 执行自身速度/变化率规则。

只保证 `abs(omega_ff) <= omega_available`，不承诺任意大 yaw 误差下 FF+P 都不饱和，也不承诺角加速度在 FF 原始量上可行。最终 omega 的硬上限和 slew 仍由 Limiter 保证。

`HolonomicSpeedPlan` 仅新增一个有明确用途的字段：

```cpp
double yaw_ff_speed_limit{std::numeric_limits<double>::infinity()};
```

现有 `max_abs_target_yaw_rate_per_meter` 保持“窗口 rate”的含义；日志用 Reference 的 current rate 表示实际 FF rate。`speed_limit_reason` 仍表示 target 的主导约束，不能把它误称为所有最终截断的完整原因。

### 3. 时间连续性与生命周期

每个控制周期只读取一次 `now`，正常 dt 为本次时间减去上次成功返回命令的时间。用 `rclcpp::Time`/Duration 做差，不继续使用 `1/pid_frequency` 驱动新闭环。

| 事件 | Tracker / Reference | Planner | Limiter | Rotation settle |
| --- | --- | --- | --- | --- |
| 正常周期 | update 一次 | plan(real dt) | 从 previous limited command 推进 | 连续有效观测才累计 |
| activate 后首次周期 | 获取当前 Reference | 按实测速度 seed；本周期不调用 plan | dt=0 的 reseed 分支返回实测速度的硬约束投影 | 清零，本周期不 ack |
| dt<=0 或 dt>gap threshold | 正常获取有效 Reference | 同上 | 同上，不消费虚构/巨大加速度预算 | 清零，不把空档当稳定时间 |
| 合法 setPlan | 每次 reset 并安装新 Path | setPath 清除旧路径约束，保留参考速度历史，下一周期受新约束限制 | 保持 previous limited command | 无条件清零，包含相同末点/相同 segment index |
| 非法 Path 或 active 计算异常 | 本周期拒绝命令 | 下次有效周期重新 seed | 下次有效周期重新 seed | 清零且不 ack |
| deactivate / cleanup | 生命周期停用；cleanup 释放 active 路径/组件 | 下次激活需新 seed | 下次激活需新 seed | 清零 |

正常 setPlan 不制造一次 Limiter reset，也不把新 Path 误当作新的控制时间起点。

Planner 的平移 seed：

```cpp
const double delta = reference.current.path_tangent - robot_yaw;
const double forward_projection = std::max(
  0.0, measured.linear.x * std::cos(delta) +
  measured.linear.y * std::sin(delta));
```

Translation 用该正向投影 seed；RotationPending/Terminal 用 0 seed。Limiter 用实测 body vx/vy/omega，通过 `limit(measured_body, measured_body, 0.0, cap)` 完成 seed 和硬约束投影。

首周期不调用目前拒绝 dt=0 的 `Planner::plan()`，也不构造假的 1/15 s。日志标记 `control_phase=reseed`、`planner_step=false`；没有执行的 planner 结果标记不可用，不能伪装成正常计划值。

### 4. 一次完整的 active 控制调用

正常周期固定顺序：

```text
校验生命周期、Path、pose、measured velocity
  → 读取 now 和一次 external cap 快照
  → Tracker.update 一次
  → 构造有效 Reference
  → 判断是否 reseed；是则返回受硬约束的 seed 命令
  → Planner.plan(real dt, cap)
  → 根据 reference.mode 构造 ControlLaw 输入
  → ControlLaw
  → Limiter.limit(raw, measured, real dt, cap)
  → 校验最终三轴命令
  → Rotation 完成判断及本周期最多一次 ack
  → 保存命令时间，发布 carrot 和诊断，返回 Twist
```

Translation 使用 accepted current 位置、current tangent、current target_yaw/rate 和最终 reference_velocity。移除 active 中的 start-heading alignment、Legacy 输出分支、`max(0,vx)`、最终 `linear.y=0`，允许负 body vx。

RotationPending 保留 `vx_raw=vy_raw=0`，只计算 `kp_yaw * yaw_error`。Limiter 的历史平移命令可以逐步减到零，因此不将“raw 为零”错误解释成“最终输出本周期必须为零”。C5 不增加 Rotation 的 XY hold。

Terminal 使用终点位置及 yaw，reference_velocity=0、omega_ff=0，位置/yaw P 继续收敛。位置已到而 yaw 未到时允许只旋转；“普通 Translation 不先转头”不限制 Terminal 的朝向收敛。

不依赖 Legacy `goal_x_/goal_y_` 缓存决定新 Reference，也不在 C5 重新启用旧 `#if 0` 的到达逻辑。Nav2/现有 GoalChecker 仍拥有任务完成判定。

### 5. Rotation 确认与错误处理

沿用 `8f5c4cb` 的已提交行为：同位置、同 translation arc boundary 上相邻 Rotation 是一组；跟踪整组最终 yaw；pending index 是组首 index；ack 一次跨过整组。

这意味着 `0°→90°→0°` 可直接满足最终 yaw。C5 将这些中间 yaw 视为采样点，不支持必须逐点停留、指定绕转方向或累计多圈动作。不同位置、由 Translation 分隔的 Rotation 仍依次处理。若后续要求中间 yaw 是强制动作，属于重新定义 C4 路径语义的独立变更，不能在 C5 中悄然加入。

settle 仍要求 yaw 误差、实测 omega、实测 XY 合速度同时在各自容差内。进入 Rotation、pending index 变化、setPlan、reseed、上一采样不满足条件时，首个恢复合格的采样以 dt=0 建立稳定区间起点；从之后连续合格的样本开始累计。

可复用现有 RotationCompletionTracker，控制器记录上一 Rotation 样本是否合格，传入可信的累计 dt；不修改组件数学定义或增加新的完成条件。

实际 ack 在全部会拒绝本次输出的算法计算和有限性检查成功之后执行。本周期保持 Rotation 命令，下周期 update 才观察到 ack 后的新模式；禁止为了同周期跳到 Translation 再调用一次 update 或进行无界 ack 循环。

非法 Path 继续报 `nav2_core::InvalidPath`；Reference、Planner、ControlLaw、Limiter、Rotation/ack 的运行时计算失败报 `nav2_core::NoValidControl`，同时清除时间连续性与 settle 状态。不能保留 shadow 的“警告后继续输出 Legacy”路径，不能自动回退到旧控制行为。

### 6. 参数、外部限速和低速配置

configure 内只加载一份共有的速度/加速度数值，再分别填入 PlannerConfig 和 LimiterConfig；不维护同名参数的两套独立默认值。新参数只在 configure 使用，不引入动态参数回调。

当前控制器公开头文件继续用前向声明和 `unique_ptr` 持有私有组件；不让安装后的公开头文件包含 `src/` 内部头文件。

控制器新增/替换的状态包括：

```cpp
class HolonomicSpeedPlanner;
class HolonomicCommandLimiter;

std::unique_ptr<HolonomicSpeedPlanner> holonomic_speed_planner_;
std::unique_ptr<HolonomicCommandLimiter> holonomic_command_limiter_;
double kp_x_{1.0};
double kp_y_{1.0};
double max_xy_speed_{0.2};
double control_gap_threshold_{0.5};
std::atomic<double> external_speed_limit_{
  std::numeric_limits<double>::infinity()};
rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
bool control_time_valid_{false};
bool rotation_last_all_conditions_{false};
```

保留已有 kp_yaw 与 RotationCompletionTracker。将现有 shadow rotation 状态转为 active 状态，避免复制两套状态机。

以下是建议的首轮仿真测试限制，属于人为施加的低速测试包络，不代表已测得的底盘能力，不可据此宣称实车安全上限：

```yaml
kp_x: 1.0
kp_y: 1.0
kp_yaw: 1.0
nominal_tracking_speed: 0.20
max_vx: 0.20
max_vy: 0.20
max_xy_speed: 0.20
max_omega: 0.30
max_ax: 0.20
max_ay: 0.20
max_alpha: 0.50
a_ref_up: 0.20
a_ref_down: 0.20
yaw_feedback_reserve: 0.05
configured_min_rate_window: 0.30
control_gap_threshold: 0.50
rotation_yaw_tolerance: 0.05
rotation_omega_tolerance: 0.05
rotation_xy_speed_tolerance: 0.05
rotation_settle_time: 0.30
```

沿用现有 lookahead/frame 参数。kp_x/kp_y/kp_yaw 的数学合法性仍为有限非负；该仿真验收配置使用正增益确保收敛能力。校验 `a_ref_up/down <= min(max_ax,max_ay)`、`yaw_feedback_reserve < max_omega`，以及所有速度/加速度/时间参数的既有合法范围。

0.2 m/s Gate 对最终 `hypot(vx_command,vy_command)` 生效，不能只降低 nominal，因为位置 P 会叠加速度。

setSpeedLimit 复用已有 `convertExternalSpeedLimitToAbsolute()`：

```cpp
void PidTrackingController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  try {
    const double cap = convertExternalSpeedLimitToAbsolute(
      speed_limit, percentage, max_xy_speed_);
    external_speed_limit_.store(cap, std::memory_order_relaxed);
  } catch (const std::invalid_argument & exception) {
    RCLCPP_WARN(logger_, "PID_LOG rejected_speed_limit reason=%s", exception.what());
  }
}
```

入口 0 表示解除限制（内部 +infinity），不等于急停。百分比 0–100，按 max_xy_speed 换算，而不是 nominal_tracking_speed；测试必须设置这两个参数不同。负数、NaN、infinity、超过 100 的百分比拒绝并保留旧 cap。内部 cap=0 仍是 Limiter/Planner 支持的硬零速度，但不是该 Nav2 callback 的停止指令；不要混用入口语义和内部语义。

每周期只读取一次 cap，Planner 和 Limiter 使用同一快照；降低在第一个读取到新 cap 的输出周期立即满足，增加/解除通过正常斜坡恢复。不 reset Tracker 或 Limiter。不宣称能追回已发送给下游的旧命令。

### 7. 必要诊断和观测边界

按最新审查收缩日志：active 不再调用 Legacy 统计/长日志函数，仅输出一条约 1 Hz 摘要（模式、重规划、reseed、candidate 变化可触发），加一条实际 ack 事件；不新增 topic、类、统计框架或常驻 FF/FB 大字段集。Legacy 诊断函数和其字段保留给 C6 清理，不能作为 active 验收依据。

摘要字段：

```text
plan_revision / mode / planner_step
reference_target / reference_effective / limit_reason / external_limit
raw_vx / raw_vy / raw_omega
limited_vx / limited_vy / limited_omega / translation_lambda
reseed / control_gap / rotation_index / rotation_settled / ack_candidate
```

实际 `PID_LOG rotation_ack` 区别于 candidate。reseed 周期没有执行 Planner：`planner_step=false`、`reference_target=nan`、`limit_reason=not_planned`；`reference_effective=0` 表示该周期没有平移 FF，最终 command 是实测速度的硬限幅投影。普通周期的 `limit_reason` 指 Planner target 的主导限制，不表示 Limiter 全部截断原因。

FF/FB 解耦、同一 effective reference 驱动 omega_ff、连续性和 XY 合速度包络由行为测试保护。active 不输出 Legacy 的“vx 减实测合速度”统计，也不为 C5 新建统计体系。

必要诊断变更控制在新增日志语句 18 条以内，目标不超过 5 条新增事件日志。以约 50–80 行诊断部分 diff 为目标；若需要超过用户诊断修改预算，执行时先说明具体原因和超出范围，不能借完整 C5 规模掩盖日志扩张。

当前下游配置允许 vy，但还存在 velocity_smoother；PID 日志能证明插件返回值，不能单独证明底盘收到/执行的命令。仿真验收需要结合下游命令与 odom。当前 smoother 的逐轴限制可能在瞬态改变方向，不把它误判为 ControlLaw 耦合。

## 文件职责与预计规模

以下是最终实现相对 `8f5c4cb` 的估算；“改动行数”指新增+删除，不是净增。文档本身不计入生产代码规模。

| 文件/组 | 职责 | 预计改动行数 |
| --- | --- | ---: |
| `src/pid_tracking_controller.cpp` | active 编排、时间、参数、限速、ack、必要诊断 | 350–600 |
| `include/pid_tracking_controller/pid_tracking_controller.hpp` | 组件所有权和连续性状态 | 35–65 |
| `src/holonomic_speed_planner.cpp` | 当前和未来真实 FF rate 的约束 | 50–90 |
| `src/holonomic_speed_planner.hpp` | 当前 FF cap 结果字段 | 3–8 |
| `param/jazzy/pid_tracking_controller.yaml` | 低速仿真参数及用途说明 | 20–35 |
| `test/test_path_and_control.cpp` | active Twist、时间、模式、异常、限速契约 | 220–350 |
| `test/test_lifecycle_smoke.cpp` | 生命周期、有效配置、真实日志与发布链 | 160–270 |
| `test/test_holonomic_speed_planner.cpp` | 短段、相反 yaw rate 和未来制动回归 | 65–110 |
| `test/test_holonomic_tracking_reference.cpp` | 明确整组最终 yaw 的边界用例 | 15–35 |
| `test/controller_test_access.hpp` | 复用现有 friend fixture，初始化新组件 | 20–40 |
| `CMakeLists.txt`、`package.xml` | 测试包含路径和 rcl 测试依赖 | 8–20 |
| **合计** | **预计 12 个代码/配置/测试文件** | **约 950–1,700** |

中心估计约 1,200–1,400 行 diff；生产代码/配置约 460–800 行，余量主要来自替换旧 active 行为断言及新状态边界测试。现有 C1/C3/C4 算法主体不计划重写。

以下是原规划估计，实际规模见文末执行记录。控制器 cpp 基线 1,252 行；替换旧 active 分支和 shadow 编排后，预计净增约 50–250 行，最终约 1,300–1,500 行。它仍只负责编排；不为行数目标拆类。若实际预计突破约 1,600 行，应先复核是否重复保留了 active/shadow 实现或扩张了诊断职责。

## C5-A：补齐 Planner / ControlLaw 的 rate 契约

**Files:** 修改 `src/holonomic_speed_planner.cpp`、`.hpp`、`test/test_holonomic_speed_planner.cpp`。

**Interfaces:** 消费既有 `HolonomicTrackingReference::current.target_yaw_rate_per_meter`、PreparedPath translation segments 和 PlannerConfig；保留 `HolonomicSpeedPlanner::plan(reference, robot_yaw, dt, external_speed_limit)` 签名，产出新 `HolonomicSpeedPlan::yaw_ff_speed_limit` 及受其约束的 reference_velocity。

- [x] **A1：增加一个代表性短段回归及同组边界数据。** 使用现有 makePath/makeReference/makeConfig，核心用例完整如下：

```cpp
TEST(HolonomicSpeedPlanner, currentShortSegmentKeepsActualYawFeedforwardFeasible)
{
  const auto path = preparePath(makePath({
    {0.0, 0.0, 0.0}, {0.01, 0.0, 0.1},
    {1.01, 0.0, 0.1}, {2.01, 0.0, 0.1}}));
  auto config = makeConfig();
  config.nominal_tracking_speed = 0.2;
  config.max_omega = 0.3;
  config.yaw_feedback_reserve = 0.05;
  config.max_ax = config.max_ay = 0.2;
  config.a_ref_up = config.a_ref_down = 0.2;
  config.configured_min_rate_window = 0.3;
  HolonomicSpeedPlanner planner(config);
  planner.setPath(path);
  planner.seed(0.2);
  const auto reference = makeReference(path, makePoint(0.0, 0.0));
  const auto result = planner.plan(reference, 0.0, 0.1);
  EXPECT_LE(
    std::abs(reference.current.target_yaw_rate_per_meter * result.reference_velocity),
    config.max_omega - config.yaw_feedback_reserve + 1e-12);
}
```

同组输入还包括：短段在路径未来 0.1 m 处；相邻短段 yaw 先加 0.1 再减 0.1；跨 ±pi 的最短角；Rotation 后的 yaw 变化不跨边界参与制动。对未来短段检查 `reference_velocity_target <= sqrt(v_i*v_i+2*a_ref_down*d_i)`；断言数值语义，不锁定候选遍历顺序。

- [x] **A2：在构建预算内运行一次有意义的 Red。** 上述代表性用例预期因 FF 超预算失败。若失败来自环境或无关 fixture，先静态核对，不重复制造 Red。
- [x] **A3：将“修订设计 2”公式加入现有约束扫描，新增当前 FF cap 字段和斜坡后的硬约束。** 保持 `HolonomicControlLaw`、Reference 的 target_yaw 和 Rate 插值不变；Rotation/Terminal 仍立即清零 reference。
- [x] **A4：增量构建并运行 speed planner、control law 和 tracking reference 测试。** 检查原 temporal-slew 测试只在硬约束未收紧的条件下要求斜坡；新增当前 FF cap 突降场景保护明确的优先级。

**Gate A:** 上述 rate 不变量通过，Legacy 仍为真实输出；本批次可独立审查。

## C5-B：完整 active 控制链切换

**Files:** 修改 controller hpp/cpp、Jazzy YAML、`test/controller_test_access.hpp`、`test/test_path_and_control.cpp`、`test/test_lifecycle_smoke.cpp`、CMakeLists、package.xml。

**Interfaces:** 复用 `PathProgressTracker::update()`、`buildHolonomicTrackingReference()`、`HolonomicSpeedPlanner::setPath/seed/plan()`、`computeHolonomicRawCommand()`、`HolonomicCommandLimiter::limit()`、`RotationCompletionTracker::update/reset()`、`acknowledgeRotationComplete()`。新增状态采用上文名称；既有公开插件接口不变。

- [x] **B1：准备可控时间和新组件 fixture，替换与 C5 明确冲突的旧 active 断言。** 在测试侧使用 ROS clock override，不新增生产时钟接口，也不靠 sleep 制造 dt：

```cpp
auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
ASSERT_EQ(
  rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL), RCL_RET_OK);
PidTrackingControllerTestAccess::setClock(controller_, clock);
const auto seed_command = controller_.computeVelocityCommands(
  makeRobotPoseAt(0.0, 0.1), geometry_msgs::msg::Twist(), nullptr);
EXPECT_DOUBLE_EQ(seed_command.twist.linear.x, 0.0);
EXPECT_DOUBLE_EQ(seed_command.twist.linear.y, 0.0);
ASSERT_EQ(
  rcl_set_ros_time_override(clock->get_clock_handle(), 1100000000LL), RCL_RET_OK);
const auto tracking_command = controller_.computeVelocityCommands(
  makeRobotPoseAt(0.0, 0.1), geometry_msgs::msg::Twist(), nullptr);
EXPECT_GT(tracking_command.twist.linear.x, 0.0);
EXPECT_LT(tracking_command.twist.linear.y, 0.0);
EXPECT_NEAR(tracking_command.twist.angular.z, 0.0, 1e-12);
EXPECT_LE(
  std::hypot(tracking_command.twist.linear.x, tracking_command.twist.linear.y),
  0.2 + 1e-12);
```

fixture 在此之前通过已有 friend 初始化 Planner/Limiter/RotationCompletionTracker，并安装长直线路径；三个组件配置与上文仿真限制一致。保持 Legacy 直接函数 fixture 可用，不将新配置注入 Legacy 回归。

在 `BUILD_TESTING` 内增加 `find_package(rcl REQUIRED)`，给直接使用 override API 的测试 target 加 rcl；package.xml 增加 `<test_depend>rcl</test_depend>`。仅给使用内部组件的测试 target 增加 `${CMAKE_CURRENT_SOURCE_DIR}/src` include path。不要给生产 target 添加新的外部库。

- [x] **B2：在 configure 内加载并校验全向配置，创建 Planner/Limiter，接入 setSpeedLimit。** 使用上文完整 callback 和单 cap 快照规则；运行期间配置不可变。Legacy 参数暂保留，但不得再决定 active 控制能力或使不使用的 Legacy 模型校验阻止全向启动。
- [x] **B3：按状态表改造 activate/deactivate/cleanup/setPlan。** 合法 setPlan 给 Tracker 和 Planner 安装同一 PreparedPath；Limiter 状态连续。非法 setPlan 保留既有“拒绝后不能继续使用旧计划”的契约。`resetRotationShadow()` 改为 active completion reset，不保留第二套 shadow timer。
- [x] **B4：按固定顺序替换 computeVelocityCommands。** 使用下列完整的 ControlLaw 输入规则：

```cpp
HolonomicControlInput input;
input.robot_x = pose.pose.position.x;
input.robot_y = pose.pose.position.y;
input.robot_yaw = robot_yaw;
input.reference_x = reference.current.position.x;
input.reference_y = reference.current.position.y;
input.path_tangent = reference.current.path_tangent;
input.target_yaw = reference.current.target_yaw;
input.target_yaw_rate_per_meter = reference.current.target_yaw_rate_per_meter;
input.reference_velocity = speed_plan.reference_velocity;
HolonomicControlGains gains{kp_x_, kp_y_, kp_yaw_};
if (reference.mode == HolonomicReferenceMode::kRotationPending) {
  input.reference_x = input.robot_x;
  input.reference_y = input.robot_y;
  input.target_yaw = reference.rotation_target_yaw;
  input.target_yaw_rate_per_meter = 0.0;
  input.reference_velocity = 0.0;
  gains.kp_x = gains.kp_y = 0.0;
} else if (reference.mode == HolonomicReferenceMode::kTerminal) {
  input.target_yaw_rate_per_meter = 0.0;
  input.reference_velocity = 0.0;
}
const auto raw = computeHolonomicRawCommand(input, gains);
const HolonomicBodyCommand raw_body{raw.vx, raw.vy, raw.omega};
const HolonomicBodyCommand measured_body{
  velocity.linear.x, velocity.linear.y, velocity.angular.z};
const auto limited = holonomic_command_limiter_->limit(
  raw_body, measured_body, dt, external_cap);
cmd_vel.twist.linear.x = limited.command.vx;
cmd_vel.twist.linear.y = limited.command.vy;
cmd_vel.twist.angular.z = limited.command.omega;
```

上面的 `reference` 是本周期有效 Reference，`speed_plan` 是正常周期 Planner 结果，`dt/external_cap` 是统一周期快照；reseed 分支提前处理，不进入此段。body frame_id 沿用现有有效参数，stamp 使用周期开始的 now。

- [x] **B5：接入真实 completion/ack 及异常转换。** 保留三项实测条件，使用上一合格样本标志决定 settle dt；只有有效正常周期才可能 ack。发生算法异常时设置 `control_time_valid_=false`、重置 completion，再抛 Nav2 控制异常；不返回未完成计算的 Twist。
- [x] **B6：接入紧凑 active 诊断。** 按修订设计 7 记录真实全向 command、Planner 结果、reseed 和实际 ack；停止调用旧 Legacy 统计，不伪造 LegacyPlan，不扩张为新统计系统。
- [x] **B7：增量构建并运行本批受影响测试。** 至少覆盖 lateral-offset、yaw 与 tangent 相差 90°、允许负 body vx、Rotation 后进入下一 Translation、单点 Terminal 位置/yaw P、外部限速降低、首次 seed 和一个异常 gap。

**Gate B:** active 输出三轴齐备，三模式均有基本命令级验证；必要诊断可观察真实输出，旧 Legacy 直接函数测试仍可运行。此阶段表示代码接入通过，尚不代表低速仿真验收完成。

## C5-C：状态边界、语义和观测回归

**Files:** 主要修改 `test/test_path_and_control.cpp`、`test/test_lifecycle_smoke.cpp`；复用已有 `test/test_holonomic_tracking_reference.cpp`，不重复添加组件测试。只在新证据发现缺陷时做针对性的生产修正，不顺带重写 C1–C4。

**Interfaces:** 通过公开 configure/activate/setPlan/compute/setSpeedLimit/deactivate/cleanup 和实际 Twist/模式后继行为检查 B 的交付；受控时钟只存在于测试侧。

- [x] **C1：批量补齐以下可观察状态边界。**

| 场景 | 输入/操作 | 断言 |
| --- | --- | --- |
| 首次/重复时间/回退/长 gap | 1.0→1.1→1.1→1.0→3.0 s | 异常周期只 reseed，不突增 slew 预算、不 ack；下一正常周期恢复 |
| 正常控制连续性 | measured 维持不变，连续 0.1 s 调用 | command 按 previous command 推进，不每次退回 odom |
| setPlan 连续性 | 运动中安装同/不同 Path | Tracker 按新 Path reset，Limiter 不因 setPlan 突跳或反复启动 |
| 重规划时 Rotation index 相同 | 已累计 settle 后 setPlan | 新 Path 不继承旧 settle，不立即 ack |
| settle 中断 | 合格→yaw 或任一实测速度不合格→合格 | 清零，恢复后的首个合格采样不累计之前间隔 |
| 外部限速降低 | 已输出 0.2，设置绝对 0.08 | 下一快照周期合速度<=0.08，即使违反普通减速连续性 |
| 限速提高/解除 | 0.08→0.15；入口 speed_limit=0 | 限幅恢复受 Planner/Limiter 约束；0 不作急停 |
| 非法外部限速 | -1、NaN、infinity、percentage=101 | 保持之前 cap，记录拒绝，不破坏导航 callback |
| 非有限输入与异常恢复 | pose/velocity 无效；或经现有 friend 构造缺失 active 组件 | 抛 NoValidControl，无实际 ack；恢复后 reseed |
| Rotation 三轴输入 | raw 平移为零，但前一命令仍运动 | final 平移按 slew 收敛；实测未停稳不能 ack |
| Terminal | 单点路径有 XY/yaw 偏差；终端 Rotation 完成 | FF=0，P 使误差收敛；不因位置已到丢失 yaw 控制 |
| preview 无副作用 | stationary pose 下改变 lookahead | accepted current 不因 preview 前进，不能拿 carrot 做 XY P |

用时间覆盖执行相同命令序列，命令 slew 断言采用实际 0.1 s 预算和数值容差；不测试仅为当前实现服务的私有布尔默认值。

- [x] **C2：验证 Rotation 分组语义。** 本轮复用既有 reference/ack 分组测试，并在 Controller 层覆盖中间 yaw 反向后仍只跟踪最终 yaw 和一次 ack。下列原计划示例为语义说明，不要求重复增加到 reference 测试：

```cpp
const auto path = preparePath(makePath({
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.5 * kPi}, {0.0, 0.0, 0.0}}));
PathProgressTracker tracker;
tracker.setPath(path);
const auto progress = tracker.update(makePoint(0.0, 0.0), 0.2);
const auto reference = buildHolonomicTrackingReference(*path, progress, 0.2);
ASSERT_EQ(reference.mode, HolonomicReferenceMode::kRotationPending);
EXPECT_NEAR(reference.rotation_target_yaw, 0.0, 1e-12);
ASSERT_TRUE(reference.pending_rotation_segment_index.has_value());
tracker.acknowledgeRotationComplete(*reference.pending_rotation_segment_index);
const auto terminal = buildHolonomicTrackingReference(
  *path, tracker.update(makePoint(0.0, 0.0), 0.2), 0.2);
EXPECT_EQ(terminal.mode, HolonomicReferenceMode::kTerminal);
```

由 Translation 分隔的 Rotation 按现有测试依次执行；不要求每个离散采样 yaw 都单独停留。

- [x] **C3：验证真实配置、生命周期、日志及 carrot 发布。** configure 无效参数应失败；纠正后可正常 configure；deactivate/activate 后首周期 reseed；cleanup 后无旧 active 路径输出；全向命令使用 XY 合速度限制；日志中的 command 三轴等于返回 Twist；实际 ack 后下一周期模式推进，而不只查 candidate 文本。
- [x] **C4：核对验收口径。** 路径切线方向不变指世界系平移 FF；允许位置 FB 改变合成方向，允许 Limiter 在历史速度约束下暂时偏离 target。共同 lambda 沿 previous→target 连线推进，不保证 final vector 始终与 target 同方向。大 lateral error 不得直接进入 omega 公式；同时存在 yaw 误差或 rate 需求时，不要求 omega 永不饱和。
- [x] **C5：增量构建并运行功能回归一次。** 复用已有测试 targets；不为此新增测试框架、独立 benchmark 或全工作区矩阵。

**Gate C:** active 状态边界与错误恢复通过，日志含义与 Twist 一致；可以进入受控低速仿真。

## C5-D：0.2 m/s 低速仿真验收

**Files:** 默认不新增或修改生产代码。若测量暴露实现缺陷，记录触发条件后回到所属批次做小范围修复；若需要调整测试包络，只改本包 YAML 并重新记录有效参数。

**Interfaces:** 当前插件返回 Twist、导航日志 `/home/CNF2026987648/s100_docker/start_nav_jazzy.log`、实际下游命令和 odom；不修改 FinalApproach 切换逻辑。

- [ ] **D1：确认运行的是新库和新参数。** 配置日志应显示 holonomic、三轴限制和 0.2 合速度 cap；定位实际 cmd_vel→smoother→底盘链路。读取现有配置只能作为前置核对，最终以该次运行有效配置为准。
- [ ] **D2：按顺序运行以下仿真场景。** 初始静止，路径足够长，避免把首周期 reseed 当成跟踪结果：

| 场景 | 要观察的结果 |
| --- | --- |
| 长直线，无横向/yaw 偏差 | 平稳起步，最终合速度不超过 0.2，omega 近零 |
| 长直线，初始横向偏差 0.1–0.2 m | vy 的方向消除横向误差；omega 不随横向误差符号左右翻转 |
| tangent=0，robot yaw=target yaw=pi/2 | 通过 body 横向运动沿世界 x 前进，不先对齐 tangent |
| tangent 恒定，target yaw 连续变化 | 世界系 FF 方向稳定，独立转动跟踪 yaw，FF cap 生效 |
| Translation→Rotation→Translation | 提前减速、实测停稳后累计 settle、一次实际 ack、恢复平移 |
| 末端 Rotation 或单点 Terminal | 插件自身的 Terminal 反馈方向和减速行为正确；任务完成另行归因 |

速度检查区分插件 command、下游 command 和 odom；实测速度可能有底盘动态响应，不能把命令限制等同于测量严格上限。

- [ ] **D3：对 Terminal 做独立归因。** 当前 `FinalApproachGoalChecker` 要求 policy_state=`final_approach`、controller_status=`settled` 及位置/yaw/速度容差。C5 用直接命令/集成测试证明自身 Terminal 行为；整次导航成功只算系统证据。若真实运行先切到 FinalApproach，则记录 C5 Terminal 未被该运行覆盖，不把 FinalApproach 的收敛算给 C5。
- [ ] **D4：记录验收结果。** 记录场景、有效参数、对应日志区间、是否实际经过 C5、三轴 command/actual 与 ack 模式转换。常规日志无法证明每个 15 Hz 周期都符合 slew，逐周期硬约束由测试保护；需要解释瞬态时才做短时受授权采样，不长期增加高频日志。

**Gate D:** 低速场景有可归因证据且没有未解释的控制失败。只有到这里才能标记“C5 验收完成”；如果仅 A–C 完成，应报告“C5 代码/测试完成，低速仿真待验收”，不能据此开始 C6 删除 Legacy。

## 构建、测试与最终检查

目标 3–5 次显式增量 build，架构级正常上限 7 次。允许为真实编译/API/语义问题追加并说明原因；不因日志目录/DDS 权限失败重新 build。

每次首次准备编译或测试前只读检查进程：

```bash
ps -eo pid,ppid,args | rg '[n]av2_controller|[n]av2_planner|[n]av2_container|[s]tart_nav_jazzy|[w]ebot_nav.py'
```

确认确有 RBS 导航运行才执行：

```bash
cd /home/CNF2026987648/s100_docker/src/webots_ros2_demo
./kill_ros.sh --rbs
```

增量 build（复用当前 `BUILD_TESTING=ON` 和构建类型，不改缓存）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/CNF2026987648/s100_docker
MAKEFLAGS="-j12" colcon build --packages-select pid_tracking_controller
```

A 的最小测试组：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/CNF2026987648/s100_docker
source install/setup.bash
ROS_LOG_DIR=/tmp ctest --test-dir build/pid_tracking_controller \
  -R '^test_holonomic_(speed_planner|control_law|tracking_reference)$' \
  --output-on-failure
```

B 的命令/生命周期组：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/CNF2026987648/s100_docker
source install/setup.bash
ROS_LOG_DIR=/tmp ctest --test-dir build/pid_tracking_controller \
  -R '^test_(path_and_control|lifecycle_smoke|legacy_behavior)$' \
  --output-on-failure
```

C 的功能回归（仅本包既有 gtest，不重跑无关工作区）：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/CNF2026987648/s100_docker
source install/setup.bash
ROS_LOG_DIR=/tmp ctest --test-dir build/pid_tracking_controller \
  -R '^test_' --output-on-failure
```

如遇 ROS 本地 IPC/DDS sandbox 拒绝，在不 rebuild 的前提下申请最小本地执行权限重跑对应现有测试。测试日志目录失败换用 `mktemp -d /tmp/pid-c5-ros.XXXXXX` 产生的目录；不得把环境失败报告成控制行为 Red。

代码最终检查：

```bash
cd /home/CNF2026987648/s100_docker/src/pid_tracking_controller
git diff --check -- . ':!LICENSE' ':!CONTRIBUTING.md'
git diff --stat -- . ':!LICENSE' ':!CONTRIBUTING.md'
git diff -- . ':!LICENSE' ':!CONTRIBUTING.md'
wc -l src/pid_tracking_controller.cpp include/pid_tracking_controller/pid_tracking_controller.hpp
```

审查不出现新的 profiling/benchmark、动态配置、临时生产接口、Legacy 自动回退或未授权跨包变更。报告新增/删除行数、主要文件净增长、构建次数、测试结果与仿真覆盖范围，不以编译成功代替行为正确。

## 方案覆盖自检

| 本次审查问题 | 处理位置 |
| --- | --- |
| 窗口 yaw rate 与实际 FF rate 不一致 | 设计 2、C5-A |
| dt、seed、gap 与 settle 不统一 | 设计 3/5、C5-B/C |
| 外部限速入口与低速合速度限制缺失 | 设计 6、C5-B/C/D |
| 必要诊断拖到 C6、XY 速度统计错误 | 设计 1/7、C5-B/C |
| shadow 异常策略不能用于 active | 设计 4/5、C5-B/C |
| 连续 Rotation 与旧摘要语义不一致 | 设计 5、C5-C2 |
| 首周期/raw/final/Terminal/导航成功验收混淆 | 设计 3/4/7、C5-C4/D3 |

方案审阅完成后按 C5-A → C5-B → C5-C → C5-D 执行。各 Gate 单独报告；执行 C5 的授权不自动包含 C6/C7。


## 执行结果（2026-09-09）

状态：**C5 代码修改和功能测试已完成；C5-D 真实低速导航验收待执行。没有进入 C6。**

- 先运行完整链 Shadow 接线测试：施加仅 Shadow 的 0.001 m/s cap 后，真实 Legacy Twist 与对照一致、vy=0；随后复用同一 helper 切 active，删除临时 Shadow 调用和过渡期测试。最终运行期无 Shadow/Legacy fallback。
- 5 次本包增量构建调用：第 1 次构建代表性 Red 测试；第 2 次发现新 helper 调用旧非 const yaw 接口的编译错误；第 3 次修正接口使用并验证 Shadow；第 4 次验证 active；第 5 次完成闭环/溢出回归且无编译告警。不执行 clean build，不重编译其他包。
- Red 证据：原 Planner 在短段产生 2.0 rad/s FF，超过 0.25 rad/s 可用预算；原 Controller 横向误差返回 vy=0、omega=-0.1。修改后对应回归通过。
- 最终 12 个功能测试目标的 XML 汇总：**130 个用例，0 failures、0 errors**。包含 22 个路径/Controller 测试、3 个 ROS 生命周期测试、现有各组件和 Legacy 直接函数回归。独立只读代码审查未发现需修正问题。
- 新增理想运动学闭环：从 x=-0.1、y=0.2、yaw=pi/2 开始，经过 2 m 平移及末端旋转，位置/yaw/速度收敛；全程检查 0.20 m/s XY、0.30 rad/s 及各轴变化率包络。它不替代真实底盘、smoother 和 odom 验收。
- ROS 生命周期清理存在未定位的间歇性停顿：一次直接整组运行需 SIGINT 解除，一次 colcon 批量运行在 45 s 超时退出。没有把这两次运行记为正常通过，也未改动包外 ROS 代码。相同最终二进制的单项及完整三项重跑通过（最终三项 119 ms），最新 XML 已由该正常运行更新。该停顿仍作为测试稳定性风险记录。
- 仅运行功能目标，不重跑既有 uncrustify 风格检查；工作区旧 lint XML 仍存在，不能把未经筛选的历史 test-result 汇总当作本轮功能测试结论。
- `git diff --check` 通过。11 个代码/配置/测试文件：**+757 / -796**（约 1,553 行 diff）。预先存在且未跟踪的本计划文档另行更新，不把其全部文本当作本轮新增代码。
- Controller cpp：**1,252 → 1,147 行（-105）**；公开 hpp：**219 → 237 行（+18）**。没有新增生产类，没有扩大到超大源文件，无其他模块修改、README 修改、提交或分支操作。

后续实际导航应检查 `PID_LOG holonomic_parameters` 的 0.20 m/s 包络及 `PID_LOG holonomic` 摘要，结合下游命令和 odom。本轮没有生成新的导航测试日志，不能用此前 Legacy/Shadow 日志声称 active 实车或仿真验收已通过。
