#include <limits>
#include "gtest/gtest.h"
#include "motion_transition_observer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace pid_tracking_controller
{
namespace
{
using M = MotionObservation;
using namespace std::chrono_literals;
constexpr int64_t anchor = 10000000000LL;
class TransitionObserverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    motion.activate();
    for (int i = 0; i < 4; ++i) {record(static_cast<M::Stage>(i), anchor, i == 0 ? -.5 : .8);}
    history = motion.history();
    geometry.snapshot.emplace(); geometry.reason = "ready";
    auto & g = *geometry.snapshot;
    g.header.stamp.sec = 10; g.header.frame_id = "map"; g.pose = {0., 0., 0.};
    g.map = std::make_unique<nav2_costmap_2d::Costmap2D>(200, 200, .02, -2., -2., 0);
    for (auto xy : {std::pair<double, double>{-.02, -.02}, {.02, -.02}, {.02, .02}, {-.02, .02}}) {
      geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second; g.footprint.push_back(p);
    }
    evidence.epoch = history.epoch; evidence.revision = 7; evidence.candidate_count = 1;
    evidence.pose_stamp_ns = anchor; evidence.queue_complete = true;
    evidence.model.candidate_execution = {0., 0.}; evidence.model.candidate_duration = .5;
    evidence.model.acceleration = evidence.model.deceleration = evidence.model.braking_deceleration = {.8, .8, .8};
  }
  void record(M::Stage stage, int64_t stamp, double vx, const std::string & frame = "base", uint64_t revision = 7)
  {
    std_msgs::msg::Header h; h.stamp.sec = stamp / 1000000000LL; h.stamp.nanosec = stamp % 1000000000LL;
    geometry_msgs::msg::Twist v; v.linear.x = vx;
    motion.record(stage, h, v, frame, stage == M::Candidate ? revision : 0);
  }
  TransitionObservation evaluate(const TransitionExecutionEvidence * execution, int64_t pose = anchor)
  {return observer.evaluate(geometry, history, pose, anchor + 100000000LL, M::Wall::now(), execution);}
  M motion{"base"};
  M::History history;
  SafetyInputResult geometry;
  TransitionExecutionEvidence evidence;
  MotionTransitionObserver observer{ExecutionSafetyOptions{.025, 512, 500000, 1.}};
};

TEST_F(TransitionObserverTest, TopicsAloneNeverBecomeExecutionEvidence)
{
  const auto r = evaluate(nullptr);
  EXPECT_STREQ(r.sweep.reason, "execution_evidence_missing");
  EXPECT_EQ(r.sweep.status, SweepStatus::kUnavailable);
  EXPECT_EQ(r.geometry_reason, "ready"); EXPECT_EQ(r.motion_reason, "ready");
  EXPECT_FALSE(r.evaluated); EXPECT_FALSE(r.execution_present);
}

TEST_F(TransitionObserverTest, UsesAlignedMeasuredAndCandidateVelocitiesForConditionalSweep)
{
  // Model defaults are zero; observed odometry/candidate must replace them.
  const auto clear = evaluate(&evidence);
  ASSERT_EQ(clear.sweep.status, SweepStatus::kClear) << clear.sweep.reason << clear.motion_reason;
  EXPECT_TRUE(clear.evaluated); EXPECT_NEAR(clear.sweep.pose.x, .4, 1e-8);
  EXPECT_NEAR(clear.sweep.time, 1., 1e-8);
  unsigned int x, y; ASSERT_TRUE(geometry.snapshot->map->worldToMap(.25, 0., x, y));
  geometry.snapshot->map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto collision = evaluate(&evidence);
  EXPECT_EQ(collision.sweep.status, SweepStatus::kCollision);
  EXPECT_TRUE(collision.sweep.collision_cell); EXPECT_GT(collision.sweep.time, 0.);
}

TEST_F(TransitionObserverTest, NominalPredictionIncludesDelayAndBrakingWithoutCertifyingExecution)
{
  const NominalTransitionAssumptions nominal{.2, .5, {.8, .8, .8}, {.8, .8, .8}, {.8, .8, .8}};
  const auto predict = [&]() {
      return observer.evaluate(geometry, history, anchor, anchor + 100000000LL,
        M::Wall::now(), nullptr, &nominal);
    };
  auto r = predict();
  ASSERT_TRUE(r.nominal_sweep);
  EXPECT_EQ(r.nominal_sweep->status, SweepStatus::kClear);
  // .8*.2 holding + mean(.8,.4)*.5 ramp + .4^2/(2*.8) braking.
  EXPECT_NEAR(r.nominal_sweep->pose.x, .56, 1e-8);
  EXPECT_NEAR(r.nominal_sweep->time, 1.2, 1e-8);
  EXPECT_EQ(r.sweep.status, SweepStatus::kUnavailable);
  EXPECT_STREQ(r.sweep.reason, "execution_evidence_missing");
  EXPECT_FALSE(r.execution_present); EXPECT_FALSE(r.queue_complete); EXPECT_FALSE(r.evaluated);
  unsigned int x, y; ASSERT_TRUE(geometry.snapshot->map->worldToMap(.54, 0., x, y));
  geometry.snapshot->map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  r = predict();
  ASSERT_TRUE(r.nominal_sweep);
  EXPECT_EQ(r.nominal_sweep->status, SweepStatus::kCollision);
  EXPECT_EQ(r.nominal_sweep->phase, SweepPhase::kBraking);
  EXPECT_EQ(r.sweep.status, SweepStatus::kUnavailable);
}

TEST_F(TransitionObserverTest, NominalPredictionCannotBypassInputOrExecutionEvidenceFailures)
{
  const NominalTransitionAssumptions nominal{.2, .5, {.8, .8, .8}, {.8, .8, .8}, {.8, .8, .8}};
  const auto predict = [&](const TransitionExecutionEvidence * e = nullptr) {
      return observer.evaluate(geometry, history, anchor, anchor + 100000000LL,
        M::Wall::now(), e, &nominal);
    };
  auto complete = history;
  history.stages[M::Published] = {};
  EXPECT_FALSE(predict().nominal_sweep);
  history = complete; history.last_drop = M::Wall::now();
  EXPECT_FALSE(predict().nominal_sweep);
  history = complete;
  evidence.queue_complete = false;
  EXPECT_FALSE(predict(&evidence).nominal_sweep);
  evidence.queue_complete = true; ++evidence.candidate_count;
  EXPECT_FALSE(predict(&evidence).nominal_sweep);
  record(M::Candidate, anchor, -.4); history = motion.history();
  EXPECT_EQ(predict().motion_reason, "anchor_ambiguous");
  EXPECT_FALSE(predict().nominal_sweep);
  history = complete; geometry.snapshot.reset(); geometry.reason = "map_missing_or_stale";
  EXPECT_FALSE(predict().nominal_sweep);
}

TEST_F(TransitionObserverTest, NominalInvalidResponseUnknownSpaceAndBudgetCannotReportClear)
{
  NominalTransitionAssumptions nominal{.2, .5, {.8, .8, .8}, {.8, .8, .8}, {.8, .8, .8}};
  const auto predict = [&](M::Wall::time_point start) {
      return observer.evaluate(geometry, history, anchor, anchor + 100000000LL,
        start, nullptr, &nominal);
    };
  nominal.braking_deceleration.vx = 0.;
  auto r = predict(M::Wall::now());
  ASSERT_TRUE(r.nominal_sweep);
  EXPECT_EQ(r.nominal_sweep->status, SweepStatus::kUnavailable);
  nominal.braking_deceleration.vx = .8;
  geometry.snapshot->map->setCost(100, 100, nav2_costmap_2d::NO_INFORMATION);
  r = predict(M::Wall::now()); ASSERT_TRUE(r.nominal_sweep);
  EXPECT_NE(r.nominal_sweep->status, SweepStatus::kClear);
  r = predict(M::Wall::now() - 2s);
  EXPECT_FALSE(r.nominal_sweep);
  EXPECT_STREQ(r.sweep.reason, "observer_budget_exceeded");
}

TEST_F(TransitionObserverTest, MissingOrInvalidGeometryAndEachMotionStreamFailClosed)
{
  geometry.snapshot.reset(); geometry.reason = "map_missing_or_stale";
  auto r = evaluate(&evidence);
  EXPECT_STREQ(r.sweep.reason, "geometry_unavailable"); EXPECT_FALSE(r.evaluated);
  EXPECT_EQ(r.geometry_reason, "map_missing_or_stale");
  for (size_t i = 0; i < 4; ++i) {
    const auto saved = history.stages[i]; history.stages[i] = {};
    r = evaluate(&evidence); EXPECT_NE(r.motion_reason.find("missing"), std::string::npos);
    EXPECT_EQ(r.sweep.status, SweepStatus::kUnavailable); history.stages[i] = saved;
  }
  geometry.reason = "ready";
  EXPECT_EQ(evaluate(&evidence).geometry_reason, "geometry_missing");
}

TEST_F(TransitionObserverTest, RequiresExactOdometryAnchorAndUnambiguousSamples)
{
  EXPECT_EQ(evaluate(&evidence, anchor + 1).motion_reason, "odometry_anchor_missing");
  EXPECT_EQ(evaluate(&evidence, anchor - 1).motion_reason, "candidate_anchor_missing");
  record(M::Odometry, anchor, .9); history = motion.history();
  const auto ambiguous = evaluate(&evidence);
  EXPECT_EQ(ambiguous.motion_reason, "anchor_ambiguous"); EXPECT_FALSE(ambiguous.evaluated);
}

TEST_F(TransitionObserverTest, FutureOdometryDoesNotReplaceOrInvalidateExactPastAnchor)
{
  // /odom arrives one /clock tick ahead. Its very different velocity must not
  // replace the exact footprint-time measurement used by the sweep.
  record(M::Odometry, anchor + 110000000LL, 5.);
  history = motion.history();
  const auto r = evaluate(&evidence);
  EXPECT_EQ(r.motion_reason, "ready");
  ASSERT_EQ(r.sweep.status, SweepStatus::kClear) << r.sweep.reason;
  EXPECT_NEAR(r.sweep.pose.x, .4, 1e-8);
  EXPECT_STREQ(evaluate(nullptr).sweep.reason, "execution_evidence_missing");
}

TEST_F(TransitionObserverTest, FutureOdometryDoesNotAllowMissingStaleOrInvalidAnchors)
{
  record(M::Odometry, anchor + 110000000LL, 5.);
  history = motion.history();
  EXPECT_EQ(evaluate(&evidence, anchor + 1).motion_reason, "odometry_anchor_missing");
  history.stages[M::Odometry].samples[0].received -= 1s;
  EXPECT_EQ(evaluate(&evidence).motion_reason, "odometry:receipt_stale");
  history = motion.history();
  history.stages[M::Odometry].samples[0].payload = "frame_mismatch";
  EXPECT_EQ(evaluate(&evidence).sweep.status, SweepStatus::kUnavailable);
  history = motion.history();
  history.stages[M::Odometry].samples[0].stamp_ns = anchor + 105000000LL;
  EXPECT_EQ(evaluate(&evidence).motion_reason, "odometry:header_future");
  record(M::Odometry, anchor + 120000000LL, 5., "map");
  history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "odometry:frame_mismatch");
}

TEST_F(TransitionObserverTest, FutureCommandsCannotBeIgnoredLikeOdometry)
{
  record(M::Candidate, anchor + 110000000LL, -.5);
  history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "candidate:header_future");
  EXPECT_FALSE(evaluate(&evidence).evaluated);
}

TEST_F(TransitionObserverTest, NewerValidSampleDoesNotHideInvalidAlignedSample)
{
  record(M::Odometry, anchor + 10000000LL, .8, "map");
  record(M::Odometry, anchor + 20000000LL, .8);
  history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "history_invalid");
}

TEST_F(TransitionObserverTest, RejectsUnknownFramesStaleEvidenceAndRecentDrops)
{
  record(M::Smoothed, anchor, .8, ""); history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "smoothed:frame_unverified");
  record(M::Smoothed, anchor, .8); history = motion.history();
  auto & latest = history.stages[M::Published].samples[0]; latest.received -= 1s;
  EXPECT_EQ(evaluate(&evidence).motion_reason, "published:receipt_stale");
  history = motion.history(); history.last_drop = M::Wall::now(); history.dropped = 1;
  EXPECT_EQ(evaluate(&evidence).motion_reason, "history_gap");
  history.last_drop -= 1s;
  EXPECT_STREQ(evaluate(nullptr).sweep.reason, "execution_evidence_missing");
}

TEST_F(TransitionObserverTest, QueueIdentityAndTimingMustBeExplicit)
{
  evidence.queue_complete = false;
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_queue_incomplete");
  evidence.queue_complete = true;
  for (auto field : {&evidence.epoch, &evidence.revision, &evidence.candidate_count}) {
    ++*field; EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_evidence_mismatch"); --*field;
  }
  ++evidence.pose_stamp_ns;
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_evidence_mismatch"); --evidence.pose_stamp_ns;
  evidence.model.candidate_execution = {0., .1};
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_timing_unknown");
  evidence.model.candidate_execution = {0., 0.}; evidence.model.braking_deceleration.vx = 0.;
  EXPECT_EQ(evaluate(&evidence).sweep.status, SweepStatus::kUnavailable);
  evidence.model.pending.resize(17);
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "motion_history_limit");
}

TEST_F(TransitionObserverTest, HistoryBusyInactiveAndUnknownRevisionCannotEvaluate)
{
  history.state = "history_busy"; EXPECT_EQ(evaluate(&evidence).motion_reason, "history_busy");
  motion.deactivate(); history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "inactive");
  motion.activate();
  for (int i = 0; i < 4; ++i) {record(static_cast<M::Stage>(i), anchor, .1, "base", 0);}
  history = motion.history();
  EXPECT_EQ(evaluate(&evidence).motion_reason, "candidate_revision_unknown");
}

TEST_F(TransitionObserverTest, NewPlanAndLifecycleCannotReuseOldExecutionEvidence)
{
  const auto old_epoch = history.epoch;
  record(M::Candidate, anchor, -.5, "base", 8);
  history = motion.history(); EXPECT_GT(history.epoch, old_epoch);
  EXPECT_EQ(evaluate(&evidence).motion_reason, "published:missing");
  for (int i = 1; i < 4; ++i) {record(static_cast<M::Stage>(i), anchor, .8);}
  history = motion.history();
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_evidence_mismatch");
  motion.deactivate(); motion.activate();
  for (int i = 0; i < 4; ++i) {record(static_cast<M::Stage>(i), anchor, .8);}
  history = motion.history();
  EXPECT_STREQ(evaluate(&evidence).sweep.reason, "execution_evidence_mismatch");
}

TEST_F(TransitionObserverTest, BudgetIncludesUpstreamCaptureAndHistoryCopy)
{
  const auto r = observer.evaluate(geometry, history, anchor, anchor,
      M::Wall::now() - 2s, &evidence);
  EXPECT_STREQ(r.sweep.reason, "observer_budget_exceeded");
  EXPECT_FALSE(r.evaluated); EXPECT_GT(r.total_ms, 1000.);
}

TEST_F(TransitionObserverTest, UnknownSpaceNeverReportsClear)
{
  unsigned int x, y; ASSERT_TRUE(geometry.snapshot->map->worldToMap(.25, 0., x, y));
  geometry.snapshot->map->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
  const auto r = evaluate(&evidence);
  EXPECT_EQ(r.sweep.status, SweepStatus::kUnavailable); EXPECT_STREQ(r.sweep.reason, "unknown_space");
}
}  // namespace
}  // namespace pid_tracking_controller
