#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

#include "gtest/gtest.h"

#include "path_progress_tracker.hpp"
#include "holonomic_tracking_reference.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{
namespace
{

geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

void setYaw(geometry_msgs::msg::PoseStamped & pose, double yaw)
{
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  pose.pose.orientation.w = std::cos(0.5 * yaw);
}

nav_msgs::msg::Path makePath(
  std::initializer_list<std::pair<double, double>> coordinates)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & [x, y] : coordinates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = makePoint(x, y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path makeTwoRotationPath()
{
  auto path = makePath(
    {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0},
      {2.0, 0.0}, {3.0, 0.0}});
  setYaw(path.poses[2], M_PI_4);
  setYaw(path.poses[3], M_PI_4);
  setYaw(path.poses[4], M_PI_2);
  setYaw(path.poses[5], M_PI_2);
  return path;
}

nav_msgs::msg::Path makeDenseHairpinPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const auto append = [&path](double x, double y) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = makePoint(x, y);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    };

  for (int index = 0; index <= 20; ++index) {
    append(0.05 * static_cast<double>(index), 0.0);
  }
  append(1.0, 0.05);
  append(1.0, 0.10);
  for (int index = 19; index >= 0; --index) {
    append(0.05 * static_cast<double>(index), 0.10);
  }
  return path;
}

void setPreparedPath(PathProgressTracker & tracker, const nav_msgs::msg::Path & path)
{
  tracker.setPath(preparePath(path));
}

void advanceAlongOutboundBranch(PathProgressTracker & tracker)
{
  for (int index = 0; index <= 8; ++index) {
    tracker.update(makePoint(0.05 * static_cast<double>(index), 0.0), 0.10);
  }
}

void expectFinite(const PathProgressReference & reference)
{
  EXPECT_TRUE(std::isfinite(reference.projection.x));
  EXPECT_TRUE(std::isfinite(reference.projection.y));
  EXPECT_TRUE(std::isfinite(reference.carrot.x));
  EXPECT_TRUE(std::isfinite(reference.carrot.y));
  EXPECT_TRUE(std::isfinite(reference.segment_fraction));
  EXPECT_TRUE(std::isfinite(reference.progress_arc_length));
  EXPECT_TRUE(std::isfinite(reference.carrot_arc_length));
  EXPECT_TRUE(std::isfinite(reference.remaining_path_length));
  EXPECT_TRUE(std::isfinite(reference.progress_budget));
  EXPECT_TRUE(std::isfinite(reference.tangent));
  EXPECT_TRUE(std::isfinite(reference.curvature));
  EXPECT_TRUE(std::isfinite(reference.signed_lateral_error));
}

TEST(PathProgressTracker, denseSamplingDoesNotCapObservedForwardMotion)
{
  for (const double spacing : {0.01, 0.02, 0.05}) {
    SCOPED_TRACE(spacing);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    const int count = static_cast<int>(std::lround(4.0 / spacing));
    for (int i = 0; i <= count; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = makePoint(i * spacing, 0.0);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    PathProgressTracker tracker;
    setPreparedPath(tracker, path);
    tracker.update(makePoint(0.0, 0.0), 0.3);
    const double step = 0.8 / 15.0;
    double maximum_progress_error = 0.0;
    double maximum_reserve_error = 0.0;
    for (int i = 1; i <= 45; ++i) {
      const double x = i * step;
      const auto reference = tracker.update(makePoint(x, 0.0), 0.3);
      maximum_progress_error = std::fmax(
        maximum_progress_error, std::abs(reference.progress_arc_length - x));
      maximum_reserve_error = std::fmax(
        maximum_reserve_error, std::abs(reference.progress_budget - 2.0 * spacing));
    }
    EXPECT_NEAR(maximum_progress_error, 0.0, 1e-8);
    EXPECT_NEAR(maximum_reserve_error, 0.0, 1e-8);
  }
}

TEST(PathProgressTracker, sharedTranslationEndpointUsesOutgoingCursor)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}));
  tracker.update(makePoint(0.0, 0.0), 0.3);
  for (int i = 1; i < 10; ++i) {
    tracker.update(makePoint(0.1 * i, 0.0), 0.3);
  }
  const auto reference = tracker.update(makePoint(1.0, 0.0), 0.3);
  EXPECT_NEAR(reference.progress_arc_length, 1.0, 1e-10);
  EXPECT_EQ(reference.cursor.segment_index, 1u);
  EXPECT_NEAR(reference.cursor.segment_fraction, 0.0, 1e-10);
  EXPECT_NEAR(reference.projection.x, 1.0, 1e-10);
  EXPECT_NEAR(reference.projection.y, 0.0, 1e-10);
}

TEST(PathProgressTracker, sharedEndpointPreservesReserveAndUsesOutgoingMotion)
{
  for (const double direction : {-1.0, 0.0, 1.0}) {
    SCOPED_TRACE(direction);
    const bool straight = direction == 0.0;
    PathProgressTracker tracker;
    setPreparedPath(
      tracker, makePath(
        {{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0}, {0.2, 0.0},
          {straight ? 0.3 : 0.2, direction * 0.1},
          {straight ? 0.7 : 0.2, direction * 0.5}}));

    // Initial anchoring consumes the reserve; further progress needs the outgoing tangent.
    const auto boundary = tracker.update(makePoint(0.2, 0.0), 0.05);
    ASSERT_NEAR(boundary.progress_budget, 0.0, 1e-12);
    for (int sample = 0; sample < 3; ++sample) {
      const auto stationary = tracker.update(makePoint(0.2, 0.0), 0.05);
      EXPECT_EQ(stationary.cursor.segment_index, 2u);
      EXPECT_EQ(stationary.segment_index, 3u);  // The duplicate source pose was skipped.
      EXPECT_NEAR(stationary.cursor.segment_fraction, 0.0, 1e-12);
      EXPECT_NEAR(stationary.progress_arc_length, 0.2, 1e-12);
      EXPECT_NEAR(stationary.progress_budget, 0.0, 1e-12);
    }
    const auto outgoing = tracker.update(
      makePoint(straight ? 0.25 : 0.2, direction * 0.05), 0.05);
    EXPECT_NEAR(outgoing.progress_arc_length, 0.25, 1e-12);
    EXPECT_NEAR(outgoing.cursor.segment_fraction, 0.5, 1e-12);
    EXPECT_NEAR(outgoing.progress_budget, 0.0, 1e-12);
  }
}

TEST(PathProgressTracker, futureNearbyBranchCannotReplaceCurrentOrderedBranch)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makeDenseHairpinPath());
  advanceAlongOutboundBranch(tracker);

  const auto reference = tracker.update(makePoint(0.40, 0.08), 0.10);

  EXPECT_NEAR(reference.projection.x, 0.40, 1e-12);
  EXPECT_NEAR(reference.projection.y, 0.0, 1e-12);
  EXPECT_LT(reference.segment_index, 20u);
  EXPECT_GT(reference.carrot.x, reference.projection.x);
  EXPECT_NEAR(reference.carrot.y, 0.0, 1e-12);
}

TEST(PathProgressTracker, unchangedRobotPoseCannotRatchetProgressForward)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makeDenseHairpinPath());
  advanceAlongOutboundBranch(tracker);

  const auto robot_position = makePoint(0.40, 0.08);
  const double first_progress =
    tracker.update(robot_position, 0.10).progress_arc_length;

  for (int sample = 0; sample < 20; ++sample) {
    EXPECT_DOUBLE_EQ(
      tracker.update(robot_position, 0.10).progress_arc_length,
      first_progress);
  }
}

TEST(PathProgressTracker, sparsePathCarrotInterpolatesAcrossSegmentsByArcLength)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}));

  const auto reference = tracker.update(makePoint(0.0, 0.0), 1.20);

  EXPECT_EQ(reference.segment_index, 0u);
  EXPECT_NEAR(reference.segment_fraction, 0.0, 1e-12);
  EXPECT_NEAR(reference.progress_arc_length, 0.0, 1e-12);
  EXPECT_NEAR(reference.remaining_path_length, 2.0, 1e-12);
  EXPECT_NEAR(reference.progress_budget, 2.0, 1e-12);
  EXPECT_NEAR(reference.carrot.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.carrot.y, 0.20, 1e-12);
  EXPECT_NEAR(reference.carrot_arc_length, 1.20, 1e-12);
}

TEST(PathProgressTracker, nonuniformSamplingPreservesArcLengthProgressAndProvenance)
{
  PathProgressTracker tracker;
  setPreparedPath(
    tracker,
    makePath({{0.0, 0.0}, {0.05, 0.0}, {0.15, 0.0}, {0.90, 0.0}, {1.0, 0.0}}));

  const auto first = tracker.update(makePoint(0.07, 0.0), 0.30);
  const auto second = tracker.update(makePoint(0.17, 0.0), 0.30);
  const auto third = tracker.update(makePoint(0.27, 0.0), 0.30);

  EXPECT_NEAR(first.progress_arc_length, 0.07, 1e-12);
  EXPECT_EQ(first.segment_index, 1u);
  EXPECT_NEAR(first.segment_fraction, 0.20, 1e-12);
  EXPECT_EQ(first.cursor.segment_index, 1u);
  EXPECT_NEAR(first.cursor.segment_fraction, 0.20, 1e-12);
  EXPECT_NEAR(first.cursor.translation_arc_length, 0.07, 1e-12);
  EXPECT_NEAR(first.carrot_arc_length, 0.37, 1e-12);
  EXPECT_NEAR(first.carrot.x, 0.37, 1e-12);
  EXPECT_NEAR(first.remaining_path_length, 0.93, 1e-12);
  EXPECT_NEAR(first.progress_budget, 0.13, 1e-12);

  EXPECT_NEAR(second.progress_arc_length, 0.17, 1e-12);
  EXPECT_EQ(second.segment_index, 2u);
  EXPECT_NEAR(second.segment_fraction, 2.0 / 75.0, 1e-12);
  EXPECT_NEAR(second.remaining_path_length, 0.83, 1e-12);
  EXPECT_NEAR(second.progress_budget, 0.13, 1e-12);
  EXPECT_GT(second.progress_arc_length, first.progress_arc_length);

  EXPECT_NEAR(third.progress_arc_length, 0.27, 1e-12);
  EXPECT_EQ(third.segment_index, 2u);
  EXPECT_NEAR(third.segment_fraction, 12.0 / 75.0, 1e-12);
  EXPECT_NEAR(third.remaining_path_length, 0.73, 1e-12);
  EXPECT_NEAR(third.progress_budget, 0.13, 1e-12);
  EXPECT_GT(third.progress_arc_length, second.progress_arc_length);
}

TEST(PathProgressTracker, repeatedLateralMotionCannotRatchetProgressForward)
{
  PathProgressTracker tracker;
  setPreparedPath(
    tracker,
    makePath(
      {{0.0, 0.0}, {0.05, 0.0}, {0.10, 0.0}, {0.15, 0.0}, {0.20, 0.0},
        {0.25, 0.0}, {0.30, 0.0}, {0.35, 0.0}, {0.40, 0.0}, {0.45, 0.0},
        {0.50, 0.0}}));

  const auto initial = tracker.update(makePoint(0.40, 0.0), 0.10);
  ASSERT_NEAR(initial.progress_arc_length, 0.10, 1e-12);
  ASSERT_NEAR(initial.progress_budget, 0.0, 1e-12);

  for (int sample = 0; sample < 20; ++sample) {
    const double lateral_offset = sample % 2 == 0 ? 0.10 : -0.10;
    const auto reference = tracker.update(makePoint(0.40, lateral_offset), 0.10);
    EXPECT_NEAR(reference.progress_arc_length, initial.progress_arc_length, 1e-12);
    EXPECT_NEAR(reference.progress_budget, initial.progress_budget, 1e-12);
  }
}

TEST(PathProgressTracker, lateralErrorUsesAcceptedProgressSegment)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makeDenseHairpinPath());
  advanceAlongOutboundBranch(tracker);

  const auto reference = tracker.update(makePoint(0.40, 0.08), 0.10);

  EXPECT_NEAR(reference.signed_lateral_error, 0.08, 1e-12);
}

TEST(PathProgressTracker, curvaturePreservesLegacyTurnSignAndMagnitude)
{
  const double diagonal = std::sqrt(0.5);

  PathProgressTracker left_turn_tracker;
  setPreparedPath(
    left_turn_tracker,
    makePath({{1.0, 0.0}, {diagonal, diagonal}, {0.0, 1.0}}));
  const auto left_turn = left_turn_tracker.update(makePoint(1.0, 0.0), 0.10);

  PathProgressTracker right_turn_tracker;
  setPreparedPath(
    right_turn_tracker,
    makePath({{1.0, 0.0}, {diagonal, -diagonal}, {0.0, -1.0}}));
  const auto right_turn = right_turn_tracker.update(makePoint(1.0, 0.0), 0.10);

  PathProgressTracker straight_tracker;
  setPreparedPath(straight_tracker, makePath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}));
  const auto straight = straight_tracker.update(makePoint(0.0, 0.0), 0.10);

  EXPECT_NEAR(left_turn.curvature, 1.0, 1e-12);
  EXPECT_NEAR(right_turn.curvature, -1.0, 1e-12);
  EXPECT_DOUBLE_EQ(straight.curvature, 0.0);
}

TEST(PathProgressTracker, onePointPathReturnsFiniteStationaryReference)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makePath({{0.2, -0.1}}));

  const auto reference = tracker.update(makePoint(0.2, -0.1), 0.30);

  expectFinite(reference);
  EXPECT_DOUBLE_EQ(reference.carrot.x, 0.2);
  EXPECT_DOUBLE_EQ(reference.carrot.y, -0.1);
  EXPECT_DOUBLE_EQ(reference.progress_arc_length, 0.0);
  EXPECT_DOUBLE_EQ(reference.signed_lateral_error, 0.0);
}

TEST(PathProgressTracker, rejectsMissingPathAndInvalidUpdateInputs)
{
  PathProgressTracker tracker;
  EXPECT_THROW(tracker.setPath(PreparedPathPtr()), std::invalid_argument);

  setPreparedPath(tracker, makePath({{0.0, 0.0}, {0.1, 0.0}}));
  auto invalid_position = makePoint(0.0, 0.0);
  invalid_position.y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(tracker.update(invalid_position, 0.1), std::domain_error);
  EXPECT_THROW(tracker.update(makePoint(0.0, 0.0), -0.1), std::domain_error);
  EXPECT_THROW(
    tracker.update(
      makePoint(0.0, 0.0), std::numeric_limits<double>::quiet_NaN()),
    std::domain_error);
}

TEST(PathProgressTracker, cursorUsesPreparedSegmentIndexAndKeepsLegacyProvenance)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makePath({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}}));

  const auto reference = tracker.update(makePoint(0.20, 0.0), 0.30);

  EXPECT_EQ(reference.cursor.segment_index, 0u);
  EXPECT_NEAR(reference.cursor.segment_fraction, 0.20, 1e-12);
  EXPECT_NEAR(reference.cursor.translation_arc_length, 0.20, 1e-12);
  EXPECT_EQ(reference.segment_index, 1u);
  EXPECT_NEAR(reference.segment_fraction, 0.20, 1e-12);
  EXPECT_NEAR(reference.progress_arc_length, 0.20, 1e-12);
}

TEST(PathProgressTracker, legacyCarrotStillCrossesShadowRotationSegment)
{
  auto path = makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  path.poses[2].pose.orientation.z = std::sin(M_PI_4);
  path.poses[2].pose.orientation.w = std::cos(M_PI_4);
  path.poses[3].pose.orientation = path.poses[2].pose.orientation;

  PathProgressTracker tracker;
  setPreparedPath(tracker, path);
  const auto reference = tracker.update(makePoint(0.80, 0.0), 0.50);

  EXPECT_EQ(reference.cursor.segment_index, 0u);
  EXPECT_EQ(reference.segment_index, 0u);
  EXPECT_NEAR(reference.progress_arc_length, 0.80, 1e-12);
  EXPECT_NEAR(reference.carrot_arc_length, 1.30, 1e-12);
  EXPECT_NEAR(reference.carrot.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.carrot.y, 0.30, 1e-12);
  EXPECT_NEAR(reference.tangent, M_PI_2, 1e-12);
}

TEST(PathProgressTracker, acknowledgementPreservesProgressAndExcludesOlderSegments)
{
  auto path = makePath(
    {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  setYaw(path.poses[2], M_PI_4);
  setYaw(path.poses[3], M_PI_2);
  setYaw(path.poses[4], M_PI_2);

  PathProgressTracker tracker;
  setPreparedPath(tracker, path);
  const auto before = tracker.update(makePoint(1.0, 0.0), 0.50);
  ASSERT_EQ(before.cursor.segment_index, 0u);
  ASSERT_EQ(before.prepared_segment_lower_bound, 0u);

  tracker.acknowledgeRotationComplete(1u);
  const auto after = tracker.update(makePoint(1.0, 0.0), 0.50);

  EXPECT_EQ(after.prepared_segment_lower_bound, 3u);
  EXPECT_EQ(after.cursor.segment_index, 3u);
  EXPECT_DOUBLE_EQ(after.progress_arc_length, before.progress_arc_length);
  EXPECT_DOUBLE_EQ(after.progress_budget, before.progress_budget);

  for (int sample = 0; sample < 5; ++sample) {
    const auto stationary = tracker.update(makePoint(1.0, 0.0), 0.50);
    EXPECT_EQ(stationary.prepared_segment_lower_bound, 3u);
    EXPECT_EQ(stationary.cursor.segment_index, 3u);
  }
}

TEST(PathProgressTracker, acknowledgementRequiresReachedEarliestRotationAndIsMonotonic)
{
  PathProgressTracker missing_path;
  EXPECT_THROW(missing_path.acknowledgeRotationComplete(0u), std::logic_error);

  PathProgressTracker tracker;
  setPreparedPath(tracker, makeTwoRotationPath());
  tracker.update(makePoint(0.0, 0.0), 0.10);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(1u), std::logic_error);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(0u), std::invalid_argument);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(9u), std::out_of_range);

  const auto reached = tracker.update(makePoint(2.0, 0.0), 0.10);
  ASSERT_NEAR(reached.progress_arc_length, 1.0, 1e-12);
  EXPECT_EQ(reached.cursor.segment_index, 0u);
  EXPECT_NEAR(reached.cursor.segment_fraction, 1.0, 1e-12);
  EXPECT_NEAR(reached.progress_budget, 2.0, 1e-12);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(3u), std::logic_error);

  tracker.acknowledgeRotationComplete(1u);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(1u), std::logic_error);
  EXPECT_THROW(tracker.acknowledgeRotationComplete(3u), std::logic_error);
  const auto second_boundary = tracker.update(makePoint(2.0, 0.0), 0.10);
  EXPECT_NEAR(second_boundary.progress_arc_length, 2.0, 1e-12);
  EXPECT_EQ(second_boundary.cursor.segment_index, 2u);
  tracker.acknowledgeRotationComplete(3u);

  const auto acknowledged = tracker.update(makePoint(2.0, 0.0), 0.10);
  EXPECT_EQ(acknowledged.prepared_segment_lower_bound, 4u);
  EXPECT_EQ(acknowledged.cursor.segment_index, 4u);

  setPreparedPath(tracker, makeTwoRotationPath());
  const auto reset = tracker.update(makePoint(0.0, 0.0), 0.10);
  EXPECT_EQ(reset.prepared_segment_lower_bound, 0u);
}

TEST(PathProgressTracker, initialRotationBlocksTranslationUntilAcknowledged)
{
  auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}});
  setYaw(path.poses[1], M_PI_2);
  setYaw(path.poses[2], M_PI_2);
  PathProgressTracker tracker;
  setPreparedPath(tracker, path);

  for (const double x : {0.0, 0.2}) {
    const auto pending = tracker.update(makePoint(x, 0.0), 0.1);
    EXPECT_EQ(pending.cursor.segment_index, 0u);
    EXPECT_DOUBLE_EQ(pending.progress_arc_length, 0.0);
    EXPECT_DOUBLE_EQ(pending.projection.x, 0.0);
    EXPECT_DOUBLE_EQ(pending.progress_budget, 2.0);
    EXPECT_NEAR(pending.carrot_arc_length, 0.1, 1e-12);
    EXPECT_NEAR(pending.carrot.x, 0.1, 1e-12);
    EXPECT_NEAR(pending.carrot.y, 0.0, 1e-12);
  }
  tracker.acknowledgeRotationComplete(0u);
  const auto after = tracker.update(makePoint(0.2, 0.0), 0.1);
  EXPECT_EQ(after.cursor.segment_index, 1u);
  EXPECT_NEAR(after.progress_arc_length, 0.2, 1e-12);
}

TEST(PathProgressTracker, acknowledgementPreservesPositionAndAcceptedTangentHistory)
{
  auto path = makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  setYaw(path.poses[2], M_PI_2);
  setYaw(path.poses[3], M_PI_2);

  PathProgressTracker tracker;
  setPreparedPath(tracker, path);
  const auto before = tracker.update(makePoint(1.0, 0.0), 0.20);
  ASSERT_NEAR(before.progress_budget, 1.0, 1e-12);

  EXPECT_THROW(tracker.acknowledgeRotationComplete(0u), std::invalid_argument);
  tracker.acknowledgeRotationComplete(1u);
  const auto after = tracker.update(makePoint(1.20, 0.0), 0.20);

  EXPECT_EQ(after.cursor.segment_index, 2u);
  EXPECT_DOUBLE_EQ(after.progress_arc_length, before.progress_arc_length);
  EXPECT_NEAR(after.progress_budget, before.progress_budget + 0.20, 1e-12);
}

TEST(PathProgressTracker, handoffAnchorsDelayedDensePlanWithoutSpendingMotionBudget)
{
  for (const double spacing : {0.01, 0.05, 0.20}) {
    SCOPED_TRACE(spacing);
    auto path = makePath({{3.5, 0.0}, {3.5, 0.0}});
    setYaw(path.poses.front(), 0.2);
    for (int i = 1; i <= static_cast<int>(std::lround(3.0 / spacing)); ++i) {
      auto pose = path.poses.back();
      pose.pose.position.x = 3.5 - i * spacing;
      path.poses.push_back(pose);
    }
    const auto prepared = preparePath(path);
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto handoff = tracker.initializePathHandoff(makePoint(1.925, -0.02), 0.3);
    EXPECT_NEAR(handoff.progress_arc_length, 1.575, 1e-10);
    EXPECT_NEAR(handoff.projection.x, 1.925, 1e-10);
    EXPECT_NEAR(handoff.projection.y, 0.0, 1e-10);
    EXPECT_NEAR(handoff.progress_budget, 2.0 * spacing, 1e-10);
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, handoff, 0.3).mode,
      HolonomicReferenceMode::kTranslation);
    EXPECT_THROW(tracker.acknowledgeRotationComplete(0u), std::logic_error);

    const auto forward = tracker.update(makePoint(1.855, -0.02), 0.3);
    EXPECT_NEAR(forward.progress_arc_length, 1.645, 1e-10);
    EXPECT_NEAR(forward.progress_budget, 2.0 * spacing, 1e-10);
    for (int i = 0; i < 8; ++i) {
      const auto lateral = tracker.update(makePoint(1.855, i % 2 ? 0.01 : -0.03), 0.3);
      EXPECT_NEAR(lateral.progress_arc_length, 1.645, 1e-10);
    }
    EXPECT_NEAR(tracker.update(makePoint(1.90, 0.0), 0.3).progress_arc_length, 1.645, 1e-10);
  }
}

TEST(PathProgressTracker, handoffUsesOnlyTheNewRevisionsArcAndMotionHistory)
{
  PathProgressTracker tracker;
  for (const double start : {4.0, 2.0, 3.0, 1.0}) {
    auto path = makePath({{start, 0.0}, {0.0, 0.0}});
    setPreparedPath(tracker, path);
    const auto handoff = tracker.initializePathHandoff(makePoint(0.5, 0.0), 0.1);
    EXPECT_NEAR(handoff.progress_arc_length, start - 0.5, 1e-12);
    EXPECT_NEAR(handoff.projection.x, 0.5, 1e-12);
    EXPECT_NEAR(tracker.update(makePoint(0.4, 0.0), 0.1).progress_arc_length,
      start - 0.4, 1e-12);
  }
}

TEST(PathProgressTracker, handoffSkipsOnlyRotationsStrictlyBeforeItsBoundary)
{
  auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0},
      {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {2.0, 0.0}});
  const double yaws[] = {0.0, 0.3, 0.6, 0.6, 1.0, 1.3, 1.3, 0.0};
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    setYaw(path.poses[i], yaws[i]);
  }
  const auto prepared = preparePath(path);
  struct Case {double x; int pending_rotation;};
  for (const auto & test : {Case{0.0, 0}, Case{0.5, -1}, Case{1.0 - 1e-8, -1},
      Case{1.0, 3}, Case{1.0 + 5e-11, 3}, Case{1.0 + 1e-8, -1}, Case{1.5, -1},
      Case{2.0, 6}})
  {
    SCOPED_TRACE(test.x);
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto handoff = tracker.initializePathHandoff(makePoint(test.x, 0.0), 0.3);
    const auto reference = buildHolonomicTrackingReference(*prepared, handoff, 0.3);
    EXPECT_NEAR(handoff.progress_arc_length, test.x, 1e-10);
    if (test.pending_rotation >= 0) {
      EXPECT_EQ(reference.mode, HolonomicReferenceMode::kRotationPending);
      ASSERT_TRUE(reference.pending_rotation_segment_index);
      EXPECT_EQ(*reference.pending_rotation_segment_index,
        static_cast<std::size_t>(test.pending_rotation));
      tracker.acknowledgeRotationComplete(*reference.pending_rotation_segment_index);
      const auto after = tracker.update(makePoint(test.x, 0.0), 0.3);
      EXPECT_NE(buildHolonomicTrackingReference(*prepared, after, 0.3).mode,
        HolonomicReferenceMode::kRotationPending);
    } else {
      EXPECT_EQ(reference.mode, HolonomicReferenceMode::kTranslation);
    }
  }
}

TEST(PathProgressTracker, handoffPreservesInitialRotationForRecordedStartOffsets)
{
  struct Case
  {
    const char * name;
    double x, y, yaw_before, yaw_after, next_x, next_y, end_x, end_y, robot_x, robot_y;
  };
  // First four recorded path poses and first controller observation, 2026-10-26.
  const Case cases[] = {
    {"P2", -0.376336756405, 2.432739440695, 1.542931601429, -1.859451041216,
      -0.376335888368, 2.432739226195, -0.393415966111, 2.423967253735,
      -0.376639534000, 2.432365074000},
    {"P6", 4.614394463688, -0.445546884611, -0.200973577834, -1.731624046366,
      4.596950453675, -0.435789807092, 4.579446944392, -0.426139865532,
      4.614094483000, -0.445434164000},
    {"P9", -0.465840518337, -0.351243173739, 0.500324181007, -1.818115521602,
      -0.465904590934, -0.311243211574, -0.465968663531, -0.271243249843,
      -0.466236923000, -0.351229146000},
    {"P7", -0.498012673423, -0.547499244270, -1.727530214720, 0.855339007114,
      -0.498013131742, -0.547499965004, -0.515156686046, -0.538928187852,
      -0.497781653000, -0.547562884000}
  };
  for (const auto & test : cases) {
    SCOPED_TRACE(test.name);
    auto path = makePath({{test.x, test.y}, {test.x, test.y},
        {test.next_x, test.next_y}, {test.end_x, test.end_y}});
    setYaw(path.poses.front(), test.yaw_before);
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      setYaw(path.poses[i], test.yaw_after);
    }
    const auto prepared = preparePath(path);
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto robot = makePoint(test.robot_x, test.robot_y);
    const auto handoff = tracker.initializePathHandoff(robot, 0.5);
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, handoff, 0.5).mode,
      HolonomicReferenceMode::kRotationPending);
    // Later position observations cannot silently consume the pending rotation.
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, tracker.update(robot, 0.5), 0.5).mode,
      HolonomicReferenceMode::kRotationPending);
    EXPECT_NO_THROW(tracker.acknowledgeRotationComplete(0u));
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, tracker.update(robot, 0.5), 0.5).mode,
      HolonomicReferenceMode::kTranslation);
  }
}

TEST(PathProgressTracker, initialRotationHandoffRequiresBothPositionAndArcProximity)
{
  auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}});
  setYaw(path.poses[1], 0.3);
  setYaw(path.poses[2], 0.6);
  setYaw(path.poses[3], 0.6);
  const auto prepared = preparePath(path);
  struct Case {double x, y; bool pending;};
  for (const auto & test : {
      Case{-0.005, 0.0, true}, Case{0.0, -0.005, true}, Case{0.0, 0.005, true},
      Case{0.005, 0.0, true}, Case{0.01, 0.0, true}, Case{0.019, 0.0, true},
      Case{0.02, 0.0, true}, Case{0.020001, 0.0, false}, Case{0.03, 0.0, false},
      Case{0.05, 0.0, false}, Case{0.005, 0.019, true},
      Case{0.005, 0.020, false}, Case{0.005, -0.020, false}})
  {
    SCOPED_TRACE(::testing::Message() << test.x << ", " << test.y);
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto robot = makePoint(test.x, test.y);
    const auto handoff = tracker.initializePathHandoff(robot, 0.1);
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, handoff, 0.1).mode,
      test.pending ? HolonomicReferenceMode::kRotationPending :
      HolonomicReferenceMode::kTranslation);
    if (test.pending) {
      EXPECT_NO_THROW(tracker.acknowledgeRotationComplete(0u));
      EXPECT_EQ(buildHolonomicTrackingReference(*prepared, tracker.update(robot, 0.1), 0.1).mode,
        HolonomicReferenceMode::kTranslation);
    }
  }

  // A return leg is near the initial anchor but has already passed it along the path.
  auto returning = makePath({{0.0, 0.0}, {0.0, 0.0}, {0.04, 0.0},
      {0.04, 0.04}, {0.005, 0.04}, {0.005, 0.015}});
  for (std::size_t i = 1; i < returning.poses.size(); ++i) {
    setYaw(returning.poses[i], 0.6);
  }
  const auto return_prepared = preparePath(returning);
  PathProgressTracker tracker;
  tracker.setPath(return_prepared);
  const auto handoff = tracker.initializePathHandoff(makePoint(0.005, 0.015), 0.1);
  EXPECT_NEAR(handoff.progress_arc_length, 0.14, 1e-10);
  EXPECT_NE(buildHolonomicTrackingReference(*return_prepared, handoff, 0.1).mode,
    HolonomicReferenceMode::kRotationPending);
}

TEST(PathProgressTracker, initialRotationProtectionDoesNotBypassBranchAmbiguity)
{
  auto path = makePath({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0},
      {1.0, 0.01}, {0.0, 0.01}});
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    setYaw(path.poses[i], 0.6);
  }
  PathProgressTracker tracker;
  tracker.setPath(preparePath(path));
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.005, 0.005), 0.1), std::domain_error);
}

TEST(PathProgressTracker, handoffSelectsClearlyNearestBranchDespiteRemoteAlternatives)
{
  for (const bool reverse_order : {false, true}) {
    const auto path = reverse_order ?
      makePath({{0.0, 0.47}, {2.0, 0.47}, {2.0, 0.0}, {0.0, 0.0}}) :
      makePath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.47}, {0.0, 0.47}});
    PathProgressTracker tracker;
    setPreparedPath(tracker, path);
    // Distances are 0.02 m and 0.45 m: a second branch is not itself ambiguity.
    const auto handoff = tracker.initializePathHandoff(makePoint(0.4, 0.02), 0.1);
    EXPECT_NEAR(handoff.projection.x, 0.4, 1e-12);
    EXPECT_NEAR(handoff.projection.y, 0.0, 1e-12);
    EXPECT_NEAR(handoff.progress_arc_length, reverse_order ? 4.07 : 0.4, 1e-12);
  }

  // Dense sampling and a nearby return leg must not reject a clear nearest leg.
  for (const double y : {0.02, 0.08}) {
    PathProgressTracker tracker;
    setPreparedPath(tracker, makeDenseHairpinPath());
    const auto handoff = tracker.initializePathHandoff(makePoint(0.4, y), 0.1);
    EXPECT_NEAR(handoff.progress_arc_length, y < 0.05 ? 0.4 : 1.7, 1e-12);
  }
}

TEST(PathProgressTracker, handoffRejectsCrossingsReversalsAndNearlyEquidistantBranches)
{
  const auto crossing = makePath({{-1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}, {1.0, -1.0}});
  const auto reversal = makePath({{0.0, 0.0}, {2.0, 0.0}, {0.0, 0.0}});
  const auto parallel = makePath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0},
      {-1.0, 1.0}, {-1.0, 0.1}, {2.0, 0.1}});
  for (const auto & path : {crossing, reversal, makeDenseHairpinPath(), parallel}) {
    PathProgressTracker tracker;
    setPreparedPath(tracker, path);
    const auto point = path.poses.front().pose.position.x < 0.0 ?
      makePoint(0.0, 0.0) : makePoint(0.4, 0.049);
    for (int attempt = 0; attempt < 2; ++attempt) {
      EXPECT_THROW(tracker.initializePathHandoff(point, 0.1), std::domain_error);
    }
  }

  // A numerical nearest point is insufficient when distinct branches differ by 3 mm.
  PathProgressTracker tracker;
  setPreparedPath(tracker,
    makePath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.045}, {0.0, 0.045}}));
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.4, 0.021), 0.1), std::domain_error);
  // An exact match must not mask a different, almost overlapping return leg.
  setPreparedPath(tracker,
    makePath({{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.005}, {0.0, 0.005}}));
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.4, 0.0), 0.1), std::domain_error);
}

TEST(PathProgressTracker, nearestHandoffKeepsRotationBoundaryAndSkipsOnlyPassedPrefix)
{
  auto path = makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0},
      {2.0, 0.0}, {2.0, 0.47}, {0.0, 0.47}});
  for (std::size_t i = 2; i < path.poses.size(); ++i) {
    setYaw(path.poses[i], 0.5);
  }
  const auto prepared = preparePath(path);
  for (const double x : {1.0, 1.1}) {
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto handoff = tracker.initializePathHandoff(makePoint(x, 0.02), 0.1);
    EXPECT_NEAR(handoff.progress_arc_length, x, 1e-12);
    EXPECT_NEAR(handoff.projection.y, 0.0, 1e-12);
    const auto reference = buildHolonomicTrackingReference(*prepared, handoff, 0.1);
    if (x == 1.0) {
      EXPECT_EQ(reference.mode, HolonomicReferenceMode::kRotationPending);
      ASSERT_TRUE(reference.pending_rotation_segment_index);
      EXPECT_EQ(*reference.pending_rotation_segment_index, 1u);
    } else {
      EXPECT_EQ(reference.mode, HolonomicReferenceMode::kTranslation);
      EXPECT_THROW(tracker.acknowledgeRotationComplete(1u), std::logic_error);
    }
  }
}

TEST(PathProgressTracker, handoffFailureDoesNotCommitAndAcceptedBranchKeepsNormalProtection)
{
  PathProgressTracker tracker;
  setPreparedPath(tracker, makeDenseHairpinPath());
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.4, 0.049), 0.1), std::domain_error);
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.4, 0.0), -0.1), std::domain_error);
  auto invalid = makePoint(0.4, 0.0);
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.initializePathHandoff(invalid, 0.1), std::domain_error);
  const auto handoff = tracker.initializePathHandoff(makePoint(0.4, 0.0), 0.1);
  EXPECT_NEAR(handoff.progress_arc_length, 0.4, 1e-12);
  for (int i = 0; i < 10; ++i) {
    const auto nearby = tracker.update(makePoint(0.4, 0.08), 0.1);
    EXPECT_NEAR(nearby.projection.y, 0.0, 1e-12);
    EXPECT_NEAR(nearby.progress_arc_length, 0.4, 1e-12);
  }
  EXPECT_THROW(tracker.initializePathHandoff(makePoint(0.4, 0.1), 0.1), std::logic_error);
}

TEST(PathProgressTracker, handoffTreatsOneBendAsOneBranchAndOwnsSharedTranslationEndpoint)
{
  for (const double spacing : {0.05, 1.0}) {
    auto path = makePath({{0.0, 0.0}});
    for (int i = 1; i <= static_cast<int>(std::lround(1.0 / spacing)); ++i) {
      auto pose = path.poses.back();
      pose.pose.position.x = i * spacing;
      path.poses.push_back(pose);
    }
    for (int i = 1; i <= static_cast<int>(std::lround(1.0 / spacing)); ++i) {
      auto pose = path.poses.back();
      pose.pose.position.y = i * spacing;
      path.poses.push_back(pose);
    }
    PathProgressTracker tracker;
    setPreparedPath(tracker, path);
    const auto incoming = tracker.initializePathHandoff(makePoint(0.8, 0.1), 0.1);
    EXPECT_NEAR(incoming.progress_arc_length, 0.8, 1e-10);
    EXPECT_NEAR(incoming.projection.y, 0.0, 1e-10);
    setPreparedPath(tracker, path);
    const auto outgoing = tracker.initializePathHandoff(makePoint(0.9, 0.2), 0.1);
    EXPECT_NEAR(outgoing.progress_arc_length, 1.2, 1e-10);
    EXPECT_NEAR(outgoing.projection.x, 1.0, 1e-10);
    setPreparedPath(tracker, path);
    const auto boundary = tracker.initializePathHandoff(makePoint(1.0, 0.0), 0.1);
    EXPECT_NEAR(boundary.cursor.segment_fraction, 0.0, 1e-10);
    EXPECT_NEAR(tracker.update(makePoint(1.0, 0.05), 0.1).progress_arc_length, 1.05, 1e-10);
  }
}

TEST(PathProgressTracker, handoffPreservesSinglePoseAndRotationOnlyPaths)
{
  for (const bool rotation : {false, true}) {
    auto path = makePath({{1.0, 2.0}});
    if (rotation) {
      path.poses.push_back(path.poses.back());
      setYaw(path.poses.back(), 0.5);
    }
    const auto prepared = preparePath(path);
    PathProgressTracker tracker;
    tracker.setPath(prepared);
    const auto handoff = tracker.initializePathHandoff(makePoint(1.01, 1.98), 0.1);
    EXPECT_EQ(buildHolonomicTrackingReference(*prepared, handoff, 0.1).mode,
      rotation ? HolonomicReferenceMode::kRotationPending : HolonomicReferenceMode::kTerminal);
    EXPECT_NEAR(handoff.projection.x, 1.0, 1e-12);
    EXPECT_NEAR(handoff.projection.y, 2.0, 1e-12);
  }
}

}  // namespace
}  // namespace pid_tracking_controller
