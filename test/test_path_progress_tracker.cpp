#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

#include "gtest/gtest.h"

#include "path_progress_tracker.hpp"

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

TEST(PathProgressTracker, futureNearbyBranchCannotReplaceCurrentOrderedBranch)
{
  PathProgressTracker tracker;
  tracker.setPath(makeDenseHairpinPath());
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
  tracker.setPath(makeDenseHairpinPath());
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
  tracker.setPath(makePath({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}));

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
  tracker.setPath(makePath(
      {{0.0, 0.0}, {0.05, 0.0}, {0.15, 0.0}, {0.90, 0.0}, {1.0, 0.0}}));

  const auto first = tracker.update(makePoint(0.07, 0.0), 0.30);
  const auto second = tracker.update(makePoint(0.17, 0.0), 0.30);
  const auto third = tracker.update(makePoint(0.27, 0.0), 0.30);

  EXPECT_NEAR(first.progress_arc_length, 0.07, 1e-12);
  EXPECT_EQ(first.segment_index, 1u);
  EXPECT_NEAR(first.segment_fraction, 0.20, 1e-12);
  EXPECT_NEAR(first.carrot_arc_length, 0.37, 1e-12);
  EXPECT_NEAR(first.carrot.x, 0.37, 1e-12);
  EXPECT_NEAR(first.remaining_path_length, 0.93, 1e-12);
  EXPECT_NEAR(first.progress_budget, 0.13, 1e-12);

  EXPECT_NEAR(second.progress_arc_length, 0.17, 1e-12);
  EXPECT_EQ(second.segment_index, 2u);
  EXPECT_NEAR(second.segment_fraction, 2.0 / 75.0, 1e-12);
  EXPECT_NEAR(second.remaining_path_length, 0.83, 1e-12);
  EXPECT_NEAR(second.progress_budget, 0.10, 1e-12);
  EXPECT_GT(second.progress_arc_length, first.progress_arc_length);

  EXPECT_NEAR(third.progress_arc_length, 0.27, 1e-12);
  EXPECT_EQ(third.segment_index, 2u);
  EXPECT_NEAR(third.segment_fraction, 12.0 / 75.0, 1e-12);
  EXPECT_NEAR(third.remaining_path_length, 0.73, 1e-12);
  EXPECT_NEAR(third.progress_budget, 0.10, 1e-12);
  EXPECT_GT(third.progress_arc_length, second.progress_arc_length);
}

TEST(PathProgressTracker, repeatedLateralMotionCannotRatchetProgressForward)
{
  PathProgressTracker tracker;
  tracker.setPath(makePath(
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
  tracker.setPath(makeDenseHairpinPath());
  advanceAlongOutboundBranch(tracker);

  const auto reference = tracker.update(makePoint(0.40, 0.08), 0.10);

  EXPECT_NEAR(reference.signed_lateral_error, 0.08, 1e-12);
}

TEST(PathProgressTracker, onePointPathReturnsFiniteStationaryReference)
{
  PathProgressTracker tracker;
  tracker.setPath(makePath({{0.2, -0.1}}));

  const auto reference = tracker.update(makePoint(0.2, -0.1), 0.30);

  expectFinite(reference);
  EXPECT_DOUBLE_EQ(reference.carrot.x, 0.2);
  EXPECT_DOUBLE_EQ(reference.carrot.y, -0.1);
  EXPECT_DOUBLE_EQ(reference.progress_arc_length, 0.0);
  EXPECT_DOUBLE_EQ(reference.signed_lateral_error, 0.0);
}

TEST(PathProgressTracker, rejectsInvalidPathAndUpdateInputs)
{
  PathProgressTracker tracker;
  EXPECT_THROW(tracker.setPath(nav_msgs::msg::Path()), std::invalid_argument);

  auto invalid_path = makePath({{0.0, 0.0}, {0.1, 0.0}});
  invalid_path.poses.back().pose.position.x =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(tracker.setPath(invalid_path), std::invalid_argument);

  tracker.setPath(makePath({{0.0, 0.0}, {0.1, 0.0}}));
  auto invalid_position = makePoint(0.0, 0.0);
  invalid_position.y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(tracker.update(invalid_position, 0.1), std::domain_error);
  EXPECT_THROW(tracker.update(makePoint(0.0, 0.0), -0.1), std::domain_error);
  EXPECT_THROW(
    tracker.update(
      makePoint(0.0, 0.0), std::numeric_limits<double>::quiet_NaN()),
    std::domain_error);
}

}  // namespace
}  // namespace pid_tracking_controller
