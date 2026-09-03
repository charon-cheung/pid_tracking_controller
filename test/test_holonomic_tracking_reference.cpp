#include <cmath>
#include <initializer_list>

#include "gtest/gtest.h"

#include "holonomic_tracking_reference.hpp"
#include "path_progress_tracker.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct PathPose
{
  double x;
  double y;
  double yaw;
};

geometry_msgs::msg::Quaternion makeQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  return point;
}

nav_msgs::msg::Path makePath(std::initializer_list<PathPose> path_poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & path_pose : path_poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = makePoint(path_pose.x, path_pose.y);
    pose.pose.orientation = makeQuaternion(path_pose.yaw);
    path.poses.push_back(pose);
  }
  return path;
}

TEST(HolonomicTrackingReference, separatesAcceptedCurrentFromPreviewGeometry)
{
  const auto path = preparePath(
    makePath({{0.0, 0.0, 0.20}, {1.0, 0.0, 0.60}, {1.0, 1.0, 1.00}}));
  PathProgressTracker tracker;
  tracker.setPath(path);
  const auto progress = tracker.update(makePoint(0.25, 0.0), 1.0);

  const auto reference = buildHolonomicTrackingReference(*path, progress, 1.0);

  EXPECT_EQ(reference.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_NEAR(reference.current.position.x, 0.25, 1e-12);
  EXPECT_NEAR(reference.current.position.y, 0.0, 1e-12);
  EXPECT_EQ(reference.current.cursor.segment_index, 0u);
  EXPECT_NEAR(reference.current.cursor.segment_fraction, 0.25, 1e-12);
  EXPECT_NEAR(reference.current.path_tangent, 0.0, 1e-12);
  EXPECT_NEAR(reference.current.target_yaw, 0.30, 1e-12);
  EXPECT_NEAR(reference.current.target_yaw_rate_per_meter, 0.40, 1e-12);

  EXPECT_NEAR(reference.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.preview.position.y, 0.25, 1e-12);
  EXPECT_EQ(reference.preview.cursor.segment_index, 1u);
  EXPECT_NEAR(reference.preview.cursor.segment_fraction, 0.25, 1e-12);
  EXPECT_NEAR(reference.preview.cursor.translation_arc_length, 1.25, 1e-12);
  EXPECT_NEAR(reference.preview.path_tangent, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(reference.preview.target_yaw, 0.70, 1e-12);
  EXPECT_NEAR(reference.preview.target_yaw_rate_per_meter, 0.40, 1e-12);
  EXPECT_FALSE(reference.preview_clamped_by_rotation);
  EXPECT_NEAR(progress.progress_arc_length, 0.25, 1e-12);
}

TEST(HolonomicTrackingReference, clampsPreviewBeforeRotationWithoutAdvancingProgress)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}, {1.0, 1.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);
  const auto progress = tracker.update(makePoint(0.80, 0.0), 1.0);

  const auto reference = buildHolonomicTrackingReference(*path, progress, 1.0);

  EXPECT_EQ(reference.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_NEAR(reference.current.position.x, 0.80, 1e-12);
  EXPECT_NEAR(reference.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.preview.position.y, 0.0, 1e-12);
  EXPECT_EQ(reference.preview.cursor.segment_index, 0u);
  EXPECT_NEAR(reference.preview.cursor.segment_fraction, 1.0, 1e-12);
  EXPECT_NEAR(reference.preview.cursor.translation_arc_length, 1.0, 1e-12);
  EXPECT_TRUE(reference.preview_clamped_by_rotation);
  EXPECT_NEAR(reference.rotation_target_yaw, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(progress.progress_arc_length, 0.80, 1e-12);

  const auto unchanged = tracker.update(makePoint(0.80, 0.0), 1.0);
  EXPECT_NEAR(unchanged.progress_arc_length, progress.progress_arc_length, 1e-12);
}

TEST(HolonomicTrackingReference, cursorDistinguishesRotationBoundarySides)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}, {1.0, 1.0, 0.5 * kPi}}));

  PathProgressReference before_rotation;
  before_rotation.projection = makePoint(1.0, 0.0);
  before_rotation.cursor = {0u, 1.0, 1.0};
  const auto pending = buildHolonomicTrackingReference(
    *path, before_rotation, 0.20);

  EXPECT_EQ(pending.mode, HolonomicReferenceMode::kRotationPending);
  EXPECT_TRUE(pending.preview_clamped_by_rotation);
  EXPECT_EQ(pending.preview.cursor.segment_index, 0u);
  EXPECT_NEAR(pending.rotation_target_yaw, 0.5 * kPi, 1e-12);

  PathProgressReference after_rotation;
  after_rotation.projection = makePoint(1.0, 0.0);
  after_rotation.cursor = {2u, 0.0, 1.0};
  const auto translation = buildHolonomicTrackingReference(
    *path, after_rotation, 0.20);

  EXPECT_EQ(translation.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_FALSE(translation.preview_clamped_by_rotation);
  EXPECT_EQ(translation.current.cursor.segment_index, 2u);
  EXPECT_EQ(translation.preview.cursor.segment_index, 2u);
  EXPECT_NEAR(translation.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(translation.preview.position.y, 0.20, 1e-12);
}

TEST(HolonomicTrackingReference, reportsTerminalForFinalEndpointAndSinglePose)
{
  const auto translation_path = preparePath(
    makePath({{0.0, 0.0, 0.2}, {1.0, 0.0, 0.6}}));
  PathProgressTracker tracker;
  tracker.setPath(translation_path);
  const auto progress = tracker.update(makePoint(1.0, 0.0), 0.30);
  const auto terminal = buildHolonomicTrackingReference(
    *translation_path, progress, 0.30);

  EXPECT_EQ(terminal.mode, HolonomicReferenceMode::kTerminal);
  EXPECT_NEAR(terminal.current.position.x, 1.0, 1e-12);
  EXPECT_NEAR(terminal.current.target_yaw, 0.6, 1e-12);
  EXPECT_NEAR(terminal.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(terminal.preview.target_yaw, 0.6, 1e-12);

  const auto single_pose_path = preparePath(makePath({{0.2, -0.1, 0.4}}));
  PathProgressTracker single_pose_tracker;
  single_pose_tracker.setPath(single_pose_path);
  const auto single_pose_progress = single_pose_tracker.update(
    makePoint(0.2, -0.1), 0.30);
  const auto single_pose = buildHolonomicTrackingReference(
    *single_pose_path, single_pose_progress, 0.30);

  EXPECT_EQ(single_pose.mode, HolonomicReferenceMode::kTerminal);
  EXPECT_NEAR(single_pose.current.position.x, 0.2, 1e-12);
  EXPECT_NEAR(single_pose.current.position.y, -0.1, 1e-12);
  EXPECT_NEAR(single_pose.current.target_yaw, 0.4, 1e-12);
  EXPECT_NEAR(single_pose.preview.target_yaw, 0.4, 1e-12);
  EXPECT_TRUE(std::isfinite(single_pose.current.path_tangent));
  EXPECT_TRUE(std::isfinite(single_pose.current.target_yaw_rate_per_meter));
}

}  // namespace
}  // namespace pid_tracking_controller
