#include <cmath>
#include <initializer_list>

#include "gtest/gtest.h"

#include "holonomic_tracking_reference.hpp"
#include "legacy_carrot_velocity_planner.hpp"
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

TEST(HolonomicTrackingReference, keepsPathTangentIndependentFromTargetYaw)
{
  const auto path = preparePath(
    makePath({{0.0, 0.0, 0.5 * kPi}, {2.0, 0.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);
  const auto progress = tracker.update(makePoint(0.40, 0.0), 0.60);

  const auto reference = buildHolonomicTrackingReference(*path, progress, 0.60);

  EXPECT_NEAR(reference.current.position.x, progress.projection.x, 1e-12);
  EXPECT_NEAR(reference.current.position.y, progress.projection.y, 1e-12);
  EXPECT_NEAR(reference.current.path_tangent, 0.0, 1e-12);
  EXPECT_NEAR(reference.current.target_yaw, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(reference.current.target_yaw_rate_per_meter, 0.0, 1e-12);
  EXPECT_NEAR(reference.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(reference.preview.path_tangent, 0.0, 1e-12);
  EXPECT_NEAR(reference.preview.target_yaw, 0.5 * kPi, 1e-12);
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
  const auto repeated_pending = buildHolonomicTrackingReference(
    *path, before_rotation, 0.20);

  EXPECT_EQ(pending.mode, HolonomicReferenceMode::kRotationPending);
  ASSERT_TRUE(pending.pending_rotation_segment_index.has_value());
  EXPECT_EQ(*pending.pending_rotation_segment_index, 1u);
  EXPECT_EQ(repeated_pending.mode, HolonomicReferenceMode::kRotationPending);
  EXPECT_TRUE(pending.preview_clamped_by_rotation);
  EXPECT_EQ(pending.preview.cursor.segment_index, 0u);
  EXPECT_NEAR(pending.rotation_target_yaw, 0.5 * kPi, 1e-12);

  PathProgressReference after_rotation;
  after_rotation.projection = makePoint(1.0, 0.0);
  after_rotation.cursor = {2u, 0.0, 1.0};
  after_rotation.prepared_segment_lower_bound = 2u;
  const auto translation = buildHolonomicTrackingReference(
    *path, after_rotation, 0.20);

  EXPECT_EQ(translation.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_FALSE(translation.pending_rotation_segment_index.has_value());
  EXPECT_FALSE(translation.preview_clamped_by_rotation);
  EXPECT_EQ(translation.current.cursor.segment_index, 2u);
  EXPECT_EQ(translation.preview.cursor.segment_index, 2u);
  EXPECT_NEAR(translation.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(translation.preview.position.y, 0.20, 1e-12);
}

TEST(HolonomicTrackingReference, exactPendingRotationClearsAfterAcknowledgement)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}, {1.0, 1.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto crossed_progress = tracker.update(makePoint(1.0, 0.20), 0.20);
  ASSERT_EQ(crossed_progress.cursor.segment_index, 2u);
  const auto pending = buildHolonomicTrackingReference(
    *path, crossed_progress, 0.20);
  ASSERT_EQ(pending.mode, HolonomicReferenceMode::kRotationPending);
  ASSERT_TRUE(pending.pending_rotation_segment_index.has_value());
  EXPECT_EQ(*pending.pending_rotation_segment_index, 1u);

  tracker.acknowledgeRotationComplete(1u);
  const auto acknowledged_progress = tracker.update(makePoint(1.0, 0.20), 0.20);
  const auto translation = buildHolonomicTrackingReference(
    *path, acknowledged_progress, 0.20);
  EXPECT_EQ(translation.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_FALSE(translation.pending_rotation_segment_index.has_value());
  EXPECT_EQ(translation.current.cursor.segment_index, 2u);
}

TEST(HolonomicTrackingReference, oneMeterBoundaryPendingAlwaysPermitsAcknowledgement)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto outside_progress = tracker.update(makePoint(1.0 - 5e-10, 0.0), 0.20);
  const auto outside = buildHolonomicTrackingReference(
    *path, outside_progress, 0.20);
  EXPECT_EQ(outside.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_FALSE(outside.pending_rotation_segment_index.has_value());

  const auto inside_progress = tracker.update(makePoint(1.0 - 5e-11, 0.0), 0.20);
  const auto inside = buildHolonomicTrackingReference(*path, inside_progress, 0.20);
  ASSERT_EQ(inside.pending_rotation_segment_index, std::optional<std::size_t>(1u));
  EXPECT_NO_THROW(tracker.acknowledgeRotationComplete(1u));
}

TEST(HolonomicTrackingReference, shortBoundaryPendingAlwaysPermitsAcknowledgement)
{
  constexpr double kShortTranslationLength = 2e-4;
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {kShortTranslationLength, 0.0, 0.0},
        {kShortTranslationLength, 0.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto outside_progress = tracker.update(
    makePoint(kShortTranslationLength - 5e-10, 0.0), 0.20);
  const auto outside = buildHolonomicTrackingReference(
    *path, outside_progress, 0.20);
  EXPECT_EQ(outside.mode, HolonomicReferenceMode::kTranslation);
  EXPECT_FALSE(outside.pending_rotation_segment_index.has_value());

  const auto inside_progress = tracker.update(
    makePoint(kShortTranslationLength - 5e-11, 0.0), 0.20);
  const auto inside = buildHolonomicTrackingReference(*path, inside_progress, 0.20);
  ASSERT_EQ(inside.pending_rotation_segment_index, std::optional<std::size_t>(1u));
  EXPECT_NO_THROW(tracker.acknowledgeRotationComplete(1u));
}

TEST(HolonomicTrackingReference, finalRotationAcknowledgementProducesTerminalSentinel)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto progress = tracker.update(makePoint(1.0, 0.0), 0.20);
  const auto pending = buildHolonomicTrackingReference(*path, progress, 0.20);
  ASSERT_EQ(pending.mode, HolonomicReferenceMode::kRotationPending);
  ASSERT_EQ(pending.pending_rotation_segment_index, std::optional<std::size_t>(1u));

  tracker.acknowledgeRotationComplete(1u);
  const auto terminal_progress = tracker.update(makePoint(1.0, 0.0), 0.20);
  const auto terminal = buildHolonomicTrackingReference(
    *path, terminal_progress, 0.20);

  EXPECT_EQ(terminal_progress.cursor.segment_index, path->segments.size());
  EXPECT_EQ(terminal_progress.prepared_segment_lower_bound, path->segments.size());
  EXPECT_EQ(terminal.mode, HolonomicReferenceMode::kTerminal);
  EXPECT_FALSE(terminal.pending_rotation_segment_index.has_value());
  EXPECT_NEAR(terminal.current.position.x, 1.0, 1e-12);
  EXPECT_NEAR(terminal.current.target_yaw, 0.5 * kPi, 1e-12);
}

TEST(HolonomicTrackingReference, initialAndAllRotationPathsAdvanceInStrictOrder)
{
  const auto path = preparePath(
    makePath(
      {{0.3, -0.2, 0.0}, {0.3, -0.2, 0.25 * kPi},
        {0.3, -0.2, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto first_progress = tracker.update(makePoint(0.3, -0.2), 0.20);
  const auto first = buildHolonomicTrackingReference(*path, first_progress, 0.20);
  ASSERT_EQ(first.mode, HolonomicReferenceMode::kRotationPending);
  ASSERT_EQ(first.pending_rotation_segment_index, std::optional<std::size_t>(0u));
  EXPECT_NEAR(first.rotation_target_yaw, 0.25 * kPi, 1e-12);

  tracker.acknowledgeRotationComplete(0u);
  const auto second_progress = tracker.update(makePoint(0.3, -0.2), 0.20);
  const auto second = buildHolonomicTrackingReference(*path, second_progress, 0.20);
  ASSERT_EQ(second.mode, HolonomicReferenceMode::kRotationPending);
  ASSERT_EQ(second.pending_rotation_segment_index, std::optional<std::size_t>(1u));
  EXPECT_NEAR(second.rotation_target_yaw, 0.5 * kPi, 1e-12);

  tracker.acknowledgeRotationComplete(1u);
  const auto terminal_progress = tracker.update(makePoint(0.3, -0.2), 0.20);
  const auto terminal = buildHolonomicTrackingReference(
    *path, terminal_progress, 0.20);
  EXPECT_EQ(terminal_progress.cursor.segment_index, path->segments.size());
  EXPECT_EQ(terminal.mode, HolonomicReferenceMode::kTerminal);
  EXPECT_FALSE(terminal.pending_rotation_segment_index.has_value());
}

TEST(HolonomicTrackingReference, initialRotationBoundaryUsesRotationBeginYaw)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.25 * kPi},
        {1.0, 0.0, 0.25 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto progress = tracker.update(makePoint(0.0, 0.0), 0.20);
  const auto reference = buildHolonomicTrackingReference(*path, progress, 0.20);

  ASSERT_EQ(reference.pending_rotation_segment_index, std::optional<std::size_t>(0u));
  EXPECT_EQ(reference.preview.cursor.segment_index, 0u);
  EXPECT_NEAR(reference.preview.target_yaw, 0.0, 1e-12);
  EXPECT_NEAR(reference.rotation_target_yaw, 0.25 * kPi, 1e-12);
}

TEST(HolonomicTrackingReference, consecutiveFinalRotationsRemainPendingInOrder)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.25 * kPi}, {1.0, 0.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);

  const auto first_progress = tracker.update(makePoint(1.0, 0.0), 0.20);
  const auto first = buildHolonomicTrackingReference(*path, first_progress, 0.20);
  ASSERT_EQ(first.pending_rotation_segment_index, std::optional<std::size_t>(1u));

  tracker.acknowledgeRotationComplete(1u);
  const auto second_progress = tracker.update(makePoint(1.0, 0.0), 0.20);
  const auto second = buildHolonomicTrackingReference(*path, second_progress, 0.20);
  EXPECT_EQ(second_progress.cursor.segment_index, 2u);
  EXPECT_EQ(second.mode, HolonomicReferenceMode::kRotationPending);
  EXPECT_EQ(second.pending_rotation_segment_index, std::optional<std::size_t>(2u));
  EXPECT_GE(
    second.current.cursor.segment_index,
    second_progress.prepared_segment_lower_bound);
  EXPECT_GE(
    second.preview.cursor.segment_index,
    second_progress.prepared_segment_lower_bound);
  EXPECT_NEAR(second.current.target_yaw, 0.25 * kPi, 1e-12);
  EXPECT_NEAR(second.preview.target_yaw, 0.25 * kPi, 1e-12);
  EXPECT_NEAR(second.rotation_target_yaw, 0.5 * kPi, 1e-12);

  tracker.acknowledgeRotationComplete(2u);
  const auto terminal_progress = tracker.update(makePoint(1.0, 0.0), 0.20);
  const auto terminal = buildHolonomicTrackingReference(
    *path, terminal_progress, 0.20);
  EXPECT_EQ(terminal_progress.cursor.segment_index, path->segments.size());
  EXPECT_EQ(terminal.mode, HolonomicReferenceMode::kTerminal);
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

TEST(HolonomicTrackingReference, shadowConstructionPreservesLegacyReferenceAndPlan)
{
  const auto path = preparePath(
    makePath(
      {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.5 * kPi}, {1.0, 1.0, 0.5 * kPi}}));
  PathProgressTracker tracker;
  tracker.setPath(path);
  const auto robot_position = makePoint(0.80, 0.0);
  const auto before_reference = tracker.update(robot_position, 0.50);

  LegacyCarrotVelocityPlannerConfig config;
  config.model_k_v = 1.9;
  config.model_k_w = 2.1;
  config.characteristic_length = 0.2;
  config.cruise_speed = 0.8;
  config.max_angular_speed = 0.8;
  config.angular_speed_reserve = 0.1;
  const auto before_plan = planLegacyCarrotVelocity(
    before_reference.carrot.x - robot_position.x,
    before_reference.carrot.y - robot_position.y, config);

  const auto shadow = buildHolonomicTrackingReference(
    *path, before_reference, 0.50);
  const auto after_reference = tracker.update(robot_position, 0.50);
  const auto after_plan = planLegacyCarrotVelocity(
    after_reference.carrot.x - robot_position.x,
    after_reference.carrot.y - robot_position.y, config);

  ASSERT_TRUE(shadow.preview_clamped_by_rotation);
  EXPECT_NEAR(shadow.preview.position.x, 1.0, 1e-12);
  EXPECT_NEAR(shadow.preview.position.y, 0.0, 1e-12);
  EXPECT_NEAR(before_reference.carrot.x, 1.0, 1e-12);
  EXPECT_NEAR(before_reference.carrot.y, 0.30, 1e-12);
  EXPECT_EQ(
    after_reference.cursor.segment_index,
    before_reference.cursor.segment_index);
  EXPECT_NEAR(
    after_reference.cursor.segment_fraction,
    before_reference.cursor.segment_fraction, 1e-12);
  EXPECT_NEAR(
    after_reference.progress_arc_length,
    before_reference.progress_arc_length, 1e-12);
  EXPECT_NEAR(
    after_reference.carrot_arc_length,
    before_reference.carrot_arc_length, 1e-12);
  EXPECT_NEAR(after_reference.carrot.x, before_reference.carrot.x, 1e-12);
  EXPECT_NEAR(after_reference.carrot.y, before_reference.carrot.y, 1e-12);
  EXPECT_NEAR(after_reference.tangent, before_reference.tangent, 1e-12);
  EXPECT_NEAR(after_reference.curvature, before_reference.curvature, 1e-12);
  EXPECT_NEAR(
    after_reference.signed_lateral_error,
    before_reference.signed_lateral_error, 1e-12);
  EXPECT_EQ(after_plan.speed_limit_reason, before_plan.speed_limit_reason);
  EXPECT_NEAR(after_plan.reference_speed, before_plan.reference_speed, 1e-12);
  EXPECT_NEAR(after_plan.raw_linear_command, before_plan.raw_linear_command, 1e-12);
  EXPECT_NEAR(after_plan.raw_angular_command, before_plan.raw_angular_command, 1e-12);
}

}  // namespace
}  // namespace pid_tracking_controller
