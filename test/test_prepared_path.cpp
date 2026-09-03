#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "prepared_path.hpp"

namespace pid_tracking_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion makeQuaternion(double yaw, double scale = 1.0)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = scale * std::sin(0.5 * yaw);
  quaternion.w = scale * std::cos(0.5 * yaw);
  return quaternion;
}

void appendPose(
  nav_msgs::msg::Path & path,
  double x,
  double y,
  double yaw,
  double quaternion_scale = 1.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = makeQuaternion(yaw, quaternion_scale);
  path.poses.push_back(pose);
}

nav_msgs::msg::Path makePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  return path;
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

TEST(PreparedPath, preparesTranslationGeometryAndCumulativeArcLength)
{
  auto path = makePath();
  appendPose(path, 0.0, 0.0, 0.0, 2.0);
  appendPose(path, 1.0, 0.0, 0.5 * kPi);
  appendPose(path, 1.0, 2.0, kPi);

  const auto prepared = preparePath(path);

  ASSERT_NE(prepared, nullptr);
  ASSERT_EQ(prepared->segments.size(), 2u);
  EXPECT_EQ(prepared->frame_id, "map");
  EXPECT_NEAR(prepared->total_translation_length, 3.0, 1e-12);
  EXPECT_NEAR(quaternionNorm(prepared->poses.front().pose.orientation), 1.0, 1e-12);

  const auto & first = prepared->segments[0];
  EXPECT_EQ(first.type, PathSegmentType::kTranslation);
  EXPECT_EQ(first.start_pose_index, 0u);
  EXPECT_EQ(first.end_pose_index, 1u);
  EXPECT_NEAR(first.translation_s_begin, 0.0, 1e-12);
  EXPECT_NEAR(first.translation_s_end, 1.0, 1e-12);
  EXPECT_NEAR(first.translation_length, 1.0, 1e-12);
  EXPECT_NEAR(first.path_tangent, 0.0, 1e-12);
  EXPECT_NEAR(first.target_yaw_begin, 0.0, 1e-12);
  EXPECT_NEAR(first.target_yaw_delta, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(first.target_yaw_rate_per_meter, 0.5 * kPi, 1e-12);

  const auto & second = prepared->segments[1];
  EXPECT_EQ(second.type, PathSegmentType::kTranslation);
  EXPECT_EQ(second.start_pose_index, 1u);
  EXPECT_EQ(second.end_pose_index, 2u);
  EXPECT_NEAR(second.translation_s_begin, 1.0, 1e-12);
  EXPECT_NEAR(second.translation_s_end, 3.0, 1e-12);
  EXPECT_NEAR(second.translation_length, 2.0, 1e-12);
  EXPECT_NEAR(second.path_tangent, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(second.target_yaw_begin, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(second.target_yaw_delta, 0.5 * kPi, 1e-12);
  EXPECT_NEAR(second.target_yaw_rate_per_meter, 0.25 * kPi, 1e-12);
}

TEST(PreparedPath, usesShortestYawDeltaAcrossPi)
{
  auto path = makePath();
  appendPose(path, 0.0, 0.0, 179.0 * kPi / 180.0);
  appendPose(path, 1.0, 0.0, -179.0 * kPi / 180.0);

  const auto prepared = preparePath(path);

  ASSERT_EQ(prepared->segments.size(), 1u);
  const double expected_delta = 2.0 * kPi / 180.0;
  EXPECT_NEAR(prepared->segments.front().target_yaw_delta, expected_delta, 1e-12);
  EXPECT_NEAR(
    prepared->segments.front().target_yaw_rate_per_meter,
    expected_delta,
    1e-12);
}

TEST(PreparedPath, classifiesRotationAndIgnoresNoOpWithSourceProvenance)
{
  auto path = makePath();
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.0, 0.0, 0.5 * kPi);
  appendPose(path, 1.0, 0.0, 0.5 * kPi);

  const auto prepared = preparePath(path);

  ASSERT_EQ(prepared->segments.size(), 2u);
  const auto & rotation = prepared->segments[0];
  EXPECT_EQ(rotation.type, PathSegmentType::kRotation);
  EXPECT_EQ(rotation.start_pose_index, 1u);
  EXPECT_EQ(rotation.end_pose_index, 2u);
  EXPECT_DOUBLE_EQ(rotation.translation_s_begin, 0.0);
  EXPECT_DOUBLE_EQ(rotation.translation_s_end, 0.0);
  EXPECT_DOUBLE_EQ(rotation.translation_length, 0.0);
  EXPECT_DOUBLE_EQ(rotation.path_tangent, 0.0);
  EXPECT_NEAR(rotation.target_yaw_delta, 0.5 * kPi, 1e-12);
  EXPECT_DOUBLE_EQ(rotation.target_yaw_rate_per_meter, 0.0);

  const auto & translation = prepared->segments[1];
  EXPECT_EQ(translation.type, PathSegmentType::kTranslation);
  EXPECT_EQ(translation.start_pose_index, 2u);
  EXPECT_EQ(translation.end_pose_index, 3u);
  EXPECT_DOUBLE_EQ(translation.translation_s_begin, 0.0);
  EXPECT_DOUBLE_EQ(translation.translation_s_end, 1.0);
  EXPECT_DOUBLE_EQ(prepared->total_translation_length, 1.0);
}

TEST(PreparedPath, preservesSinglePoseAndFallsBackToPoseFrame)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = -0.1;
  pose.pose.orientation = makeQuaternion(0.4, 0.98);
  path.poses.push_back(pose);

  const auto prepared = preparePath(path);

  ASSERT_NE(prepared, nullptr);
  EXPECT_EQ(prepared->frame_id, "odom");
  ASSERT_EQ(prepared->poses.size(), 1u);
  EXPECT_TRUE(prepared->segments.empty());
  EXPECT_DOUBLE_EQ(prepared->total_translation_length, 0.0);
  EXPECT_NEAR(quaternionNorm(prepared->poses.front().pose.orientation), 1.0, 1e-12);
}

TEST(PreparedPath, rejectsEmptyPathAndInvalidPoseData)
{
  EXPECT_THROW(preparePath(nav_msgs::msg::Path()), std::invalid_argument);

  auto invalid_position = makePath();
  appendPose(invalid_position, 0.0, 0.0, 0.0);
  invalid_position.poses.front().pose.position.x =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(preparePath(invalid_position), std::invalid_argument);

  auto nonfinite_quaternion = makePath();
  appendPose(nonfinite_quaternion, 0.0, 0.0, 0.0);
  nonfinite_quaternion.poses.front().pose.orientation.w =
    std::numeric_limits<double>::infinity();
  EXPECT_THROW(preparePath(nonfinite_quaternion), std::invalid_argument);

  auto zero_quaternion = makePath();
  appendPose(zero_quaternion, 0.0, 0.0, 0.0);
  auto & quaternion = zero_quaternion.poses.front().pose.orientation;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;
  quaternion.w = 0.0;
  EXPECT_THROW(preparePath(zero_quaternion), std::invalid_argument);
}

}  // namespace
}  // namespace pid_tracking_controller
