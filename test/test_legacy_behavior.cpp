#include "gtest/gtest.h"

#include "controller_test_access.hpp"

namespace pid_tracking_controller
{

TEST(LegacyBehavior, modelControllerKeepsHandDerivedBodyMapping)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.0, 0.2);

  const Eigen::Vector2d output = controller.UseModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.3, 0.1, 1.2),
    Eigen::Vector2d(0.6, -0.4));

  EXPECT_NEAR(output.x(), 0.3, 1e-12);
  EXPECT_NEAR(output.y(), 0.5, 1e-12);
}

TEST(LegacyBehavior, lookaheadKeepsSpeedTimeClamp)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setLookaheadParameters(
    controller, 0.5, 0.3, 0.9);

  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.0), 0.3);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(0.8), 0.4);
  EXPECT_DOUBLE_EQ(controller.getLookAheadDistance(3.0), 0.9);
}

TEST(LegacyBehavior, noModelControllerKeepsFirstCyclePidOutput)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setNoModelParameters(controller, 0.1);

  const Eigen::Vector2d output = controller.NoModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.01, 0.0, 0.01),
    Eigen::Vector2d(0.05, 0.0));

  EXPECT_NEAR(output.x(), 0.20005, 1e-10);
  EXPECT_NEAR(output.y(), 0.2, 1e-10);
}

TEST(LegacyBehavior, regularizationKeepsMeasuredVelocityIncrementSemantics)
{
  PidTrackingController controller;
  geometry_msgs::msg::Twist measured;

  measured.linear.x = 0.1;
  EXPECT_DOUBLE_EQ(controller.linearRegularization(measured, 0.8), 0.6);

  measured.angular.z = -0.1;
  EXPECT_DOUBLE_EQ(controller.angularRegularization(measured, 0.2), 0.1);
}

}  // namespace pid_tracking_controller
