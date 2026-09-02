#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "controller_test_access.hpp"
#include "nav2_core/controller_exceptions.hpp"

namespace pid_tracking_controller
{

TEST(LegacyBehavior, modelControllerUsesIndependentCruiseSpeed)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.7, 1.9, 0.2);
  PidTrackingControllerTestAccess::setModelSpeedParameters(controller, 0.51, 0.1, 0.6);

  const Eigen::Vector2d output = controller.UseModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.3, 0.0, 1.2),
    Eigen::Vector2d(0.6, -0.4));

  EXPECT_NEAR(output.x(), 0.51, 1e-12);
  EXPECT_NEAR(output.y(), 0.0, 1e-12);
}

TEST(LegacyBehavior, modelControllerLimitsAngularDemandByScalingBothCommands)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.7, 1.9, 0.2);
  PidTrackingControllerTestAccess::setModelSpeedParameters(controller, 0.51, 0.1, 0.6);

  const Eigen::Vector2d output = controller.UseModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.2819077862357725, 0.1026060429977006, 0.0),
    Eigen::Vector2d::Zero());

  EXPECT_NEAR(output.x(), 0.245826927003835, 1e-12);
  EXPECT_NEAR(output.y(), 0.5, 1e-12);
}

TEST(LegacyBehavior, splitModelGainsAreInvariantToWorldFrameRotation)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.7, 1.9, 0.2);
  PidTrackingControllerTestAccess::setModelSpeedParameters(controller, 0.51, 0.1, 0.6);
  const double half_pi = std::acos(-1.0) / 2.0;

  const Eigen::Vector2d unrotated = controller.UseModelPidControl(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.3, 0.1, 0.0),
    Eigen::Vector2d::Zero());
  const Eigen::Vector2d rotated = controller.UseModelPidControl(
    Eigen::Vector3d(1.0, -2.0, half_pi),
    Eigen::Vector3d(0.9, -1.7, half_pi),
    Eigen::Vector2d::Zero());

  EXPECT_NEAR(rotated.x(), unrotated.x(), 1e-12);
  EXPECT_NEAR(rotated.y(), unrotated.y(), 1e-12);
}

TEST(LegacyBehavior, nonFiniteModelIntermediateThrowsNoValidControl)
{
  PidTrackingController controller;
  PidTrackingControllerTestAccess::setModelParameters(controller, 1.7, 1.9, 0.2);
  PidTrackingControllerTestAccess::setModelSpeedParameters(controller, 0.51, 0.1, 0.6);
  const double maximum = std::numeric_limits<double>::max();

  EXPECT_THROW(
    controller.UseModelPidControl(
      Eigen::Vector3d(-maximum, 0.0, 0.0),
      Eigen::Vector3d(maximum, 0.0, 0.0),
      Eigen::Vector2d::Zero()),
    nav2_core::NoValidControl);
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
