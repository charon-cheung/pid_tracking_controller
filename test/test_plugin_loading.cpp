#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"

TEST(PidTrackingControllerPlugin, loadsFromNav2CoreRegistry)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> controller;
  EXPECT_NO_THROW(
    controller = loader.createSharedInstance(
      "pid_tracking_controller::PidTrackingController"));
  ASSERT_NE(controller, nullptr);
}
