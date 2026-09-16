#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rotation_entry_gate.hpp"

namespace pid_tracking_controller
{
namespace
{

TEST(RotationEntryGate, requiresBothCommandAndMeasuredStopForAContinuousInterval)
{
  for (const HolonomicBodyCommand moving : {
    HolonomicBodyCommand{0.04, 0.04, 0.0}, {0.0, -0.06, 0.0}, {0.0, 0.0, -0.06}})
  {
    for (const bool measured_moves : {false, true}) {
      RotationEntryGate gate({0.05, 0.05, 0.2, 8.0});
      gate.observeBraking({}, {}, 0.1);  // First observation provides no stable interval.
      gate.observeBraking({}, {}, 0.1);
      gate.observeBraking(measured_moves ? moving : HolonomicBodyCommand{},
        measured_moves ? HolonomicBodyCommand{} : moving, 0.1);
      EXPECT_FALSE(gate.beginCycle({}));
      gate.observeBraking({}, {}, 0.1);
      gate.observeBraking({}, {}, 0.1);
      EXPECT_FALSE(gate.beginCycle({}));
      gate.observeBraking({}, {}, 0.1);
      EXPECT_TRUE(gate.beginCycle({}));
      EXPECT_TRUE(gate.beginCycle({0.0, 0.0, 0.4}));  // Rotation does not re-arm its own gate.
    }
  }
}

TEST(RotationEntryGate, latestMovingMeasurementRevokesPermissionBeforeFirstTurn)
{
  RotationEntryGate gate({0.05, 0.05, 0.2, 8.0});
  for (int i = 0; i < 3; ++i) {
    gate.observeBraking({}, {}, 0.1);
  }
  EXPECT_FALSE(gate.beginCycle({0.1, 0.0, 0.0}));
  gate.observeBraking({}, {}, 0.1);
  gate.observeBraking({}, {}, 0.1);
  EXPECT_FALSE(gate.beginCycle({}));
  gate.observeBraking({}, {}, 0.1);
  EXPECT_TRUE(gate.beginCycle({}));
}

TEST(RotationEntryGate, replansPreserveBrakingButDiscontinuitiesAndNewRotationsReconfirm)
{
  RotationEntryGate gate({0.05, 0.05, 0.2, 8.0});
  gate.observeBraking({}, {}, 0.1);
  gate.observeBraking({}, {}, 0.1);
  gate.reset(true);
  gate.observeBraking({}, {}, 0.1);
  EXPECT_TRUE(gate.beginCycle({}));
  gate.reset(true);  // A new rotation after admission must brake again.
  EXPECT_FALSE(gate.beginCycle({}));
  gate.observeBraking({}, {}, 0.1);
  gate.observeBraking({}, {}, 0.1);
  gate.reset();  // A control gap discards all previous stop evidence.
  gate.observeBraking({}, {}, 0.0);
  gate.observeBraking({}, {}, 0.1);
  EXPECT_FALSE(gate.beginCycle({}));
  gate.observeBraking({}, {}, 0.1);
  EXPECT_TRUE(gate.beginCycle({}));
}

TEST(RotationEntryGate, cannotWaitForeverOrHideTimeoutWithReplans)
{
  RotationEntryGate gate({0.05, 0.05, 0.2, 1.0});
  gate.observeBraking({0.1, 0.0, 0.0}, {}, 0.0);
  for (int i = 0; i < 3; ++i) {
    gate.reset(true);
    gate.observeBraking({0.1, 0.0, 0.0}, {}, 0.25);
  }
  EXPECT_THROW(gate.observeBraking({}, {}, 0.25), std::runtime_error);
  EXPECT_FALSE(gate.beginCycle({}));
}

TEST(RotationEntryGate, rejectsInvalidConfigurationAndInputs)
{
  for (const RotationEntryGateConfig config : {
    RotationEntryGateConfig{-0.1, 0.05, 0.2, 8.0}, {0.05, -0.1, 0.2, 8.0},
    {0.05, 0.05, 0.0, 8.0}, {0.05, 0.05, 0.2, 0.2},
    {0.05, 0.05, 0.2, std::numeric_limits<double>::infinity()}})
  {
    EXPECT_THROW(RotationEntryGate gate(config), std::invalid_argument);
  }
  RotationEntryGate gate({0.05, 0.05, 0.2, 8.0});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(gate.beginCycle({nan, 0.0, 0.0}), std::domain_error);
  EXPECT_THROW(gate.observeBraking({}, {0.0, nan, 0.0}, 0.1), std::domain_error);
  EXPECT_THROW(gate.observeBraking({}, {}, -0.1), std::domain_error);
  EXPECT_THROW(gate.observeBraking({}, {}, nan), std::domain_error);
}

}  // namespace
}  // namespace pid_tracking_controller
