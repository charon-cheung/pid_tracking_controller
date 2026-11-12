#ifndef PID_TRACKING_CONTROLLER_TEST__DIRECT_COSTMAP_FIXTURE_HPP_
#define PID_TRACKING_CONTROLLER_TEST__DIRECT_COSTMAP_FIXTURE_HPP_
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
namespace pid_tracking_controller
{
// A deterministic layer drives the real LayeredCostmap update/initialization path.
class TestCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  TestCostmapLayer() {current_ = true; enabled_ = true;}
  void setCurrent(bool value) {current_ = value;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double * x0, double * y0, double * x1, double * y1) override
  {*x0 = *y0 = -100.; *x1 = *y1 = 100.;}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
};
}  // namespace pid_tracking_controller
#endif
