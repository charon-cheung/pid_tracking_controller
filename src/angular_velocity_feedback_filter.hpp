#ifndef PID_TRACKING_CONTROLLER__ANGULAR_VELOCITY_FEEDBACK_FILTER_HPP_
#define PID_TRACKING_CONTROLLER__ANGULAR_VELOCITY_FEEDBACK_FILTER_HPP_

#include <cstdint>
#include <mutex>
#include <optional>

namespace pid_tracking_controller
{

struct AngularVelocityFeedbackSample
{
  double raw{0.0};
  double filtered{0.0};
  int64_t stamp_ns{0};
};

// Updated by odometry callbacks, read without waiting by the control loop.
class AngularVelocityFeedbackFilter
{
public:
  explicit AngularVelocityFeedbackFilter(double time_constant);
  void update(double omega, int64_t stamp_ns);
  void reset();
  std::optional<AngularVelocityFeedbackSample> sample(int64_t now_ns) const;

private:
  const double time_constant_;
  mutable std::mutex mutex_;
  std::optional<AngularVelocityFeedbackSample> sample_;
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__ANGULAR_VELOCITY_FEEDBACK_FILTER_HPP_
