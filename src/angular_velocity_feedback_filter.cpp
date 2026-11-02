#include "angular_velocity_feedback_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace pid_tracking_controller
{
namespace
{
constexpr int64_t kMaxSampleAgeNs = 100000000LL;
}

AngularVelocityFeedbackFilter::AngularVelocityFeedbackFilter(double time_constant)
: time_constant_(time_constant)
{
  if (!std::isfinite(time_constant_) || time_constant_ <= 0.0) {
    throw std::invalid_argument("omega feedback filter time constant must be finite and positive");
  }
}

void AngularVelocityFeedbackFilter::update(double omega, int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(omega) || stamp_ns <= 0) {
    sample_.reset();
    return;
  }
  double filtered = omega;
  if (sample_ && stamp_ns > sample_->stamp_ns &&
    stamp_ns - sample_->stamp_ns <= kMaxSampleAgeNs)
  {
    const double dt = static_cast<double>(stamp_ns - sample_->stamp_ns) * 1e-9;
    const double alpha = -std::expm1(-dt / time_constant_);
    // Convex form avoids overflow in omega - previous for finite extreme inputs.
    filtered = (1.0 - alpha) * sample_->filtered + alpha * omega;
  }
  sample_ = AngularVelocityFeedbackSample{omega, filtered, stamp_ns};
}

void AngularVelocityFeedbackFilter::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  sample_.reset();
}

std::optional<AngularVelocityFeedbackSample> AngularVelocityFeedbackFilter::sample(
  int64_t now_ns) const
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !sample_ || now_ns < sample_->stamp_ns ||
    now_ns - sample_->stamp_ns > kMaxSampleAgeNs)
  {
    return std::nullopt;
  }
  return sample_;
}

}  // namespace pid_tracking_controller
