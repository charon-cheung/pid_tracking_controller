// Offline visualization of the production nominal model, NOT collision checking.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "motion_transition.hpp"

using namespace pid_tracking_controller;

int main()
{
  MotionTransitionInput in;
  double delay, dt;
  int resume;
  const auto read = [](HolonomicBodyCommand & v) {
      return bool(std::cin >> v.vx >> v.vy >> v.omega);
    };
  if (!read(in.measured) || !read(in.candidate) || !read(in.acceleration) ||
    !read(in.deceleration) || !read(in.braking_deceleration) ||
    !(std::cin >> delay >> in.candidate_duration >> dt >> resume) ||
    !std::isfinite(dt) || dt < 0.0001 || dt > 0.1 || (resume != 0 && resume != 1))
  {std::cerr << "invalid input\n"; return 2;}
  std::string extra;
  if (std::cin >> extra) {std::cerr << "unexpected input\n"; return 2;}
  if (resume) {
    const auto ramp = [](double a, double b, double up, double down) {
        return a * b < 0 ? std::abs(a) / down + std::abs(b) / up :
               std::abs(b - a) / (std::abs(b) > std::abs(a) ? up : down);
      };
    in.candidate_duration += std::max({
      ramp(in.measured.vx, in.candidate.vx, in.acceleration.vx, in.deceleration.vx),
      ramp(in.measured.vy, in.candidate.vy, in.acceleration.vy, in.deceleration.vy),
      ramp(in.measured.omega, in.candidate.omega,
        in.acceleration.omega, in.deceleration.omega)});
  }
  in.active_command = in.measured;
  in.active_valid_for = delay;
  in.candidate_execution = {delay, delay};
  const auto model = buildMotionTransition(in);
  if (!model.available) {std::cerr << model.reason << '\n'; return 2;}
  double duration = 0;
  for (const auto & s : model.segments) {duration += s.duration;}
  if (duration / dt > 1000000) {std::cerr << "sample limit\n"; return 2;}
  double t = 0, x = 0, y = 0, yaw = 0, distance = 0;
  std::cout << std::setprecision(12) << "time,x,y,yaw,distance,vx,vy,omega,phase\n";
  const auto emit = [&](const HolonomicBodyCommand & v, SweepPhase phase) {
      std::cout << t << ',' << x << ',' << y << ',' << yaw << ',' << distance << ',' <<
        v.vx << ',' << v.vy << ',' << v.omega << ',' << static_cast<int>(phase) << '\n';
    };
  emit(in.measured, SweepPhase::kInitial);
  for (const auto & s : model.segments) {
    const auto count = std::max(1, static_cast<int>(std::ceil(s.duration / dt)));
    const double h = s.duration / count;
    const auto velocity = [&](double u) {
        return HolonomicBodyCommand{
          s.start.vx + (s.end.vx - s.start.vx) * u,
          s.start.vy + (s.end.vy - s.start.vy) * u,
          s.start.omega + (s.end.omega - s.start.omega) * u};
      };
    for (int i = 0; i < count; ++i) {
      const auto a = velocity(double(i) / count), m = velocity((i + 0.5) / count);
      const auto b = velocity(double(i + 1) / count);
      const double mid_yaw = yaw + (a.omega + m.omega) * h / 4;
      x += h * (std::cos(mid_yaw) * m.vx - std::sin(mid_yaw) * m.vy);
      y += h * (std::sin(mid_yaw) * m.vx + std::cos(mid_yaw) * m.vy);
      yaw += (a.omega + b.omega) * h / 2;
      distance += std::hypot(m.vx, m.vy) * h;
      t += h;
      emit(b, s.phase);
    }
  }
}
