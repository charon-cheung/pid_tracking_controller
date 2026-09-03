#ifndef PID_TRACKING_CONTROLLER__PATH_PROGRESS_TRACKER_HPP_
#define PID_TRACKING_CONTROLLER__PATH_PROGRESS_TRACKER_HPP_

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "prepared_path.hpp"

namespace pid_tracking_controller
{

struct PathProgressReference
{
  geometry_msgs::msg::Point projection;
  geometry_msgs::msg::Point carrot;
  std::string frame_id;
  PathCursor cursor;
  std::size_t segment_index{0};
  double segment_fraction{0.0};
  double progress_arc_length{0.0};
  double carrot_arc_length{0.0};
  double remaining_path_length{0.0};
  double progress_budget{0.0};
  double tangent{0.0};
  double curvature{0.0};
  double signed_lateral_error{0.0};
};

class PathProgressTracker
{
public:
  void setPath(PreparedPathPtr path);

  PathProgressReference update(
    const geometry_msgs::msg::Point & robot_position,
    double lookahead_distance);

  bool hasPath() const;

private:
  PreparedPathPtr path_;
  double progress_arc_length_{0.0};
  double progress_budget_{0.0};
  double maximum_progress_budget_{0.0};
  geometry_msgs::msg::Point previous_robot_position_;
  double previous_accepted_tangent_x_{0.0};
  double previous_accepted_tangent_y_{0.0};
  bool previous_robot_position_valid_{false};
  bool previous_accepted_tangent_valid_{false};
};

}  // namespace pid_tracking_controller

#endif  // PID_TRACKING_CONTROLLER__PATH_PROGRESS_TRACKER_HPP_
