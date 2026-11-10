# Observation only: no motion authorization. Ordered index into the ORIGINAL path.
std_msgs/Header header
string controller_instance
uint64 plan_revision
std_msgs/Header path_header
uint64 path_fingerprint
builtin_interfaces/Time loaded_at
bool valid
uint64 segment_index
float64 segment_fraction
bool rotating
geometry_msgs/PoseStamped robot_pose
