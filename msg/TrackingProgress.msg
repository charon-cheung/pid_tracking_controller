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
# Observation of the gate's selected output, not proof of physical standstill.
# Retained when valid becomes false so action termination cannot hide a safety stop.
bool safety_inhibited
# Explicit execution end (reset/deactivate), distinct from temporary invalid progress.
# Does not assert physical standstill; MotionHandoff remains authoritative.
bool execution_finished
