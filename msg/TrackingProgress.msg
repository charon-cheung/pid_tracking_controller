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
# Latched obstacle cause survives a later input failure and action termination.
uint8 SAFETY_NONE=0
uint8 SAFETY_INPUT_UNAVAILABLE=1
uint8 SAFETY_OBSTACLE=2
uint8 safety_stop_cause
# Nominal gate standstill with a complete current safety check, not physical proof.
# BT must still finish action termination and MotionHandoff before taking ownership.
bool safety_stopped
