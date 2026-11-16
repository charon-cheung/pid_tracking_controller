# Path observer input for conditional motion checking, not motion authorization.
# Identity and source stamps must match the current loaded path and remain fresh.
uint8 UNAVAILABLE=0
uint8 CLEAR=1
uint8 CLEARANCE_LOW=2
uint8 COLLISION_AHEAD=3
std_msgs/Header header
std_msgs/Header path_header
string controller_instance
uint64 plan_revision
uint64 path_fingerprint
builtin_interfaces/Time progress_stamp
builtin_interfaces/Time map_stamp
uint8 state
# Distance along the checked path to the first red envelope, measured from progress_stamp.
# risk_pose is in header.frame_id; it is the robot centre at that envelope, not an obstacle point.
float64 risk_distance
geometry_msgs/Point risk_pose

# Additional evidence from the actual supplied polygon, without envelope padding.
# Shares the path identity, map and progress timestamps above. False is no evidence.
bool footprint_collision
float64 footprint_distance
geometry_msgs/Point footprint_pose
builtin_interfaces/Time footprint_origin_stamp
