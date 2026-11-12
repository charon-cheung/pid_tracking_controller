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
