# Atomic direct-route Planner -> Pure Pursuit sample.
# The legacy Trajectory topic remains available for visualization, but control
# consumers use this message so geometry and behavior mode cannot be observed
# from different planner generations.
std_msgs/Header header
uint64 generation
# Semantic execution identity. generation is the wire publication sequence;
# these fields remain stable across republishes of the same corridor owner and
# change only when Planner commits new geometry.
uint64 owner_commit_id
uint64 geometry_revision
float64 valid_until_sec
float32 remaining_arc_m
string<=64 mode
bool emergency_stop
string<=128 reason
# Execution envelope for any fully validated, committed pass corridor. These
# fields are atomic with trajectory so PP never applies clearance from another
# planner generation. corridor_side is -1 for right and +1 for left.
bool corridor_clearance_profile
int8 corridor_side
# Backward-compatible classification of the wall-following d(s) subset.
bool wall_edge_profile
int8 wall_side
float32 wall_clearance_reserve_m
float32 opponent_clearance_reserve_m
float32 corridor_clearance_reserve_m
float32 desired_control_reserve_m
autoware_auto_planning_msgs/Trajectory trajectory
