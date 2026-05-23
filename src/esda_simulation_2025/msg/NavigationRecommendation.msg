std_msgs/Header header
#string source           # "follow the gap", "track follower"

# Node source
uint8 FOLLOW_THE_GAP = 1
uint8 TRACK_FOLLOWER = 2
uint8 BEHAVIOUR_TREE = 3

uint8 source

# Reason
uint8 NO_OBSTACLE = 1   # e.g. We want to use the track follower
uint8 OBSTACLE_DETECTED = 2 # e.g. We ideally want to use follow the gap
uint8 SAFE_TO_PROCEED = 3 # e.g. Can continue
uint8 UNSAFE_TO_PROCEED = 4 # e,g, Can't continue as it is unsafe
uint8 BOTH_LANES_NORMAL_LANES_SHIFTED_RIGHT = 5
uint8 BOTH_LANES_NORMAL_LANES_SHIFTED_LEFT = 6
uint8 BOTH_LANES_NORMAL_LANES_CENTRED = 7

uint8 BOTH_LANES_TOO_NARROW_SHIFTED_RIGHT = 8
uint8 BOTH_LANES_TOO_NARROW_SHIFTED_LEFT = 9
uint8 BOTH_LANES_TOO_NARROW_CENTRED = 10

uint8 BOTH_LANES_TOO_WIDE_SHIFTED_RIGHT = 11
uint8 BOTH_LANES_TOO_WIDE_SHIFTED_LEFT = 12
uint8 BOTH_LANES_TOO_WIDE_CENTRED = 13

uint8 ONLY_LEFT_LANE_VISIBLE = 14
uint8 ONLY_RIGHT_LANE_VISIBLE = 15
uint8 NO_LANES_VISIBLE = 16

uint8 INVALID_LANE_PAIR = 17

uint8 reason


uint8 mode              # e.g. ENUMs that correspond to the behaviour tree

bool valid              # e.g. is this usable? true or false

float32 confidence      # 0.0 -> 1.0
float32 linear_x
float32 angular_z

# string reason