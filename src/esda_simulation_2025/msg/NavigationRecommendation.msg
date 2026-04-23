std_msgs/Header header
string source           # "follow the gap", "track follower"

uint8 mode              # e.g. ENUMs that correspond to the behaviour tree

bool valid              # e.g. is this usable? true or false

float32 confidence      # 0.0 -> 1.0
float32 linear_x
float32 angular_z

string reason