# Fixed poses shared by all parts.
# Source of truth: box_comm/common.py FIXED_POSES. DB keeps a copy;
# control compares the two before starting (preflight).
bool available         # false if DB has no stored poses
float64[6] home_j      # HOME joint angles [deg]
float64[3] measure_zone  # measure zone position x, y, z [mm, robot base frame]
