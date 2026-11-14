# Fixed poses shared by all parts. Source of truth: DB.
# Control reads them with GetJobData and sends them in every Action Goal.
bool available           # false = not provided (parts fall back to box_comm/common.py defaults)
float64[6] home_j        # HOME joint angles [deg]
float64[3] measure_zone  # measure zone position x, y, z [mm, robot base frame]
