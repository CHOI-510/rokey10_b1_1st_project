# Place zone of one class. Source of truth: DB.
# x, y = 1st tier (innermost column) position [mm, robot base frame].
# Tiers grow in +y (tier n: y + (n-1) * pitch). Pitch, z and heights live only in the place part.
string class_id        # "A" / "B" / "C" / "D"
float64 x
float64 y
