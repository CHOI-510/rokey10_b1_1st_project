# One event log line. Published by control on topic /event_log (fire-and-forget).
# DB subscribes, stores it and shows it in UI. Parts never publish this directly:
# they send Feedback with log_msg, and control forwards it.
int32 seq           # 1, 2, 3 ... per job_id (11/18). DB skips lines it already wrote (QoS replays last 100)
string stamp        # "2026-11-16 14:03:12" (control local time)
string job_id       # control job id, e.g. "20261116_140012"
string source       # "control" / "scan" / "judge" / "place"
string stage        # part stage at that moment ("" for control events)
string level        # "INFO" / "WARN" / "ERROR"
string code         # see box_comm/common.py LOG_CODES
string box_id       # "" if not related to a box
string class_id     # "" if unknown
int32 expected      # -1 = not used (COUNT_MISMATCH only)
int32 measured      # -1 = not used (COUNT_MISMATCH only)
string message      # human-readable line, shown as is in UI
