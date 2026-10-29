"""Keep the tests' walking statistics (fake robot, sim) out of robot_dump/walk_stats.jsonl, the real robot's record."""
import os
import tempfile

_tmp = tempfile.mkdtemp(prefix="yanshee_stats_")
os.environ.setdefault("YANSHEE_WALK_STATS", os.path.join(_tmp, "walk_stats.jsonl"))
os.environ.setdefault("YANSHEE_DUMP_DIR", _tmp)  # the sim-to-real log of fake-robot sessions
