#!/usr/bin/env python3
"""Run an existing YanAPI script against the simulator, unmodified.

    python3 scripts/run_in_sim.py ../"yanshee robot"/yanshee/yanshee_examples/yanshee_examples/0custom0/singleJoint.py
    YANSHEE_SIM_FIXED=1 python3 scripts/run_in_sim.py my_show.py      # torso held in the air

Both `import YanAPI` and `from YansheeRobot import YanAPI` are redirected to yanshee_sim.yanapi_sim.
"""
import os
import runpy
import sys
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import yanapi_sim  # noqa: E402


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    script = os.path.abspath(sys.argv[1])
    sys.modules["YanAPI"] = yanapi_sim
    pkg = types.ModuleType("YansheeRobot")
    pkg.YanAPI = yanapi_sim
    sys.modules["YansheeRobot"] = pkg
    sys.modules["YansheeRobot.YanAPI"] = yanapi_sim
    sys.argv = sys.argv[1:]
    try:
        runpy.run_path(script, run_name="__main__")
    except KeyboardInterrupt:
        pass
    bot = yanapi_sim._robot
    if bot is not None and bot._viewer is not None:
        print("script finished; close the viewer window to exit")
        while bot.is_running():
            time.sleep(0.2)
    if bot is not None:
        bot.close()


if __name__ == "__main__":
    main()
