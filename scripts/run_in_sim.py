#!/usr/bin/env python3
"""Run an existing YanAPI script against the simulator, unmodified.

    python3 scripts/run_in_sim.py path/to/script.py [script args]
    python3 scripts/run_in_sim.py ../"yanshee robot"/yanshee/yanshee_examples/yanshee_examples/0custom0/singleJoint.py
    YANSHEE_SIM_FIXED=1 python3 scripts/run_in_sim.py my_show.py      # torso held in the air
    YANSHEE_SIM_HEADLESS=1 python3 scripts/run_in_sim.py my_show.py   # no window

Both `import YanAPI` and `from YansheeRobot import YanAPI` are redirected to yanshee_sim.yanapi_sim.
The script runs as `python3 script.py` would: its folder is on sys.path and the exit code is its own
(1 after an uncaught exception, 130 after Ctrl+C). The simulator is always closed on the way out.
"""
import os
import runpy
import sys
import time
import traceback
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from yanshee_sim import yanapi_sim  # noqa: E402


def _install_shim():
    sys.modules["YanAPI"] = yanapi_sim
    pkg = types.ModuleType("YansheeRobot")
    pkg.YanAPI = yanapi_sim
    sys.modules["YansheeRobot"] = pkg
    sys.modules["YansheeRobot.YanAPI"] = yanapi_sim


def _wait_for_viewer():
    bot = yanapi_sim._robot
    if bot is not None and bot._viewer is not None:
        print("script finished; close the viewer window to exit", file=sys.stderr)
        while bot.is_running():
            time.sleep(0.2)


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0 if len(sys.argv) >= 2 else 2
    script = os.path.abspath(sys.argv[1])
    if not os.path.isfile(script):
        print(f"run_in_sim: no such script: {sys.argv[1]}", file=sys.stderr)
        return 2
    _install_shim()
    sys.path.insert(0, os.path.dirname(script))  # like `python3 script.py` (YanAPI.py next to it stays shadowed)
    sys.argv = sys.argv[1:]
    code, interrupted = 0, False
    try:
        try:
            runpy.run_path(script, run_name="__main__")
        except SystemExit as e:
            code = e.code
        except KeyboardInterrupt:
            code, interrupted = 130, True
        except BaseException:
            traceback.print_exc()
            code = 1
        if not interrupted:
            _wait_for_viewer()
    except KeyboardInterrupt:
        pass
    finally:
        yanapi_sim._shutdown()  # stop playback and the physics thread before the interpreter tears down
    return code


if __name__ == "__main__":
    sys.exit(main())
