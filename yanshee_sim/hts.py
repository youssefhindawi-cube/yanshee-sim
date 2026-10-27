"""Yanshee .hts motion files (the robot's own v1 keyframe format) <-> this package's motion dicts.

Layout (from UBTECH's 2018 Yanshee-SDK yanshee_api.c and BehaviorEditor yansheehts.cpp, checked against 46
app-made files): a 33-byte all-zero header, one 33-byte record per frame, and a 33-byte trailer whose last
four bytes are the total time in ms (little-endian). A frame record is

    0xFB 0xBF | kind | position | frames LE16 | index LE16 | 17 angles | 3 reserved | run | total BE16 | sum | 0xED

kind 38 in app and editor files (1 in the 2018 SDK), position 1 first / 2 middle / 3 last, angles in servo
degrees in YanAPI order (servo 1..17, 0xFF = leave that servo alone), run = move time in 20 ms ticks (one
byte), total = move + wait time in 20 ms ticks, sum = the low byte of the sum of bytes 2..30. The robot
needs at least 200 ms per frame and the editor says it will not run a frame over 4000 ms.

    python3 -m yanshee_sim.hts FILE.hts            # print the frames
    python3 -m yanshee_sim.hts motion.json out.hts  # convert (either direction, by extension)

Built-in motions live on the robot (ssh pi@ROBOT: the ROS firmware keeps user uploads in
/home/pi/Documents/motions); there is no REST download, so copy them with scp.
"""
import os
import struct
import sys

from .servos import NAMES, RESET_POSE

RECORD = 33
TICK_MS = 20
KIND_APP = 38
MAX_RUN_TICKS = 200  # 4000 ms, the editor's limit (the byte could hold 5100 ms)
RESET_FRAME_MS = 1000  # the closing reset frame written for motions with end_in_reset


class HtsError(ValueError):
    pass


def parse(data, name="motion"):
    """Motion dict from the bytes of an .hts file. Raises HtsError on a malformed file."""
    if len(data) < 3 * RECORD or len(data) % RECORD:
        raise HtsError(f"{name}: size {len(data)} is not a multiple of {RECORD} bytes with header and trailer")
    n = len(data) // RECORD - 2
    frames, total = [], 0
    for i in range(1, n + 1):
        f = data[i * RECORD:(i + 1) * RECORD]
        if f[0] != 0xFB or f[1] != 0xBF or f[32] != 0xED:
            raise HtsError(f"{name}: frame {i} has bad markers {f[0]:02x} {f[1]:02x} .. {f[32]:02x}")
        if sum(f[2:31]) & 0xFF != f[31]:
            raise HtsError(f"{name}: frame {i} checksum mismatch")
        run, tot = f[28], (f[29] << 8) | f[30]
        angles = {n: a for n, a in zip(NAMES, f[8:25]) if a != 0xFF}
        frames.append({"angles": angles, "runtime": run * TICK_MS, "hold": max(tot - run, 0) * TICK_MS})
        total += tot * TICK_MS
    trailer_ms = struct.unpack("<I", data[-4:])[0]
    return {"name": name, "description": f"from {name}.hts ({n} frames, {trailer_ms} ms)", "frames": frames,
            "end_in_reset": False, "hts_total_ms": trailer_ms}


def load(path):
    with open(path, "rb") as f:
        return parse(f.read(), os.path.splitext(os.path.basename(path))[0])


def _ticks(ms, lo, hi):
    return min(max(int(round(ms / TICK_MS)), lo), hi)


def build(motion, kind=KIND_APP):
    """Bytes of an .hts file for a motion dict. Servos a frame does not name are left alone (0xFF); angles
    are rounded to whole degrees and clamped to 0-180; move times to 20 ms ticks within 200-4000 ms. A
    motion with end_in_reset gets a closing RESET_POSE frame of RESET_FRAME_MS."""
    frames = [fr for fr in motion.get("frames", []) if isinstance(fr, dict)]
    if motion.get("end_in_reset", False):
        frames = frames + [{"angles": dict(RESET_POSE), "runtime": RESET_FRAME_MS}]
    if not frames:
        raise HtsError("a motion needs at least one frame")
    out, total = bytearray(RECORD), 0
    for i, fr in enumerate(frames):
        angles = fr.get("angles") if isinstance(fr.get("angles"), dict) else {}
        run = _ticks(fr.get("runtime", 500), 10, MAX_RUN_TICKS)
        tot = min(run + _ticks(fr.get("hold", 0), 0, 0xFFFF), 0xFFFF)
        rec = bytearray(RECORD)
        rec[0], rec[1], rec[2] = 0xFB, 0xBF, kind
        rec[3] = 1 if i == 0 else 3 if i == len(frames) - 1 else 2
        rec[4:6] = struct.pack("<H", len(frames))
        rec[6:8] = struct.pack("<H", i)
        for k, n in enumerate(NAMES):
            rec[8 + k] = 0xFF if n not in angles else min(max(int(round(angles[n])), 0), 180)
        rec[28] = run
        rec[29], rec[30] = tot >> 8, tot & 0xFF
        rec[31] = sum(rec[2:31]) & 0xFF
        rec[32] = 0xED
        out += rec
        total += tot * TICK_MS
    trailer = bytearray(RECORD)
    trailer[-4:] = struct.pack("<I", total)
    return bytes(out + trailer)


def save(motion, path):
    with open(path, "wb") as f:
        f.write(build(motion))


def describe(motion):
    lines = []
    for i, fr in enumerate(motion["frames"]):
        a = " ".join(f"{fr['angles'][n]:3d}" if n in fr["angles"] else " --" for n in NAMES)
        lines.append(f"{i + 1:3d}  move {fr['runtime']:5d} ms  hold {fr['hold']:5d} ms | {a}")
    return "\n".join(lines)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) == 1 and argv[0].lower().endswith(".hts"):
        mo = load(argv[0])
        print(f"{mo['name']}: {len(mo['frames'])} frames, {mo['hts_total_ms']} ms  (servo columns: {' '.join(NAMES)})")
        print(describe(mo))
        return 0
    if len(argv) == 2:
        from . import motion as motion_mod
        src, dst = argv
        if src.lower().endswith(".hts") and dst.lower().endswith(".json"):
            motion_mod.save(load(src), dst)
        elif dst.lower().endswith(".hts"):
            save(motion_mod.load(src), dst)
        else:
            print("usage: hts FILE.hts | hts motion.json out.hts | hts in.hts out.json", file=sys.stderr)
            return 2
        print(f"wrote {dst}")
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
