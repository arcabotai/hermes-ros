#!/usr/bin/env python3
"""Offline demo: no ROS, no Docker, no API key. Drives the built-in mock robot to the red cone through the real
tool handlers + safety interlock, then shows the interlock refusing unsafe requests.

    python demo/mock_demo.py [--out demo-out] [--present]
"""
import argparse
import importlib.util
import json
import math
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("hermes_ros", ROOT / "__init__.py", submodule_search_locations=[str(ROOT)])
pkg = importlib.util.module_from_spec(spec)
sys.modules["hermes_ros"] = pkg
spec.loader.exec_module(pkg)
import logging  # noqa: E402
logging.getLogger("hermes_ros").setLevel(logging.CRITICAL)
logging.getLogger("hermes_ros.interlock").setLevel(logging.CRITICAL)
from hermes_ros.runtime import Runtime  # noqa: E402
from hermes_ros.tools import make_handlers  # noqa: E402

BOLD, DIM, RED, GREEN, YEL, END = "\033[1m", "\033[2m", "\033[31m", "\033[32m", "\033[33m", "\033[0m"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="demo-out")
    ap.add_argument("--present", action="store_true",
                    help="pause between beats for a screen recording; stays under 90 seconds")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    present = args.present

    def pause(seconds):
        if present:
            time.sleep(seconds)
    rt = Runtime(None, Path(tempfile.mkdtemp(prefix="hermes-ros-demo-")),
                 {"camera": {"output_dir": str(out)}, "mock": {"persist": False},
                  "safety": {"geofence": {"enabled": True, "x_min": -4, "x_max": 4, "y_min": -4, "y_max": 4}}})
    h = make_handlers(rt)

    def call(tool, **args):
        """Exactly what Hermes does: pre_tool_call hook first, then the handler."""
        shown = ", ".join(f"{k}={v}" for k, v in args.items())
        print(f"{BOLD}> {tool}({shown}){END}")
        d = rt.interlock.pre_tool_call(tool_name=tool, args=args)
        if d and d["action"] == "block":
            print(f"  {RED}{d['message']}{END}")
            return None
        if d and d["action"] == "modify":
            args = {**args, **d["args"]}
        res = h[tool](args)
        res = json.loads(res["text_summary"] if isinstance(res, dict) else res)
        return res

    def say(s):
        print(f"{DIM}{s}{END}")

    say("== SIMULATED robot (built-in mock). No ROS and no hardware. ==")
    say("== Interlock covers hermes-ros tools only. Not a sandbox. Not a hardware safety system. ==")
    say("== Red cone at (3.5, 0.8) in a 10x10 m arena. Limits 0.5 m/s, 1 rad/s, 5 s. Geofence ±4 m. ==")
    pause(4.0)
    r = call("ros_camera_snapshot")
    (out / "camera-start.png").write_bytes(Path(r["path"]).read_bytes())
    print(f"  {GREEN}saved {r['path']}{END}  ({r['width']}x{r['height']}; copy {out / 'camera-start.png'})")
    pause(5.0)
    say("-- approach the cone: turn toward it, then drive in small steps")
    cone = call("ros_call_service", service="/mock/get_cones")["response"]["cones"][0]
    for _ in range(8):
        p = call("ros_get_pose")["pose"]
        dx, dy = cone["x"] - p["x"], cone["y"] - p["y"]
        dist = math.hypot(dx, dy)
        if dist < 0.8:
            break
        err = math.atan2(math.sin(math.atan2(dy, dx) - p["yaw_rad"]), math.cos(math.atan2(dy, dx) - p["yaw_rad"]))
        if abs(err) > 0.05:
            call("ros_drive", angular=0.5 if err > 0 else -0.5, duration_s=round(abs(err) / 0.5, 2))
        res = call("ros_drive", linear=0.5, duration_s=round(min(max(dist - 0.6, 0.2), 4.0), 2))
        print(f"  {GREEN}-> at ({res['pose_after']['x']:.2f}, {res['pose_after']['y']:.2f}), {res['displacement_m']} m moved{END}")
        pause(0.8)
    r = call("ros_camera_snapshot")
    (out / "camera-at-cone.png").write_bytes(Path(r["path"]).read_bytes())
    print(f"  {GREEN}saved {r['path']}{END}  (copy {out / 'camera-at-cone.png'})")
    (out / "topdown.png").write_bytes(rt.backend().topdown_png())
    pause(5.0)
    say("-- the user asks: 'floor it into the wall' from here")
    pause(1.2)
    held = call("ros_get_pose")["pose"]
    call("ros_drive", linear=3.0, duration_s=30)
    still = call("ros_get_pose")["pose"]
    print(f"  {GREEN}pose held at ({still['x']:.2f}, {still['y']:.2f}); was ({held['x']:.2f}, {held['y']:.2f}){END}")
    pause(5.0)
    say("-- now something unsafe: drive out of the geofence, then touch a topic that is not allowlisted")
    pause(1.0)
    call("ros_drive", linear=0.5, duration_s=5)
    call("ros_publish", topic="/arm/gripper", msg_type="std_msgs/Float64", message={"data": 1.0})
    pause(4.0)
    say("-- human latches the e-stop; further motion is refused. The agent cannot clear it.")
    pause(1.2)
    call("ros_stop", estop=True, reason="demo e-stop")
    call("ros_drive", linear=0.1, duration_s=1)
    pause(5.0)
    rt.interlock.clear_estop("human")
    say("-- blocked-call audit log (plugin tools only; a terminal is outside this log)")
    for e in rt.interlock.recent_blocked(20):
        print(f"  {YEL}{e['iso']}  {e['tool']:<16} [{e['rule']}] {e['reason'][:90]}{END}")
    print(f"\nPictures in {out}/  (camera-start.png, camera-at-cone.png, topdown.png)")
    say("Plugin tools only. Independent hardware limits are still required on a real robot.")
    say("Default rosbridge, if you leave the mock, is unencrypted ws:// on loopback. Do not publish that port publicly.")
    pause(4.0)


if __name__ == "__main__":
    main()
