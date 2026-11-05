"""``/ros`` slash command: human-only controls (the agent cannot type slash commands)."""
from __future__ import annotations

import json
import time
from typing import Callable

from .backend_base import BackendError
from .runtime import Runtime

HELP = """hermes-ros — robot link and safety controls
  /ros status          backend, limits, geofence, allowlists, e-stop
  /ros estop [reason]  latch the e-stop now (blocks all motion)
  /ros clear           clear the e-stop latch (human-only)
  /ros log [n]         last n blocked calls with reasons
  /ros snapshot        grab a camera frame
  /ros stop            send a zero-velocity burst (no latch)"""


def make_command(rt: Runtime) -> Callable[[str], str]:
    from .tools import make_handlers
    handlers = make_handlers(rt)

    def handle(raw_args: str = "") -> str:
        parts = (raw_args or "").strip().split(None, 1)
        sub = parts[0].lower() if parts else "status"
        rest = parts[1] if len(parts) > 1 else ""
        if sub in ("help", "-h", "--help"):
            return HELP
        if sub == "estop":
            rec = rt.interlock.engage_estop(rest or "human e-stop via /ros estop", source="human")
            try:
                handlers["ros_stop"]({"estop": False})
            except Exception:
                pass
            return f"E-STOP LATCHED ({rec['reason']}). All motion and service calls are blocked. Clear with /ros clear."
        if sub == "clear":
            was = rt.interlock.clear_estop("human")
            return "E-stop cleared; motion is allowed again within the safety limits." if was else "E-stop was not latched."
        if sub == "stop":
            return handlers["ros_stop"]({"estop": False})
        if sub == "log":
            try:
                n = int(rest or 10)
            except ValueError:
                n = 10
            rows = rt.interlock.recent_blocked(n)
            if not rows:
                return "No blocked calls recorded this session."
            return "\n".join(f"{r['iso']}  {r['tool']}  [{r['rule']}]  {r['reason']}" for r in rows)
        if sub == "snapshot":
            raw = handlers["ros_camera_snapshot"]({})
            try:
                res = json.loads(raw if isinstance(raw, str) else raw["text_summary"])
            except Exception as exc:
                return f"snapshot failed: {exc}"
            return f"{res.get('error', 'Camera frame saved')}\n{res.get('media_tag', '')}"
        if sub == "status":
            res = json.loads(handlers["ros_status"]({}))
            est = res["estop"]
            lim = res["limits"]
            gf = res["geofence"]
            be = res["backend"]
            lines = [f"backend: {be.get('backend') if isinstance(be, dict) else be}"]
            if isinstance(be, dict) and be.get("url"):
                lines.append(f"link: {be['url']} ({'encrypted' if be.get('encrypted') else 'unencrypted'})")
            if isinstance(be, dict) and be.get("transport_warning"):
                lines.append(f"transport: {be['transport_warning']}")
            if isinstance(be, dict) and be.get("error"):
                lines.append(f"link error: {be['error']}")
            lines += [
                f"e-stop: {'LATCHED — ' + str(est.get('reason')) if est.get('latched') else 'clear'}",
                f"limits: ≤{lim['max_linear_mps']} m/s, ≤{lim['max_angular_rps']} rad/s, ≤{lim['max_duration_s']} s ({lim['over_limit_action']} when exceeded)",
                f"geofence: {'x[%s,%s] y[%s,%s]' % (gf['x_min'], gf['x_max'], gf['y_min'], gf['y_max']) if gf.get('enabled') else 'off'}",
                f"write allowlist: topics={res['write_allowlist']['topics']} services={res['write_allowlist']['services']}",
                f"calls: {res['counters']}",
            ]
            return "\n".join(lines)
        return f"unknown subcommand {sub!r}\n{HELP}"

    return handle
