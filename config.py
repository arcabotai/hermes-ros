"""Plugin settings: defaults + deep-merge over ``plugins.entries.hermes-ros.settings``.

Settings are read through ``ctx.get_config`` on every use, so edits to config.yaml take effect on the next
tool call without restarting Hermes. No new HERMES_* environment variables are introduced.
"""
from __future__ import annotations

import copy
import os
from typing import Any, Callable, Dict

# Shown on ros_status / /ros status when backend=rosbridge talks plaintext to a host outside the loopback.
# Localhost (the default, and the docker demo) stays quiet: that link never leaves the machine.
PLAINTEXT_REMOTE_WARNING = (
    "Unencrypted rosbridge websocket (ws://) to a non-loopback host. "
    "Velocity commands and camera frames travel in plaintext. "
    "Use this only on a trusted network, and do not expose an unencrypted rosbridge to the public internet. "
    "Set rosbridge.tls: true for wss://; the server certificate and hostname are verified, and that check cannot be turned off."
)

DEFAULTS: Dict[str, Any] = {
    # "mock" (built-in 2D sim, no ROS needed) or "rosbridge" (real robot / sim via a rosbridge websocket)
    "backend": "mock",
    "rosbridge": {
        "host": "localhost", "port": 9090, "connect_timeout_s": 6.0, "call_timeout_s": 8.0,
        "tls": False,  # true: wss:// with certificate + hostname verification (there is no skip switch)
        "ca_file": "",  # PEM CA bundle used only when tls is true; empty means the system trust store
    },
    "topics": {
        "cmd_vel": "/cmd_vel",
        "cmd_vel_type": "geometry_msgs/Twist",  # TurtleBot3 on Jazzy wants geometry_msgs/TwistStamped
        "odom": "/odom",
        "odom_type": "nav_msgs/Odometry",  # use turtlesim/Pose with /turtle1/pose
        "camera": "/camera/image_raw",
    },
    "camera": {
        "timeout_s": 6.0,
        "output_dir": "",  # default: <HERMES_HOME>/plugin-data/hermes-ros/snapshots
        "keep_last": 20,
        "return_mode": "path",  # "path": MEDIA:<file> + vision_analyze hint; "inline": multimodal tool result
    },
    "safety": {
        "max_linear_mps": 0.5,
        "max_angular_rps": 1.0,
        "max_duration_s": 5.0,
        "default_duration_s": 1.0,
        "over_limit_action": "block",  # "block" | "clamp" (clamp speed/duration to the limit and tell the agent)
        "max_publish_rate_hz": 30.0,
        "geofence": {
            "enabled": False,
            "x_min": -4.0, "x_max": 4.0, "y_min": -4.0, "y_max": 4.0,
            "max_odom_age_s": 5.0, "margin_m": 0.25,
        },
        "write_allowlist": {
            "topics": ["/cmd_vel"],
            "services": ["/mock/*"],
        },
        "motion_topic_patterns": ["*cmd_vel*"],  # publishes here must parse as a Twist or are refused
        "blocked_log_max": 500,
    },
    "mock": {
        "half_x": 5.0, "half_y": 5.0,
        "start": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "cones": [{"x": 3.5, "y": 0.8}],
        "realtime": False,  # sleep in real time while the sim runs (nice for a live dashboard)
        "persist": True,  # remember the robot pose between processes (plugin-data/hermes-ros/mock_state.json)
    },
}


def _coerce_tls(value: Any) -> bool:
    """Bool-ish config without treating the string ``\"false\"`` as true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off", ""):
            return False
    raise ValueError("rosbridge.tls must be true or false")


def _is_loopback(host: str) -> bool:
    """Hosts whose rosbridge traffic stays on this machine. Anything else is treated as remote."""
    name = host.strip().lower().strip("[]").rstrip(".")
    if name in ("localhost", "::1") or name.endswith(".localhost"):
        return True
    parts = name.split(".")
    if len(parts) == 4 and all(part.isdigit() and 0 <= int(part) <= 255 for part in parts):
        return int(parts[0]) == 127
    return False


def describe_rosbridge(rb: Any) -> Dict[str, Any]:
    """URL, encryption flag and plaintext warning for a rosbridge settings dict. No network I/O.

    ``host`` is a hostname or IP, never a URL. Encryption is ``tls: true`` (wss://). Certificate and
    hostname checks are always on for that path; this function has no field that turns them off.
    """
    if not isinstance(rb, dict):
        raise ValueError("rosbridge settings must be a mapping with host, port, and optional tls and ca_file")
    host = str(rb.get("host", "localhost")).strip()
    if (not host or "://" in host or "/" in host or "@" in host or "\\" in host
            or any(ch.isspace() for ch in host)):
        raise ValueError(
            "rosbridge.host must be a hostname or IP address, not a URL. "
            "Set rosbridge.tls: true for an encrypted wss:// connection."
        )
    try:
        port = int(rb.get("port", 9090))
    except (TypeError, ValueError) as exc:
        raise ValueError("rosbridge.port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise ValueError("rosbridge.port must be between 1 and 65535")
    tls = _coerce_tls(rb.get("tls", False))
    raw_ca = rb.get("ca_file")
    ca_file = "" if raw_ca is None else os.path.expanduser(str(raw_ca).strip())
    bare = host.strip("[]")
    url_host = f"[{bare}]" if ":" in bare else bare
    scheme = "wss" if tls else "ws"
    warning = None if tls or _is_loopback(bare) else PLAINTEXT_REMOTE_WARNING
    return {
        "host": bare, "port": port, "tls": tls, "ca_file": ca_file,
        "url": f"{scheme}://{url_host}:{port}", "encrypted": tls, "warning": warning,
    }


def deep_merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


class Settings:
    """Live view over the plugin's configuration."""

    def __init__(self, getter: Callable[[str, Any], Any] | None = None, overrides: Dict[str, Any] | None = None):
        self._getter = getter
        self._overrides = overrides or {}

    def snapshot(self) -> Dict[str, Any]:
        merged = copy.deepcopy(DEFAULTS)
        for key in DEFAULTS:
            value = None
            if self._getter is not None:
                try:
                    value = self._getter(key, None)
                except Exception:  # config problems must never crash a tool call
                    value = None
            if value is None:
                continue
            if isinstance(DEFAULTS[key], dict) and isinstance(value, dict):
                merged[key] = deep_merge(DEFAULTS[key], value)
            elif not isinstance(DEFAULTS[key], dict):
                merged[key] = value
        return deep_merge(merged, self._overrides)

    def __getitem__(self, key: str) -> Any:
        return self.snapshot()[key]

    @property
    def safety(self) -> Dict[str, Any]:
        return self.snapshot()["safety"]
