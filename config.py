"""Plugin settings: defaults + deep-merge over ``plugins.entries.hermes-ros.settings``.

Settings are read through ``ctx.get_config`` on every use, so edits to config.yaml take effect on the next
tool call without restarting Hermes. No new HERMES_* environment variables are introduced.
"""
from __future__ import annotations

import copy
from typing import Any, Callable, Dict

DEFAULTS: Dict[str, Any] = {
    # "mock" (built-in 2D sim, no ROS needed) or "rosbridge" (real robot / sim via a rosbridge websocket)
    "backend": "mock",
    "rosbridge": {"host": "localhost", "port": 9090, "connect_timeout_s": 6.0, "call_timeout_s": 8.0},
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
