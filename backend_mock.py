"""Built-in mock robot: a 2D kinematic (unicycle) sim with a rendered fake camera.

Runs anywhere with no ROS, rosbridge or Docker. It speaks the same ``Backend`` interface as the real
rosbridge client, so every tool and the whole safety interlock behave identically against it.

World: a rectangular arena ``[-half_x, half_x] x [-half_y, half_y]`` metres, one or more red traffic cones.
Robot: radius 0.2 m, differential drive, stops dead on walls and cones (``collided`` is reported in odom/pose).
Like a real base controller, a velocity command expires 0.5 s (sim time) after the last publish.
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

from . import imaging
from .backend_base import Backend, BackendError

ROBOT_RADIUS = 0.2
CONE_RADIUS = 0.2
CMD_TIMEOUT_S = 0.5
SIM_DT = 0.02

_TOPICS = [
    ("/cmd_vel", "geometry_msgs/Twist"),
    ("/odom", "nav_msgs/Odometry"),
    ("/camera/image_raw", "sensor_msgs/Image"),
    ("/camera/image_raw/compressed", "sensor_msgs/CompressedImage"),
    ("/rosout", "rcl_interfaces/Log"),
]
_SERVICES = [
    ("/mock/reset", "std_srvs/Trigger"),
    ("/mock/teleport", "mock_msgs/Teleport"),
    ("/mock/get_cones", "mock_msgs/GetCones"),
]
_NODES = ["/mock_robot", "/mock_camera", "/rosbridge_websocket"]


def _yaw_to_quat(yaw: float) -> Dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class MockBackend(Backend):
    name = "mock"
    description = "built-in 2D kinematic sim (no ROS)"

    def __init__(self, cfg: Dict[str, Any], data_dir: Optional[Path] = None):
        self._lock = threading.RLock()
        self.half_x = float(cfg.get("half_x", 5.0))
        self.half_y = float(cfg.get("half_y", 5.0))
        self.cones: List[Tuple[float, float]] = [(float(c["x"]), float(c["y"])) for c in cfg.get("cones", [])]
        self.realtime = bool(cfg.get("realtime", False))
        start = cfg.get("start", {})
        self.x, self.y, self.yaw = float(start.get("x", 0)), float(start.get("y", 0)), float(start.get("yaw", 0))
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.t = 0.0
        self.collided: Optional[str] = None
        self.trail: Deque[Tuple[float, float]] = deque(maxlen=400)
        self.trail.append((self.x, self.y))
        self.published: Deque[Dict[str, Any]] = deque(maxlen=50)
        self._persist_path: Optional[Path] = None
        if cfg.get("persist", True) and data_dir is not None:
            self._persist_path = Path(data_dir) / "mock_state.json"
            self._load()

    # ----------------------------------------------------------- persistence
    def _load(self) -> None:
        try:
            s = json.loads(self._persist_path.read_text())  # type: ignore[union-attr]
            self.x, self.y, self.yaw = float(s["x"]), float(s["y"]), float(s["yaw"])
            self.t = float(s.get("t", 0.0))
            self.trail.clear()
            for p in s.get("trail", [])[-400:]:
                self.trail.append((float(p[0]), float(p[1])))
            if not self.trail:
                self.trail.append((self.x, self.y))
        except Exception:
            pass

    def _save(self) -> None:
        if self._persist_path is None:
            return
        try:
            self._persist_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._persist_path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"x": self.x, "y": self.y, "yaw": self.yaw, "t": self.t,
                                       "trail": list(self.trail)[-200:]}))
            tmp.replace(self._persist_path)
        except OSError:
            pass

    # --------------------------------------------------------------- physics
    def _blocked_at(self, nx: float, ny: float, margin: float = 0.0) -> Optional[str]:
        if abs(nx) > self.half_x - ROBOT_RADIUS - margin or abs(ny) > self.half_y - ROBOT_RADIUS - margin:
            return "wall"
        for cx, cy in self.cones:
            if math.hypot(nx - cx, ny - cy) < ROBOT_RADIUS + CONE_RADIUS + margin:
                return "cone"
        return None

    def _step(self, dt: float) -> None:
        live = (self.t - self.cmd_t) <= CMD_TIMEOUT_S
        lin, ang = self.cmd if live else (0.0, 0.0)
        self.w = ang
        nx = self.x + lin * math.cos(self.yaw) * dt
        ny = self.y + lin * math.sin(self.yaw) * dt
        hit = self._blocked_at(nx, ny)
        if hit:
            self.collided = hit
            self.v = 0.0
        else:
            if lin != 0.0 and not self._blocked_at(nx, ny, margin=0.02):
                self.collided = None  # only once clearly free again (avoids flicker at the contact boundary)
            self.x, self.y, self.v = nx, ny, lin
        self.yaw = _wrap(self.yaw + ang * dt)
        self.t += dt

    def _advance(self, seconds: float) -> None:
        remaining = seconds
        while remaining > 1e-6:  # ignore float-dust slivers; they would nudge the robot out of a contact
            dt = min(SIM_DT, remaining)
            self._step(dt)
            remaining -= dt
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.03:
            self.trail.append((self.x, self.y))

    def sleep(self, seconds: float) -> None:
        with self._lock:
            self._advance(max(0.0, seconds))
            self._save()
        if self.realtime:
            time.sleep(max(0.0, seconds))

    # ------------------------------------------------------------- Backend API
    def list_topics(self) -> List[Dict[str, str]]:
        return [{"name": n, "type": t} for n, t in _TOPICS]

    def list_services(self) -> List[Dict[str, str]]:
        return [{"name": n, "type": t} for n, t in _SERVICES]

    def list_nodes(self) -> List[str]:
        return list(_NODES)

    def _odom_msg(self) -> Dict[str, Any]:
        sec = int(self.t)
        return {
            "header": {"stamp": {"sec": sec, "nanosec": int((self.t - sec) * 1e9)}, "frame_id": "odom"},
            "child_frame_id": "base_link",
            "pose": {"pose": {"position": {"x": self.x, "y": self.y, "z": 0.0}, "orientation": _yaw_to_quat(self.yaw)},
                     "covariance": [0.0] * 36},
            "twist": {"twist": {"linear": {"x": self.v, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": self.w}},
                      "covariance": [0.0] * 36},
        }

    def echo_once(self, topic: str, msg_type: Optional[str], timeout: float) -> Dict[str, Any]:
        with self._lock:
            if topic == "/odom":
                return self._odom_msg()
            if topic == "/cmd_vel":
                lin, ang = self.cmd if (self.t - self.cmd_t) <= CMD_TIMEOUT_S else (0.0, 0.0)
                return {"linear": {"x": lin, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": ang}}
            if topic in ("/camera/image_raw", "/camera/image_raw/compressed"):
                png = self._render_forward()
                return {"header": {"frame_id": "camera_link"}, "format": "png", "data": f"<{len(png)} bytes PNG>"}
            if topic == "/rosout":
                return {"level": 20, "name": "mock_robot", "msg": f"collided={self.collided}", "stamp": self.t}
        raise BackendError(f"no publisher on {topic!r} (known: {[t['name'] for t in self.list_topics()]})")

    def publish(self, topic: str, msg_type: str, message: Dict[str, Any]) -> None:
        with self._lock:
            self.published.append({"topic": topic, "type": msg_type, "t": self.t})
            if topic != "/cmd_vel":
                return  # accepted, but nothing in the mock listens to it
            tw = message.get("twist", message) if isinstance(message, dict) else {}
            lin = float(((tw.get("linear") or {}).get("x", 0.0)))
            ang = float(((tw.get("angular") or {}).get("z", 0.0)))
            self.cmd, self.cmd_t = (lin, ang), self.t

    def call_service(self, service: str, srv_type: Optional[str], request: Dict[str, Any], timeout: float) -> Dict[str, Any]:
        with self._lock:
            if service == "/mock/reset":
                self.x = self.y = self.yaw = 0.0
                self.v = self.w = 0.0
                self.cmd, self.cmd_t, self.collided = (0.0, 0.0), -1e9, None
                self.trail.clear()
                self.trail.append((0.0, 0.0))
                self._save()
                return {"success": True, "message": "robot reset to origin"}
            if service == "/mock/teleport":
                nx, ny = float(request.get("x", 0)), float(request.get("y", 0))
                if self._blocked_at(nx, ny):
                    return {"success": False, "message": "target is inside a wall or cone"}
                self.x, self.y, self.yaw = nx, ny, _wrap(float(request.get("yaw", 0.0)))
                self.trail.append((self.x, self.y))
                self._save()
                return {"success": True, "message": f"teleported to ({nx:.2f}, {ny:.2f})"}
            if service == "/mock/get_cones":
                return {"cones": [{"x": cx, "y": cy} for cx, cy in self.cones]}
        raise BackendError(f"unknown service {service!r} (known: {[s['name'] for s in self.list_services()]})")

    def _pose_dict(self) -> Dict[str, Any]:
        return {"x": round(self.x, 4), "y": round(self.y, 4), "yaw_rad": round(self.yaw, 4),
                "yaw_deg": round(math.degrees(self.yaw), 1), "linear_vel": round(self.v, 4),
                "angular_vel": round(self.w, 4), "frame_id": "odom", "child_frame_id": "base_link",
                "age_s": 0.0, "collided": self.collided, "source": "mock:/odom"}

    def get_pose(self, topic: Optional[str], msg_type: Optional[str], timeout: float) -> Dict[str, Any]:
        with self._lock:
            return self._pose_dict()

    def latest_pose(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._pose_dict()

    def _render_forward(self) -> bytes:
        return imaging.render_forward(self.x, self.y, self.yaw, self.half_x, self.half_y, self.cones)

    def camera_frame(self, topic: Optional[str], timeout: float) -> Tuple[bytes, str]:
        with self._lock:
            return self._render_forward(), "mock forward camera (software raycaster, 320x240)"

    def topdown_png(self) -> bytes:
        with self._lock:
            return imaging.render_topdown(self.x, self.y, self.yaw, self.half_x, self.half_y, self.cones, list(self.trail))

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {"backend": "mock", "t": round(self.t, 2), "pose": self._pose_dict(),
                    "arena": {"half_x": self.half_x, "half_y": self.half_y}, "cones": list(self.cones)}
