"""Real robot / simulator backend: talks to a rosbridge websocket through the small built-in client in ``rosbridge_client.py``.

The Hermes host needs no ROS install. ``websocket-client`` is imported lazily and the websocket is opened on the
first tool call (never while the plugin registers), so ``hermes plugins doctor`` and CI stay offline.
Works with ROS 1 and ROS 2 rosbridge servers (``rosbridge_server`` / ``rosapi`` must be running).
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from . import imaging
from .backend_base import Backend, BackendError
from .config import describe_rosbridge
from .rosbridge_client import RosbridgeClient

logger = logging.getLogger("hermes_ros")


def _quat_to_yaw(q: Dict[str, float]) -> float:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_pose(msg: Dict[str, Any]) -> Dict[str, Any]:
    """Normalise nav_msgs/Odometry, PoseStamped, PoseWithCovarianceStamped or turtlesim/Pose to one dict."""
    out: Dict[str, Any] = {"frame_id": None, "child_frame_id": msg.get("child_frame_id")}
    hdr = msg.get("header") or {}
    out["frame_id"] = hdr.get("frame_id")
    if "theta" in msg and "x" in msg:  # turtlesim/Pose
        out.update(x=float(msg["x"]), y=float(msg["y"]), yaw_rad=float(msg["theta"]),
                   linear_vel=float(msg.get("linear_velocity", 0.0)), angular_vel=float(msg.get("angular_velocity", 0.0)))
        out["frame_id"] = out["frame_id"] or "turtlesim"
    else:
        pose = msg.get("pose")
        if isinstance(pose, dict) and "pose" in pose:  # Odometry / PoseWithCovarianceStamped
            pose = pose["pose"]
        if not isinstance(pose, dict) or "position" not in pose:
            raise BackendError("message has no pose (expected nav_msgs/Odometry, PoseStamped or turtlesim/Pose)")
        out["x"], out["y"] = float(pose["position"]["x"]), float(pose["position"]["y"])
        out["yaw_rad"] = _quat_to_yaw(pose.get("orientation") or {})
        tw = (msg.get("twist") or {}).get("twist") or {}
        out["linear_vel"] = float((tw.get("linear") or {}).get("x", 0.0))
        out["angular_vel"] = float((tw.get("angular") or {}).get("z", 0.0))
    out["yaw_rad"] = round(out["yaw_rad"], 4)
    out["yaw_deg"] = round(math.degrees(out["yaw_rad"]), 1)
    out["x"], out["y"] = round(out["x"], 4), round(out["y"], 4)
    return out


class RosbridgeBackend(Backend):
    name = "rosbridge"

    def __init__(self, rb_cfg: Dict[str, Any], topics_cfg: Dict[str, Any]):
        try:
            self.endpoint = describe_rosbridge(rb_cfg)
        except ValueError as exc:
            raise BackendError(str(exc)) from exc
        self.host = self.endpoint["host"]
        self.port = self.endpoint["port"]
        self.connect_timeout = float(rb_cfg.get("connect_timeout_s", 6.0)) if isinstance(rb_cfg, dict) else 6.0
        self.call_timeout = float(rb_cfg.get("call_timeout_s", 8.0)) if isinstance(rb_cfg, dict) else 8.0
        self.topics_cfg = topics_cfg
        self.description = f"rosbridge {self.endpoint['url']}"
        self._ros: Any = None
        self._lock = threading.RLock()
        self._pubs: Dict[Tuple[str, str], Any] = {}
        self._odom_sub: Any = None
        self._odom_key: Optional[Tuple[str, str]] = None
        self._odom_last: Optional[Tuple[Dict[str, Any], float]] = None

    # ------------------------------------------------------------ connection
    def _client(self) -> Any:
        with self._lock:
            if self._ros is not None and self._ros.is_connected:
                return self._ros
            try:
                import websocket  # noqa: F401  (websocket-client; checked here so the error is actionable)
            except ImportError as exc:
                raise BackendError("websocket-client is not installed (pip install 'websocket-client>=1.6,<2')") from exc
            if self.endpoint["warning"]:
                logger.warning("%s (%s)", self.endpoint["warning"], self.endpoint["url"])
            ros = RosbridgeClient(self.endpoint["url"], tls=self.endpoint["tls"], ca_file=self.endpoint["ca_file"])
            try:
                ros.connect(timeout=self.connect_timeout)
            except Exception as exc:
                ros.close()
                message = f"cannot reach rosbridge at {self.endpoint['url']} ({type(exc).__name__}: {exc})."
                # A missing CA file or a failed certificate check already says why. Don't add a
                # "is the server up?" hint on top of that.
                explained = "ca_file" in str(exc) or "TLS verification failed" in str(exc)
                if not explained:
                    message += (" Is rosbridge_server listening with TLS, and does the certificate match this host?"
                                if self.endpoint["tls"] else
                                " Is rosbridge_server running and the host/port in plugin settings correct?")
                if self.endpoint["warning"]:
                    message += " " + self.endpoint["warning"]
                raise BackendError(message) from exc
            self._ros = ros
            self._pubs.clear()
            self._odom_sub = None
            self._odom_key = None
            return ros

    def close(self) -> None:
        with self._lock:
            try:
                if self._odom_sub is not None and self._ros is not None:
                    self._ros.unsubscribe(self._odom_sub[0], self._odom_sub[1])
            except Exception:
                pass
            try:
                if self._ros is not None:
                    self._ros.close()
            except Exception:
                pass
            self._ros, self._pubs, self._odom_sub = None, {}, None

    def status(self) -> Dict[str, Any]:
        connected = bool(self._ros is not None and self._ros.is_connected)
        out: Dict[str, Any] = {
            "backend": "rosbridge", "url": self.endpoint["url"], "connected": connected,
            "encrypted": self.endpoint["encrypted"],
        }
        if self.endpoint["warning"]:
            out["transport_warning"] = self.endpoint["warning"]
        return out

    # ---------------------------------------------------------------- helpers
    def _svc(self, name: str, typ: str, req: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None) -> Dict[str, Any]:
        ros = self._client()
        try:
            return ros.call_service(name, typ, req or {}, timeout=timeout or self.call_timeout)
        except Exception as exc:
            raise BackendError(f"service {name} failed: {type(exc).__name__}: {exc}") from exc

    def _topic_type(self, topic: str) -> str:
        res = self._svc("/rosapi/topic_type", "rosapi/TopicType", {"topic": topic})
        t = res.get("type") or ""
        if not t:
            raise BackendError(f"topic {topic!r} has no publisher/known type; pass msg_type explicitly or check ros_list")
        return t

    # -------------------------------------------------------------- discovery
    def list_topics(self) -> List[Dict[str, str]]:
        res = self._svc("/rosapi/topics_and_raw_types", "rosapi/TopicsAndRawTypes")
        names, types = res.get("topics", []), res.get("types", [])
        if names and len(names) == len(types):
            return [{"name": n, "type": t} for n, t in zip(names, types)]
        res = self._svc("/rosapi/topics", "rosapi/Topics")
        return [{"name": n, "type": ""} for n in res.get("topics", [])]

    def list_services(self) -> List[Dict[str, str]]:
        names = self._svc("/rosapi/services", "rosapi/Services").get("services", [])
        out = []
        for n in names[:80]:
            try:
                t = self._svc("/rosapi/service_type", "rosapi/ServiceType", {"service": n}, timeout=2.0).get("type", "")
            except BackendError:
                t = ""
            out.append({"name": n, "type": t})
        out += [{"name": n, "type": ""} for n in names[80:]]
        return out

    def list_nodes(self) -> List[str]:
        return list(self._svc("/rosapi/nodes", "rosapi/Nodes").get("nodes", []))

    # ------------------------------------------------------------------- data
    def _first_message(self, topic: str, msg_type: Optional[str], timeout: float, **kw: Any) -> Dict[str, Any]:
        ros = self._client()
        if self._odom_key is not None and topic == self._odom_key[0] and not kw:
            # rosbridge drops *every* subscription of this client on a topic when one of them unsubscribes, so a
            # one-shot subscribe/unsubscribe here would silently kill the persistent odometry feed the safety
            # interlock depends on. Serve the request from that feed instead.
            t0 = time.time()
            self._ensure_odom()
            deadline = t0 + timeout
            while time.time() < deadline:
                last = self._odom_last
                if last is not None and last[1] >= t0:
                    return dict(last[0])
                time.sleep(0.02)
            raise BackendError(f"no message on {topic} within {timeout:.1f}s (nothing publishing? wrong type {msg_type}?)")
        msg_type = msg_type or self._topic_type(topic)
        got: Dict[str, Any] = {}
        ev = threading.Event()

        def cb(m: Dict[str, Any]) -> None:
            if not ev.is_set():
                got["m"] = dict(m)
                ev.set()

        sid = ros.subscribe(topic, msg_type, cb, queue_length=1, **kw)
        try:
            if not ev.wait(timeout):
                raise BackendError(f"no message on {topic} within {timeout:.1f}s (nothing publishing? wrong type {msg_type}?)")
        finally:
            try:
                ros.unsubscribe(topic, sid)
            except Exception:
                pass
        return got["m"]

    def echo_once(self, topic: str, msg_type: Optional[str], timeout: float) -> Dict[str, Any]:
        return self._first_message(topic, msg_type, timeout)

    def publish(self, topic: str, msg_type: str, message: Dict[str, Any]) -> None:
        ros = self._client()
        ros.publish(topic, msg_type, message)

    def call_service(self, service: str, srv_type: Optional[str], request: Dict[str, Any], timeout: float) -> Dict[str, Any]:
        if not srv_type:
            res = self._svc("/rosapi/service_type", "rosapi/ServiceType", {"service": service}, timeout=3.0)
            srv_type = res.get("type")
            if not srv_type:
                raise BackendError(f"unknown service {service!r}; check ros_list kind=services")
        return self._svc(service, srv_type, request, timeout=timeout)

    def _ensure_odom(self) -> None:
        topic = self.topics_cfg.get("odom", "/odom")
        mtype = self.topics_cfg.get("odom_type") or self._topic_type(topic)
        key = (topic, mtype)
        with self._lock:
            ros = self._client()
            if self._odom_sub is not None and self._odom_key == key:
                return
            if self._odom_sub is not None:
                try:
                    ros.unsubscribe(self._odom_sub[0], self._odom_sub[1])
                except Exception:
                    pass

            def cb(m: Dict[str, Any]) -> None:
                self._odom_last = (dict(m), time.time())

            self._odom_sub = (topic, ros.subscribe(topic, mtype, cb, queue_length=1, throttle_rate=50))
            self._odom_key = key
            self._odom_last = None

    def latest_pose(self) -> Optional[Dict[str, Any]]:
        try:
            self._ensure_odom()
            deadline = time.time() + 1.5
            while self._odom_last is None and time.time() < deadline:
                time.sleep(0.05)
            if self._odom_last is None:
                return None
            msg, ts = self._odom_last
            if time.time() - ts > 2.0:  # feed went quiet: resubscribe once before reporting stale data
                try:
                    self._client().unsubscribe(self._odom_sub[0], self._odom_sub[1])
                except Exception:
                    pass
                self._odom_key, self._odom_sub, self._odom_last = None, None, None
                self._ensure_odom()
                deadline = time.time() + 1.5
                while self._odom_last is None and time.time() < deadline:
                    time.sleep(0.05)
                if self._odom_last is None:
                    return None
                msg, ts = self._odom_last
            pose = parse_pose(msg)
            pose["age_s"] = round(time.time() - ts, 3)
            pose["source"] = f"rosbridge:{self._odom_key[0] if self._odom_key else '?'}"
            return pose
        except BackendError:
            return None

    def get_pose(self, topic: Optional[str], msg_type: Optional[str], timeout: float) -> Dict[str, Any]:
        topic = topic or self.topics_cfg.get("odom", "/odom")
        configured = (topic == self.topics_cfg.get("odom", "/odom"))
        msg_type = msg_type or (self.topics_cfg.get("odom_type") if configured else None)
        pose = parse_pose(self._first_message(topic, msg_type, timeout))
        pose.update(age_s=0.0, source=f"rosbridge:{topic}")
        return pose

    def camera_frame(self, topic: Optional[str], timeout: float) -> Tuple[bytes, str]:
        topic = topic or self.topics_cfg.get("camera", "/camera/image_raw")
        msg = self._first_message(topic, None, timeout, compression="none", throttle_rate=100)
        if "format" in msg and "width" not in msg:  # sensor_msgs/CompressedImage
            data = imaging.compressed_msg_to_bytes(msg)
            imaging.sniff_image(data)  # raises on formats we cannot hand to a vision model (e.g. compressedDepth)
            return data, f"rosbridge:{topic} (CompressedImage {msg.get('format')})"
        try:
            png = imaging.image_msg_to_png(msg)
        except (KeyError, ValueError) as exc:
            raise BackendError(f"cannot decode image from {topic}: {exc}") from exc
        return png, f"rosbridge:{topic} (Image {msg.get('width')}x{msg.get('height')} {msg.get('encoding')})"
