"""Safety interlock for robot-writing tools, enforced from a ``pre_tool_call`` hook.

Rules (all configured under ``plugins.entries.hermes-ros.settings.safety``):

* speed ceiling   – linear / angular magnitude of any Twist, block or clamp
* duration cap    – a motion command always has a bounded duration (default injected when omitted)
* geofence        – optional rectangle in the odom frame; the *projected path* of the command (dead
                    reckoned from the latest odom) must stay inside, and odom must be fresh
* write allowlist – only allowlisted topics / services may be written to (glob patterns)
* e-stop latch    – once engaged, all motion and service calls are blocked until a human clears it
                    (``/ros clear``); the agent has no tool that clears it
* audit trail     – every blocked call is logged with its rule id and reason (logger, ring buffer, JSONL)

Zero-velocity commands and the stop tools are never blocked: stopping must always be possible.
"""
from __future__ import annotations

import copy
import fnmatch
import json
import logging
import math
import secrets
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

logger = logging.getLogger("hermes_ros.interlock")

MOTION_TOOLS = {"ros_drive", "ros_publish"}
SERVICE_TOOLS = {"ros_call_service"}
READ_TOOLS = {"ros_list", "ros_echo", "ros_get_pose", "ros_camera_snapshot", "ros_status"}
STOP_TOOLS = {"ros_stop"}
ALL_TOOLS = MOTION_TOOLS | SERVICE_TOOLS | READ_TOOLS | STOP_TOOLS
TWIST_TYPES = {"geometry_msgs/Twist", "geometry_msgs/msg/Twist", "geometry_msgs/TwistStamped",
               "geometry_msgs/msg/TwistStamped"}


class Blocked(Exception):
    def __init__(self, rule: str, reason: str):
        super().__init__(reason)
        self.rule, self.reason = rule, reason


def norm_name(name: Any) -> str:
    n = str(name or "").strip()
    return n if n.startswith("/") else "/" + n


def matches_any(name: str, patterns: List[str]) -> bool:
    return any(fnmatch.fnmatchcase(name, norm_name(p)) for p in (patterns or []))


def _num(v: Any, what: str, default: Optional[float] = None) -> float:
    if v is None:
        if default is None:
            raise Blocked("invalid_args", f"{what} is required")
        return default
    if isinstance(v, bool) or not isinstance(v, (int, float, str)):
        raise Blocked("invalid_args", f"{what} must be a number, got {type(v).__name__}")
    try:
        f = float(v)
    except ValueError:
        raise Blocked("invalid_args", f"{what} must be a number, got {v!r}")
    if not math.isfinite(f):
        raise Blocked("invalid_args", f"{what} must be finite, got {v!r}")
    return f


def extract_twist(message: Any) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Return ((lx, ly, lz), (ax, ay, az)) from a Twist or TwistStamped dict, else None."""
    if not isinstance(message, dict):
        return None
    tw = message.get("twist") if isinstance(message.get("twist"), dict) and "linear" not in message else message
    if not isinstance(tw, dict) or ("linear" not in tw and "angular" not in tw):
        return None

    def vec(key: str) -> Tuple[float, float, float]:
        v = tw.get(key) or {}
        if not isinstance(v, dict):
            raise Blocked("invalid_args", f"twist.{key} must be an object with x/y/z")
        return (_num(v.get("x", 0.0), f"twist.{key}.x", 0.0), _num(v.get("y", 0.0), f"twist.{key}.y", 0.0),
                _num(v.get("z", 0.0), f"twist.{key}.z", 0.0))

    return vec("linear"), vec("angular")


def set_twist(message: Dict[str, Any], lin: Tuple[float, float, float], ang: Tuple[float, float, float]) -> Dict[str, Any]:
    out = copy.deepcopy(message)
    tw = out["twist"] if isinstance(out.get("twist"), dict) and "linear" not in out else out
    tw["linear"] = dict(zip("xyz", lin))
    tw["angular"] = dict(zip("xyz", ang))
    return out


def project_path(pose: Dict[str, Any], vx: float, vy: float, wz: float, duration: float,
                 dt: float = 0.1) -> List[Tuple[float, float]]:
    """Dead-reckon the robot-frame twist from ``pose`` for ``duration`` seconds."""
    x, y, yaw = float(pose["x"]), float(pose["y"]), float(pose.get("yaw_rad", 0.0))
    pts = [(x, y)]
    steps = max(1, int(math.ceil(duration / dt)))
    step = duration / steps
    for _ in range(steps):
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * step
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * step
        yaw += wz * step
        pts.append((x, y))
    return pts


class Interlock:
    def __init__(self, settings_fn: Callable[[], Dict[str, Any]],
                 pose_fn: Callable[[], Optional[Dict[str, Any]]],
                 data_dir_fn: Callable[[], Optional[Path]] = lambda: None):
        self._settings_fn = settings_fn
        self._pose_fn = pose_fn
        self._data_dir_fn = data_dir_fn
        self._lock = threading.RLock()
        self.nonce = secrets.token_hex(12)
        self._latched: Optional[Dict[str, Any]] = None
        self._latch_loaded = False
        self.blocked: Deque[Dict[str, Any]] = deque(maxlen=500)
        self.counters = {"allowed": 0, "blocked": 0, "modified": 0}

    # ------------------------------------------------------------------ e-stop
    def _latch_path(self) -> Optional[Path]:
        d = self._data_dir_fn()
        return None if d is None else Path(d) / "estop.json"

    def _load_latch(self) -> None:
        if self._latch_loaded:
            return
        self._latch_loaded = True
        p = self._latch_path()
        if p and p.exists():
            try:
                data = json.loads(p.read_text())
                if data.get("latched"):
                    self._latched = data
            except Exception:
                # unreadable latch file: fail closed
                self._latched = {"latched": True, "reason": "estop.json unreadable (failing closed)", "ts": time.time()}

    def estop_status(self) -> Dict[str, Any]:
        with self._lock:
            self._load_latch()
            return dict(self._latched) if self._latched else {"latched": False}

    @property
    def latched(self) -> bool:
        with self._lock:
            self._load_latch()
            return self._latched is not None

    def engage_estop(self, reason: str = "", source: str = "agent") -> Dict[str, Any]:
        with self._lock:
            self._load_latch()
            rec = {"latched": True, "reason": reason or "e-stop engaged", "source": source, "ts": time.time()}
            self._latched = rec
            self._write_latch(rec)
        logger.warning("E-STOP engaged by %s: %s", source, rec["reason"])
        return rec

    def clear_estop(self, source: str = "human") -> bool:
        with self._lock:
            self._load_latch()
            was = self._latched is not None
            self._latched = None
            self._write_latch({"latched": False, "cleared_by": source, "ts": time.time()})
        if was:
            logger.warning("E-stop cleared by %s", source)
        return was

    def _write_latch(self, rec: Dict[str, Any]) -> None:
        p = self._latch_path()
        if not p:
            return
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            tmp = p.with_suffix(".tmp")
            tmp.write_text(json.dumps(rec))
            tmp.replace(p)
        except OSError as exc:
            logger.error("could not persist e-stop latch: %s", exc)

    # --------------------------------------------------------------- audit log
    def record_block(self, tool: str, args: Dict[str, Any], rule: str, reason: str) -> Dict[str, Any]:
        safe_args = {k: v for k, v in (args or {}).items() if not str(k).startswith("_")}
        text = json.dumps(safe_args, default=str)
        entry = {"ts": time.time(), "iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "tool": tool, "rule": rule,
                 "reason": reason, "args": text if len(text) < 400 else text[:400] + "…"}
        with self._lock:
            self.blocked.append(entry)
            self.counters["blocked"] += 1
        logger.warning("BLOCKED %s [%s]: %s | args=%s", tool, rule, reason, entry["args"])
        d = self._data_dir_fn()
        if d:
            try:
                path = Path(d) / "blocked.jsonl"
                path.parent.mkdir(parents=True, exist_ok=True)
                with open(path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(entry) + "\n")
                if path.stat().st_size > 1_000_000:  # keep the file bounded
                    lines = path.read_text(encoding="utf-8").splitlines()[-300:]
                    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            except OSError:
                pass
        return entry

    def recent_blocked(self, n: int = 20) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self.blocked)[-n:]

    # ------------------------------------------------------------------- rules
    def _check_allowlist(self, kind: str, name: str, cfg: Dict[str, Any]) -> None:
        allow = (cfg.get("write_allowlist") or {}).get(kind + "s") or []
        if not matches_any(name, allow):
            raise Blocked(f"{kind}_not_allowlisted",
                          f"{kind} {name!r} is not on the write allowlist {list(allow)}. Writing to it is not permitted; "
                          f"a human can add it under safety.write_allowlist.{kind}s.")

    def _check_geofence(self, cfg: Dict[str, Any], vx: float, vy: float, wz: float, duration: float) -> Optional[str]:
        gf = cfg.get("geofence") or {}
        if not gf.get("enabled"):
            return None
        pose = self._pose_fn()
        if not pose:
            raise Blocked("geofence_no_odom", "geofence is enabled but no odometry is available; refusing to move blind "
                          "(check the odom topic / rosbridge connection)")
        age = float(pose.get("age_s", 0.0) or 0.0)
        max_age = float(gf.get("max_odom_age_s", 5.0))
        if age > max_age:
            raise Blocked("geofence_odom_stale", f"latest odom is {age:.1f}s old (max {max_age:.1f}s); refusing to move on stale data")
        m = float(gf.get("margin_m", 0.0))
        x0, x1 = float(gf["x_min"]) + m, float(gf["x_max"]) - m
        y0, y1 = float(gf["y_min"]) + m, float(gf["y_max"]) - m
        pts = project_path(pose, vx, vy, wz, duration)
        for i, (px, py) in enumerate(pts):
            if not (x0 <= px <= x1 and y0 <= py <= y1):
                t = duration * i / max(1, len(pts) - 1)
                raise Blocked("geofence",
                              f"projected path leaves the geofence x[{gf['x_min']},{gf['x_max']}] y[{gf['y_min']},{gf['y_max']}] "
                              f"(margin {m}) at t≈{t:.1f}s near ({px:.2f}, {py:.2f}); robot is at "
                              f"({pose['x']:.2f}, {pose['y']:.2f}). Choose a shorter/slower/different move.")
        return f"geofence ok (end ≈ ({pts[-1][0]:.2f}, {pts[-1][1]:.2f}))"

    def _check_motion(self, tool: str, args: Dict[str, Any], cfg: Dict[str, Any], topics_cfg: Dict[str, Any]) -> Dict[str, Any]:
        notes: List[str] = []
        patch: Dict[str, Any] = {}
        clamp = str(cfg.get("over_limit_action", "block")).lower() == "clamp"

        if tool == "ros_drive":
            topic = norm_name(args.get("topic") or topics_cfg.get("cmd_vel", "/cmd_vel"))
            lin = (_num(args.get("linear", 0.0), "linear", 0.0), 0.0, 0.0)
            ang = (0.0, 0.0, _num(args.get("angular", 0.0), "angular", 0.0))
            msg_type = topics_cfg.get("cmd_vel_type", "geometry_msgs/Twist")
            message: Any = None
            twist: Optional[tuple] = (lin, ang)
            is_motion_topic = True
        else:
            topic = norm_name(args.get("topic"))
            msg_type = str(args.get("msg_type") or "")
            message = args.get("message")
            if not isinstance(message, dict):
                raise Blocked("invalid_args", "message must be an object (the ROS message as JSON)")
            twist = extract_twist(message)
            is_motion_topic = matches_any(topic, cfg.get("motion_topic_patterns") or [])
            if twist is None and (is_motion_topic or msg_type in TWIST_TYPES):
                raise Blocked("unparseable_motion_message",
                              f"{topic} looks like a velocity topic but the message is not a readable Twist/TwistStamped; "
                              "refusing because its speed cannot be verified")
            lin, ang = twist if twist else ((0.0,) * 3, (0.0,) * 3)

        self._check_allowlist("topic", topic, cfg)

        duration_arg = args.get("duration_s")
        rate_arg = args.get("rate_hz")
        max_dur = float(cfg.get("max_duration_s", 5.0))
        default_dur = float(cfg.get("default_duration_s", 1.0))

        if twist is None:  # non-motion publish to an allowlisted topic
            if self.latched:
                raise Blocked("estop_latched", f"e-stop is latched ({self.estop_status().get('reason')}); writes are blocked until a human clears it with /ros clear")
            if duration_arg is not None:
                d = _num(duration_arg, "duration_s")
                if d > max_dur:
                    if not clamp:
                        raise Blocked("over_duration", f"duration {d:.1f}s exceeds max {max_dur:.1f}s")
                    patch["duration_s"] = max_dur
                    notes.append(f"duration clamped {d:.1f}s -> {max_dur:.1f}s")
            return {"patch": patch, "notes": notes}

        lin_mag = math.sqrt(sum(c * c for c in lin))
        ang_mag = math.sqrt(sum(c * c for c in ang))
        if lin_mag == 0.0 and ang_mag == 0.0:
            notes.append("zero-velocity command (stop) always allowed")
            return {"patch": patch, "notes": notes}

        if self.latched:
            raise Blocked("estop_latched", f"e-stop is latched ({self.estop_status().get('reason')}); motion is blocked "
                          "until a human clears it with /ros clear")

        max_lin, max_ang = float(cfg.get("max_linear_mps", 0.5)), float(cfg.get("max_angular_rps", 1.0))
        if lin_mag > max_lin + 1e-9:
            if not clamp:
                raise Blocked("over_speed_linear", f"linear speed {lin_mag:.2f} m/s exceeds the limit of {max_lin:.2f} m/s")
            s = max_lin / lin_mag
            lin = tuple(c * s for c in lin)  # type: ignore[assignment]
            notes.append(f"linear speed clamped {lin_mag:.2f} -> {max_lin:.2f} m/s")
        if ang_mag > max_ang + 1e-9:
            if not clamp:
                raise Blocked("over_speed_angular", f"angular speed {ang_mag:.2f} rad/s exceeds the limit of {max_ang:.2f} rad/s")
            s = max_ang / ang_mag
            ang = tuple(c * s for c in ang)  # type: ignore[assignment]
            notes.append(f"angular speed clamped {ang_mag:.2f} -> {max_ang:.2f} rad/s")

        if duration_arg is None:
            duration = default_dur
            notes.append(f"no duration given; defaulted to {default_dur:.1f}s")
        else:
            duration = _num(duration_arg, "duration_s")
            if duration <= 0:
                raise Blocked("invalid_args", "duration_s must be > 0 for a moving command")
            if duration > max_dur + 1e-9:
                if not clamp:
                    raise Blocked("over_duration", f"duration {duration:.1f}s exceeds the maximum of {max_dur:.1f}s")
                notes.append(f"duration clamped {duration:.1f}s -> {max_dur:.1f}s")
                duration = max_dur
        patch["duration_s"] = duration

        max_rate = float(cfg.get("max_publish_rate_hz", 30.0))
        if rate_arg is not None:
            rate = _num(rate_arg, "rate_hz")
            if rate <= 0:
                raise Blocked("invalid_args", "rate_hz must be > 0")
            if rate > max_rate:
                patch["rate_hz"] = max_rate
                notes.append(f"rate clamped {rate:.0f} -> {max_rate:.0f} Hz")

        gf_note = self._check_geofence(cfg, lin[0], lin[1], ang[2], duration)
        if gf_note:
            notes.append(gf_note)

        if tool == "ros_drive":
            patch["linear"], patch["angular"] = lin[0], ang[2]
        else:
            patch["message"] = set_twist(message, lin, ang)  # type: ignore[arg-type]
        return {"patch": patch, "notes": notes}

    # -------------------------------------------------------------- the hook
    def evaluate(self, tool: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Return ``{"patch": {...}, "notes": [...]}`` or raise :class:`Blocked`."""
        settings = self._settings_fn()
        cfg, topics_cfg = settings["safety"], settings["topics"]
        if tool in MOTION_TOOLS:
            return self._check_motion(tool, args, cfg, topics_cfg)
        if tool in SERVICE_TOOLS:
            name = norm_name(args.get("service"))
            if not isinstance(args.get("request", {}) or {}, dict):
                raise Blocked("invalid_args", "request must be an object")
            self._check_allowlist("service", name, cfg)
            if self.latched:
                raise Blocked("estop_latched", f"e-stop is latched ({self.estop_status().get('reason')}); service calls are blocked until a human clears it with /ros clear")
            return {"patch": {}, "notes": []}
        return {"patch": {}, "notes": []}

    def pre_tool_call(self, tool_name: str = "", args: Optional[Dict[str, Any]] = None, **kwargs: Any) -> Optional[Dict[str, Any]]:
        """Hermes ``pre_tool_call`` callback. Fails closed for robot-writing tools."""
        if tool_name not in ALL_TOOLS:
            return None
        args = args if isinstance(args, dict) else {}
        if tool_name in READ_TOOLS or tool_name in STOP_TOOLS:
            return None
        try:
            result = self.evaluate(tool_name, args)
        except Blocked as b:
            self.record_block(tool_name, args, b.rule, b.reason)
            return {"action": "block", "message": f"hermes-ros interlock BLOCKED {tool_name} [{b.rule}]: {b.reason}"}
        except Exception as exc:  # never fail open
            logger.exception("interlock error")
            self.record_block(tool_name, args, "interlock_error", repr(exc))
            return {"action": "block", "message": f"hermes-ros interlock BLOCKED {tool_name} [interlock_error]: {exc!r} (failing closed)"}
        with self._lock:
            self.counters["modified" if result["notes"] else "allowed"] += 1
        patch = dict(result["patch"])
        patch["_interlock"] = {"nonce": self.nonce, "notes": result["notes"]}
        if result["notes"]:
            logger.info("%s adjusted/annotated: %s", tool_name, "; ".join(result["notes"]))
        return {"action": "modify", "args": patch}

    def verify_token(self, args: Dict[str, Any]) -> List[str]:
        """Handlers of motion/service tools call this: refuse to act unless the hook vetted the call."""
        tok = args.get("_interlock")
        if not isinstance(tok, dict) or tok.get("nonce") != self.nonce:
            raise Blocked("interlock_not_applied", "this call was not vetted by the hermes-ros safety interlock "
                          "(pre_tool_call hook did not run); refusing to act")
        return list(tok.get("notes") or [])
