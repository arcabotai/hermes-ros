"""Tool handlers. Every handler returns a JSON string and never raises."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from . import imaging
from .backend_base import BackendError
from .runtime import Runtime
from .safety import Blocked, norm_name, set_twist

ZERO_TWIST = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}


def _ok(**kw: Any) -> str:
    return json.dumps({"ok": True, **kw}, default=str)


def _err(msg: str, **kw: Any) -> str:
    return json.dumps({"ok": False, "error": msg, **kw}, default=str)


def summarize(obj: Any, max_str: int = 160, max_list: int = 16, depth: int = 0) -> Any:
    """Shrink big ROS messages (image data, covariance matrices) so they fit in the model context."""
    if isinstance(obj, dict):
        return {k: summarize(v, max_str, max_list, depth + 1) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        if len(obj) > max_list:
            return [summarize(v, max_str, max_list, depth + 1) for v in obj[:max_list]] + [f"… (+{len(obj) - max_list} more)"]
        return [summarize(v, max_str, max_list, depth + 1) for v in obj]
    if isinstance(obj, str) and len(obj) > max_str:
        return obj[:max_str] + f"… ({len(obj)} chars)"
    if isinstance(obj, float):
        return round(obj, 5)
    return obj


def _as_float(v: Any, default: float) -> float:
    try:
        f = float(v)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default


def _twist_for(rt: Runtime, linear: float, angular: float) -> Dict[str, Any]:
    """Build the cmd_vel message in the configured type (Twist or TwistStamped)."""
    msg = {"linear": {"x": linear, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": angular}}
    if "Stamped" in str(rt.settings["topics"].get("cmd_vel_type", "")):
        return {"header": {"frame_id": "base_link"}, "twist": msg}
    return msg


def _send_stop(rt: Runtime, topic: str, msg_type: str, stamped: bool, n: int = 3) -> None:
    be = rt.backend()
    zero = {"header": {"frame_id": "base_link"}, "twist": ZERO_TWIST} if stamped else ZERO_TWIST
    for _ in range(n):
        try:
            be.publish(topic, msg_type, zero)
            be.sleep(0.02)
        except BackendError:
            break


def _run_timed(rt: Runtime, topic: str, msg_type: str, message: Dict[str, Any], duration: float, rate: float,
               is_twist: bool) -> Dict[str, Any]:
    """Publish ``message`` at ``rate`` Hz for ``duration`` s, aborting on e-stop or a geofence breach, then stop."""
    be = rt.backend()
    cfg = rt.settings.safety
    gf = cfg.get("geofence") or {}
    period = 1.0 / rate
    elapsed, sent, reason = 0.0, 0, "completed"
    try:
        while elapsed < duration - 1e-9:
            if rt.interlock.latched:
                reason = "aborted: e-stop latched"
                break
            if is_twist and gf.get("enabled"):
                pose = be.latest_pose()
                if pose and not (gf["x_min"] <= pose["x"] <= gf["x_max"] and gf["y_min"] <= pose["y"] <= gf["y_max"]):
                    reason = "aborted: robot left the geofence"
                    rt.interlock.record_block("ros_drive", {"runtime": True}, "geofence_runtime", reason)
                    break
            be.publish(topic, msg_type, message)
            sent += 1
            step = min(period, duration - elapsed)
            be.sleep(step)
            elapsed += step
    finally:
        if is_twist:
            _send_stop(rt, topic, msg_type, "twist" in message and "linear" not in message)
    return {"published": sent, "ran_for_s": round(elapsed, 2), "end_reason": reason}


def make_handlers(rt: Runtime) -> Dict[str, Callable[..., str]]:
    def guarded(fn: Callable[[Dict[str, Any]], str]) -> Callable[..., str]:
        def handler(args: Dict[str, Any], **kwargs: Any) -> str:
            args = args if isinstance(args, dict) else {}
            try:
                return fn(args)
            except Blocked as b:
                rt.interlock.record_block(fn.__name__, args, b.rule, b.reason)
                return _err(f"hermes-ros interlock BLOCKED [{b.rule}]: {b.reason}")
            except BackendError as exc:
                return _err(str(exc))
            except Exception as exc:  # tool contract: always JSON, never raise
                return _err(f"{type(exc).__name__}: {exc}")
        handler.__name__ = fn.__name__
        return handler

    def ros_list(a: Dict[str, Any]) -> str:
        kind = (a.get("kind") or "all").lower()
        flt = (a.get("filter") or "").lower()
        be = rt.backend()

        def keep(item: Any) -> bool:
            s = (item if isinstance(item, str) else f"{item['name']} {item.get('type', '')}").lower()
            return not flt or flt in s

        out: Dict[str, Any] = {"backend": be.name, "via": be.description}
        if kind in ("topics", "all"):
            out["topics"] = [t for t in be.list_topics() if keep(t)]
        if kind in ("services", "all"):
            out["services"] = [s for s in be.list_services() if keep(s)]
        if kind in ("nodes", "all"):
            out["nodes"] = [n for n in be.list_nodes() if keep(n)]
        return _ok(**out)

    def ros_echo(a: Dict[str, Any]) -> str:
        topic = norm_name(a.get("topic"))
        msg = rt.backend().echo_once(topic, a.get("msg_type") or None, _as_float(a.get("timeout_s"), 5.0))
        return _ok(topic=topic, message=summarize(msg))

    def _result_pose(label: str) -> Optional[Dict[str, Any]]:
        try:
            return rt.backend().latest_pose()
        except BackendError:
            return None

    def ros_drive(a: Dict[str, Any]) -> str:
        notes = rt.interlock.verify_token(a)
        cfg = rt.settings.snapshot()
        topic = norm_name(a.get("topic") or cfg["topics"]["cmd_vel"])
        msg_type = cfg["topics"].get("cmd_vel_type", "geometry_msgs/Twist")
        linear, angular = _as_float(a.get("linear"), 0.0), _as_float(a.get("angular"), 0.0)
        duration = _as_float(a.get("duration_s"), cfg["safety"]["default_duration_s"])
        before = _result_pose("before")
        run = _run_timed(rt, topic, msg_type, _twist_for(rt, linear, angular), duration, 10.0, True)
        after = _result_pose("after")
        rt.record_pose(after)
        disp = None
        if before and after:
            disp = round(math.hypot(after["x"] - before["x"], after["y"] - before["y"]), 3)
        return _ok(topic=topic, command={"linear": linear, "angular": angular, "duration_s": duration}, **run,
                   interlock=notes, pose_before=before, pose_after=after, displacement_m=disp,
                   collided=(after or {}).get("collided"))

    def ros_publish(a: Dict[str, Any]) -> str:
        notes = rt.interlock.verify_token(a)
        topic, msg_type, message = norm_name(a.get("topic")), str(a.get("msg_type")), a.get("message")
        from .safety import extract_twist
        twist = extract_twist(message)
        rate = _as_float(a.get("rate_hz"), 10.0)
        rate = max(0.1, min(rate, float(rt.settings.safety.get("max_publish_rate_hz", 30.0))))
        duration = a.get("duration_s")
        if duration is None:
            rt.backend().publish(topic, msg_type, message)
            return _ok(topic=topic, published=1, interlock=notes)
        run = _run_timed(rt, topic, msg_type, message, float(duration), rate, twist is not None)
        return _ok(topic=topic, interlock=notes, **run)

    def ros_call_service(a: Dict[str, Any]) -> str:
        notes = rt.interlock.verify_token(a)
        service = norm_name(a.get("service"))
        res = rt.backend().call_service(service, a.get("srv_type") or None, a.get("request") or {}, _as_float(a.get("timeout_s"), 8.0))
        return _ok(service=service, response=summarize(res), interlock=notes)

    def ros_get_pose(a: Dict[str, Any]) -> str:
        pose = rt.backend().get_pose(a.get("topic") or None, None, _as_float(a.get("timeout_s"), 5.0))
        rt.record_pose(pose)
        return _ok(pose=pose)

    def ros_camera_snapshot(a: Dict[str, Any]) -> str:
        cfg = rt.settings.snapshot()["camera"]
        data, source = rt.backend().camera_frame(a.get("topic") or None, _as_float(a.get("timeout_s"), cfg["timeout_s"]))
        mime, ext, size = imaging.sniff_image(data)
        out_dir = rt.snapshot_dir()
        path = out_dir / f"ros-snapshot-{time.strftime('%Y%m%d-%H%M%S')}-{int((time.time() % 1) * 1000):03d}.{ext}"
        path.write_bytes(data)
        _prune(out_dir, int(cfg.get("keep_last", 20)))
        info = {"path": str(path), "mime": mime, "bytes": len(data), "source": source,
                "width": size[0] if size else None, "height": size[1] if size else None,
                "media_tag": f"MEDIA:{path}",
                "next_step": f"Call vision_analyze with image_url='{path}' to describe what the robot sees. "
                             f"To show the picture to the user, include the line MEDIA:{path} in your reply."}
        text = _ok(**info)
        if str(cfg.get("return_mode", "path")).lower() == "inline":
            import base64
            return {"_multimodal": True,  # type: ignore[return-value]
                    "content": [{"type": "text", "text": text},
                                {"type": "image_url", "image_url": {"url": f"data:{mime};base64,{base64.b64encode(data).decode()}"}}],
                    "text_summary": text}
        return text

    def ros_stop(a: Dict[str, Any]) -> str:
        cfg = rt.settings.snapshot()
        topic = norm_name(cfg["topics"]["cmd_vel"])
        err = None
        try:
            _send_stop(rt, topic, cfg["topics"].get("cmd_vel_type", "geometry_msgs/Twist"),
                       "Stamped" in cfg["topics"].get("cmd_vel_type", ""), n=5)
        except Exception as exc:  # still latch if requested
            err = f"{type(exc).__name__}: {exc}"
        latched = None
        if a.get("estop"):
            latched = rt.interlock.engage_estop(str(a.get("reason") or "agent requested e-stop"), source="agent")
        res = {"stop_sent_to": topic, "estop": latched or rt.interlock.estop_status()}
        if err:
            return _err(f"could not reach the robot to send zero velocity ({err}); e-stop latched={bool(latched)}", **res)
        return _ok(**res, note="e-stop latched: only a human can clear it with /ros clear" if latched else "velocity zeroed (no latch)")

    def ros_status(a: Dict[str, Any]) -> str:
        cfg = rt.settings.snapshot()
        try:
            be = rt.backend().status()
        except BackendError as exc:
            be = {"error": str(exc)}
        s = cfg["safety"]
        return _ok(backend=be, estop=rt.interlock.estop_status(),
                   limits={"max_linear_mps": s["max_linear_mps"], "max_angular_rps": s["max_angular_rps"],
                           "max_duration_s": s["max_duration_s"], "over_limit_action": s["over_limit_action"]},
                   geofence=s["geofence"], write_allowlist=s["write_allowlist"], counters=rt.interlock.counters,
                   recent_blocked=rt.interlock.recent_blocked(5))

    fns = [ros_list, ros_echo, ros_drive, ros_publish, ros_call_service, ros_get_pose, ros_camera_snapshot, ros_stop, ros_status]
    return {f.__name__: guarded(f) for f in fns}


def _prune(directory: Path, keep: int) -> None:
    try:
        files = sorted((p for p in directory.glob("ros-snapshot-*") if p.is_file()), key=lambda p: p.stat().st_mtime)
        for p in files[:-max(1, keep)]:
            p.unlink(missing_ok=True)
    except OSError:
        pass
