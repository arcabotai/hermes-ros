"""hermes-ros dashboard backend, mounted at /api/plugins/hermes-ros/. Read-only.

The dashboard is a separate process from the agent, so it reads what the plugin leaves in its data directory:
blocked.jsonl (audit log), estop.json (latch), last_pose.json, snapshots/ and the effective settings.
It never moves the robot and never clears the e-stop.
"""
from __future__ import annotations

import base64
import importlib.util
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter

router = APIRouter()
_PLUGIN_ROOT = Path(__file__).resolve().parents[1]
_MAX_IMAGE_BYTES = 1_500_000


def _hermes_home() -> Path:
    try:
        from hermes_constants import get_hermes_home
        return Path(get_hermes_home())
    except Exception:
        return Path(os.environ.get("HERMES_HOME") or Path.home() / ".hermes")


def _data_dir() -> Optional[Path]:
    """The plugin's data dir is plugin-data/<namespace>; the namespace carries the plugin name, so find it by name."""
    root = _hermes_home() / "plugin-data"
    if not root.is_dir():
        return None
    cands = [d for d in root.iterdir() if d.is_dir() and "hermes-ros" in d.name]
    return max(cands, key=lambda d: d.stat().st_mtime) if cands else None


def _defaults() -> Dict[str, Any]:
    spec = importlib.util.spec_from_file_location("_hermes_ros_config", _PLUGIN_ROOT / "config.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # config.py is stdlib-only
    return mod


def _settings() -> Dict[str, Any]:
    cfgmod = _defaults()
    user: Dict[str, Any] = {}
    try:
        from hermes_cli.config import load_config_readonly
        entry = ((load_config_readonly() or {}).get("plugins", {}).get("entries", {}) or {})
        user = (entry.get("hermes-ros") or {}).get("settings") or {}
    except Exception:
        pass
    return cfgmod.deep_merge(cfgmod.DEFAULTS, user if isinstance(user, dict) else {})


def _read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _blocked(data: Path, limit: int) -> List[Dict[str, Any]]:
    path = data / "blocked.jsonl"
    if not path.exists():
        return []
    try:
        lines = path.read_text().splitlines()[-limit:]
    except Exception:
        return []
    out = []
    for line in reversed(lines):
        try:
            e = json.loads(line)
            out.append({k: e.get(k) for k in ("ts", "iso", "tool", "rule", "reason", "args")})
        except Exception:
            continue
    return out


def _latest_snapshot(settings: Dict[str, Any], data: Path) -> Optional[Path]:
    custom = (settings["camera"].get("output_dir") or "").strip()
    d = Path(custom).expanduser() if custom else data / "snapshots"
    if not d.is_dir():
        return None
    files = [f for f in d.iterdir() if f.name.startswith("ros-snapshot-") and f.suffix in (".png", ".jpg", ".jpeg")]
    return max(files, key=lambda f: f.stat().st_mtime) if files else None


@router.get("/state")
def state(since: float = 0.0, blocked: int = 30) -> Dict[str, Any]:
    settings = _settings()
    data = _data_dir()
    safety = settings["safety"]
    out: Dict[str, Any] = {
        "ok": True, "now": time.time(), "backend": settings["backend"],
        "rosbridge": f"ws://{settings['rosbridge']['host']}:{settings['rosbridge']['port']}" if settings["backend"] == "rosbridge" else None,
        "limits": {k: safety[k] for k in ("max_linear_mps", "max_angular_rps", "max_duration_s", "over_limit_action")},
        "geofence": safety["geofence"], "allowlist": safety["write_allowlist"],
        "estop": {"latched": False}, "pose": None, "pose_age_s": None, "blocked": [], "blocked_total": 0, "snapshot": None,
        "data_dir_found": data is not None,
    }
    if data is None:
        return out
    latch = _read_json(data / "estop.json")
    if isinstance(latch, dict) and latch.get("latched"):
        out["estop"] = latch
    lp = _read_json(data / "last_pose.json")
    if isinstance(lp, dict) and lp.get("pose"):
        out["pose"], out["pose_age_s"] = lp["pose"], round(time.time() - float(lp.get("ts", 0)), 1)
    out["blocked"] = _blocked(data, max(1, min(int(blocked), 200)))
    try:
        out["blocked_total"] = sum(1 for _ in (data / "blocked.jsonl").open())
    except Exception:
        pass
    snap = _latest_snapshot(settings, data)
    if snap is not None:
        mtime = snap.stat().st_mtime
        info: Dict[str, Any] = {"name": snap.name, "mtime": mtime, "age_s": round(time.time() - mtime, 1)}
        if mtime != since and snap.stat().st_size <= _MAX_IMAGE_BYTES:
            mime = "image/png" if snap.suffix == ".png" else "image/jpeg"
            info["data_url"] = f"data:{mime};base64," + base64.b64encode(snap.read_bytes()).decode()
        out["snapshot"] = info
    return out
