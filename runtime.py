"""Plugin runtime: settings, backend lifecycle, interlock, data directory. One instance per plugin load."""
from __future__ import annotations

import json
import logging
import os
import tempfile
import time
import threading
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from .backend_base import Backend, BackendError
from .config import Settings
from .safety import Interlock

logger = logging.getLogger("hermes_ros")


class Runtime:
    def __init__(self, get_config: Optional[Callable[[str, Any], Any]] = None,
                 data_dir: Optional[Path] = None, overrides: Optional[Dict[str, Any]] = None):
        self.settings = Settings(get_config, overrides)
        self._data_dir = Path(data_dir) if data_dir else None
        self._backend: Optional[Backend] = None
        self._backend_key: Any = None
        self._lock = threading.RLock()
        self.interlock = Interlock(self.settings.snapshot, self._latest_pose_for_interlock, lambda: self.data_dir)

    # ----------------------------------------------------------------- paths
    @property
    def data_dir(self) -> Path:
        if self._data_dir is None:  # only if the host gave us no data dir (e.g. bare unit use)
            self._data_dir = Path(tempfile.mkdtemp(prefix="hermes-ros-"))
        return self._data_dir

    def snapshot_dir(self) -> Path:
        custom = (self.settings["camera"].get("output_dir") or "").strip()
        d = Path(custom).expanduser() if custom else self.data_dir / "snapshots"
        d.mkdir(parents=True, exist_ok=True)
        return d

    # --------------------------------------------------------------- backend
    def backend(self) -> Backend:
        cfg = self.settings.snapshot()
        kind = str(cfg["backend"]).lower()
        key = (kind, cfg["rosbridge"]["host"], cfg["rosbridge"]["port"]) if kind == "rosbridge" else (kind,)
        with self._lock:
            if self._backend is None or key != self._backend_key:
                if self._backend is not None:
                    try:
                        self._backend.close()
                    except Exception:
                        pass
                if kind == "mock":
                    from .backend_mock import MockBackend
                    self._backend = MockBackend(cfg["mock"], self.data_dir)
                elif kind == "rosbridge":
                    from .backend_rosbridge import RosbridgeBackend
                    self._backend = RosbridgeBackend(cfg["rosbridge"], cfg["topics"])
                else:
                    raise BackendError(f"unknown backend {kind!r}; set plugins.entries.hermes-ros.settings.backend to 'mock' or 'rosbridge'")
                self._backend_key = key
            elif kind == "rosbridge":
                self._backend.topics_cfg = cfg["topics"]  # type: ignore[attr-defined]  # live topic edits
            return self._backend

    def _latest_pose_for_interlock(self) -> Optional[Dict[str, Any]]:
        try:
            return self.backend().latest_pose()
        except BackendError:
            return None

    def record_pose(self, pose: Optional[Dict[str, Any]]) -> None:
        """Remember the latest pose in ``last_pose.json`` so the dashboard tab (a separate process) can show it."""
        if not pose:
            return
        try:
            path = self.data_dir / "last_pose.json"
            tmp = path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"pose": pose, "ts": time.time(), "backend": self.settings["backend"]}))
            os.replace(tmp, path)
        except Exception:
            pass  # purely informational

    def shutdown(self) -> None:
        with self._lock:
            if self._backend is not None:
                try:
                    self._backend.close()
                except Exception:
                    pass
                self._backend = None
