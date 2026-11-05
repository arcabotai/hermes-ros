"""Backend interface shared by the mock robot and the rosbridge client."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple


class BackendError(RuntimeError):
    """A recoverable backend failure (not connected, timeout, unknown topic, ...)."""


class Backend:
    name = "base"
    description = ""

    # -- discovery
    def list_topics(self) -> List[Dict[str, str]]: raise NotImplementedError
    def list_services(self) -> List[Dict[str, str]]: raise NotImplementedError
    def list_nodes(self) -> List[str]: raise NotImplementedError

    # -- data
    def echo_once(self, topic: str, msg_type: Optional[str], timeout: float) -> Dict[str, Any]: raise NotImplementedError
    def publish(self, topic: str, msg_type: str, message: Dict[str, Any]) -> None: raise NotImplementedError
    def call_service(self, service: str, srv_type: Optional[str], request: Dict[str, Any], timeout: float) -> Dict[str, Any]:
        raise NotImplementedError
    def get_pose(self, topic: Optional[str], msg_type: Optional[str], timeout: float) -> Dict[str, Any]: raise NotImplementedError
    def latest_pose(self) -> Optional[Dict[str, Any]]:
        """Most recent odometry pose without blocking, or None. Includes ``age_s``."""
        raise NotImplementedError
    def camera_frame(self, topic: Optional[str], timeout: float) -> Tuple[bytes, str]:
        """Return ``(png_or_jpeg_bytes, source_description)``."""
        raise NotImplementedError

    # -- timing (the mock advances simulated time here)
    def sleep(self, seconds: float) -> None:
        time.sleep(max(0.0, seconds))

    def close(self) -> None:
        pass

    def status(self) -> Dict[str, Any]:
        return {"backend": self.name}
