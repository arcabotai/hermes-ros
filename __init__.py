"""hermes-ros: drive and observe ROS robots from Hermes Agent, with a safety interlock and a built-in mock robot."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from . import schemas
from .commands import make_command
from .runtime import Runtime
from .tools import make_handlers

logger = logging.getLogger(__name__)

_RUNTIME: Optional[Runtime] = None


def get_runtime() -> Optional[Runtime]:
    """The live runtime (used by the dashboard API and tests)."""
    return _RUNTIME


def register(ctx) -> None:
    """Register tools, the pre_tool_call safety interlock, /ros, and the bundled skill. Opens no connections."""
    global _RUNTIME
    try:
        data_dir = Path(ctx.state.data_dir)
    except Exception:
        data_dir = None
    rt = Runtime(get_config=ctx.get_config, data_dir=data_dir)
    _RUNTIME = rt

    handlers = make_handlers(rt)
    for schema in schemas.ALL:
        ctx.register_tool(name=schema["name"], toolset="ros", schema=schema, handler=handlers[schema["name"]],
                          emoji="🤖", description=schema["description"].split(". ")[0])

    ctx.register_hook("pre_tool_call", rt.interlock.pre_tool_call)
    ctx.register_command("ros", make_command(rt), description="Robot link and safety: status, estop, clear, log, snapshot",
                         args_hint="[status|estop|clear|log|snapshot|stop]")

    skill = Path(__file__).parent / "skills" / "ros-safe-operation" / "SKILL.md"
    if skill.exists():
        ctx.register_skill("ros-safe-operation", skill)
    ctx.on_unload(rt.shutdown)
