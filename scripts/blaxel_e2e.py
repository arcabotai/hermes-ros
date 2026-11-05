#!/usr/bin/env python3
"""Prove hermes-ros against REAL ROS 2 on Blaxel (a cloud sandbox) instead of your own machine.

Blaxel sandboxes are Alpine microVMs without docker, so this builds an Ubuntu 24.04 chroot, installs ROS 2 Jazzy +
rosbridge_server + turtlesim, starts the demo robot (docker/run_demo.sh), runs the full pytest suite with the live tests
enabled, copies the logs and camera snapshots back, and ALWAYS deletes the sandbox.

    export BLAXEL_API_KEY=...          # never printed
    python scripts/blaxel_e2e.py [--ref main] [--out runs/e2e] [--memory 4096]

Requires `pip install blaxel`. Cost: ~4 GB RAM for roughly 5 minutes. A 45 minute server-side TTL is the safety net.
"""
import argparse
import asyncio
import base64
import json
import os
import sys
import time
import uuid
import urllib.request
from pathlib import Path

API = "https://api.blaxel.ai/v0"
REPO = "https://github.com/arcabotai/hermes-ros"
HERE = Path(__file__).resolve().parent

REMOTE = r"""#!/bin/bash
set -u
L=/tmp/logs; mkdir -p $L; RC=0
say() { echo "=== $*"; }
say "1/6 host packages"; apk add --no-cache curl tar git bash xz >$L/apk.log 2>&1 || { tail -5 $L/apk.log; exit 1; }
say "2/6 ubuntu 24.04 rootfs"; mkdir -p /ros
curl -fsSL https://cdimage.ubuntu.com/ubuntu-base/releases/24.04/release/ubuntu-base-24.04.5-base-amd64.tar.gz | tar -xz -C /ros || exit 1
mount -t proc proc /ros/proc; mount --rbind /dev /ros/dev; cp /etc/resolv.conf /ros/etc/resolv.conf
say "3/6 ROS 2 jazzy + rosbridge + turtlesim"; cp /tmp/chroot_setup.sh /ros/tmp/; chroot /ros /bin/bash /tmp/chroot_setup.sh >$L/ros_setup.log 2>&1 || { tail -20 $L/ros_setup.log; exit 1; }
tail -2 $L/ros_setup.log
say "4/6 plugin @ $REF"; chroot /ros /bin/bash -c "git clone -q $REPO /opt/hermes-ros && cd /opt/hermes-ros && git checkout -q $REF && git log -1 --format='%H %s'"
say "5/6 start the demo robot"
nohup chroot /ros /bin/bash /opt/hermes-ros/docker/run_demo.sh > $L/demo.log 2>&1 &
chroot /ros /bin/bash -c 'for i in $(seq 1 60); do python3 -c "import socket;socket.create_connection((\"127.0.0.1\",9090),1)" 2>/dev/null && exit 0; sleep 1; done; exit 1' || { tail -20 $L/demo.log; exit 1; }
sleep 4; tail -5 $L/demo.log
say "6/6 pytest: unit + live (real ROS 2 over rosbridge)"
chroot /ros /bin/bash -c 'python3 -m venv /opt/venv && /opt/venv/bin/pip install -q websocket-client websockets pytest pyyaml && mkdir -p /tmp/artifacts && cd /opt/hermes-ros/tests && HERMES_ROS_ARTIFACTS=/tmp/artifacts HERMES_ROS_LIVE=127.0.0.1:9090 /opt/venv/bin/python -m pytest -rA -p no:cacheprovider' 2>&1 | tee $L/pytest.log | grep -E "PASSED|FAILED|ERROR|passed|failed|error" | sed 's/^PASSED /  ok  /'
RC=${PIPESTATUS[0]}
cd /ros/tmp/artifacts 2>/dev/null && tar -czf /tmp/artifacts.tgz . ; cp /ros/tmp/artifacts.tgz /tmp/ 2>/dev/null
grep -qE "[0-9]+ (failed|errors?)( |,|$)" $L/pytest.log && RC=1
echo "RC=$RC" | tee $L/rc.txt
"""


def api(path):
    req = urllib.request.Request(API + path, headers={"Authorization": f"Bearer {os.environ['BLAXEL_API_KEY']}"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def setup_env():
    if not os.environ.get("BLAXEL_API_KEY"):
        sys.exit("BLAXEL_API_KEY not set")
    ws = os.environ.get("BL_WORKSPACE") or api("/workspaces")[0]
    if isinstance(ws, dict):
        ws = ws["name"]
    os.environ["BL_WORKSPACE"], os.environ["BL_API_KEY"] = ws, os.environ["BLAXEL_API_KEY"]
    return ws


async def main(a):
    ws = setup_env()
    from blaxel.core.sandbox import SandboxInstance
    name = f"hermes-ros-e2e-{uuid.uuid4().hex[:8]}"
    out = Path(a.out or f"runs/{time.strftime('%Y%m%d-%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    print(f"[blaxel-e2e] workspace={ws} sandbox={name} memory={a.memory}MB ref={a.ref}")
    rc, t0, sb = 2, time.time(), None
    try:
        sb = await SandboxInstance.create({"name": name, "image": "blaxel/base-image:latest", "memory": a.memory,
                                           "ttl": "45m", "region": os.environ.get("BL_REGION", "us-was-1"),
                                           "labels": {"purpose": "hermes-ros-e2e"}})
        await sb.fs.write("/tmp/chroot_setup.sh", (HERE / "chroot_setup.sh").read_text())
        await sb.fs.write("/tmp/run.sh", f"export REF={a.ref} REPO={REPO}\n" + REMOTE)
        try:
            await sb.process.exec({"name": "run", "command": "bash /tmp/run.sh", "wait_for_completion": True,
                                   "timeout": a.timeout, "keep_alive": True,
                                   "on_log": lambda l: print(l, end="" if l.endswith("\n") else "\n", flush=True)})
        except Exception as exc:  # the stream can drop; the logs are on disk
            print(f"[blaxel-e2e] stream error ({type(exc).__name__}); fetching logs anyway")
        for n in ("pytest", "demo", "ros_setup", "rc"):
            try:
                path = "/tmp/logs/rc.txt" if n == "rc" else f"/tmp/logs/{n}.log"
                (out / f"{n}.log").write_text(await sb.fs.read(path))
            except Exception:
                pass
        try:
            p = await sb.process.exec({"name": "b64", "command": "base64 -w0 /tmp/artifacts.tgz", "wait_for_completion": True, "timeout": 60})
            blob = (getattr(p, "logs", "") or "").strip()
            if blob:
                (out / "artifacts.tgz").write_bytes(base64.b64decode(blob))
        except Exception:
            pass
        rc_txt = (out / "rc.log").read_text() if (out / "rc.log").exists() else ""
        rc = 0 if "RC=0" in rc_txt else 1 if "RC=1" in rc_txt else 2
    finally:
        for i in range(5):
            try:
                await SandboxInstance.delete(name)
                print(f"[blaxel-e2e] deleted {name}")
                break
            except Exception as exc:
                if "404" in str(exc) or "not found" in str(exc).lower():
                    break
                await asyncio.sleep(3)
        print(f"[blaxel-e2e] elapsed={time.time() - t0:.0f}s logs={out} rc={rc}")
    return rc


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", default="main", help="git ref of the plugin to test (default main)")
    ap.add_argument("--memory", type=int, default=4096)
    ap.add_argument("--timeout", type=int, default=1500)
    ap.add_argument("--out")
    sys.exit(asyncio.run(main(ap.parse_args())))
