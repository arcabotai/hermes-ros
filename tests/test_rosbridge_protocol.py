"""The rosbridge client + backend against an in-process fake rosbridge server (no ROS needed, runs in CI).

The fake speaks the real rosbridge v2 JSON protocol: rosapi services, subscribe/unsubscribe, advertise/publish.
The live suite (tests/live) covers a real ROS 2 rosbridge.
"""
import json
import threading
import time

import pytest

pytest.importorskip("websocket")
sync_server = pytest.importorskip("websockets.sync.server")

from conftest import Harness  # noqa: E402
from helpers import GEOFENCE as _G  # noqa: E402

GEOFENCE = _G["safety"]["geofence"]


class FakeRosbridge:
    def __init__(self):
        self.received_twists = []
        self.pose = {"x": 0.0, "y": 0.0}
        self.unsub_count = 0
        self.log = []
        self.clients = []
        self._stop = threading.Event()
        self.server = sync_server.serve(self._handle, "127.0.0.1", 0)
        self.port = self.server.socket.getsockname()[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self._stop.set()
        threading.Thread(target=self.server.shutdown, daemon=True).start()  # may linger on a half-closed client

    def drop_clients(self):
        for ws in list(self.clients):  # the server-side close handshake can linger: do it off the test thread
            threading.Thread(target=lambda w=ws: w.close(), daemon=True).start()

    def _handle(self, ws):
        self.clients.append(ws)
        subs = {}
        lock = threading.Lock()

        def send(obj):
            with lock:
                try:
                    ws.send(json.dumps(obj))
                except Exception:
                    pass

        def feeder():
            while not self._stop.is_set():
                for topic in list(subs):
                    msg = {"pose": {"pose": {"position": {"x": self.pose["x"], "y": self.pose["y"], "z": 0.0},
                                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1.0}}},
                           "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
                           "header": {"frame_id": "odom"}}
                    send({"op": "publish", "topic": topic, "msg": msg})
                time.sleep(0.05)

        threading.Thread(target=feeder, daemon=True).start()
        try:
            for raw in ws:
                m = json.loads(raw)
                self.log.append(m)
                op = m["op"]
                if op == "subscribe":
                    subs[m["topic"]] = m
                elif op == "unsubscribe":
                    self.unsub_count += 1
                    subs.pop(m["topic"], None)  # like rosbridge: drops *all* of this client's subs on the topic
                elif op == "publish" and m["topic"] == "/cmd_vel":
                    self.received_twists.append(m["msg"])
                elif op == "call_service":
                    v, ok = self._service(m["service"], m.get("args", {}))
                    send({"op": "service_response", "service": m["service"], "id": m["id"], "values": v, "result": ok})
        except Exception:
            pass

    @staticmethod
    def _service(name, args):
        if name == "/rosapi/topics_and_raw_types":
            return {"topics": ["/cmd_vel", "/odom"], "types": ["geometry_msgs/Twist", "nav_msgs/Odometry"]}, True
        if name == "/rosapi/topic_type":
            return {"type": {"/odom": "nav_msgs/Odometry", "/cmd_vel": "geometry_msgs/Twist"}.get(args.get("topic"), "")}, True
        if name == "/rosapi/nodes":
            return {"nodes": ["/fake_base"]}, True
        if name == "/rosapi/services":
            return {"services": ["/reset"]}, True
        if name == "/rosapi/service_type":
            return {"type": "std_srvs/Empty"}, True
        if name == "/reset":
            return {}, True
        if name == "/explode":
            return "service blew up", False
        return "no such service", False


@pytest.fixture
def fake():
    f = FakeRosbridge()
    yield f
    f.close()


def overrides(port, **safety):
    return {"backend": "rosbridge", "rosbridge": {"host": "127.0.0.1", "port": port, "connect_timeout_s": 3.0, "call_timeout_s": 3.0},
            "safety": {"max_duration_s": 3.0, "write_allowlist": {"topics": ["/cmd_vel"], "services": ["/reset", "/explode"]}, **safety}}


@pytest.fixture
def bot(fake, tmp_path):
    h = Harness(tmp_path, overrides(fake.port))
    yield h
    h.rt.shutdown()


def test_discovery_over_the_wire(bot):
    r = bot.call("ros_list")
    assert r["ok"]
    assert {"name": "/odom", "type": "nav_msgs/Odometry"} in r["topics"]
    assert r["nodes"] == ["/fake_base"]


def test_pose_comes_from_the_odom_feed(bot, fake):
    fake.pose.update(x=1.5, y=-0.5)
    r = bot.call("ros_get_pose")
    assert r["ok"] and r["pose"]["x"] == pytest.approx(1.5) and r["pose"]["y"] == pytest.approx(-0.5)


def test_drive_publishes_twists_then_zero_twists(bot, fake):
    r = bot.call("ros_drive", {"linear": 0.3, "duration_s": 0.5})
    assert r["ok"], r
    moving = [t for t in fake.received_twists if t["linear"]["x"] == pytest.approx(0.3)]
    assert 3 <= len(moving) <= 8  # ~10 Hz for 0.5 s
    assert fake.received_twists[-1]["linear"]["x"] == 0.0 and fake.received_twists[-1]["angular"]["z"] == 0.0
    adverts = [m for m in fake.log if m["op"] == "advertise"]
    assert len(adverts) == 1 and adverts[0]["type"] == "geometry_msgs/Twist"


def test_blocked_drive_never_reaches_the_wire(bot, fake):
    r = bot.call("ros_drive", {"linear": 2.0, "duration_s": 1})
    assert r["blocked"]
    assert not [t for t in fake.received_twists if t["linear"]["x"]]


def test_one_shot_echo_of_odom_does_not_kill_the_persistent_feed(fake, tmp_path):
    """Regression: rosbridge drops all of a client's subscriptions on a topic when one unsubscribes."""
    bot = Harness(tmp_path, overrides(fake.port, geofence={**GEOFENCE, "enabled": True}))
    try:
        assert bot.call("ros_drive", {"linear": 0.1, "duration_s": 0.2})["ok"]  # starts the feed
        for _ in range(3):
            assert bot.call("ros_get_pose")["ok"]
            assert bot.call("ros_echo", {"topic": "/odom"})["ok"]
        assert fake.unsub_count == 0  # served from the feed, never a one-shot subscribe/unsubscribe on /odom
        r = bot.call("ros_drive", {"linear": 0.1, "duration_s": 0.2})
        assert r["ok"], r  # geofence still sees fresh odom
    finally:
        bot.rt.shutdown()


def test_geofence_uses_live_odom_from_the_wire(fake, tmp_path):
    bot = Harness(tmp_path, overrides(fake.port, geofence={**GEOFENCE, "enabled": True}))
    try:
        fake.pose.update(x=GEOFENCE["x_max"] - 0.1, y=0.0)
        assert bot.call("ros_get_pose")["ok"]
        r = bot.call("ros_drive", {"linear": 0.4, "duration_s": 2})
        assert r["blocked"] and "geofence" in r["message"]
    finally:
        bot.rt.shutdown()


def test_service_calls_allowlist_and_failures(bot):
    assert bot.call("ros_call_service", {"service": "/reset"})["ok"]
    failed = bot.call("ros_call_service", {"service": "/explode"})
    assert not failed["blocked"] and not failed["ok"] and "blew up" in json.dumps(failed)
    assert bot.call("ros_call_service", {"service": "/rosapi/nodes"})["blocked"]  # not allowlisted


def test_unreachable_rosbridge_is_a_clean_error(tmp_path):
    bot = Harness(tmp_path, overrides(1))  # nothing listens on port 1
    try:
        r = bot.call("ros_get_pose")
        assert not r["ok"] and "rosbridge" in json.dumps(r).lower()
    finally:
        bot.rt.shutdown()


def test_server_going_away_fails_calls_fast_and_reconnects(fake, tmp_path):
    bot = Harness(tmp_path, overrides(fake.port))
    try:
        assert bot.call("ros_list")["ok"]
        fake.drop_clients()
        time.sleep(0.3)
        assert bot.call("ros_list")["ok"]  # the dead link is detected and a new one is opened transparently
    finally:
        bot.rt.shutdown()
