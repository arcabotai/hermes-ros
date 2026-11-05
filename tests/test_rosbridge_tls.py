"""Transport contracts for rosbridge.

These talk to a real TLS listener on 127.0.0.1. Certificate and hostname checks stay on:
a private CA is trusted only when ``ca_file`` names it, and nothing in the settings
(including keys that look like "turn verification off") may make a bad certificate succeed.
"""
import json
import logging
import ssl
import subprocess
from pathlib import Path

import pytest

pytest.importorskip("websocket")
pytest.importorskip("websockets")

from conftest import Harness  # noqa: E402
from hermes_ros.commands import make_command  # noqa: E402
from hermes_ros.config import Settings, describe_rosbridge  # noqa: E402
from hermes_ros.runtime import Runtime  # noqa: E402
from hermes_ros.tools import make_handlers  # noqa: E402
from test_rosbridge_protocol import FakeRosbridge  # noqa: E402


def _openssl(args, cwd: Path) -> str:
    proc = subprocess.run(["openssl", *args], cwd=cwd, check=True, capture_output=True, text=True)
    return proc.stdout


def _leaf(tmp: Path, name: str, san: str) -> None:
    _openssl(["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", f"{name}.key"], tmp)
    _openssl(["req", "-new", "-key", f"{name}.key", "-out", f"{name}.csr", "-subj", f"/CN={name}",
              "-addext", f"subjectAltName={san}"], tmp)
    _openssl(["x509", "-req", "-in", f"{name}.csr", "-CA", "ca.pem", "-CAkey", "ca.key",
              "-CAcreateserial", "-out", f"{name}.crt", "-days", "2", "-copy_extensions", "copy"], tmp)


@pytest.fixture(scope="module")
def certs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("rosbridge-certs")
    _openssl(["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", "ca.key"], tmp)
    _openssl(["req", "-new", "-x509", "-key", "ca.key", "-out", "ca.pem", "-days", "2",
              "-subj", "/CN=hermes-ros-test-ca"], tmp)
    _openssl(["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", "other-ca.key"], tmp)
    _openssl(["req", "-new", "-x509", "-key", "other-ca.key", "-out", "other-ca.pem", "-days", "2",
              "-subj", "/CN=hermes-ros-other-ca"], tmp)
    _leaf(tmp, "server", "DNS:localhost,IP:127.0.0.1")
    _leaf(tmp, "mismatch", "DNS:rosbridge.invalid")
    server_san = _openssl(["x509", "-in", "server.crt", "-noout", "-ext", "subjectAltName"], tmp)
    mismatch_san = _openssl(["x509", "-in", "mismatch.crt", "-noout", "-ext", "subjectAltName"], tmp)
    assert "127.0.0.1" in server_san and "localhost" in server_san
    assert "rosbridge.invalid" in mismatch_san and "127.0.0.1" not in mismatch_san
    return tmp


def _server_tls(cert: Path, key: Path) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(cert, key)
    return ctx


def _cfg(port, **rosbridge):
    return {
        "backend": "rosbridge",
        "rosbridge": {"host": "127.0.0.1", "port": port, "connect_timeout_s": 3.0, "call_timeout_s": 3.0, **rosbridge},
        "safety": {"write_allowlist": {"topics": ["/cmd_vel"], "services": ["/reset"]}},
    }


def _saw_service(fake) -> bool:
    return any(m.get("op") == "call_service" for m in fake.log)


def test_wss_round_trip_is_tls_and_accepts_only_the_configured_ca(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))
    bot = Harness(tmp_path, _cfg(fake.port, tls=True, ca_file=str(certs / "ca.pem")))
    try:
        status = bot.call("ros_status")
        assert status["ok"] and status["backend"]["url"] == f"wss://127.0.0.1:{fake.port}"
        assert status["backend"]["encrypted"] is True
        assert "transport_warning" not in status["backend"]
        called = bot.call("ros_call_service", {"service": "/reset"})
        assert called["ok"], called
        assert _saw_service(fake)
        sock = fake.client_sockets[-1]
        assert isinstance(sock, ssl.SSLSocket)
        assert sock.version() in {"TLSv1.2", "TLSv1.3"}
        cipher = sock.cipher()
        assert cipher and cipher[2] >= 128
    finally:
        bot.rt.shutdown()
        fake.close()


def test_wss_rejects_a_server_the_trust_store_does_not_know(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))
    bot = Harness(tmp_path, _cfg(fake.port, tls=True))
    try:
        failed = bot.call("ros_call_service", {"service": "/reset"})
        assert failed["ok"] is False
        assert "TLS verification failed" in failed["error"]
        assert "no option to disable" in failed["error"]
        assert "hostname" in failed["error"]
        assert not _saw_service(fake)
    finally:
        bot.rt.shutdown()
        fake.close()


def test_wss_rejects_a_different_ca_file(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))
    bot = Harness(tmp_path, _cfg(fake.port, tls=True, ca_file=str(certs / "other-ca.pem")))
    try:
        failed = bot.call("ros_call_service", {"service": "/reset"})
        assert failed["ok"] is False and "TLS verification failed" in failed["error"]
        assert not _saw_service(fake)
    finally:
        bot.rt.shutdown()
        fake.close()


def test_wss_rejects_a_certificate_for_a_different_hostname(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "mismatch.crt", certs / "mismatch.key"))
    bot = Harness(tmp_path, _cfg(fake.port, tls=True, ca_file=str(certs / "ca.pem")))
    try:
        failed = bot.call("ros_call_service", {"service": "/reset"})
        assert failed["ok"] is False and "TLS verification failed" in failed["error"]
        assert not _saw_service(fake)
    finally:
        bot.rt.shutdown()
        fake.close()


def test_insecure_looking_settings_do_not_turn_verification_off(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))
    bot = Harness(tmp_path, _cfg(
        fake.port, tls=True, ca_file=str(certs / "other-ca.pem"),
        insecure=True, verify=False, ssl_verify=False, cert_reqs=0, check_hostname=False,
    ))
    try:
        failed = bot.call("ros_call_service", {"service": "/reset"})
        assert failed["ok"] is False and "TLS verification failed" in failed["error"]
        assert not _saw_service(fake)
    finally:
        bot.rt.shutdown()
        fake.close()


def test_missing_ca_file_fails_closed_before_a_handshake(certs, tmp_path):
    fake = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))
    missing = tmp_path / "missing.pem"
    bot = Harness(tmp_path, _cfg(fake.port, tls=True, ca_file=str(missing)))
    try:
        failed = bot.call("ros_call_service", {"service": "/reset"})
        assert failed["ok"] is False
        assert "ca_file" in failed["error"] and "cannot be disabled" in failed["error"]
        assert "listening with TLS" not in failed["error"]
        assert fake.client_sockets == []
        assert not _saw_service(fake)
    finally:
        bot.rt.shutdown()
        fake.close()


def test_loopback_plaintext_stays_the_default_and_is_not_tls(tmp_path):
    fake = FakeRosbridge()
    bot = Harness(tmp_path, _cfg(fake.port))
    try:
        assert bot.call("ros_call_service", {"service": "/reset"})["ok"]
        assert fake.client_sockets and not isinstance(fake.client_sockets[-1], ssl.SSLSocket)
        status = bot.call("ros_status")
        assert status["backend"]["url"] == f"ws://127.0.0.1:{fake.port}"
        assert status["backend"]["encrypted"] is False
        assert "transport_warning" not in status["backend"]
        assert "public internet" not in make_command(bot.rt)("status")
    finally:
        bot.rt.shutdown()
        fake.close()


def test_unreachable_loopback_does_not_carry_the_remote_warning(tmp_path):
    bot = Harness(tmp_path, _cfg(1, connect_timeout_s=0.5))
    try:
        failed = bot.call("ros_get_pose")
        assert failed["ok"] is False
        assert "ws://127.0.0.1:1" in failed["error"]
        assert "public internet" not in failed["error"]
    finally:
        bot.rt.shutdown()


def test_remote_plaintext_is_warned_on_status_and_on_a_failed_connect(tmp_path, caplog):
    status_bot = Harness(tmp_path, {"backend": "rosbridge", "rosbridge": {"host": "robot.lan", "port": 9090}})
    try:
        status = status_bot.call("ros_status")
        warning = status["backend"]["transport_warning"]
        assert status["backend"]["url"] == "ws://robot.lan:9090" and status["backend"]["encrypted"] is False
        assert "plaintext" in warning and "public internet" in warning and "certificate" in warning
        text = make_command(status_bot.rt)("status")
        assert "ws://robot.lan:9090 (unencrypted)" in text and "transport: " in text and "public internet" in text
    finally:
        status_bot.rt.shutdown()

    fail_bot = Harness(tmp_path, {"backend": "rosbridge", "rosbridge": {
        "host": "192.0.2.1", "port": 9, "connect_timeout_s": 0.4, "call_timeout_s": 0.4}})
    try:
        with caplog.at_level(logging.WARNING, logger="hermes_ros"):
            failed = fail_bot.call("ros_get_pose")
        assert failed["ok"] is False
        assert "public internet" in failed["error"] and "plaintext" in failed["error"]
        assert any("public internet" in rec.message for rec in caplog.records)
    finally:
        fail_bot.rt.shutdown()


def test_tls_flag_is_applied_on_the_next_call_without_a_restart(certs, tmp_path):
    plain = FakeRosbridge()
    secure = FakeRosbridge(_server_tls(certs / "server.crt", certs / "server.key"))

    class Live:
        def __init__(self):
            self.rb = {"host": "127.0.0.1", "port": plain.port, "tls": False,
                       "connect_timeout_s": 3.0, "call_timeout_s": 3.0}

        def __call__(self, key, default=None):
            if key == "backend":
                return "rosbridge"
            if key == "rosbridge":
                return self.rb
            if key == "safety":
                return {"write_allowlist": {"topics": ["/cmd_vel"], "services": ["/reset"]}}
            return default

    live = Live()
    rt = Runtime(live, tmp_path)
    try:
        def call(tool, args):
            args = dict(args)
            directive = rt.interlock.pre_tool_call(tool_name=tool, args=args)
            assert not directive or directive.get("action") != "block"
            if directive and directive.get("action") == "modify":
                args.update(directive["args"])
            return json.loads(make_handlers(rt)[tool](args))

        assert call("ros_call_service", {"service": "/reset"})["ok"]
        assert plain.client_sockets and not isinstance(plain.client_sockets[-1], ssl.SSLSocket)
        live.rb = {"host": "127.0.0.1", "port": secure.port, "tls": True, "ca_file": str(certs / "ca.pem"),
                   "connect_timeout_s": 3.0, "call_timeout_s": 3.0}
        again = call("ros_call_service", {"service": "/reset"})
        assert again["ok"], again
        assert isinstance(secure.client_sockets[-1], ssl.SSLSocket)
        described = json.loads(make_handlers(rt)["ros_status"]({}))
        assert described["backend"]["url"] == f"wss://127.0.0.1:{secure.port}"
        assert described["backend"]["encrypted"] is True
    finally:
        rt.shutdown()
        plain.close()
        secure.close()


def test_host_is_not_a_url_and_tls_strings_mean_what_they_say():
    for host in ("localhost", "LOCALHOST", "127.0.0.1", "127.8.8.8", "::1", "[::1]", "sim.localhost"):
        described = describe_rosbridge({"host": host, "port": "9090", "tls": "false"})
        assert described["warning"] is None and described["encrypted"] is False
        assert described["url"].startswith("ws://")
    assert describe_rosbridge({"host": "::1"})["url"] == "ws://[::1]:9090"
    secure = describe_rosbridge({"host": "robot.lan", "tls": "true", "ca_file": "~/robot-ca.pem"})
    assert secure["url"] == "wss://robot.lan:9090" and secure["warning"] is None
    assert secure["ca_file"] == str(Path.home() / "robot-ca.pem")
    merged = describe_rosbridge(Settings(lambda key, default=None: {"rosbridge": {"tls": True}}.get(key, default)).snapshot()["rosbridge"])
    assert merged["url"] == "wss://localhost:9090" and merged["port"] == 9090
    with pytest.raises(ValueError, match="not a URL"):
        describe_rosbridge({"host": "wss://robot.lan"})
    with pytest.raises(ValueError, match="true or false"):
        describe_rosbridge({"tls": "maybe"})


def test_mock_status_has_no_rosbridge_link(tmp_path):
    bot = Harness(tmp_path, {})
    try:
        status = bot.call("ros_status")
        assert status["backend"]["backend"] == "mock"
        assert "url" not in status["backend"] and "transport_warning" not in status["backend"]
        assert "link:" not in make_command(bot.rt)("status")
    finally:
        bot.rt.shutdown()
