(function () {
  "use strict";
  // hermes-ros dashboard tab: live camera frame, pose, safety limits and the blocked-call log. Read-only.
  const SDK = window.__HERMES_PLUGIN_SDK__;
  if (!SDK || !window.__HERMES_PLUGINS__) return;
  const React = SDK.React;
  const h = React.createElement;
  const { useState, useEffect, useRef } = SDK.hooks;
  const { Card, CardHeader, CardTitle, CardContent, Badge } = SDK.components;
  const API = "/api/plugins/hermes-ros/state";

  const f = (v, d) => (typeof v === "number" ? v.toFixed(d === undefined ? 2 : d) : "–");
  const deg = (r) => (typeof r === "number" ? ((r * 180) / Math.PI).toFixed(0) + "°" : "–");

  function kv(rows) {
    return h("dl", { className: "hros-kv" }, rows.flatMap(([k, v]) => [h("dt", { key: k + "k" }, k), h("dd", { key: k + "v" }, v)]));
  }

  function RobotPage() {
    const [st, setSt] = useState(null);
    const [err, setErr] = useState(null);
    const [img, setImg] = useState(null);
    const since = useRef(0);

    useEffect(() => {
      let alive = true;
      const tick = () =>
        SDK.fetchJSON(API + "?since=" + since.current)
          .then((d) => {
            if (!alive) return;
            setSt(d);
            setErr(null);
            if (d.snapshot && d.snapshot.data_url) {
              since.current = d.snapshot.mtime;
              setImg(d.snapshot.data_url);
            }
          })
          .catch((e) => alive && setErr(String(e)));
      tick();
      const t = setInterval(tick, 1500);
      return () => { alive = false; clearInterval(t); };
    }, []);

    if (err && !st) return h(Card, null, h(CardContent, { className: "py-6" }, "Could not reach the hermes-ros backend: " + err));
    if (!st) return h(Card, null, h(CardContent, { className: "py-6 hros-muted" }, "Loading…"));

    const latched = st.estop && st.estop.latched;
    const p = st.pose;
    const gf = st.geofence || {};
    const linkNote = st.rosbridge ? (st.encrypted ? ", encrypted" : ", unencrypted") : "";
    return h("div", { className: "space-y-4" },
      h("div", { className: "hros-banner " + (latched ? "bad" : "ok") },
        latched ? "E-STOP LATCHED — " + (st.estop.reason || "") + " (a human clears it with /ros clear)" : "E-stop clear · interlock active"),
      st.transport_warning ? h("div", { className: "hros-banner warn" }, st.transport_warning) : null,
      h("div", { className: "hros-grid" },
        h(Card, null,
          h(CardHeader, null, h(CardTitle, null, "Camera")),
          h(CardContent, null,
            img ? h("img", { className: "hros-cam", src: img, alt: "latest robot camera frame" })
                : h("div", { className: "hros-cam hros-muted", style: { padding: "4rem 1rem", textAlign: "center" } }, "No snapshot yet — ask the agent to look through the camera."),
            st.snapshot ? h("p", { className: "hros-muted text-xs", style: { marginTop: "0.5rem" } }, st.snapshot.name + " · " + st.snapshot.age_s + "s ago") : null)),
        h("div", { className: "space-y-4" },
          h(Card, null,
            h(CardHeader, null, h(CardTitle, null, "Pose")),
            h(CardContent, null, p ? kv([
              ["x, y", f(p.x) + ", " + f(p.y) + " m"],
              ["heading", deg(p.yaw_rad !== undefined ? p.yaw_rad : p.yaw)],
              ["collided", p.collided ? "YES" : "no"],
              ["last update", st.pose_age_s + "s ago"],
            ]) : h("span", { className: "hros-muted" }, "No pose recorded yet."))),
          h(Card, null,
            h(CardHeader, null, h(CardTitle, null, "Safety limits")),
            h(CardContent, null, kv([
              ["backend", st.backend + (st.rosbridge ? " (" + st.rosbridge + linkNote + ")" : "")],
              ["max speed", f(st.limits.max_linear_mps) + " m/s · " + f(st.limits.max_angular_rps) + " rad/s"],
              ["max duration", f(st.limits.max_duration_s, 1) + " s"],
              ["over limit", st.limits.over_limit_action],
              ["geofence", gf.enabled ? "x[" + gf.x_min + ", " + gf.x_max + "] y[" + gf.y_min + ", " + gf.y_max + "]" : "off"],
              ["writes allowed", (st.allowlist.topics || []).concat(st.allowlist.services || []).join(", ") || "none"],
            ])))
        )),
      h(Card, null,
        h(CardHeader, null, h(CardTitle, null, "Blocked calls ", h(Badge, { variant: "outline" }, String(st.blocked_total)))),
        h(CardContent, null,
          st.blocked.length === 0 ? h("span", { className: "hros-muted" }, "Nothing blocked yet.") :
          h("div", { className: "hros-log" }, st.blocked.map((b, i) =>
            h("div", { key: i },
              h("span", { className: "hros-rule" }, b.tool + " · " + b.rule),
              h("span", { className: "hros-muted" }, "  " + (b.iso || "")),
              h("div", null, b.reason)))))));
  }

  window.__HERMES_PLUGINS__.register("hermes-ros", RobotPage);
})();
