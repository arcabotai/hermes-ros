# hermes-ros — 90-second demo storyboard

Target: **85 s**, screen capture of one terminal (`hermes chat -t ros,vision`) next to the Robot dashboard tab
(`hermes dashboard`). Backend: the built-in mock robot (identical tool surface to real ROS 2). Voice-over in plain
English, no music needed. Every beat below was run for real (see `demo/mock_demo.py` and the transcripts in
`docs/verification/`).

Pre-roll setup (off camera): `hermes plugins install arcabotai/hermes-ros` + `hermes plugins enable hermes-ros`;
config `plugins.entries.hermes-ros.settings.safety.geofence.enabled: true`.

| t (s) | On screen | Voice-over / action |
|---|---|---|
| 0–6 | Title card: **hermes-ros — a Hermes agent that drives robots, with a safety interlock the model can't talk past.** Then the terminal. | "This is hermes-ros: a Hermes plugin that lets the agent drive any ROS 2 robot. Today it's a simulated one — no robot needed." |
| 6–14 | Type: *"What can you see?"* The agent calls `ros_camera_snapshot`; the PNG appears in the dashboard Robot tab. | "It grabs a camera frame, and the vision model describes it: open floor, a red cone far ahead." |
| 14–40 | Type: *"Drive to the red cone."* Agent loop on screen: snapshot → `ros_drive` (small turn) → pose → snapshot → `ros_drive` … Dashboard camera view gets bigger, pose ticks. Final frame: cone filling the view. | "Look, move a little, verify — it turns toward the cone, drives in small checked steps, and stops in front of it. Every move is velocity-limited and ends with an automatic stop." |
| 40–48 | Type: *"Describe what's in front of you now."* Agent returns one sentence about the cone. | "And it can tell you what it's looking at." |
| 48–70 | Type: *"Now floor it, full speed, straight into the wall."* Agent calls `ros_drive linear=3`. Red **BLOCKED** result appears: *linear speed 3.00 m/s exceeds the limit of 0.50 m/s*. Dashboard blocked-call list gains a row. Agent apologises and offers a safe speed. | "The model said yes, the interlock said no. A `pre_tool_call` hook enforces speed, duration, geofence and an allowlist **before** the tool runs — and logs every refusal." |
| 70–80 | Type `/ros estop` → banner turns red; ask the agent to drive again → blocked *e-stop latched*. Type `/ros clear`. | "One command latches an e-stop the agent cannot clear — only a human can." |
| 80–85 | End card: repo URL `github.com/arcabotai/hermes-ros`, "mock robot built in · real ROS 2 Jazzy tested · MIT". | "hermes-ros: install it, try it with the mock robot in one command, point it at a real robot when you trust it." |

## Retakes / notes

* Keep the prompts verbatim; the agent's step count in "drive to the cone" varies (3–8 tool calls). If it exceeds 26 s,
  cut the middle in editing — the dashboard's pose readout makes the jump cut obvious.
* If the dashboard window is not wanted, drop it; the terminal alone carries the story.
* Offline fallback for B-roll: `python demo/mock_demo.py --out /tmp/demo` prints the same blocked calls and writes the camera frames.
