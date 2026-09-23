# Reviewed pregrasp execution

[简体中文](operation.zh-CN.md) · [README](../README.md) · Previous: [pregrasp planning](pregrasp.md)

Execution moves the selected wrists to the reviewed pregrasp standoff and verifies arrival. It sends arm/head setpoints only. The calibrated head and an unselected arm remain fixed; no contact or gripper command follows arrival.

## Validate with the mock robot first

Run the complete synthetic pipeline:

```bash
python -m tron2_deployment.cli demo --output output/demo
```

It writes synthetic calibration evidence, a pregrasp plan, an execution JSONL log, and a result under `output/demo`. The toy model tests software behavior; its measurements, geometry, and calibration do not describe a TRON2 installation.

For interactive testing, start the mock workbench:

```bash
python -m tron2_deployment.cli operator \
  --profile configs/demo.json --mock \
  --host 127.0.0.1 --port 8787
```

Capture, segment, estimate, and plan. Review the displayed plan in RViz, then check the review box and start mock execution. The browser reports execution state and provides a mock stop button. Browser execution and stop apply only to the mock robot; real execution uses the CLI below.

## Accept the installed configuration

Complete these installation-specific checks before enabling real execution:

1. Confirm fresh, read-only D435 and joint feedback, matching camera identity, synchronized clocks, and the recorded fixed head pose.
2. Complete [calibration](calibration.md), independently check both wrist/TCP mounts, and confirm the registered object geometry and measured table frame.
3. Check that the numeric model and RViz URDF match the installed robot and passive attachments. Verify all joint mappings, limits, collision masks/exclusions, and attachment clearances.
4. Verify the controller's bounded hold behavior and physical emergency-stop procedure under supervision. Record the observed behavior, timing, and failure handling.
5. Start physical motion acceptance with one wrist and generous object clearance; repeat for the other wrist, then use both. Review each newly generated plan and record measured arrival and interruption results.

This repository has automated and synthetic validation. The migration itself does not perform or certify these hardware checks.

Compute the identity of the installed numeric model, including compiled assets:

```bash
python -m tron2_deployment.cli model-hash \
  --profile configs/local-robot-calibrated.json
```

After the relevant physical checks, record the returned value in `robot.model_hash`, retain the acceptance evidence, and prepare `configs/local-robot-accepted.json` with `calibration.source="real"`, `calibration.verified=true`, `execution.allow_real=true`, and `execution.hold_behavior_verified=true`. These values record completed acceptance; editing them cannot establish that the physical checks passed.

**Freeze the accepted profile before observation and planning.** Restart the workbench with `configs/local-robot-accepted.json`, acquire a new observation, generate a new plan, and review it using the same accepted profile. Changing any profile value after planning changes the profile hash and invalidates that plan. `apply-calibration` intentionally resets real execution to disabled.

## Execute the exact reviewed plan

Use the accepted profile and the actual plan path shown by the workbench. The example below assumes that the reviewed artifact was saved as `output/pregrasp-plan.json`. First inspect its identity and final targets:

```bash
python - <<'PY'
import json
from pathlib import Path
plan = json.loads(Path('output/pregrasp-plan.json').read_text())
print(json.dumps({key: plan[key] for key in
                 ('plan_id', 'profile_hash', 'selected_sides', 'targets')}, indent=2))
PY
```

After reviewing this exact artifact in RViz, enter its full `plan_id` and run the supervised command. Use a new log filename for every attempt:

```bash
read -r -p 'Reviewed plan_id: ' TRON2_REVIEWED_PLAN_ID
python -m tron2_deployment.cli execute \
  --profile configs/local-robot-accepted.json \
  --plan output/pregrasp-plan.json --real \
  --reviewed-plan-id "$TRON2_REVIEWED_PLAN_ID" \
  --confirm-supervised --log output/execution-001.jsonl
```

The command first checks plan integrity, profile/model identity, real observation provenance, calibration/head consistency, observation age, and fresh starting feedback. It independently rechecks target composition, combined-arm collisions, joint limits, and the same quintic velocity/acceleration bounds used during planning and RViz playback. It reads feedback again after expensive validation and rejects changes before authorizing setpoints.

The transport publishes `[left7, right7, head2]` at the accepted rate, at least 500 Hz. Feedback is mapped explicitly; passive gripper entries in the raw robot state are not arm joints. The executor monitors feedback freshness, head motion, tracking error, publisher timing, and arrival. Default configured thresholds are visible in `execution.DEFAULT_EXECUTION`; choose installation limits before planning, because changing them invalidates the profile hash.

Successful completion requires consecutive fresh feedback samples within the final joint tolerance. Review the result and JSONL evidence, including planned and measured states, timing, tracking errors, and arrival result. A successful transport send alone is not arrival verification. Physical TCP position/orientation accuracy must be measured during hardware acceptance.

## Stop and recover

Press **Ctrl+C in the executing terminal** to request a stop. A stop request or execution fault stops advancing along the path. When possible, the executor reads fresh measured joints and sends a short hold at that position; if feedback is unavailable, it uses the last validated setpoint. Hold duration defaults to 0.1 s and its completion is not verified.

This software hold is not a motor-disable command or an emergency-stop guarantee. Its physical behavior depends on the accepted controller configuration. Use the installation's verified physical emergency-stop control when immediate intervention is needed. The browser's mock stop button cannot stop a separate real CLI process.

The log records rejection, failure, stop, and hold errors. Existing log files are never overwritten. After an interruption, inspect the robot and recorded feedback, restore the calibrated head if needed, acquire a fresh observation, and generate/review a new plan from the current measured state. Do not resume from an assumed waypoint or reuse a plan whose start, object pose, or calibration no longer matches.
