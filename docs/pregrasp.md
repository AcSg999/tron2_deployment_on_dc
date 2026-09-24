# Object-aware pregrasp planning

[简体中文](pregrasp.zh-CN.md) · [README](../README.md) · Previous: [deployment](deployment.md) · Next: [operation](operation.md)

The task ends with the selected wrists at collision-checked **standoff poses**. FoundationPose supplies the object's frame; geometry and approach constraints determine wrist position and orientation. The workflow includes no contact, gripping, or gripper opening/closing.

## Define the object and wrist approach

All translations use metres. `pose7` arrays use `[x,y,z,qw,qx,qy,qz]` with unit quaternions. The observation must be expressed in `base_Link`. Configure the registered object `vision.mesh_id`, a conservative enclosing `scene.object_radius_m`, measured `scene.table_z_m`, and a positive `scene.clearance_m`.

Each selected side uses its own `pregrasp.left` or `pregrasp.right` configuration:

| Field | Meaning |
| --- | --- |
| `wrist_to_tcp_pose7` | Measured TCP frame expressed in the configured wrist body frame. |
| `symmetry` | `axial` for symmetry around the chosen object up axis; `none` for a specified object-relative approach. |
| `up_axis_object` | Object-local up/roll reference; defaults to `[0,0,1]`. |
| `standoff_m` | Positive distance from the anchor along the outward approach direction. |
| `lift_clearance_m` | Nonnegative lift above the higher of the initial and target TCP positions. |
| `radius_m`, `axial_offset_m` | For `axial`: surface anchor radius and height along the object up axis. |
| `azimuth_hint_base` | Optional `axial` outward direction in base coordinates; otherwise inferred from the current measured TCP position. |
| `anchor_object`, `outward_axis_object` | For `none`: object-local anchor and outward direction, both three-component vectors. |

For an axially symmetric bowl, the planner rotates the object's up axis into the base frame, then projects the current TCP-to-object offset into the radial plane. The radial direction selects the approach side. Object yaw about the symmetry axis therefore does not force wrist rotation. Object tilt still changes the up axis and target geometry. An explicit azimuth hint can choose a different side.

TCP `+Z` points toward the object, opposite the outward direction. TCP `+Y` follows the object up axis projected perpendicular to the approach. When that projection degenerates, the current measured TCP axes provide a roll reference. For `symmetry=none`, the anchor and outward direction rotate with the object, while these approach-axis constraints still determine wrist orientation.

```text
T_base_object = T_base_camera @ T_camera_object
p_base_pregrasp = p_base_anchor + standoff_m * outward_base
T_base_wrist = T_base_tcp @ inverse(T_wrist_tcp)
```

The anchor is a geometric reference, not a commanded contact pose. Choosing `standoff_m` does not by itself guarantee clearance: the entire installed attachment and both arms must pass the model checks.

## Capture, estimate, and plan

Start the compact workbench using the calibrated deployment profile:

```bash
python -m tron2_deployment.cli operator \
  --profile configs/local-robot-calibrated.json \
  --host 127.0.0.1 --port 8787
```

Open `http://127.0.0.1:8787/`. Startup reads configuration; an explicit operation triggers camera, vision, or state access.

1. Capture a frame from the head-mounted D455 RGB-D camera and draw a box around the configured object.
2. Generate the SAM mask, inspect it, then run FoundationPose.
3. Select left, right, or both wrists. Click **Read state and plan** (`读取状态并规划`); the service obtains fresh measured arm/head state before planning.
4. Inspect the returned checks and targets. The server saves an immutable plan in `output/pregrasp_<id>.json`, displays its actual path and RViz command, and allows a JSON download.

Capture, mask, or pose changes invalidate dependent results. Changing the selected sides invalidates the displayed plan. Editing the deployment profile or model requires a service restart and a new observation/plan. Real planning rejects mismatched profile/calibration/mesh provenance, low or invalid confidence, stale capture/state timestamps, and a head pose that differs from acquisition or calibration.

For a previously serialized, still-valid `vision.estimate()` observation, the CLI can read fresh state directly. This command plans only:

```bash
python -m tron2_deployment.cli plan \
  --profile configs/local-robot-calibrated.json \
  --observation output/observation.json --state live \
  --sides left right --output output/pregrasp-plan.json
```

For a fully synthetic run, use `operator --profile configs/demo.json --mock` or the `demo` command in [deployment](deployment.md). Synthetic observations and the toy robot cannot authorize real execution.

## Understand the planned path and checks

The planner follows one deterministic corridor: lift the TCP, translate and orient above the target, then descend to standoff. Sequential IK starts from measured joints and the preceding solution. Both selected wrists share a timeline; an unselected arm and the head remain fixed. A blocked or unreachable corridor fails without producing a partial executable plan. The planner does not search globally for an alternative route around obstacles.

Every segment uses `quintic_stop` interpolation:

```text
s(u) = 10*u^3 - 15*u^4 + 6*u^5,  0 <= u <= 1
q(u) = q_start + s(u) * (q_end - q_start)
```

Joint velocity and acceleration are zero at each segment boundary. Segment durations respect the configured per-joint velocity and acceleration limits. The final numeric IK tolerance is 1 mm position and 0.01 rad orientation; measured hardware accuracy is a separate acceptance result.

Numeric MuJoCo queries check FK, joint bounds, model self/interarm collisions, the table half-space, and the enclosing object sphere. Sampling evaluates both arms together and bounds between-sample geometry motion using link offsets, joint ranges, attachment bounding radii, and the configured clearance. The deployed model's collision masks and exclusions remain authoritative and must represent the installed hardware. MuJoCo is not an additional viewer or operator review step.

The artifact contains both `N x 7` arm arrays, `N x 2` fixed head positions, `times_s`, selected sides, TCP/wrist targets, frozen observation and initial state, profile hash, compiled-model hash, numeric checks, and a content-derived `plan_id`. Revalidation recomputes targets and checks the path; a JSON edit is not a valid way to adjust a reviewed trajectory.

## Review the exact artifact in RViz

Use the command displayed by the workbench with its actual server-side plan path. For the CLI-generated filename above:

```bash
python -m tron2_deployment.rviz \
  --profile configs/local-robot-calibrated.json \
  --plan output/pregrasp-plan.json \
  --ros-master-uri http://127.0.0.1:11331
```

RViz uses a separate local visualization ROS master. It shows the frozen object frame and enclosing sphere, pregrasp target axes, FK wrist paths, and planned joints. The real profile requires a matching `robot.urdf` including installed attachments. The preview uses the same quintic interpolation and timing as the executor. `--once` plays once and holds the final pose; `--no-gui` runs the publisher without opening RViz, and `--no-start-master` uses an already running visualization master.

Review the full path, both arms' clearance, tool approach axes, target standoff, and final stopping poses. RViz review does not issue robot commands. Continue to [operation](operation.md) with the exact reviewed plan ID.
