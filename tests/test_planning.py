import copy
from pathlib import Path
import time

import numpy as np
import pytest

import tron2_deployment
from tron2_deployment.config import profile_fingerprint
from tron2_deployment.geometry import pose_matrix
from tron2_deployment.kinematics import RobotModel
from tron2_deployment.planning import _collision_segment, plan_digest, plan_pregrasp, validate_plan_integrity, validate_plan_path


@pytest.fixture
def inputs():
    side = {"symmetry": "axial", "radius_m": 0.08, "axial_offset_m": 0,
        "standoff_m": 0.12, "lift_clearance_m": 0.05,
        "wrist_to_tcp_pose7": [0, 0, 0, 1, 0, 0, 0]}
    profile = {
        "robot": {"model_xml": str(Path(tron2_deployment.__file__).parent / "assets/demo_robot.xml"),
            "base_body": "base_Link", "wrist_bodies": {"left": "left_wrist", "right": "right_wrist"},
            "arm_joint_names": {side: [f"{side}_{i}" for i in range(7)] for side in ("left", "right")},
            "head_joint_names": ["head_0", "head_1"],
            "joint_lower": [-1.5, -1.5, 0.1, -3.14, -3.14, -3.14, -3.14] * 2,
            "joint_upper": [1.5, 1.5, 1.5, 3.14, 3.14, 3.14, 3.14] * 2,
            "velocity_limits": [0.5] * 14, "acceleration_limits": [1.0] * 14},
        "scene": {"table_z_m": 0, "clearance_m": 0.01, "object_radius_m": 0.08},
        "pregrasp": {"left": copy.deepcopy(side), "right": copy.deepcopy(side)},
    }
    observation = {"pose7": [0, 0, 0.5, 1, 0, 0, 0], "reference_frame": "base_Link", "capture_timestamp_s": time.time(), "source": "mock"}
    state = {"arm_q14": [-0.6, 0, 0.5, 0, 0, 0, 0, 0.6, 0, 0.5, 0, 0, 0, 0],
        "head_q2": [0, 0], "timestamp_s": time.time(), "source": "mock"}
    return profile, observation, state


def test_dual_pregrasp_reaches_targets_in_base_frame(inputs):
    profile, observation, state = inputs
    plan = plan_pregrasp(profile, observation, state)
    assert validate_plan_integrity(plan) is None
    checks = validate_plan_path(profile, plan)
    assert checks["max_position_error_m"] < 0.001
    assert checks["max_orientation_error_rad"] < 0.01
    assert plan["interpolation"] == "quintic_stop"
    assert "gripper" not in str(plan)
    model = RobotModel(profile, observation)
    model.set_state(plan["left_arm_qpos"][-1] + plan["right_arm_qpos"][-1], state["head_q2"])
    for side, actual in model.wrist_matrices().items():
        goal = pose_matrix(plan["targets"][side]["wrist_pregrasp_pose7_base"])
        np.testing.assert_allclose(actual, goal, atol=0.001)


def test_single_arm_keeps_other_arm_fixed(inputs):
    profile, observation, state = inputs
    plan = plan_pregrasp(profile, observation, state, ("left",))
    np.testing.assert_allclose(plan["right_arm_qpos"], np.tile(state["arm_q14"][7:], (len(plan["times_s"]), 1)), atol=0)


def test_unreachable_target_has_no_partial_plan(inputs):
    profile, observation, state = inputs
    profile["pregrasp"]["left"].update(symmetry="none", anchor_object=[-5, 0, 0], outward_axis_object=[-1, 0, 0])
    with pytest.raises(ValueError, match="unreachable"):
        plan_pregrasp(profile, observation, state, ("left",))


def test_obstacle_clearance_failure(inputs):
    profile, observation, state = inputs
    profile["scene"]["object_radius_m"] = 0.7
    with pytest.raises(ValueError, match="collision/clearance"):
        plan_pregrasp(profile, observation, state)


def test_table_clearance_failure(inputs):
    profile, observation, state = inputs
    profile["scene"]["table_z_m"] = 0.48
    with pytest.raises(ValueError, match="collision/clearance"):
        plan_pregrasp(profile, observation, state)


def test_interarm_collision_failure(inputs):
    profile, observation, state = inputs
    state["arm_q14"][7:10] = state["arm_q14"][:3]
    with pytest.raises(ValueError, match="left_tool|right_tool"):
        plan_pregrasp(profile, observation, state)


def test_thin_obstacle_between_joint_samples_is_not_skipped(inputs, tmp_path):
    profile, observation, state = inputs
    original = Path(profile["robot"]["model_xml"]).read_text()
    model_path = tmp_path / "small_tools.xml"
    model_path.write_text(original.replace('size="0.035"', 'size="0.0001"'))
    profile["robot"]["model_xml"] = str(model_path)
    profile["scene"].update(clearance_m=0.0001, object_radius_m=0.0002)
    observation["pose7"][:3] = [-0.5947, 0, 0.5]
    model = RobotModel(profile, observation)
    start = np.asarray(state["arm_q14"])
    end = start.copy()
    end[0] += 0.01
    for arms in (start, end):
        model.set_state(arms, state["head_q2"])
        model.check_collision()
    with pytest.raises(ValueError, match="deployment_object"):
        _collision_segment(model, start, end, state["head_q2"])


def test_integrity_and_revalidation_reject_edited_plan(inputs):
    profile, observation, state = inputs
    plan = plan_pregrasp(profile, observation, state, ("left",))
    edited = copy.deepcopy(plan)
    edited["left_arm_qpos"][-1][0] += 0.02
    with pytest.raises(ValueError, match="plan_id"):
        validate_plan_integrity(edited)
    edited["plan_id"] = plan_digest(edited)
    with pytest.raises(ValueError, match="final wrist FK|velocity|acceleration"):
        validate_plan_path(profile, edited)
    edited = copy.deepcopy(plan)
    edited["times_s"] = (np.asarray(edited["times_s"]) * 0.001).tolist()
    edited["plan_id"] = plan_digest(edited)
    with pytest.raises(ValueError, match="velocity|acceleration"):
        validate_plan_path(profile, edited)
    edited = copy.deepcopy(plan)
    edited["targets"]["left"]["standoff_m"] = 0
    edited["plan_id"] = plan_digest(edited)
    with pytest.raises(ValueError, match="targets"):
        validate_plan_path(profile, edited)


@pytest.mark.parametrize("key,value", [("velocity_limits", [0]*14), ("acceleration_limits", [float("nan")]*14), ("joint_lower", [float("inf")]*14)])
def test_invalid_limits_fail(inputs, key, value):
    profile, observation, state = inputs
    profile["robot"][key] = value
    with pytest.raises(ValueError):
        plan_pregrasp(profile, observation, state)


@pytest.mark.parametrize("update", [{"confidence": 0.1}, {"confidence": float("nan")},
    {"profile_hash": "old calibration"}, {"head_q2": [0.1, 0]}, {"source": "real"}])
def test_invalid_observation_provenance_fails(inputs, update):
    profile, observation, state = inputs
    observation.update(update)
    with pytest.raises(ValueError):
        plan_pregrasp(profile, observation, state)


def test_real_observation_requires_bound_calibration_and_fresh_state(inputs):
    profile, observation, state = inputs
    state["source"] = "real"
    observation["source"] = "real"
    with pytest.raises(ValueError, match="provenance"):
        plan_pregrasp(profile, observation, state)
    profile["calibration"] = {"id": "test-calibration", "source": "real", "verified": True, "head_q2": [0, 0]}
    profile["vision"] = {"mesh_id": "test-mesh"}
    observation.update(profile_hash=profile_fingerprint(profile), calibration_id="test-calibration",
                       head_q2=[0, 0], mesh_id="test-mesh", confidence=1.0)
    state["timestamp_s"] = time.time() - 10
    with pytest.raises(ValueError, match="planning state is stale"):
        plan_pregrasp(profile, observation, state)
    state["timestamp_s"] = time.time()
    observation["capture_timestamp_s"] = time.time() - 1000
    with pytest.raises(ValueError, match="object observation is stale"):
        plan_pregrasp(profile, observation, state)
    observation["capture_timestamp_s"] = time.time()
    plan = plan_pregrasp(profile, observation, state, ("left",))
    # Review may take longer than feedback freshness; independent validation uses
    # the frozen planning state, while execution checks a new measured state.
    assert validate_plan_path(profile, plan)["ik"]


def test_real_planning_and_revalidation_require_verified_calibration(inputs):
    profile, observation, state = inputs
    state["source"] = observation["source"] = "real"
    profile["calibration"] = {"id": "test-calibration", "source": "real", "verified": True, "head_q2": [0, 0]}
    profile["vision"] = {"mesh_id": "test-mesh"}
    observation.update(profile_hash=profile_fingerprint(profile), calibration_id="test-calibration",
                       head_q2=[0, 0], mesh_id="test-mesh", confidence=1.0)
    plan = plan_pregrasp(profile, observation, state, ("left",))
    for unverified in (False, None, 1, "true"):
        invalid_profile = copy.deepcopy(profile)
        invalid_profile["calibration"]["verified"] = unverified
        invalid_observation = copy.deepcopy(observation)
        invalid_observation["profile_hash"] = profile_fingerprint(invalid_profile)
        with pytest.raises(ValueError, match="independently verified calibration"):
            plan_pregrasp(invalid_profile, invalid_observation, state, ("left",))
        # Rehashing an artifact under an unverified profile cannot bypass the
        # independent calibration gate even when all identity fields agree.
        invalid_plan = copy.deepcopy(plan)
        invalid_plan["profile_hash"] = invalid_observation["profile_hash"]
        invalid_plan["source"]["observation"] = invalid_observation
        invalid_plan["plan_id"] = plan_digest(invalid_plan)
        with pytest.raises(ValueError, match="independently verified calibration"):
            validate_plan_path(invalid_profile, invalid_plan)
