"""Execution tests use fake clocks and fake transports; no hardware is contacted."""

import json
import threading
from copy import deepcopy

import numpy as np
import pytest

from tron2_deployment import execution
from tron2_deployment.config import profile_fingerprint
from tron2_deployment.robot import MockRobot, WebsocketRobot


class Clock:
    def __init__(self):
        self.now = 1_800_000_000.0
        self.elapsed = 0.0

    def time(self):
        return self.now + self.elapsed

    def monotonic(self):
        return self.elapsed

    def sleep(self, delay):
        self.elapsed += max(0, delay)


@pytest.fixture
def clock(monkeypatch):
    value = Clock()
    monkeypatch.setattr(execution.time, "time", value.time)
    monkeypatch.setattr(execution.time, "monotonic", value.monotonic)
    monkeypatch.setattr(execution.time, "sleep", value.sleep)
    return value


def recipe(clock, *, real=False):
    source = "real" if real else "mock"
    profile = {
        "schema_version": 1, "profile_id": "test",
        "robot": {"host": "unused.invalid", "joint_lower": [-2.0] * 14,
                  "joint_upper": [2.0] * 14, "velocity_limits": [1.0] * 14,
                  "acceleration_limits": [1.0] * 14, "model_hash": "accepted-model"},
        "calibration": {"id": "test-calibration", "verified": real, "source": source, "head_q2": [0.0, 0.0]},
        "vision": {"mesh_id": "test-mesh", "min_confidence": 0.5},
        "execution": {"allow_real": real, "hold_behavior_verified": real},
    }
    state = {"arm_q14": [0.0] * 14, "head_q2": [0.0, 0.0],
             "timestamp_s": clock.time(), "source": source}
    plan = {
        "schema_version": 1, "mode": "pregrasp", "interpolation": "quintic_stop",
        "times_s": [0.0, 1.0], "left_arm_qpos": [[0.0] * 7, [0.02] * 7],
        "right_arm_qpos": [[0.0] * 7, [-0.02] * 7], "head_qpos": [[0.0, 0.0]] * 2,
        "source": {"state": state, "observation": {"reference_frame": "base_Link",
            "pose7": [0.5, 0, 0.5, 1, 0, 0, 0], "capture_timestamp_s": clock.time(), "source": source,
            "profile_hash": profile_fingerprint(profile), "mesh_id": "test-mesh",
            "calibration_id": "test-calibration", "head_q2": [0, 0], "confidence": 0.9}},
        "targets": {}, "checks": {"ik": True, "joint_limits": True, "collision": True,
                                    "velocity": True, "acceleration": True, "model_hash": "accepted-model"},
        "profile_hash": profile_fingerprint(profile),
    }
    sign(plan)
    return plan, profile, state


def sign(plan):
    from tron2_deployment.planning import plan_digest
    plan["plan_id"] = plan_digest(plan)
    return plan


def test_mock_completes_timed_arm_only_motion_and_writes_evidence(clock, tmp_path):
    plan, profile, _ = recipe(clock)
    robot = MockRobot(clock=clock.time)
    path = tmp_path / "execution.jsonl"
    result = execution.execute_plan(plan, profile, robot, log_path=path)
    assert result["status"] == "completed", result
    assert not result["hardware_commanded"]
    assert result["samples_count"] >= 500
    assert result["arrival_error_rad"] <= profile.get("execution", {}).get("arrival_tolerance_rad", 0.02)
    assert np.allclose(robot.arm_q14, [0.02] * 7 + [-0.02] * 7)
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert records[0]["event"] == "preflight"
    assert records[-1]["event"] == "completed"
    assert "planned_arm_q14" in records[1] and "measured" in records[1]


def test_mock_profile_constructor_requires_explicit_demo_arms(clock):
    _, profile, _ = recipe(clock)
    with pytest.raises(ValueError, match="demo.arm_q14"):
        MockRobot(profile)
    profile["demo"] = {"arm_q14": [0.1] * 14}
    robot = MockRobot(profile, clock=clock.time)
    assert robot.read_state()["arm_q14"] == [0.1] * 14
    assert robot.read_state()["head_q2"] == profile["calibration"]["head_q2"]
    profile["demo"]["head_q2"] = [0.2, -0.3]
    assert MockRobot(profile, clock=clock.time).read_state()["head_q2"] == [0.2, -0.3]


@pytest.mark.parametrize("mutation,error", [
    (lambda p: p["left_arm_qpos"][1].__setitem__(0, 0.03), "digest"),
    (lambda p: p["checks"].__setitem__("collision", False), "digest"),
])
def test_modified_plan_is_rejected_without_commands(clock, mutation, error):
    plan, profile, _ = recipe(clock)
    mutation(plan)
    robot = MockRobot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot)
    assert result["status"] == "rejected"
    assert not robot.commands


@pytest.mark.parametrize("mutation,expected", [
    (lambda p: p.__setitem__("times_s", [0, 0.01]), "velocity"),
    (lambda p: p["left_arm_qpos"][1].__setitem__(0, 3), "joint limits"),
    (lambda p: p["head_qpos"].__setitem__(1, [0.1, 0]), "head"),
    (lambda p: p.__setitem__("gripper_events", []), "gripper"),
    (lambda p: p.__setitem__("interpolation", "linear"), "interpolation"),
    (lambda p: p["checks"].__setitem__("collision", False), "collision"),
])
def test_invalid_plan_is_rejected_even_with_new_digest(clock, mutation, expected):
    plan, profile, state = recipe(clock)
    # Deepcopy removes the intentional repeated constant head-row reference.
    plan = deepcopy(plan)
    mutation(plan)
    sign(plan)
    report = execution.preflight(plan, profile, state)
    assert not report["ok"]
    assert expected in " ".join(report["errors"])


def test_acceleration_is_checked_between_waypoints(clock):
    plan, profile, state = recipe(clock)
    plan["times_s"] = [0, 0.1]
    sign(plan)
    report = execution.preflight(plan, profile, state)
    assert not report["ok"] and "acceleration" in report["errors"][0]


@pytest.mark.parametrize("what", ["state", "observation", "future", "start", "head", "profile"])
def test_stale_or_mismatched_inputs_rejected(clock, what):
    plan, profile, state = recipe(clock)
    state = deepcopy(state)
    if what == "state":
        state["timestamp_s"] -= 1
    elif what == "future":
        state["timestamp_s"] += 1
    elif what == "observation":
        plan["source"]["observation"]["capture_timestamp_s"] -= 121
        sign(plan)
    elif what == "start":
        state["arm_q14"][0] = 0.2
    elif what == "head":
        state["head_q2"][0] = 0.2
    elif what == "profile":
        profile["profile_id"] = "different"
    assert not execution.preflight(plan, profile, state)["ok"]


def test_operator_stop_only_holds_current_pose(clock):
    plan, profile, _ = recipe(clock)
    event = threading.Event()

    class Robot(MockRobot):
        def command(self, arms, head):
            super().command(arms, head)
            if len(self.commands) == 4:
                event.set()

    robot = Robot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot, stop_event=event)
    assert result["status"] == "stopped"
    assert result["hold"]["sent"]
    assert np.allclose(robot.commands[-1]["arm_q14"], robot.commands[3]["arm_q14"])
    assert not np.allclose(robot.arm_q14, [0.02] * 7 + [-0.02] * 7)


def test_rejected_command_aborts_remaining_path(clock):
    plan, profile, _ = recipe(clock)

    class RejectingRobot(MockRobot):
        def command(self, arms, head):
            if len(self.commands) >= 3:
                return False
            return super().command(arms, head)

    robot = RejectingRobot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot)
    assert result["status"] == "failed"
    assert len(robot.commands) == 3
    assert result["hold"]["sent"] is False
    assert any("rejected" in error for error in result["errors"])


def test_repeated_feedback_cannot_prove_arrival(clock):
    plan, profile, _ = recipe(clock)

    class FrozenRobot(MockRobot):
        def read_state(self):
            state = super().read_state()
            state["timestamp_s"] = clock.now
            return state

    robot = FrozenRobot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot)
    assert result["status"] == "failed"
    assert "stale" in result["errors"][0]
    assert "stale" in result["hold_feedback_error"]


def test_blocking_feedback_is_detected_before_next_command(clock):
    plan, profile, _ = recipe(clock)

    class SlowRobot(MockRobot):
        def read_state(self):
            if len(self.commands) == 2:
                clock.sleep(0.03)
            return super().read_state()

    robot = SlowRobot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot)
    assert result["status"] == "failed"
    assert "timing" in result["errors"][0]
    assert result["samples_count"] == 2
    assert result["hold"]["sent"]


def test_preflight_rejects_non_object_json(clock):
    _, profile, state = recipe(clock)
    assert not execution.preflight([], profile, state)["ok"]


class FakeTransport:
    def __init__(self):
        self._state_lock = threading.Lock()
        self.joint_states = {"states": list(range(18)), "timestamp": 1_800_000_000_000}
        self.commands = []
        self.connected = True

    def is_connected(self):
        return self.connected

    def send_joint_cmd(self, q):
        self.commands.append(list(q))

    def disconnect(self):
        self.connected = False


def test_real_adapter_is_lazy_readonly_and_uses_correct_state_mapping(clock):
    _, profile, _ = recipe(clock, real=True)
    calls = []
    transport = FakeTransport()
    robot = WebsocketRobot(profile, transport_factory=lambda cfg: calls.append(cfg) or transport)
    assert calls == []
    with pytest.raises(RuntimeError, match="read-only"):
        robot.command([0] * 14, [0, 0])
    assert calls == []
    state = robot.read_state()
    assert state["arm_q14"] == list(range(7)) + list(range(8, 15))
    assert state["head_q2"] == [16, 17]
    assert transport.commands == []
    robot._authorize_execution({"ok": True, "real": True, "plan_id": "reviewed"})
    robot.command(list(range(14)), [0.1, 0.2])
    assert transport.commands == [list(range(14)) + [0.1, 0.2]]
    robot._revoke_execution()
    with pytest.raises(RuntimeError):
        robot.command([0] * 14, [0, 0])


@pytest.mark.parametrize("missing", ["review", "supervisor", "allow_real", "hold", "calibration", "model", "source"])
def test_real_preflight_requires_all_acceptance_gates(clock, missing, monkeypatch):
    from tron2_deployment import planning
    plan, profile, state = recipe(clock, real=True)
    monkeypatch.setattr(planning, "validate_plan_path", lambda profile, plan: plan["checks"])
    review = plan["plan_id"]
    supervisor = True
    if missing == "review":
        review = "other"
    elif missing == "supervisor":
        supervisor = False
    elif missing == "allow_real":
        profile["execution"]["allow_real"] = False
    elif missing == "hold":
        profile["execution"]["hold_behavior_verified"] = False
    elif missing == "calibration":
        profile["calibration"]["verified"] = False
    elif missing == "model":
        profile["robot"]["model_hash"] = "other"
    elif missing == "source":
        plan["source"]["observation"]["source"] = "mock"
    plan["profile_hash"] = profile_fingerprint(profile)
    plan["source"]["observation"]["profile_hash"] = profile_fingerprint(profile)
    old_id = plan["plan_id"]
    sign(plan)
    if review == old_id:
        review = plan["plan_id"]
    report = execution.preflight(plan, profile, state, real=True,
        reviewed_plan_id=review, supervisor_confirmed=supervisor)
    assert not report["ok"]


def test_real_preflight_revalidates_current_model_and_path(clock, monkeypatch):
    from tron2_deployment import planning
    plan, profile, state = recipe(clock, real=True)
    calls = []
    monkeypatch.setattr(planning, "validate_plan_path", lambda profile, plan: calls.append(plan) or plan["checks"])
    report = execution.preflight(plan, profile, state, real=True,
        reviewed_plan_id=plan["plan_id"], supervisor_confirmed=True)
    assert report["ok"], report
    assert calls == [plan]
    monkeypatch.setattr(planning, "validate_plan_path", lambda profile, plan: {**plan["checks"], "collision": False})
    assert not execution.preflight(plan, profile, state, real=True,
        reviewed_plan_id=plan["plan_id"], supervisor_confirmed=True)["ok"]


@pytest.mark.parametrize("field,value", [("profile_hash", "old"), ("mesh_id", "other"),
    ("calibration_id", "old"), ("head_q2", [0.1, 0]), ("confidence", 0.1)])
def test_real_observation_must_match_current_geometry_and_calibration(clock, field, value):
    plan, profile, state = recipe(clock, real=True)
    plan["source"]["observation"][field] = value
    sign(plan)
    report = execution.preflight(plan, profile, state, real=True,
        reviewed_plan_id=plan["plan_id"], supervisor_confirmed=True)
    assert not report["ok"]


def test_mock_mode_cannot_command_a_real_adapter(clock):
    plan, profile, _ = recipe(clock)
    robot = WebsocketRobot(profile, transport_factory=lambda _: pytest.fail("unexpected real connection"))
    result = execution.execute_plan(plan, profile, robot)
    assert result["status"] == "rejected"
    assert "mode" in result["errors"][0]


def test_existing_log_is_preserved_without_motion(clock, tmp_path):
    plan, profile, _ = recipe(clock)
    path = tmp_path / "prior.jsonl"
    path.write_text("prior evidence\n")
    robot = MockRobot(clock=clock.time)
    result = execution.execute_plan(plan, profile, robot, log_path=path)
    assert result["status"] == "rejected"
    assert not robot.commands
    assert path.read_text() == "prior evidence\n"
