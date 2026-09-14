"""WebsocketTransport — high-level ws://<ip>:5000 JSON control.

Migrated from the original ``tron2_env.robot.Tron2`` class. Changes vs. the
old implementation:

* Renamed to make the abstraction layer explicit (``RobotTransport``).
* Single-shot ``send_joint_cmd`` replaces public ``servoj``; the internal
  ``servoj_rate_limiter`` is removed. Pacing now lives in
  ``MotionController._publish_loop``.
* Added ``get_head_position`` so callers no longer reach into private state.
* Dropped methods with zero production callers: ``movep``, ``servop``,
  ``chassis_*``, ``lifter_*``, and ``set_light_effect``.
* ``emergency_stop`` is retained as a response-verified safety primitive using
  the vendor ``request_emgy_stop`` / ``response_emgy_stop`` WebSocket pair.
* ``movej`` / ``move_head`` / ``wait_until_reached`` / ``set_gripper``
  retained — used by ``_move_to_init_pose`` and ``env.reset``.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import uuid
from collections import deque
from typing import Dict, List, Optional, Union

import numpy as np
import websocket

from tron2_env.config import Tron2Config
from tron2_env.errors import CommandError, StateError
from tron2_env.joints import JointIndex


class WebsocketTransport:
    """Tron2 control via the ws://<ip>:5000 JSON protocol.

    Implements :class:`tron2_env.transport.base.RobotTransport`.

    Example:
        >>> config = Tron2Config(robot_ip="ROBOT_IP")
        >>> with WebsocketTransport(config) as t:
        ...     t.send_joint_cmd(np.zeros(16))
        ...     state = t.get_joint_state()
    """

    SERVOJ_DIM = JointIndex.SERVOJ_DIM
    min_servoj_rate_hz = 500.0

    def __init__(self, config: Optional[Tron2Config] = None) -> None:
        self.config = config or Tron2Config()
        self._setup_logger()

        # ws connection
        self.accid: Optional[str] = None
        self.ws_client: Optional[websocket.WebSocketApp] = None
        self.ws_thread: Optional[threading.Thread] = None
        self.connected = False
        self.should_exit = False
        # The vendor e-stop is asynchronous.  Retain its exact request guid so
        # a delayed response to an earlier request cannot acknowledge a new one.
        self._emergency_stop_event = threading.Event()
        self._emergency_stop_lock = threading.Lock()
        self._emergency_stop_guid: Optional[str] = None
        self._emergency_stop_result: Optional[str] = None
        # Unlike a socket write, a ``response_movej`` with ``success`` proves
        # only that the controller accepted the request.  It is still not a
        # completion signal (callers must use feedback for that), but it lets
        # us reject malformed/limit-clipped targets before waiting for motion.
        self._movej_event = threading.Event()
        self._movej_response_lock = threading.Lock()
        self._movej_request_lock = threading.Lock()
        self._movej_guid: Optional[str] = None
        self._movej_result: Optional[str] = None

        # state buffers (timestamp = client wall clock ms, robot_timestamp = server's own ts)
        self.joint_states: Dict = {
            "timestamp": -1,
            "robot_timestamp": -1,
            "states": [-1.0] * JointIndex.STATE_DIM,
            "joint_updated": False,
            "gripper_updated": False,
        }
        self.ee_pose_states: Dict = {
            "timestamp": -1,
            "left_position": [-1.0, -1.0, -1.0],
            "left_quat": [-1.0, -1.0, -1.0, -1.0],
            "right_position": [-1.0, -1.0, -1.0],
            "right_quat": [-1.0, -1.0, -1.0, -1.0],
        }
        self.joint_state_queue: deque = deque(maxlen=self.config.state_queue_maxlen)
        self.ee_pose_queue: deque = deque(maxlen=self.config.state_queue_maxlen)
        self._queue_lock = threading.Lock()
        self._state_lock = threading.Lock()

        # bring-up
        self._connect()
        time.sleep(0.1)
        self._start_polling_thread()

        self._second_joint = [
            0.000999913, -0.00449967, 1.482, -1.57, 0.0036, 0.00289989, -0.00160009,
            0.0415001, 0.1279, -1.4808, -1.57, -0.00739986, 0.0151, -0.0624998,
        ]

        if self.config.init_joints is not None or self.config.init_head is not None:
            self._move_to_init_pose()

    # ------------------------------------------------------------------ logger

    def _setup_logger(self) -> None:
        self.logger = logging.getLogger(f"WebsocketTransport-{self.config.robot_ip}")
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(
                logging.Formatter(
                    "[%(asctime)s.%(msecs)03d] [%(name)s] [%(levelname)s] %(message)s",
                    datefmt="%Y-%m-%d %H:%M:%S",
                )
            )
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)
        self.logger.propagate = False

    # --------------------------------------------------------------- websocket

    @staticmethod
    def _new_guid() -> str:
        return str(uuid.uuid4())

    def _send_request(self, title: str, data: Optional[Dict] = None) -> bool:
        if not self.ws_client or not self.connected:
            self.logger.warning("ws not connected, dropping: %s", title)
            return False
        try:
            message = {
                "accid": self.accid,
                "title": title,
                "timestamp": int(time.time() * 1000),
                "guid": self._new_guid(),
                "data": data or {},
            }
            self.ws_client.send(json.dumps(message))
            return True
        except Exception as exc:
            self.logger.error("send failed (%s): %s", title, exc)
            return False

    def _on_open(self, ws) -> None:
        self.logger.info("connected: %s:%s", self.config.robot_ip, self.config.port)
        self.connected = True

    def _on_message(self, ws, message: str) -> None:
        try:
            root = json.loads(message)
            title = root.get("title", "")
            self.accid = root.get("accid", self.accid)
            if title == "response_get_joint_state":
                self._handle_joint_state(root)
            elif title == "response_get_limx_2fclaw_state":
                self._handle_gripper_state(root)
            elif title == "response_get_move_pose":
                self._handle_ee_pose(root)
            elif title == "response_emgy_stop":
                self._handle_emergency_stop(root)
            elif title == "response_movej":
                self._handle_movej(root)
            elif title not in {
                "notify_robot_info",
                "response_servoj",
                "response_set_limx_2fclaw_cmd",
                "response_moveh",
            }:
                self.logger.debug("rx: %s", title)
        except json.JSONDecodeError:
            self.logger.error("bad json: %s", message)
        except Exception as exc:
            self.logger.error("on_message error: %s", exc)

    def _handle_joint_state(self, root: Dict) -> None:
        data = root.get("data", {})
        joint_q = data.get("q", [])
        # 用客户端 wall clock 作为统一时间基准 (与 camera.py 中的 time.time() 一致)
        # 机器人原始时间戳保留在 robot_timestamp 字段供调试。
        client_ts_ms = int(time.time() * 1000)
        with self._state_lock:
            self.joint_states["timestamp"] = client_ts_ms
            self.joint_states["robot_timestamp"] = root.get("timestamp", -1)
            self.joint_states["states"][JointIndex.LEFT_ARM] = joint_q[: JointIndex.ARM_JOINT_DIM]
            self.joint_states["states"][JointIndex.RIGHT_ARM] = joint_q[
                JointIndex.ARM_JOINT_DIM : JointIndex.TOTAL_ARM_DIM
            ]
            self.joint_states["states"][JointIndex.HEAD_PITCH] = joint_q[14]
            self.joint_states["states"][JointIndex.HEAD_YAW] = joint_q[15]
            self.joint_states["joint_updated"] = True
            self._try_commit_state()

    def _handle_gripper_state(self, root: Dict) -> None:
        data = root.get("data", {})
        with self._state_lock:
            self.joint_states["states"][JointIndex.LEFT_GRIPPER] = data.get("left_opening", -1) / 100.0
            self.joint_states["states"][JointIndex.RIGHT_GRIPPER] = data.get("right_opening", -1) / 100.0
            self.joint_states["gripper_updated"] = True
            self._try_commit_state()

    def _handle_ee_pose(self, root: Dict) -> None:
        """Store the official controller end-effector pose response."""
        data = root.get("data", {})
        pose = {
            "timestamp": data.get("timestamp", root.get("timestamp", -1)),
            "left_position": data.get(
                "left_position", [-1.0, -1.0, -1.0]),
            "left_quat": data.get(
                "left_quat", [-1.0, -1.0, -1.0, -1.0]),
            "right_position": data.get(
                "right_position", [-1.0, -1.0, -1.0]),
            "right_quat": data.get(
                "right_quat", [-1.0, -1.0, -1.0, -1.0]),
            "result": data.get("result"),
        }
        self.ee_pose_states = pose
        with self._queue_lock:
            self.ee_pose_queue.append(pose.copy())

    def _handle_emergency_stop(self, root: Dict) -> None:
        """Record an acknowledgement for the active vendor emergency stop."""
        guid = root.get("guid")
        result = root.get("data", {}).get("result")
        with self._emergency_stop_lock:
            if self._emergency_stop_guid is None or guid != self._emergency_stop_guid:
                self.logger.warning("ignoring unmatched emergency-stop response")
                return
            self._emergency_stop_result = str(result)
            self._emergency_stop_event.set()

    def _handle_movej(self, root: Dict) -> None:
        """Record the acknowledgement for the one active MoveJ request."""
        guid = root.get("guid")
        result = root.get("data", {}).get("result")
        with self._movej_response_lock:
            if self._movej_guid is None or guid != self._movej_guid:
                self.logger.warning("ignoring unmatched MoveJ response")
                return
            self._movej_result = str(result)
            self._movej_event.set()

    def _try_commit_state(self) -> None:
        """Commit to queue only when both joint and gripper frames have arrived."""
        if (
            self.joint_states["joint_updated"]
            and self.joint_states["gripper_updated"]
            and self.joint_states["states"][JointIndex.LEFT_ARM_START] != -1
            and self.joint_states["timestamp"] != -1
        ):
            with self._queue_lock:
                self.joint_state_queue.append(self.joint_states.copy())
            self.joint_states["joint_updated"] = False
            self.joint_states["gripper_updated"] = False

    def _on_close(self, ws, status_code, msg) -> None:
        self.logger.warning("ws closed: %s - %s", status_code, msg)
        self.connected = False

    def _on_error(self, ws, error) -> None:
        self.logger.error("ws error: %s", error)

    def _connect(self) -> None:
        url = f"ws://{self.config.robot_ip}:{self.config.port}"
        self.logger.info("connecting: %s", url)
        self.ws_client = websocket.WebSocketApp(
            url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_close=self._on_close,
            on_error=self._on_error,
        )
        self.ws_thread = threading.Thread(target=self._run_websocket, daemon=True)
        self.ws_thread.start()

    def _run_websocket(self) -> None:
        try:
            self.ws_client.run_forever()
        except Exception as exc:
            self.logger.error("ws thread crashed: %s", exc)

    # ---------------------------------------------------------------- polling

    def _start_polling_thread(self) -> None:
        thread = threading.Thread(target=self._poll_feedback, daemon=True)
        thread.start()
        self.logger.info("state polling started (%s Hz)", self.config.polling_rate)

    def _poll_feedback(self) -> None:
        period = 1.0 / self.config.polling_rate
        while not self.should_exit:
            t0 = time.time()
            self._send_request("request_get_joint_state")
            self._send_request("request_get_limx_2fclaw_state")
            time.sleep(max(0.0, period - (time.time() - t0)))

    # ----------------------------------------------------------- RobotTransport

    def send_joint_cmd(self, q: Union[List[float], np.ndarray]) -> None:
        """Send one 16-dim servoj setpoint (no rate limiting — caller paces)."""
        if isinstance(q, np.ndarray):
            q = q.tolist()
        if len(q) != self.SERVOJ_DIM:
            raise CommandError(f"expected {self.SERVOJ_DIM}-dim q, got {len(q)}")
        if not self._send_request("request_servoj", {"q": q, "filter_ratio": 1.0}):
            raise CommandError("servoj send failed")

    def get_joint_state(self, timeout: float = 1.0) -> Dict:
        start = time.time()
        while time.time() - start < timeout:
            with self._queue_lock:
                if self.joint_state_queue:
                    states = self.joint_state_queue.popleft()
                    if len(states["states"]) != JointIndex.STATE_DIM:
                        raise StateError(
                            f"bad state dim: expected {JointIndex.STATE_DIM}, got {len(states['states'])}"
                        )
                    return states
            time.sleep(0.001)
        raise StateError(f"get_joint_state timed out ({timeout}s)")

    def get_ee_poses(self, timeout: float = 1.0) -> Dict:
        """Request and return the latest official controller EE poses."""
        with self._queue_lock:
            self.ee_pose_queue.clear()
        if not self._send_request("request_get_move_pose"):
            raise StateError("get_ee_poses request failed")
        start = time.time()
        while time.time() - start < timeout:
            with self._queue_lock:
                if self.ee_pose_queue:
                    pose = self.ee_pose_queue.popleft()
                    result = pose.get("result")
                    if result not in (None, "success"):
                        raise StateError(f"get_ee_poses failed: {result}")
                    return pose
            time.sleep(0.001)
        raise StateError(f"get_ee_poses timed out ({timeout}s)")

    def find_nearest_state(self, target_timestamp_s: float) -> Optional[Dict]:
        """Find the state in the queue whose timestamp is closest to target.

        Non-destructive: does NOT consume the queue. Returns None if the queue
        is empty. The timestamp field in state dicts is in milliseconds.
        """
        with self._queue_lock:
            if not self.joint_state_queue:
                return None
            target_ms = target_timestamp_s * 1000.0
            best = None
            best_diff = float("inf")
            for state in self.joint_state_queue:
                diff = abs(state["timestamp"] - target_ms)
                if diff < best_diff:
                    best_diff = diff
                    best = state
            return best.copy() if best is not None else None

    def get_head_position(self) -> np.ndarray:
        """Return latest head [pitch, yaw] without dequeuing state."""
        with self._state_lock:
            states = self.joint_states["states"]
            if len(states) < JointIndex.STATE_DIM:
                return np.array([0.0, 0.0])
            return np.array(states[JointIndex.HEAD], dtype=np.float64)

    def set_gripper(
        self,
        left_opening: float = 0.0,
        right_opening: float = 0.0,
        left_speed: float = 100.0,
        left_force: float = 50.0,
        right_speed: float = 100.0,
        right_force: float = 50.0,
    ) -> None:
        data = {
            "left_opening": int(np.clip(left_opening, 0, 100)),
            "left_speed": int(np.clip(left_speed, 0, 100)),
            "left_force": int(np.clip(left_force, 0, 100)),
            "right_opening": int(np.clip(right_opening, 0, 100)),
            "right_speed": int(np.clip(right_speed, 0, 100)),
            "right_force": int(np.clip(right_force, 0, 100)),
        }
        self._send_request("request_set_limx_2fclaw_cmd", data)

    def wait_until_reached(
        self,
        target_joints: Union[List[float], np.ndarray],
        tolerance: float = 0.05,
        timeout: float = 10.0,
    ) -> bool:
        if isinstance(target_joints, np.ndarray):
            target_joints = target_joints.tolist()
        target = np.array(target_joints)
        start = time.time()
        while time.time() - start < timeout:
            try:
                states = self.get_joint_state(timeout=1.0)
                current = states["states"]
                arm = np.array(current[JointIndex.LEFT_ARM] + current[JointIndex.RIGHT_ARM])
                diff = arm - target
                err = float(np.max(np.abs(diff)))
                if err < tolerance:
                    self.logger.info("reached target (err=%.4f)", err)
                    return True
                time.sleep(0.1)
            except StateError:
                self.logger.warning("state read failed; retrying")
                continue
        self.logger.warning("wait_until_reached timed out (%.1fs)", timeout)
        return False

    def is_connected(self) -> bool:
        return self.connected

    def disconnect(self) -> None:
        self.logger.info("disconnecting")
        self.should_exit = True
        if self.ws_client:
            self.ws_client.close()
        if self.ws_thread and self.ws_thread.is_alive():
            self.ws_thread.join(timeout=2.0)
        self.connected = False
        self.logger.info("disconnected")

    def emergency_stop(self, timeout: float = 2.0) -> bool:
        """Request and verify the vendor's WebSocket emergency stop.

        The controller documentation defines this as ``request_emgy_stop``
        (sic), with a matching ``response_emgy_stop`` whose ``data.result``
        must equal ``"success"``.  This method never treats a successful
        socket write as proof that the robot stopped.
        """
        if timeout <= 0:
            raise ValueError("emergency-stop timeout must be positive")
        if not self.ws_client or not self.connected:
            self.logger.error("ws not connected; emergency-stop request was not sent")
            return False
        guid = self._new_guid()
        message = {
            "accid": self.accid,
            "title": "request_emgy_stop",
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": {},
        }
        with self._emergency_stop_lock:
            self._emergency_stop_guid = guid
            self._emergency_stop_result = None
            self._emergency_stop_event.clear()
        try:
            self.ws_client.send(json.dumps(message))
        except Exception as exc:
            self.logger.error("emergency-stop send failed: %s", exc)
            return False
        if not self._emergency_stop_event.wait(timeout):
            self.logger.error("emergency-stop acknowledgement timed out")
            return False
        with self._emergency_stop_lock:
            success = self._emergency_stop_result == "success"
            self._emergency_stop_guid = None
        if not success:
            self.logger.error("controller rejected emergency stop: %s",
                              self._emergency_stop_result)
        return success

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def __del__(self):
        if getattr(self, "connected", False):
            try:
                self.disconnect()
            except Exception:
                pass

    # ----------------------------------------------------- ws-specific helpers
    # These are NOT part of RobotTransport. They are used by ``_move_to_init_pose``
    # and the environment's reset path.

    def movej(
        self,
        joint_positions: Union[List[float], np.ndarray],
        move_time: float = 2.0,
        response_timeout: float = 2.0,
    ) -> None:
        """Request an interpolated joint-space motion and verify acceptance.

        A successful return means the controller returned
        ``response_movej.data.result == 'success'`` for this exact request.
        It does *not* mean the robot reached the target; callers must verify
        that independently from fresh joint-state feedback.
        """
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()
        if len(joint_positions) != JointIndex.MOVEJ_DIM:
            raise CommandError(f"expected {JointIndex.MOVEJ_DIM}-dim joints, got {len(joint_positions)}")
        if move_time <= 0:
            raise CommandError("movej time must be positive")
        if response_timeout <= 0:
            raise CommandError("movej response timeout must be positive")
        if not self.ws_client or not self.connected:
            raise CommandError("movej send failed: websocket is not connected")

        # The vendor protocol has no request ID queueing contract.  Serialize
        # requests locally so an earlier response cannot acknowledge a later
        # command.
        with self._movej_request_lock:
            guid = self._new_guid()
            message = {
                "accid": self.accid,
                "title": "request_movej",
                "timestamp": int(time.time() * 1000),
                "guid": guid,
                "data": {"joint": joint_positions, "time": move_time},
            }
            with self._movej_response_lock:
                self._movej_guid = guid
                self._movej_result = None
                self._movej_event.clear()
            try:
                self.ws_client.send(json.dumps(message))
            except Exception as exc:
                with self._movej_response_lock:
                    self._movej_guid = None
                raise CommandError(f"movej send failed: {exc}") from exc
            if not self._movej_event.wait(response_timeout):
                with self._movej_response_lock:
                    self._movej_guid = None
                raise CommandError(
                    f"movej acknowledgement timed out after {response_timeout:.1f}s"
                )
            with self._movej_response_lock:
                result = self._movej_result
                self._movej_guid = None
            if result != "success":
                raise CommandError(f"controller rejected movej: {result}")
        self.logger.debug("movej accepted (time=%.2fs)", move_time)

    def move_head(self, head_joint: Union[List[float], np.ndarray], move_time: float = 5.0) -> None:
        """Interpolated head motion."""
        if isinstance(head_joint, np.ndarray):
            head_joint = head_joint.tolist()
        if len(head_joint) != JointIndex.HEAD_DIM:
            raise CommandError(f"expected {JointIndex.HEAD_DIM}-dim head, got {len(head_joint)}")
        self._send_request("request_moveh", {"joint": head_joint, "time": move_time})
        self.logger.debug("move_head sent: %s", head_joint)

    @staticmethod
    def _joint_pose_needs_second_joint(current: List[float]) -> bool:
        return (
            abs(current[0]) < 0.1
            and abs(current[8]) < 0.1
            and abs(current[3]) < 0.2
            and abs(current[11]) < 0.2
        )

    def _ee_pose_needs_second_joint(self, ee_pose: Dict) -> bool:
        threshold = self.config.init_ee_z_min
        if threshold is None:
            return False
        z_values = []
        for key in ("left_position", "right_position"):
            position = ee_pose.get(key, [])
            if len(position) >= 3:
                try:
                    z_values.append(float(position[2]))
                except (TypeError, ValueError):
                    continue
        if not z_values:
            raise StateError("bad ee pose: missing left/right z position")
        min_z = min(z_values)
        if min_z < float(threshold):
            self.logger.info(
                "ee z %.4f below init threshold %.4f; routing via second joint",
                min_z, threshold)
            return True
        return False

    def _move_to_init_pose(self) -> None:
        """Bring the robot to the init pose declared in config."""
        self.logger.info("moving to init pose")
        if self.config.init_joints is not None:
            states = self.get_joint_state(timeout=1.0)
            current = states["states"]
            # If lying close to home, swing through a known intermediate first to
            # avoid the controller picking a weird shortest path.
            need_second_joint = self._joint_pose_needs_second_joint(current)
            if not need_second_joint and self.config.init_ee_z_min is not None:
                need_second_joint = self._ee_pose_needs_second_joint(
                    self.get_ee_poses(timeout=1.0))
            if need_second_joint:
                self.movej(self._second_joint, move_time=2.0)
                time.sleep(2)
            self.movej(self.config.init_joints, move_time=2.0)
            time.sleep(2)  # 2.1.5 controller bug: movej state is clobbered by move_head
        if self.config.init_head is not None:
            self.move_head(self.config.init_head, move_time=1.0)
            time.sleep(0.1)
        self.set_gripper(left_opening=100, right_opening=100)
        time.sleep(2)
        self.logger.info("init pose reached")
