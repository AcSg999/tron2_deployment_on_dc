"""Numeric model adapter. MuJoCo is used for FK/IK and collision queries only."""
from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile

import mujoco
import numpy as np
from scipy.optimize import least_squares

from .geometry import matrix_pose, pose_matrix, rotation_error, vector


def _compiled_hash(model):
    with tempfile.TemporaryDirectory(prefix="tron2-model-") as directory:
        path = Path(directory) / "model.mjb"
        mujoco.mj_saveModel(model, str(path), None)
        return hashlib.sha256(path.read_bytes()).hexdigest()


def model_hash(profile):
    return _compiled_hash(mujoco.MjModel.from_xml_path(str(profile["robot"]["model_xml"])))


class RobotModel:
    def __init__(self, profile, observation=None):
        self.profile = profile
        robot = profile["robot"]
        spec = mujoco.MjSpec.from_file(str(robot["model_xml"]))
        original = spec.compile()
        if original.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT:
            raise ValueError("robot model disables collision detection")
        self.model_hash = _compiled_hash(original)
        original_data = mujoco.MjData(original)
        mujoco.mj_forward(original, original_data)
        base_id = mujoco.mj_name2id(original, mujoco.mjtObj.mjOBJ_BODY, robot["base_body"])
        if base_id < 0:
            raise ValueError("base_body is absent from the model")
        base = np.eye(4)
        base[:3, :3] = original_data.xmat[base_id].reshape(3, 3)
        base[:3, 3] = original_data.xpos[base_id]
        # A moving/floating root requires additional measured state and is unsupported.
        ancestor = base_id
        while ancestor:
            if original.body_jntnum[ancestor]:
                raise ValueError("base_body must have a fixed transform to world")
            ancestor = int(original.body_parentid[ancestor])
        scene = profile["scene"]
        self.clearance = float(scene["clearance_m"])
        table_z = float(scene["table_z_m"])
        if not np.isfinite([self.clearance, table_z]).all() or self.clearance <= 0:
            raise ValueError("table height must be finite and collision clearance positive")
        table_position = base[:3, 3] + base[:3, :3] @ np.array([0, 0, table_z])
        spec.worldbody.add_geom(name="deployment_table", type=mujoco.mjtGeom.mjGEOM_PLANE,
            pos=table_position, quat=matrix_pose(base)[3:], size=[0, 0, 0.01],
            contype=1, conaffinity=2147483647, margin=self.clearance)
        if observation is not None:
            if observation.get("reference_frame") != "base_Link":
                raise ValueError("object pose must be expressed in base_Link")
            radius = float(scene["object_radius_m"])
            if not np.isfinite(radius) or radius <= 0:
                raise ValueError("object_radius_m must conservatively enclose the mesh")
            center = (base @ pose_matrix(observation["pose7"]))[:3, 3]
            spec.worldbody.add_geom(name="deployment_object", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                pos=center, size=[radius, 0, 0], contype=1, conaffinity=2147483647,
                margin=self.clearance)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.base_id = self._id(mujoco.mjtObj.mjOBJ_BODY, robot["base_body"])
        self.wrist_ids = {side: self._id(mujoco.mjtObj.mjOBJ_BODY, robot["wrist_bodies"][side]) for side in ("left", "right")}
        names = list(robot["arm_joint_names"]["left"]) + list(robot["arm_joint_names"]["right"])
        head_names = list(robot["head_joint_names"])
        if len(names) != 14 or len(head_names) != 2 or len(set(names + head_names)) != 16:
            raise ValueError("model mapping requires 14 distinct arm joints and 2 head joints")
        self.joint_ids = [self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
        self.head_ids = [self._id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in head_names]
        scalar_types = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        if any(int(self.model.jnt_type[j]) not in scalar_types for j in self.joint_ids + self.head_ids):
            raise ValueError("arm/head mapping must contain scalar joints")
        if self.model.nq != 16:
            raise ValueError("unmapped movable joints: provide a model with other joints fixed")
        self.arm_qadr = self.model.jnt_qposadr[self.joint_ids]
        self.head_qadr = self.model.jnt_qposadr[self.head_ids]
        self.lower = vector(robot["joint_lower"], 14, "joint_lower")
        self.upper = vector(robot["joint_upper"], 14, "joint_upper")
        if np.any(self.lower >= self.upper):
            raise ValueError("joint lower limits must be below upper limits")
        for index, joint in enumerate(self.joint_ids):
            if self.model.jnt_limited[joint]:
                low, high = self.model.jnt_range[joint]
                if self.lower[index] < low - 1e-9 or self.upper[index] > high + 1e-9:
                    raise ValueError("configured limits exceed the robot model limits")
        self.table_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, "deployment_table")
        arm_bodies = set(int(self.model.jnt_bodyid[j]) for j in self.joint_ids)
        self.arm_geoms = set()
        for geom in range(self.model.ngeom):
            body = int(self.model.geom_bodyid[geom])
            while body:
                if body in arm_bodies:
                    self.arm_geoms.add(geom)
                    break
                body = int(self.model.body_parentid[body])
            if self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]:
                self.model.geom_margin[geom] = max(self.model.geom_margin[geom], self.clearance)
        for side, joint_ids in (("left", self.joint_ids[:7]), ("right", self.joint_ids[7:])):
            joint_bodies = set(int(self.model.jnt_bodyid[j]) for j in joint_ids)
            found = False
            for geom in self.arm_geoms:
                body = int(self.model.geom_bodyid[geom])
                while body:
                    if body in joint_bodies and (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]):
                        found = True
                    body = int(self.model.body_parentid[body])
            if not found:
                raise ValueError(f"{side} arm has no active collision geometry")
        self.model.pair_margin[:] = np.maximum(self.model.pair_margin, self.clearance)
        # Upper bound on displacement of any moving collision-geometry point per
        # radian/metre of each arm joint, for all configured joint positions.
        # Downstream hinge offsets contribute at most twice their offset norm;
        # slide travel is bounded by the accepted limits. This also covers tools.
        self.motion_weights = np.zeros(14)
        arm_index = {joint: index for index, joint in enumerate(self.joint_ids)}
        for geom in self.arm_geoms:
            if not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom]):
                continue
            radius = float(self.model.geom_rbound[geom] + np.linalg.norm(self.model.geom_pos[geom]))
            if not np.isfinite(radius):
                raise ValueError("moving collision geometry must have a finite bounding radius")
            body = int(self.model.geom_bodyid[geom])
            while body:
                joints = range(int(self.model.body_jntadr[body]), int(self.model.body_jntadr[body] + self.model.body_jntnum[body]))
                for joint in joints:
                    radius += 2 * np.linalg.norm(self.model.jnt_pos[joint])
                    if self.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE:
                        index = arm_index.get(joint)
                        if index is None:
                            if not self.model.jnt_limited[joint]:
                                raise ValueError("unbounded slide joint in collision geometry")
                            radius += max(abs(self.model.jnt_range[joint]))
                        else:
                            radius += max(abs(self.lower[index]), abs(self.upper[index]))
                for joint in joints:
                    index = arm_index.get(joint)
                    if index is not None:
                        weight = 1.0 if self.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE else radius
                        self.motion_weights[index] = max(self.motion_weights[index], weight)
                radius += np.linalg.norm(self.model.body_pos[body])
                body = int(self.model.body_parentid[body])

    def _id(self, kind, name):
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"model is missing {name}")
        return value

    def set_state(self, arms, head):
        arms = vector(arms, 14, "arm_q14")
        head = vector(head, 2, "head_q2")
        if np.any(arms < self.lower - 1e-8) or np.any(arms > self.upper + 1e-8):
            raise ValueError("arm state violates configured joint limits")
        for value, joint in zip(head, self.head_ids):
            if self.model.jnt_limited[joint] and not self.model.jnt_range[joint, 0] <= value <= self.model.jnt_range[joint, 1]:
                raise ValueError("head state violates model limits")
        self.data.qpos[self.arm_qadr] = arms
        self.data.qpos[self.head_qadr] = head
        mujoco.mj_forward(self.model, self.data)

    def wrist_matrices(self):
        base = np.eye(4)
        base[:3, :3] = self.data.xmat[self.base_id].reshape(3, 3)
        base[:3, 3] = self.data.xpos[self.base_id]
        inverse = np.linalg.inv(base)
        result = {}
        for side, body in self.wrist_ids.items():
            pose = np.eye(4)
            pose[:3, :3] = self.data.xmat[body].reshape(3, 3)
            pose[:3, 3] = self.data.xpos[body]
            result[side] = inverse @ pose
        return result

    def wrist_poses(self):
        return {side: matrix_pose(value) for side, value in self.wrist_matrices().items()}

    def check_collision(self):
        for contact in self.data.contact:
            geoms = (int(contact.geom1), int(contact.geom2))
            if self.table_id in geoms and next(g for g in geoms if g != self.table_id) not in self.arm_geoms:
                continue
            if contact.dist < self.clearance - 1e-8:
                names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g) or str(g) for g in geoms]
                raise ValueError(f"collision/clearance violation: {names[0]} / {names[1]} ({contact.dist:.6f} m)")

    def solve(self, goals, seed, head, position_tolerance=0.001, orientation_tolerance=0.01):
        seed = vector(seed, 14, "IK seed")
        indices = np.array([i for side in goals for i in (range(7) if side == "left" else range(7, 14))])
        def residual(values):
            arms = seed.copy()
            arms[indices] = values
            self.set_state(arms, head)
            current = self.wrist_matrices()
            errors = []
            for side, target in goals.items():
                errors.extend((current[side][:3, 3] - target[:3, 3]) * 5)
                errors.extend(rotation_error(target[:3, :3], current[side][:3, :3]))
            errors.extend((values - seed[indices]) * 0.0001)
            return np.array(errors)
        solution = least_squares(residual, np.clip(seed[indices], self.lower[indices], self.upper[indices]),
            bounds=(self.lower[indices], self.upper[indices]), max_nfev=120,
            ftol=1e-9, xtol=1e-9, gtol=1e-9)
        arms = seed.copy()
        arms[indices] = solution.x
        self.set_state(arms, head)
        current = self.wrist_matrices()
        for side, target in goals.items():
            position_error = np.linalg.norm(current[side][:3, 3] - target[:3, 3])
            orientation_error = np.linalg.norm(rotation_error(target[:3, :3], current[side][:3, :3]))
            if position_error > position_tolerance or orientation_error > orientation_tolerance:
                raise ValueError(f"unreachable {side} target: IK error {position_error:.4f} m / {orientation_error:.4f} rad")
        self.check_collision()
        return arms
