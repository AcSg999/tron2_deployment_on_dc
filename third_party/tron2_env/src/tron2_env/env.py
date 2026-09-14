"""
Tron2 Real Robot Environment

提供 Tron2 机器人的环境封装:
- 机器人控制委托给 ``MotionController`` (transport + interpolator + publish loop)
- 观测来源切换:bridge WebSocket(图像+关节由 bridge 对齐)或 legacy(RealSense 直连)
- ``step(action)`` 非阻塞:把 16-dim 目标交给 publish 线程,后台以 ``publish_rate``
  (默认 500 Hz)持续 send_joint_cmd,在 ``eta = 1/fps`` 时间内平滑过渡
"""

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union, Any

import cv2
import numpy as np
from PIL import Image

from tron2_env.config import Tron2Config
from tron2_env.joints import JointIndex
from tron2_env.motion import MotionController, create_motion_controller
from tron2_env.bridge import BridgeConfig


# ============================================================================
# Configuration
# ============================================================================

@dataclass
class CameraConfig:
    """相机配置"""
    # 相机名称 (统一使用 obs 输出名称)
    camera_names: List[str] = field(default_factory=lambda: [
        "cam_high",
        "cam_left_wrist",
        "cam_right_wrist"
    ])
    
    # 相机分辨率 (H, W, C)
    resolution: Tuple[int, int, int] = (480, 640, 3)
    
    # 最大队列大小
    max_queue_size: int = 10
    
    # 是否保存调试图像
    save_debug_images: bool = True
    debug_image_dir: str = "./debug_images"

    


@dataclass
class EnvConfig:
    """环境配置"""
    # 机器人配置
    robot_config: Tron2Config = field(default_factory=Tron2Config)

    # 相机配置
    camera_config: CameraConfig = field(default_factory=CameraConfig)

    # 控制后端: 当前公开版本只支持 "websocket"
    control_backend: str = "websocket"

    # TRON2 WebSocket ServoJ 的 vendor 最低安全频率是 500 Hz。
    publish_rate: float = 500.0

    # consumer 节拍 / 命令到达目标的预期耗时 = 1/fps,MotionController 的
    # LinearInterpolator 用这个 ETA 在两次 command_joints 之间平滑过渡。
    fps: float = 30.0

    # 时间同步容差 (秒)
    time_sync_tolerance: float = 0.01
    time_sync_max_retries: int = 3
    legacy_use_time_sync: bool = True

    # 夹爪初始化开口度 (0-1)
    init_gripper_opening: float = 0.9

    # 原始配置字典（用于透传给其他组件）
    raw_config: Dict[str, Any] = field(default_factory=dict)

    # 观测来源: "legacy" (RealSense 直连) | "bridge" (WebSocket bridge)
    observation_source: str = "legacy"

    # 状态维度: 16 (双臂+夹爪) 或 18 (双臂+夹爪+头部)
    state_dim: int = 16

    # Bridge 模式下 state 来源: "bridge" 使用 bridge 对齐 state，"legacy" 使用机器人直连 state
    bridge_state_source: str = "bridge"

    # Bridge WebSocket 配置（observation_source="bridge" 时生效）
    bridge_config: BridgeConfig = field(default_factory=BridgeConfig)


# ============================================================================
# Tron2 Environment
# ============================================================================

class Tron2Env:
    """Tron2机器人环境
    
    Examples:
        >>> config = EnvConfig(robot_config=Tron2Config(robot_ip="ROBOT_IP"))
        >>> env = Tron2Env(config)
        >>> obs = env.reset()
        >>> action = np.zeros(16)  # 16维动作
        >>> env.step(action)
    """
    
    def __init__(self, config: Optional[EnvConfig] = None):
        """初始化环境

        Args:
            config: 环境配置，如果为None则使用默认配置
        """
        self.config = config or EnvConfig()

        # 设置日志
        self._setup_logger()

        # 初始化机器人(MotionController = transport + interpolator + publish loop)
        self.logger.info(
            "正在初始化机器人控制器 (backend=%s, publish_rate=%.0fHz, fps=%.1f)...",
            self.config.control_backend,
            self.config.publish_rate,
            self.config.fps,
        )
        self.robot: MotionController = create_motion_controller(
            self.config.robot_config,
            backend=self.config.control_backend,
            publish_rate=self.config.publish_rate,
            eta_default=1.0 / max(self.config.fps, 1e-6),
        )

        # 初始化观测来源
        if self.config.observation_source == "bridge":
            self.logger.info("观测来源: Bridge WebSocket")
            if self.config.bridge_state_source not in {"bridge", "legacy"}:
                raise ValueError(
                    f"bridge_state_source must be 'bridge' or 'legacy', got {self.config.bridge_state_source!r}"
                )
            if self.config.bridge_state_source == "legacy":
                self.logger.info("Bridge 模式 state 来源: Legacy robot WebSocket")
                if self.config.bridge_config.joint_topics:
                    self.logger.info("Bridge legacy-state 模式: 禁用 bridge joint/gripper 订阅，仅使用 bridge 图像")
                    self.config.bridge_config.joint_topics = {}
            self.camera_manager = None
            self.bridge_provider = self._init_bridge()
        else:
            self.logger.info("观测来源: Legacy (RealSense 直连)")
            self.camera_manager = self._init_camera()
            self.bridge_provider = None

        # 状态管理
        self.last_action: Optional[np.ndarray] = None
        self.init_joints = self.config.robot_config.init_joints

        # 创建调试图像目录
        if self.config.observation_source == "bridge":
            if self.config.bridge_config.save_debug_images:
                Path(self.config.bridge_config.debug_image_dir).mkdir(parents=True, exist_ok=True)
        else:
            if self.config.camera_config.save_debug_images:
                Path(self.config.camera_config.debug_image_dir).mkdir(parents=True, exist_ok=True)

        self.logger.info("环境初始化完成")
    
    def _setup_logger(self):
        """设置日志系统"""
        self.logger = logging.getLogger("Tron2Env")
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s.%(msecs)03d] [%(name)s] [%(levelname)s] %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
            self.logger.setLevel(logging.INFO)
        self.logger.propagate = False
    
    def _init_bridge(self):
        """初始化 Bridge WebSocket 观测提供者"""
        try:
            from tron2_env.bridge import BridgeObservationProvider
        except ImportError:
            self.logger.error("无法导入 BridgeObservationProvider，请确保 websockets 已安装")
            raise

        provider = BridgeObservationProvider(self.config.bridge_config)
        provider.start()

        # 等待首次观测到达
        self.logger.info("等待 Bridge 观测就绪...")
        try:
            provider.get_obs(timeout=5.0)
        except TimeoutError:
            self.logger.warning("Bridge 首次观测超时，将继续运行")
        self.logger.info("Bridge 观测就绪")

        return provider

    def _init_camera(self):
        """初始化相机管理器"""
        try:
            from tron2_env.camera import MultiCameraManager
        except ImportError:
            self.logger.error("无法导入 MultiCameraManager，请确保 pyrealsense2 已安装")
            raise
        
        # 尝试从 YAML 加载（如果存在配置字典）
        if hasattr(self.config, 'raw_config'):
            camera_manager = MultiCameraManager.from_config(self.config.raw_config)
        else:
            camera_manager = MultiCameraManager(
                max_queue_size=self.config.camera_config.max_queue_size
            )
        
        camera_manager.start_capture()
        
        # 等待相机预热
        self.logger.info("相机预热中...")
        time.sleep(2.0)
        
        return camera_manager
    
    # ========================================================================
    # Environment Interface
    # ========================================================================
    
    def reset(self) -> Dict:
        """重置环境到初始状态
        
        Returns:
            初始观测
        """
        self.logger.info("重置环境...")
        
        # 获取当前观测
        obs = self.get_obs()

        # 验证图像尺寸（仅 legacy 模式）
        if self.config.observation_source != "bridge":
            expected_shape = self.config.camera_config.resolution
            for cam_name in self.config.camera_config.camera_names:
                actual_shape = obs['images'][cam_name].shape
                if actual_shape != expected_shape:
                    self.logger.warning(
                        f"{cam_name} 分辨率不匹配: 期望{expected_shape}, 实际{actual_shape}"
                    )
        
        # 验证机器人位置
        if self.init_joints is not None:
            current_state = obs['state']
            arm_states = np.concatenate([
                current_state[JointIndex.LEFT_ARM], 
                current_state[JointIndex.RIGHT_ARM]
            ])
            init_arm = np.array(self.init_joints)
            
            error = np.abs(arm_states - init_arm).max()
            if error > 0.05:
                self.logger.warning(f"机器人未在初始位置，最大误差: {error:.4f}")
                self.robot.wait_until_reached(self.init_joints,tolerance=0.05)
        
        # 初始化夹爪
        test_action = obs['state'].copy()
        test_action[JointIndex.LEFT_GRIPPER] = self.config.init_gripper_opening
        test_action[JointIndex.RIGHT_GRIPPER] = self.config.init_gripper_opening
        self.step(test_action)
        
        self.logger.info("环境重置完成")
        return obs

    def step(self, action: Union[List[float], np.ndarray]):
        """执行一个动作.

        非阻塞:把 16-dim 目标更新到 MotionController 的内部 interpolator,
        publish 线程会在 ~1/fps 时间窗内平滑过渡到该目标。夹爪走单独通路。

        Args:
            action: 16/18维动作向量
                   - 16维: [7关节+1夹爪(左), 7关节+1夹爪(右)]
                   - 18维: [7关节+1夹爪(左), 7关节+1夹爪(右), 2头部]
        """
        # 输入验证
        if isinstance(action, list):
            action = np.array(action)

        if len(action) not in [JointIndex.SERVOJ_DIM, JointIndex.STATE_DIM]:
            raise ValueError(f"动作维度应为{JointIndex.SERVOJ_DIM}/{JointIndex.STATE_DIM}, 实际{len(action)}")

        # 提取双臂关节动作 (14-dim)
        arm_action = np.concatenate([
            action[JointIndex.LEFT_ARM],
            action[JointIndex.RIGHT_ARM]
        ])

        # 提取头部动作 (2-dim)
        if len(action) >= JointIndex.STATE_DIM:
            head_action = action[JointIndex.HEAD]
        else:
            # action 不含头部,沿用当前头部位置 (transport 缓存,无锁泄漏)
            head_action = self.robot.get_head_position()

        # 组合为 16 维 servoj 设定点 (14 臂 + 2 头)
        full_servo_action = np.concatenate([arm_action, head_action])

        # 夹爪 (归一化 0..1 → 0..100)
        gripper_action = np.clip(
            np.array([action[JointIndex.LEFT_GRIPPER], action[JointIndex.RIGHT_GRIPPER]]) * 100.0,
            0, 100,
        )
        self.robot.set_gripper(
            left_opening=gripper_action[0],
            right_opening=gripper_action[1],
        )

        # 更新 publish loop 的目标。MotionController 用 eta=1/fps 在两次
        # command_joints 之间线性插值,所以这里不需要 env 自己再做插值。
        self.robot.command_joints(full_servo_action)
        self.last_action = full_servo_action
    
    def get_obs(self) -> Dict:
        """获取当前观测

        Returns:
            观测字典: {
                'state': np.ndarray,  # 关节状态 (16/18维)
                'images': Dict[str, np.ndarray]  # 图像字典
            }

        metadata 中关于时间戳的约定：
        - ``joint_timestamp_ms`` / ``gripper_timestamp_ms`` 始终对应 ``state``
          字段实际的来源（"我们推理用的那帧 state 的时间戳"）。
        - ``state_source`` 取值 ``bridge`` 或 ``legacy``，指示 state 来自哪条路径。
        - 若 ``state_source == "legacy"`` 但图像走 bridge，``bridge_joint_timestamp_ms``
          / ``bridge_gripper_timestamp_ms`` 会保留 bridge 自己对齐到的关节时间戳，
          供调试对比，不参与正常推理。
        """
        # Bridge 模式：图像来自 bridge，可选 state 来自 bridge 或机器人直连
        if self.config.observation_source == "bridge":
            obs = self.bridge_provider.get_obs(timeout=1.0)
            metadata = dict(obs.get("metadata", {}))
            if self.config.bridge_state_source == "legacy":
                # state 走 robot ws 直拉。bridge 已经在 TopicAligner 里对齐过
                # 图像-state,这里直接信任 robot ws 自己的 timestamp,不再做二次
                # 对齐 —— 之前的 _sync_observation 调用会把 bridge ts 与 robot ws
                # ts 强行拼到一起,反而引入不同源时钟的失配,并多出 ~5ms sleep。
                bridge_joint_timestamp_ms = metadata.get("joint_timestamp_ms")
                bridge_gripper_timestamp_ms = metadata.get("gripper_timestamp_ms")
                qpos_dict = self.robot.get_joint_states(timeout=0.5)
                obs["state"] = np.asarray(
                    qpos_dict["states"][:self.config.state_dim], dtype=np.float32
                )
                metadata.update({
                    "state_source": "legacy",
                    "bridge_joint_timestamp_ms": bridge_joint_timestamp_ms,
                    "bridge_gripper_timestamp_ms": bridge_gripper_timestamp_ms,
                    "joint_timestamp_ms": qpos_dict.get("timestamp"),
                    "gripper_timestamp_ms": qpos_dict.get("timestamp"),
                })
            else:
                obs["state"] = obs["state"][:self.config.state_dim]
                metadata["state_source"] = "bridge"
            obs["metadata"] = metadata
            if self.config.bridge_config.save_debug_images:
                self._save_debug_images_bridge(obs)
            return obs

        # Legacy 模式：先拿图像，用三相机中最旧的时间戳作为 obs 参考时刻，
        # 再在 200Hz joint_state_queue 里找与该时刻最近的 joint 帧。
        # 时间基准统一为客户端 time.time()（camera 和 transport 都用同一时钟）。
        obs_start = time.time()
        # 1. 获取图像
        rgb_images = self._get_images()

        # 保存调试图像
        if self.config.camera_config.save_debug_images:
            self._save_debug_images(rgb_images)

        # 2. 确定 obs 参考时间戳 = 三相机中最旧的那帧（保证所有图像都 ≥ 该时刻）
        cam_timestamps = []
        for cam_name in self.config.camera_config.camera_names:
            ts_key = f'{cam_name}_timestamp'
            if ts_key in rgb_images:
                cam_timestamps.append(rgb_images[ts_key])
        if not cam_timestamps:
            raise RuntimeError("No camera frames available in legacy mode")
        img_timestamp = min(cam_timestamps)  # 最旧的那帧

        # 3. 获取关节状态——在 joint_state_queue 里找与 img_timestamp 最近的帧
        synced_qpos: Optional[Dict] = None
        if self.config.legacy_use_time_sync:
            synced_qpos = self.robot.find_nearest_state(img_timestamp)
            if synced_qpos is not None:
                joint_timestamp = synced_qpos['timestamp'] / 1000.0
                self.logger.debug(
                    "legacy obs sync: nearest queued joint_img=%.1fms",
                    (joint_timestamp - img_timestamp) * 1000.0,
                )
        # Fallback: 队列空 / 不启用 sync —— 走原始 popleft 取最新帧
        qpos_dict = synced_qpos
        if qpos_dict is None:
            qpos_dict = self.robot.get_joint_states(timeout=0.5)
            synced_qpos = qpos_dict
        joint_timestamp = qpos_dict['timestamp'] / 1000.0

        self.logger.debug(
            "legacy obs: ref_img=%.3fs joint=%.3fs diff=%.1fms",
            img_timestamp, joint_timestamp,
            (joint_timestamp - img_timestamp) * 1000.0,
        )

        # 4. 构建观测
        images = {}
        for cam_name in self.config.camera_config.camera_names:
            if cam_name in rgb_images:
                images[cam_name] = rgb_images[cam_name]
        image_timestamps_ms = {
            cam_name: int(rgb_images[f'{cam_name}_timestamp'] * 1000)
            for cam_name in self.config.camera_config.camera_names
            if f'{cam_name}_timestamp' in rgb_images
        }
        image_timestamp_ms = (
            min(image_timestamps_ms.values()) if image_timestamps_ms else int(img_timestamp * 1000)
        )
        joint_timestamp_ms = synced_qpos.get('timestamp')
        synced_joint_timestamp = (joint_timestamp_ms or 0) / 1000.0
        obs_end = time.time()
        image_span_ms = 0.0
        if image_timestamps_ms:
            image_span_ms = max(image_timestamps_ms.values()) - min(image_timestamps_ms.values())

        self.logger.debug(
            "legacy obs timing: sync=%s raw_joint_img=%.1fms synced_joint_img=%.1fms "
            "img_age=%.1fms joint_age=%.1fms total=%.1fms image_span=%.1fms",
            self.config.legacy_use_time_sync,
            (joint_timestamp - img_timestamp) * 1000.0,
            (synced_joint_timestamp - img_timestamp) * 1000.0,
            (obs_end - img_timestamp) * 1000.0,
            (obs_end - synced_joint_timestamp) * 1000.0 if synced_joint_timestamp > 0 else float("nan"),
            (obs_end - obs_start) * 1000.0,
            image_span_ms,
        )

        obs = {
            "state": np.array(synced_qpos['states'][:self.config.state_dim]),
            "images": images,
            "metadata": {
                "state_source": "legacy",
                "observation_ref_timestamp_ms": image_timestamp_ms,
                "bridge_ref_timestamp_ms": image_timestamp_ms,
                "joint_timestamp_ms": joint_timestamp_ms,
                "gripper_timestamp_ms": joint_timestamp_ms,
                "image_timestamp_ms": image_timestamp_ms,
                "image_timestamps_ms": image_timestamps_ms,
                "legacy_initial_joint_timestamp_ms": qpos_dict.get('timestamp'),
                "legacy_time_sync_enabled": self.config.legacy_use_time_sync,
            },
        }

        return obs
    
    # ========================================================================
    # Private Methods
    # ========================================================================

    def _get_images(self) -> Dict:
        """获取相机图像

        Returns:
            图像字典: {
                'cam_high': np.ndarray,
                'cam_high_timestamp': float,
                ...
            }
        """
        all_frames = self.camera_manager.get_all_latest_frames()
        image_dict = {}
        
        for camera_name, frame_data in all_frames.items():
            if frame_data is not None:
                # BGR转RGB
                image_dict[camera_name] = frame_data['color'][:, :, ::-1]
                image_dict[f'{camera_name}_timestamp'] = frame_data['timestamp']
        
        return image_dict
    
    def _sync_observation(
        self,
        img_timestamp: float,
        initial_qpos: Dict,
        using_sync: bool = False
    ) -> Dict:
        """同步观测时间戳

        策略: 直接在 transport 的 200Hz joint_state_queue (maxlen=7, ~35ms 历史窗口) 中
        查询与 img_timestamp 时间最近的关节帧——非阻塞，不消耗队列，避免重试 sleep。

        Args:
            img_timestamp: 图像时间戳 (秒)
            initial_qpos: get_obs 已 popleft 的最新关节状态 (作为 fallback)

        Returns:
            同步后的关节状态
        """
        joint_timestamp = initial_qpos['timestamp'] / 1000.0
        time_dif = joint_timestamp - img_timestamp

        if not using_sync:
            self.logger.debug("legacy obs sync disabled: joint_img=%.1fms", time_dif * 1000.0)
            return initial_qpos

        # 已经在容差内，直接用
        if abs(time_dif) <= self.config.time_sync_tolerance:
            self.logger.debug("legacy obs sync ok: joint_img=%.1fms", time_dif * 1000.0)
            return initial_qpos

        # 在队列里找与 img_timestamp 最近的帧
        nearest = self.robot.find_nearest_state(img_timestamp)
        if nearest is None:
            self.logger.debug(
                "legacy obs sync: no queued state available; using initial (joint_img=%.1fms)",
                time_dif * 1000.0,
            )
            return initial_qpos

        nearest_ts = nearest['timestamp'] / 1000.0
        nearest_dif = nearest_ts - img_timestamp
        # 只有当队列里的帧比 initial_qpos 更接近 image_ts 才换用
        if abs(nearest_dif) < abs(time_dif):
            self.logger.debug(
                "legacy obs sync: switched to nearer queued state (initial=%.1fms, nearest=%.1fms)",
                time_dif * 1000.0,
                nearest_dif * 1000.0,
            )
            return nearest

        self.logger.debug(
            "legacy obs sync: initial already nearest (initial=%.1fms, queued_best=%.1fms)",
            time_dif * 1000.0,
            nearest_dif * 1000.0,
        )
        return initial_qpos
    
    def _save_debug_images(self, rgb_images: Dict):
        """保存调试图像（legacy 模式）"""
        debug_dir = Path(self.config.camera_config.debug_image_dir)

        for key in ['cam_high', 'cam_left_wrist', 'cam_right_wrist']:
            if key in rgb_images:
                img = Image.fromarray(rgb_images[key])
                save_path = debug_dir / f"{key}.jpg"
                img.save(save_path)

    def _save_debug_images_bridge(self, obs: Dict):
        """保存调试图像（bridge 模式）"""
        debug_dir = Path(self.config.bridge_config.debug_image_dir)

        for cam_name, image in obs.get("images", {}).items():
            img = Image.fromarray(image)
            save_path = debug_dir / f"{cam_name}.jpg"
            img.save(save_path)
    
    def close(self):
        """关闭环境并释放资源"""
        self.logger.info("关闭环境...")

        if hasattr(self, 'robot'):
            self.robot.disconnect()

        if hasattr(self, 'camera_manager') and self.camera_manager is not None:
            self.camera_manager.stop_capture()

        if hasattr(self, 'bridge_provider') and self.bridge_provider is not None:
            self.bridge_provider.stop()

        self.logger.info("环境已关闭")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.close()


# ============================================================================
# Policy Wrapper (Example)
# ============================================================================

class PolicyWrapper:
    """策略包装器基类"""
    
    def get_action(self, observation: Dict) -> np.ndarray:
        """获取动作
        
        Args:
            observation: 观测字典
            
        Returns:
            动作数组
        """
        raise NotImplementedError


class WebsocketPolicyWrapper(PolicyWrapper):
    """基于WebSocket的策略客户端"""
    
    def __init__(self, host: str = "localhost", port: int = 8000):
        """初始化WebSocket策略客户端
        
        Args:
            host: 服务器地址
            port: 服务器端口
        """
        try:
            from openpi_client import websocket_client_policy, image_tools
            self.ws_client = websocket_client_policy.WebsocketClientPolicy(
                host=host, 
                port=port
            )
            self.image_tools = image_tools
        except ImportError as e:
            raise ImportError(f"无法导入 openpi_client: {e}")
        
        self.logger = logging.getLogger("WebsocketPolicy")
    
    def get_action(self, observation: Dict) -> np.ndarray:
        """通过WebSocket获取动作
        
        Args:
            observation: 观测字典
            
        Returns:
            动作序列 (action_horizon, action_dim)
        """
        import einops
        
        # 预处理图像
        obs = observation.copy()
        for cam_name in obs["images"]:
            img = self.image_tools.convert_to_uint8(
                self.image_tools.resize_with_pad(obs["images"][cam_name], 224, 224)
            )
            obs["images"][cam_name] = einops.rearrange(img, "h w c -> c h w")
        
        # 推理
        result = self.ws_client.infer(obs)
        actions = np.stack(result['actions'], axis=0)

        return actions
