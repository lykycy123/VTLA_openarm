#!/usr/bin/env python3
"""RBM OpenArm 数据采集：相机 + 关节 + 末端位姿 + 底盘 odom/cmd_vel。

用法（需先 source ROS / 工作空间，并启动相机、机械臂、底盘）：
  cd ~/RBM_openarm
  source source_all.sh
  conda activate openarm   # 需 h5py / opencv
  python data_collection/data_collection.py

操作：
  SPACE  开始/停止录制并保存 episode_*.hdf5
  ESC    退出
"""

from __future__ import annotations

import argparse
import json
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import h5py
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, JointState
from tf2_ros import Buffer, TransformException, TransformListener

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "config.yaml"
JPEG_QUALITY = 80

LEFT_ARM_JOINTS = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]
RIGHT_ARM_JOINTS = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]
LEFT_GRIPPER_JOINT = "openarm_left_finger_joint1"
RIGHT_GRIPPER_JOINT = "openarm_right_finger_joint1"
ALL_JOINT_NAMES = LEFT_ARM_JOINTS + [LEFT_GRIPPER_JOINT] + RIGHT_ARM_JOINTS + [RIGHT_GRIPPER_JOINT]


def _resolve_config_path(config_path: str) -> Path:
    path = Path(config_path).expanduser()
    if path.is_absolute():
        return path
    cwd_path = (Path.cwd() / path).resolve()
    if cwd_path.exists():
        return cwd_path
    return (SCRIPT_DIR / path).resolve()


def _resolve_repo_path(path_value: str) -> Path:
    path = Path(str(path_value)).expanduser()
    if path.is_absolute():
        return path
    return (REPO_DIR / path).resolve()


def _load_config(config_path: str) -> Dict:
    path = _resolve_config_path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return config


def _section(config: Dict, name: str) -> Dict:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}


def _extract_named(names: List[str], values: List[float], ordered: List[str]) -> np.ndarray:
    name_to_val = {n: float(v) for n, v in zip(names, values)}
    out = np.zeros(len(ordered), dtype=np.float64)
    for i, name in enumerate(ordered):
        if name in name_to_val:
            out[i] = name_to_val[name]
    return out


def _pose_from_transform(transform) -> np.ndarray:
    t = transform.transform.translation
    r = transform.transform.rotation
    return np.asarray([t.x, t.y, t.z, r.x, r.y, r.z, r.w], dtype=np.float64)


class EpisodeBuffer:
    def __init__(self, jpeg_quality: int):
        self.jpeg_quality = jpeg_quality
        self.reset()

    def reset(self):
        self.timestamp: List[float] = []
        self.qpos: List[np.ndarray] = []
        self.qvel: List[np.ndarray] = []
        self.eef: List[np.ndarray] = []
        self.odom: List[np.ndarray] = []
        self.cmd_vel: List[np.ndarray] = []
        self.images_head: List[np.ndarray] = []
        self.images_left: List[np.ndarray] = []
        self.images_right: List[np.ndarray] = []

    @property
    def frame_count(self) -> int:
        return len(self.timestamp)

    def append(
        self,
        timestamp: float,
        qpos: np.ndarray,
        qvel: np.ndarray,
        eef: np.ndarray,
        odom: np.ndarray,
        cmd_vel: np.ndarray,
        head_img: np.ndarray,
        left_img: np.ndarray,
        right_img: np.ndarray,
    ):
        self.timestamp.append(float(timestamp))
        self.qpos.append(np.asarray(qpos, dtype=np.float64).reshape(-1).copy())
        self.qvel.append(np.asarray(qvel, dtype=np.float64).reshape(-1).copy())
        self.eef.append(np.asarray(eef, dtype=np.float64).reshape(14).copy())
        self.odom.append(np.asarray(odom, dtype=np.float64).reshape(13).copy())
        self.cmd_vel.append(np.asarray(cmd_vel, dtype=np.float64).reshape(6).copy())
        self.images_head.append(np.asarray(head_img, dtype=np.uint8).copy())
        self.images_left.append(np.asarray(left_img, dtype=np.uint8).copy())
        self.images_right.append(np.asarray(right_img, dtype=np.uint8).copy())

    def save(self, output_path: Path, config: Dict):
        if self.frame_count == 0:
            raise RuntimeError("No frames to save")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = output_path.with_suffix(".tmp.hdf5")

        with h5py.File(str(tmp_path), "w") as h5:
            h5.attrs["schema"] = "rbm_openarm_data_collection"
            h5.attrs["schema_version"] = "1.0"
            data_group = h5.create_group("data")
            demo_group = data_group.create_group("demo_0")
            obs_group = demo_group.create_group("observations")

            demo_group.create_dataset(
                "timestamp", data=np.asarray(self.timestamp, dtype=np.float64)
            )
            obs_group.create_dataset("qpos", data=np.stack(self.qpos, axis=0))
            obs_group.create_dataset("qvel", data=np.stack(self.qvel, axis=0))
            obs_group.create_dataset("eef", data=np.stack(self.eef, axis=0))
            obs_group.create_dataset("odom", data=np.stack(self.odom, axis=0))
            obs_group.create_dataset("cmd_vel", data=np.stack(self.cmd_vel, axis=0))
            # 动作侧：当前帧关节与底盘命令（示教回放常用）
            demo_group.create_dataset("actions", data=np.stack(self.qpos, axis=0))
            demo_group.create_dataset("actions_eef", data=np.stack(self.eef, axis=0))
            demo_group.create_dataset("actions_base", data=np.stack(self.cmd_vel, axis=0))

            image_group = obs_group.create_group("images")
            self._write_compressed_images(image_group, "head", self.images_head)
            self._write_compressed_images(image_group, "left_wrist", self.images_left)
            self._write_compressed_images(image_group, "right_wrist", self.images_right)

            meta_group = h5.create_group("meta")
            meta_group.create_dataset("config", data=json.dumps(config))
            meta_group.create_dataset("num_episodes", data=1)
            meta_group.create_dataset(
                "joint_names",
                data=np.asarray(ALL_JOINT_NAMES, dtype=h5py.string_dtype()),
            )

        os.replace(tmp_path, output_path)

    def _write_compressed_images(self, group, name: str, images: List[np.ndarray]):
        padded, lengths = self._encode_jpeg(images)
        group.create_dataset(
            name,
            data=padded,
            chunks=(1,) + padded.shape[1:],
            compression="lzf",
        )
        group.create_dataset(f"{name}_lengths", data=lengths)

    def _encode_jpeg(self, images: List[np.ndarray]):
        encoded = []
        lengths = []
        for image in images:
            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(
                ".jpg",
                bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            flat = buf.reshape(-1)
            encoded.append(flat)
            lengths.append(flat.size)

        max_len = max(lengths)
        padded = np.stack(
            [np.pad(item, (0, max_len - item.size), constant_values=0) for item in encoded],
            axis=0,
        )
        return padded, np.asarray(lengths, dtype=np.int32)


class StdinKeyboard:
    def __init__(self, on_space, on_escape):
        self._on_space = on_space
        self._on_escape = on_escape
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _loop(self):
        if not sys.stdin.isatty():
            while self._running:
                time.sleep(0.1)
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                ready, _, _ = select.select([fd], [], [], 0.1)
                if not ready:
                    continue
                char = os.read(fd, 1)
                if char == b" ":
                    self._on_space()
                elif char == b"\x1b":
                    self._on_escape()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class DataCollectionNode(Node):
    def __init__(self, args):
        super().__init__("openarm_data_collection")
        self.args = args
        self.datasets_dir = Path(args.datasets_dir).expanduser().resolve()
        self.config = vars(args).copy()
        self.config["datasets_dir"] = str(self.datasets_dir)

        self.episode = EpisodeBuffer(jpeg_quality=args.jpeg_quality)
        self._recording = False
        self._saving = False
        self._shutdown_requested = False
        self._lock = threading.RLock()
        self._sync_count = 0
        self._received_count = {"head": 0, "left": 0, "right": 0}
        self._last_missing_state_warning = 0.0

        self._latest = {
            "timestamp": 0.0,
            "qpos": np.zeros(len(ALL_JOINT_NAMES), dtype=np.float64),
            "qvel": np.zeros(len(ALL_JOINT_NAMES), dtype=np.float64),
            "eef": np.zeros(14, dtype=np.float64),
            "odom": np.zeros(13, dtype=np.float64),
            "cmd_vel": np.zeros(6, dtype=np.float64),
            "has_joint": False,
            "has_eef": False,
            "has_odom": False,
        }

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, args.joint_states_topic, self._on_joint, 20)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 20)
        self.create_subscription(Twist, args.cmd_vel_topic, self._on_cmd_vel, 20)

        self._init_image_sync()
        self.create_timer(1.0 / max(1.0, args.control_hz), self._sample_state)
        self.create_timer(2.0, self._report_status)
        self.create_timer(0.1, self._check_shutdown)

        self._keyboard = StdinKeyboard(self.toggle_recording, self.request_shutdown)
        self._keyboard.start()

        self.get_logger().info(
            "Ready. Topics: joints=%s odom=%s cmd_vel=%s ee=[%s, %s]"
            % (
                args.joint_states_topic,
                args.odom_topic,
                args.cmd_vel_topic,
                args.left_ee_frame,
                args.right_ee_frame,
            )
        )
        self.get_logger().info("Press SPACE to start/stop collection, ESC to quit.")

    def _init_image_sync(self):
        self.get_logger().info(
            "Image topics: head=%s left=%s right=%s"
            % (
                self.args.img_head_topic,
                self.args.img_left_topic,
                self.args.img_right_topic,
            )
        )
        self.head_sub = Subscriber(
            self,
            CompressedImage,
            self.args.img_head_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.left_sub = Subscriber(
            self,
            CompressedImage,
            self.args.img_left_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.right_sub = Subscriber(
            self,
            CompressedImage,
            self.args.img_right_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.head_sub.registerCallback(lambda _msg: self._mark_received("head"))
        self.left_sub.registerCallback(lambda _msg: self._mark_received("left"))
        self.right_sub.registerCallback(lambda _msg: self._mark_received("right"))

        self.sync = ApproximateTimeSynchronizer(
            [self.head_sub, self.left_sub, self.right_sub],
            queue_size=self.args.queue_size,
            slop=self.args.sync_slop,
        )
        self.sync.registerCallback(self._on_images_synced)

    def _mark_received(self, key: str):
        self._received_count[key] += 1

    def _on_joint(self, msg: JointState):
        names = list(msg.name)
        pos = list(msg.position) if msg.position else [0.0] * len(names)
        vel = list(msg.velocity) if msg.velocity else [0.0] * len(names)
        if len(vel) < len(names):
            vel = vel + [0.0] * (len(names) - len(vel))
        with self._lock:
            self._latest["qpos"] = _extract_named(names, pos, ALL_JOINT_NAMES)
            self._latest["qvel"] = _extract_named(names, vel, ALL_JOINT_NAMES)
            self._latest["has_joint"] = True
            self._latest["timestamp"] = float(time.perf_counter())

    def _on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist
        value = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ],
            dtype=np.float64,
        )
        with self._lock:
            self._latest["odom"] = value
            self._latest["has_odom"] = True

    def _on_cmd_vel(self, msg: Twist):
        value = np.asarray(
            [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ],
            dtype=np.float64,
        )
        with self._lock:
            self._latest["cmd_vel"] = value

    def _lookup_ee(self, frame: str) -> Optional[np.ndarray]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.args.world_frame,
                frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            return _pose_from_transform(transform)
        except TransformException:
            return None

    def _sample_state(self):
        left = self._lookup_ee(self.args.left_ee_frame)
        right = self._lookup_ee(self.args.right_ee_frame)
        if left is None or right is None:
            return
        with self._lock:
            self._latest["eef"] = np.concatenate([left, right])
            self._latest["has_eef"] = True
            self._latest["timestamp"] = float(time.perf_counter())

    def _on_images_synced(
        self,
        head_msg: CompressedImage,
        left_msg: CompressedImage,
        right_msg: CompressedImage,
    ):
        self._sync_count += 1
        with self._lock:
            recording = self._recording
            state = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in self._latest.items()}

        if not recording:
            return

        if not state["has_joint"] or not state["has_eef"]:
            now = time.perf_counter()
            if now - self._last_missing_state_warning > 1.0:
                self._last_missing_state_warning = now
                self.get_logger().warning(
                    "Missing joint/eef state yet; skipping frame "
                    "(has_joint=%s has_eef=%s has_odom=%s)."
                    % (state["has_joint"], state["has_eef"], state["has_odom"])
                )
            return

        try:
            head_img = self._decode_compressed_image(head_msg)
            left_img = self._decode_compressed_image(left_msg)
            right_img = self._decode_compressed_image(right_msg)
            with self._lock:
                if not self._recording:
                    return
                self.episode.append(
                    timestamp=float(state["timestamp"]),
                    qpos=state["qpos"],
                    qvel=state["qvel"],
                    eef=state["eef"],
                    odom=state["odom"],
                    cmd_vel=state["cmd_vel"],
                    head_img=head_img,
                    left_img=left_img,
                    right_img=right_img,
                )
                frames = self.episode.frame_count
            if frames % 30 == 0:
                self.get_logger().info(f"episode_frames={frames}")
        except Exception as exc:
            self.get_logger().error(f"Failed to record synced frame: {exc}")

    @staticmethod
    def _decode_compressed_image(msg: CompressedImage) -> np.ndarray:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError("Compressed image decode failed")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def toggle_recording(self):
        with self._lock:
            if self._saving:
                self.get_logger().warning("Still saving; please wait.")
                return
            if not self._recording:
                self.episode.reset()
                self._recording = True
                self.get_logger().info("Data collection started.")
                return
        self.stop_recording(save=True)

    def stop_recording(self, save: bool):
        with self._lock:
            if not self._recording:
                return
            self._recording = False
            frames = self.episode.frame_count

        self.get_logger().info(f"Data collection stopped, frames={frames}.")
        if save and frames > 0:
            self._save_current_episode()
        elif frames == 0:
            self.get_logger().warning("Episode discarded because it has no frames.")

    def _save_current_episode(self):
        with self._lock:
            self._saving = True
        try:
            output_path = self._next_episode_path()
            self.get_logger().info(f"Saving episode to {output_path}")
            self.episode.save(output_path, self.config)
            self.get_logger().info(f"Saved episode to {output_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to save episode: {exc}")
        finally:
            with self._lock:
                self._saving = False

    def _next_episode_path(self) -> Path:
        self.datasets_dir.mkdir(parents=True, exist_ok=True)
        max_index = -1
        for path in self.datasets_dir.glob("episode_*.hdf5"):
            if path.suffixes == [".tmp", ".hdf5"]:
                continue
            try:
                max_index = max(max_index, int(path.stem.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        return self.datasets_dir / f"episode_{max_index + 1}.hdf5"

    def _report_status(self):
        with self._lock:
            recording = self._recording
            frames = self.episode.frame_count
            has_joint = self._latest["has_joint"]
            has_eef = self._latest["has_eef"]
            has_odom = self._latest["has_odom"]
        status = "recording" if recording else "ready"
        self.get_logger().info(
            "Status=%s frames=%s synced=%s recv=h:%s/l:%s/r:%s "
            "joint=%s eef=%s odom=%s"
            % (
                status,
                frames,
                self._sync_count,
                self._received_count["head"],
                self._received_count["left"],
                self._received_count["right"],
                has_joint,
                has_eef,
                has_odom,
            )
        )

    def request_shutdown(self):
        self._shutdown_requested = True

    def _check_shutdown(self):
        if self._shutdown_requested and rclpy.ok():
            rclpy.shutdown()

    def close(self):
        self._keyboard.stop()
        self.stop_recording(save=True)


def parse_args(argv=None):
    clean_argv = remove_ros_args(args=argv if argv is not None else sys.argv)
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    config_args, _ = config_parser.parse_known_args(clean_argv[1:])
    try:
        config = _load_config(config_args.config)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Failed to load config: {exc}") from exc

    paths_config = _section(config, "paths")
    topics_config = _section(config, "topics")
    camera_config = _section(config, "camera")
    collection_config = _section(config, "collection")

    parser = argparse.ArgumentParser(
        description="RBM OpenArm data collection (cameras + arms + base).",
        parents=[config_parser],
    )
    parser.add_argument(
        "--datasets-dir",
        default=str(_resolve_repo_path(paths_config.get("datasets_dir", "datasets"))),
    )
    parser.add_argument(
        "--joint-states-topic",
        default=topics_config.get("joint_states", "/joint_states"),
    )
    parser.add_argument("--odom-topic", default=topics_config.get("odom", "/odom"))
    parser.add_argument("--cmd-vel-topic", default=topics_config.get("cmd_vel", "/cmd_vel"))
    parser.add_argument(
        "--left-ee-frame",
        default=topics_config.get("left_ee_frame", "openarm_left_hand_tcp"),
    )
    parser.add_argument(
        "--right-ee-frame",
        default=topics_config.get("right_ee_frame", "openarm_right_hand_tcp"),
    )
    parser.add_argument(
        "--world-frame",
        default=topics_config.get("world_frame", "world"),
    )
    parser.add_argument(
        "--img-head-topic",
        default=camera_config.get(
            "head_topic", "/camera/camera_h/color/image_rect_raw/compressed"
        ),
    )
    parser.add_argument(
        "--img-left-topic",
        default=camera_config.get(
            "left_topic", "/camera/camera_l/color/image_rect_raw/compressed"
        ),
    )
    parser.add_argument(
        "--img-right-topic",
        default=camera_config.get(
            "right_topic", "/camera/camera_r/color/image_rect_raw/compressed"
        ),
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        default=collection_config.get("control_hz", 30.0),
    )
    parser.add_argument(
        "--queue-size", type=int, default=collection_config.get("queue_size", 10)
    )
    parser.add_argument(
        "--sync-slop", type=float, default=collection_config.get("sync_slop", 0.05)
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=collection_config.get("jpeg_quality", JPEG_QUALITY),
    )
    args = parser.parse_args(clean_argv[1:])
    args.config = str(_resolve_config_path(args.config))
    if args.control_hz <= 0.0:
        parser.error("--control-hz must be positive")
    if args.queue_size <= 0:
        parser.error("--queue-size must be positive")
    if args.sync_slop < 0.0:
        parser.error("--sync-slop must be non-negative")
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=argv)
    node = None
    try:
        node = DataCollectionNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
