#!/usr/bin/env python3
"""RBM OpenArm 数据回放：读取 data_collection 保存的 HDF5，发布臂/夹爪/底盘命令。

用法（需先 source ROS / 工作空间，并启动 forward_position 驱动；回放底盘时还需底盘驱动）：
  cd ~/Medibot_openarm
  source source_all.sh
  conda activate openarm
  python data_collection/replay.py
  # 或指定文件：
  # python data_collection/replay.py --replay-hdf5-path datasets/episode_0.hdf5

操作：
  SPACE  开始/暂停回放
  ESC    退出（可选回 home）
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional

import h5py
import numpy as np
import rclpy
import yaml
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "config.yaml"

# 与 data_collection.py 中 ALL_JOINT_NAMES 布局一致
QPOS_DIM = 16
LEFT_ARM_SLICE = slice(0, 7)
LEFT_GRIP_INDEX = 7
RIGHT_ARM_SLICE = slice(8, 15)
RIGHT_GRIP_INDEX = 15
CMD_VEL_DIM = 6
HOME_ARM = np.zeros(7, dtype=np.float64)
HOME_GRIPPER_OPEN = 0.044


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


def _load_config(config_path: str) -> dict:
    path = _resolve_config_path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {path}")
    return config


def _section(config: dict, name: str) -> dict:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}


def _list_episode_files(datasets_dir: Path) -> list[tuple[int, Path]]:
    """Return sorted (episode_index, path) for episode_*.hdf5 under datasets_dir."""
    episodes: list[tuple[int, Path]] = []
    for path in sorted(datasets_dir.glob("episode_*.hdf5")):
        if path.suffixes == [".tmp", ".hdf5"]:
            continue
        try:
            index = int(path.stem.split("_", 1)[1])
        except (IndexError, ValueError):
            continue
        episodes.append((index, path))
    episodes.sort(key=lambda item: item[0])
    return episodes


def _prompt_select_episode(episodes: list[tuple[int, Path]]) -> Path:
    """Print dataset list and ask user to pick by list index or episode number."""
    print("", flush=True)
    print("========== 可选数据集 ==========", flush=True)
    for list_idx, (ep_idx, path) in enumerate(episodes):
        size_mb = path.stat().st_size / (1024.0 * 1024.0)
        print(
            "  [%d] episode_%d.hdf5  (%.1f MB)  %s"
            % (list_idx, ep_idx, size_mb, path),
            flush=True,
        )
    print("================================", flush=True)

    default_list_idx = len(episodes) - 1
    default_ep_idx = episodes[default_list_idx][0]
    while True:
        try:
            raw = input(
                "请输入数据集序号 [%d-%d]，或 episode 编号（直接 Enter=最新 episode_%d）："
                % (0, len(episodes) - 1, default_ep_idx)
            ).strip()
        except EOFError as exc:
            raise SystemExit("No dataset selected (stdin closed).") from exc

        if raw == "":
            selected = episodes[default_list_idx]
            print(
                "[INFO] [replay]: 已选择最新数据集 [%d] episode_%d.hdf5"
                % (default_list_idx, selected[0]),
                flush=True,
            )
            return selected[1]

        try:
            value = int(raw)
        except ValueError:
            print("[WARN] 请输入整数序号。", flush=True)
            continue

        # Prefer list index when in range; otherwise match episode_N number.
        if 0 <= value < len(episodes):
            selected = episodes[value]
            print(
                "[INFO] [replay]: 已选择 [%d] episode_%d.hdf5"
                % (value, selected[0]),
                flush=True,
            )
            return selected[1]

        for list_idx, (ep_idx, path) in enumerate(episodes):
            if ep_idx == value:
                print(
                    "[INFO] [replay]: 已选择 [%d] episode_%d.hdf5"
                    % (list_idx, ep_idx),
                    flush=True,
                )
                return path

        print(
            "[WARN] 无效序号：%s（可用列表下标 0-%d，或已有 episode 编号）"
            % (raw, len(episodes) - 1),
            flush=True,
        )


class Logger:
    def info(self, message: str):
        print(f"[INFO] [replay]: {message}", flush=True)

    def warning(self, message: str):
        print(f"[WARN] [replay]: {message}", flush=True)

    def error(self, message: str):
        print(f"[ERROR] [replay]: {message}", flush=True)


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


class ReplayEpisode:
    def __init__(self, path: Path, demo_index: int, fallback_hz: float):
        self.path = path
        self.demo_index = demo_index
        self.actions, self.base_actions, self.rel_timestamps = self._load(
            path, demo_index, fallback_hz
        )

    @property
    def frame_count(self) -> int:
        return int(self.actions.shape[0])

    @staticmethod
    def _load(path: Path, demo_index: int, fallback_hz: float):
        if not path.exists():
            raise FileNotFoundError(f"Replay file not found: {path}")

        with h5py.File(str(path), "r") as h5:
            demo_key = f"demo_{demo_index}"
            if "data" not in h5 or demo_key not in h5["data"]:
                raise KeyError(f"Missing data/{demo_key} in {path}")

            demo_group = h5["data"][demo_key]
            if "actions" in demo_group:
                actions = np.asarray(demo_group["actions"], dtype=np.float64)
            elif "observations" in demo_group and "qpos" in demo_group["observations"]:
                actions = np.asarray(demo_group["observations"]["qpos"], dtype=np.float64)
            else:
                raise KeyError(
                    "Replay episode must contain data/demo_x/actions "
                    "or data/demo_x/observations/qpos"
                )

            if actions.ndim != 2 or actions.shape[1] != QPOS_DIM:
                raise ValueError(
                    f"Invalid replay action shape {actions.shape}, expected (N, {QPOS_DIM})"
                )

            if "actions_base" in demo_group:
                base_actions = np.asarray(demo_group["actions_base"], dtype=np.float64)
            elif "observations" in demo_group and "cmd_vel" in demo_group["observations"]:
                base_actions = np.asarray(
                    demo_group["observations"]["cmd_vel"], dtype=np.float64
                )
            else:
                base_actions = np.zeros((actions.shape[0], CMD_VEL_DIM), dtype=np.float64)

            if base_actions.ndim != 2 or base_actions.shape[1] != CMD_VEL_DIM:
                raise ValueError(
                    f"Invalid base action shape {base_actions.shape}, "
                    f"expected (N, {CMD_VEL_DIM})"
                )

            if "timestamp" in demo_group:
                timestamps = np.asarray(demo_group["timestamp"], dtype=np.float64)
            else:
                timestamps = np.arange(actions.shape[0], dtype=np.float64) / fallback_hz

        frame_count = min(actions.shape[0], base_actions.shape[0], timestamps.shape[0])
        if frame_count <= 0:
            raise ValueError("Replay episode is empty")

        actions = actions[:frame_count]
        base_actions = base_actions[:frame_count]
        timestamps = timestamps[:frame_count]

        if frame_count > 1 and np.all(np.diff(timestamps) >= 0.0):
            rel_timestamps = timestamps - timestamps[0]
        else:
            rel_timestamps = np.arange(frame_count, dtype=np.float64) / fallback_hz

        return actions, base_actions, rel_timestamps


class OpenArmReplayDriver(Node):
    """通过 ROS 2 话题/Action 回放 OpenArm + 底盘。"""

    def __init__(self, args):
        super().__init__("rbm_openarm_replay")
        self.args = args
        self.pub_left = self.create_publisher(
            Float64MultiArray, args.left_arm_topic, 10
        )
        self.pub_right = self.create_publisher(
            Float64MultiArray, args.right_arm_topic, 10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self._left_gripper = ActionClient(
            self, GripperCommand, args.left_gripper_action
        )
        self._right_gripper = ActionClient(
            self, GripperCommand, args.right_gripper_action
        )
        self._last_left_grip: Optional[float] = None
        self._last_right_grip: Optional[float] = None
        self._last_action = np.zeros(QPOS_DIM, dtype=np.float64)
        self._last_action[LEFT_GRIP_INDEX] = HOME_GRIPPER_OPEN
        self._last_action[RIGHT_GRIP_INDEX] = HOME_GRIPPER_OPEN

    def wait_for_servers(self, timeout_s: float = 5.0) -> None:
        ok_l = self._left_gripper.wait_for_server(timeout_sec=timeout_s)
        ok_r = self._right_gripper.wait_for_server(timeout_sec=timeout_s)
        if not ok_l or not ok_r:
            self.get_logger().warning(
                "Gripper action server not ready (left=%s right=%s); "
                "arm joints will still replay." % (ok_l, ok_r)
            )

    def set_action(self, action: np.ndarray, base: np.ndarray) -> None:
        action = np.asarray(action, dtype=np.float64).reshape(QPOS_DIM)
        base = np.asarray(base, dtype=np.float64).reshape(CMD_VEL_DIM)
        self._last_action = action.copy()

        left = Float64MultiArray()
        left.data = [float(x) for x in action[LEFT_ARM_SLICE]]
        right = Float64MultiArray()
        right.data = [float(x) for x in action[RIGHT_ARM_SLICE]]
        self.pub_left.publish(left)
        self.pub_right.publish(right)

        self._send_gripper(self._left_gripper, float(action[LEFT_GRIP_INDEX]), "left")
        self._send_gripper(self._right_gripper, float(action[RIGHT_GRIP_INDEX]), "right")

        if self.args.replay_base:
            max_lin = float(self.args.max_linear_speed)
            max_ang = float(self.args.max_angular_speed)
            twist = Twist()
            twist.linear.x = float(np.clip(base[0], -max_lin, max_lin))
            twist.linear.y = float(np.clip(base[1], -max_lin, max_lin))
            twist.linear.z = float(np.clip(base[2], -max_lin, max_lin))
            twist.angular.x = float(np.clip(base[3], -max_ang, max_ang))
            twist.angular.y = float(np.clip(base[4], -max_ang, max_ang))
            twist.angular.z = float(np.clip(base[5], -max_ang, max_ang))
            self.pub_cmd_vel.publish(twist)

    def _send_gripper(self, client: ActionClient, position: float, side: str) -> None:
        position = float(np.clip(position, 0.0, HOME_GRIPPER_OPEN))
        last = self._last_left_grip if side == "left" else self._last_right_grip
        if last is not None and abs(position - last) < 0.001:
            return
        if not client.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0
        client.send_goal_async(goal)
        if side == "left":
            self._last_left_grip = position
        else:
            self._last_right_grip = position

    def stop_base(self) -> None:
        if self.args.replay_base:
            self.pub_cmd_vel.publish(Twist())

    def go_home(self, duration_s: float = 2.5) -> None:
        """关节空间插值到 MoveIt home（臂全零、夹爪张开）。"""
        self.get_logger().info("Moving arms to home (%.1fs)..." % duration_s)
        start = self._last_action.copy()
        target = np.zeros(QPOS_DIM, dtype=np.float64)
        target[LEFT_GRIP_INDEX] = HOME_GRIPPER_OPEN
        target[RIGHT_GRIP_INDEX] = HOME_GRIPPER_OPEN
        zero_base = np.zeros(CMD_VEL_DIM, dtype=np.float64)

        t0 = time.monotonic()
        while True:
            alpha = (time.monotonic() - t0) / max(0.5, duration_s)
            if alpha >= 1.0:
                self.set_action(target, zero_base)
                break
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            q = (1.0 - s) * start + s * target
            self.set_action(q, zero_base)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self.stop_base()
        for _ in range(5):
            self.set_action(target, zero_base)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.get_logger().info("Arms at home.")

    def transition_to(self, action: np.ndarray, duration_s: float) -> None:
        if duration_s <= 0.0:
            self.set_action(action, np.zeros(CMD_VEL_DIM))
            return
        self.get_logger().info("Transition to first frame (%.1fs)..." % duration_s)
        start = self._last_action.copy()
        target = np.asarray(action, dtype=np.float64).reshape(QPOS_DIM)
        zero_base = np.zeros(CMD_VEL_DIM, dtype=np.float64)
        t0 = time.monotonic()
        while True:
            alpha = (time.monotonic() - t0) / duration_s
            if alpha >= 1.0:
                self.set_action(target, zero_base)
                break
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            self.set_action((1.0 - s) * start + s * target, zero_base)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


class ReplayRunner:
    def __init__(self, args, driver: OpenArmReplayDriver):
        self.args = args
        self.driver = driver
        self.logger = Logger()
        self._lock = threading.RLock()
        self._running = True
        self._playing = False
        self._frame_idx = 0
        self._start_time: Optional[float] = None
        self._pause_until: Optional[float] = None
        self._pause_started_at: Optional[float] = None
        self._actions_since_pause = 0
        self._episode = self._load_episode()
        self._keyboard = StdinKeyboard(self.toggle_playback, self.request_shutdown)

    def run(self):
        self.driver.wait_for_servers()
        if not self.args.skip_home:
            self.driver.go_home(duration_s=self.args.home_wait)
        if self.args.transition_duration > 0.0 and self._episode.frame_count > 0:
            self.driver.transition_to(
                self._episode.actions[0], self.args.transition_duration
            )

        self._keyboard.start()
        self.logger.info(
            "Replay loaded: file=%s demo=%s frames=%s"
            % (self._episode.path, self._episode.demo_index, self._episode.frame_count)
        )
        self.logger.info("Ready. Press SPACE to start/stop replay, ESC to quit.")

        try:
            period = 1.0 / self.args.control_hz
            while self._running:
                start = time.perf_counter()
                self._step()
                try:
                    rclpy.spin_once(self.driver, timeout_sec=0.0)
                except Exception:
                    pass
                elapsed = time.perf_counter() - start
                time.sleep(max(0.0, period - elapsed))
        finally:
            self.request_shutdown()
            self._keyboard.stop()
            self.driver.stop_base()
            if self.args.home_on_shutdown:
                self._return_home("shutdown")

    def toggle_playback(self):
        with self._lock:
            if self._playing:
                self._playing = False
                self.driver.stop_base()
                self.logger.info("Replay paused at frame %s." % self._frame_idx)
                return

            if self._frame_idx >= self._episode.frame_count:
                self._frame_idx = 0
                self._actions_since_pause = 0
            self._playing = True
            self._pause_until = None
            self._pause_started_at = None
            current_rel_time = self._episode.rel_timestamps[self._frame_idx]
            self._start_time = time.perf_counter() - current_rel_time / self.args.speed
            self.logger.info("Replay started at frame %s." % self._frame_idx)

    def request_shutdown(self):
        with self._lock:
            self._running = False
            self._playing = False
            self._pause_until = None
            self._pause_started_at = None

    def _step(self):
        self._finish_pause_if_needed()
        with self._lock:
            playing = self._playing
            frame_idx = self._frame_idx
            start_time = self._start_time
            pause_until = self._pause_until

        if not playing or start_time is None or pause_until is not None:
            return

        elapsed = (time.perf_counter() - start_time) * self.args.speed
        rel_timestamps = self._episode.rel_timestamps
        total_frames = self._episode.frame_count

        next_idx = frame_idx
        while next_idx < total_frames and rel_timestamps[next_idx] <= elapsed:
            next_idx += 1

        if next_idx == frame_idx:
            return

        send_idx = next_idx - 1
        try:
            self.driver.set_action(
                self._episode.actions[send_idx],
                self._episode.base_actions[send_idx],
            )
        except Exception as exc:
            self.logger.error("Failed to send replay action: %s" % exc)
            with self._lock:
                self._playing = False
            return

        with self._lock:
            self._frame_idx = next_idx
            return_home = False
            if self._frame_idx >= total_frames:
                self.logger.info("Replay finished.")
                self.driver.stop_base()
                if self.args.loop:
                    self._frame_idx = 0
                    self._start_time = time.perf_counter()
                    self._playing = True
                    self._actions_since_pause = 0
                else:
                    self._playing = False
                    self._start_time = None
                    return_home = self.args.home_after_finish
            else:
                self._actions_since_pause += 1
                if self._actions_since_pause >= self.args.stop_every_actions:
                    self._actions_since_pause = 0
                    self._begin_pause_locked()

        if return_home:
            self._return_home("after finish")

    def _finish_pause_if_needed(self):
        now = time.perf_counter()
        with self._lock:
            if self._pause_until is None:
                return
            if now < self._pause_until:
                return
            pause_started_at = self._pause_started_at
            pause_until = self._pause_until
            if self._start_time is not None and pause_started_at is not None:
                self._start_time += pause_until - pause_started_at
            self._pause_until = None
            self._pause_started_at = None
            self.logger.info(
                "Replay pause finished; resuming at frame %s." % self._frame_idx
            )

    def _begin_pause_locked(self):
        if self.args.stop_every_actions <= 0 or self.args.stop_time <= 0.0:
            return
        now = time.perf_counter()
        self._pause_started_at = now
        self._pause_until = now + self.args.stop_time
        self.driver.stop_base()
        self.logger.info(
            "Replay pause for %.3fs after %s actions."
            % (self.args.stop_time, self.args.stop_every_actions)
        )

    def _return_home(self, reason: str):
        self.logger.info("Returning arms home (%s)..." % reason)
        try:
            self.driver.go_home(duration_s=self.args.home_wait)
            self.logger.info("Both arms returned home.")
        except Exception as exc:
            self.logger.error("Failed to return home: %s" % exc)

    def _load_episode(self) -> ReplayEpisode:
        replay_path = self._resolve_replay_file()
        return ReplayEpisode(
            path=replay_path,
            demo_index=max(0, self.args.demo_index),
            fallback_hz=self.args.fallback_hz,
        )

    def _resolve_replay_file(self) -> Path:
        if self.args.replay_hdf5_path:
            path = Path(self.args.replay_hdf5_path).expanduser()
            if path.is_absolute():
                return path
            datasets_path = Path(self.args.datasets_dir).expanduser().resolve() / path
            if datasets_path.exists():
                return datasets_path
            return (Path.cwd() / path).resolve()

        datasets_dir = Path(self.args.datasets_dir).expanduser().resolve()
        episodes = _list_episode_files(datasets_dir)
        if not episodes:
            raise FileNotFoundError(
                "No replay file found under %s. Expected files like episode_0.hdf5."
                % datasets_dir
            )
        if self.args.latest:
            return episodes[-1][1]
        return _prompt_select_episode(episodes)


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
    replay_config = _section(config, "replay")

    parser = argparse.ArgumentParser(
        description="RBM OpenArm HDF5 replay (arms + optional base).",
        parents=[config_parser],
    )
    parser.add_argument(
        "--datasets-dir",
        default=str(_resolve_repo_path(paths_config.get("datasets_dir", "datasets"))),
    )
    parser.add_argument(
        "--replay-hdf5-path",
        default=replay_config.get(
            "dataset", replay_config.get("replay_hdf5_path", "")
        ),
        help="Explicit HDF5 path; skips interactive dataset selection when set.",
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        default=bool(replay_config.get("latest", False)),
        help="Use newest episode_*.hdf5 without prompting.",
    )
    parser.add_argument(
        "--demo-index", type=int, default=replay_config.get("demo_index", 0)
    )
    parser.add_argument(
        "--control-hz", type=float, default=replay_config.get("control_hz", 50.0)
    )
    parser.add_argument(
        "--fallback-hz", type=float, default=replay_config.get("fallback_hz", 30.0)
    )
    parser.add_argument(
        "--home-wait", type=float, default=replay_config.get("home_wait", 2.5)
    )
    parser.add_argument(
        "--skip-home",
        action=argparse.BooleanOptionalAction,
        default=replay_config.get("skip_home", False),
    )
    parser.add_argument("--speed", type=float, default=replay_config.get("speed", 1.0))
    parser.add_argument(
        "--stop-every-actions",
        type=int,
        default=replay_config.get("stop_every_actions", 0),
    )
    parser.add_argument(
        "--stop-time",
        type=float,
        default=replay_config.get("stop_time", 0.5),
    )
    parser.add_argument(
        "--transition-duration",
        type=float,
        default=replay_config.get("transition_duration", 1.0),
    )
    parser.add_argument(
        "--loop",
        action=argparse.BooleanOptionalAction,
        default=replay_config.get("loop", False),
    )
    parser.add_argument(
        "--home-after-finish",
        action=argparse.BooleanOptionalAction,
        default=replay_config.get("home_after_finish", True),
    )
    parser.add_argument(
        "--home-on-shutdown",
        action=argparse.BooleanOptionalAction,
        default=replay_config.get("home_on_shutdown", True),
    )
    parser.add_argument(
        "--replay-base",
        action=argparse.BooleanOptionalAction,
        default=replay_config.get("replay_base", True),
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=replay_config.get("max_linear_speed", 0.1),
        help="Clamp |cmd_vel.linear.*| during base replay (m/s).",
    )
    parser.add_argument(
        "--max-angular-speed",
        type=float,
        default=replay_config.get("max_angular_speed", 0.2),
        help="Clamp |cmd_vel.angular.*| during base replay (rad/s).",
    )
    parser.add_argument(
        "--left-arm-topic",
        default=topics_config.get(
            "left_arm_cmd", "/left_forward_position_controller/commands"
        ),
    )
    parser.add_argument(
        "--right-arm-topic",
        default=topics_config.get(
            "right_arm_cmd", "/right_forward_position_controller/commands"
        ),
    )
    parser.add_argument(
        "--cmd-vel-topic",
        default=topics_config.get("cmd_vel", "/cmd_vel"),
    )
    parser.add_argument(
        "--left-gripper-action",
        default=topics_config.get(
            "left_gripper_action", "left_gripper_controller/gripper_cmd"
        ),
    )
    parser.add_argument(
        "--right-gripper-action",
        default=topics_config.get(
            "right_gripper_action", "right_gripper_controller/gripper_cmd"
        ),
    )
    args = parser.parse_args(clean_argv[1:])
    args.config = str(_resolve_config_path(args.config))
    if args.control_hz <= 0.0:
        parser.error("--control-hz must be positive")
    if args.fallback_hz <= 0.0:
        parser.error("--fallback-hz must be positive")
    if args.speed <= 0.0:
        parser.error("--speed must be positive")
    if args.max_linear_speed < 0.0:
        parser.error("--max-linear-speed must be non-negative")
    if args.max_angular_speed < 0.0:
        parser.error("--max-angular-speed must be non-negative")
    if args.stop_every_actions < 0:
        parser.error("--stop-every-actions must be non-negative")
    if args.stop_time < 0.0:
        parser.error("--stop-time must be non-negative")
    if args.transition_duration < 0.0:
        parser.error("--transition-duration must be non-negative")
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=argv)
    driver = None
    runner = None
    try:
        driver = OpenArmReplayDriver(args)
        runner = ReplayRunner(args, driver)
        runner.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        Logger().error(str(exc))
        return 1
    finally:
        if runner is not None:
            runner.request_shutdown()
        if driver is not None:
            driver.stop_base()
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
