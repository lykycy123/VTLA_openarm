#!/usr/bin/env python3
"""Browse RBM OpenArm HDF5 episodes (images + joints + eef + base).

No ROS required. Needs: h5py, numpy, opencv-python.

  conda activate openarm
  python datasets/browse_hdf5.py
  python datasets/browse_hdf5.py episode_18.hdf5
  python datasets/browse_hdf5.py /path/to/episode_18.hdf5 --demo 0

Keys:
  SPACE        play / pause
  A / D        previous / next frame  (or Left / Right)
  W / S        skip ±30 frames
  N / P        next / previous episode
  , / .        slower / faster
  G            toggle joint overlay
  Q / ESC      quit
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import h5py
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
WINDOW_NAME = "RBM HDF5 browser"

DEFAULT_JOINT_NAMES = [
    "L1",
    "L2",
    "L3",
    "L4",
    "L5",
    "L6",
    "L7",
    "Lg",
    "R1",
    "R2",
    "R3",
    "R4",
    "R5",
    "R6",
    "R7",
    "Rg",
]


def _as_str(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def find_episodes(directory: Path) -> List[Path]:
    files = sorted(directory.glob("episode_*.hdf5"))
    files += sorted(
        p for p in directory.glob("*.hdf5") if p not in files and "tmp" not in p.name
    )
    return files


def decode_jpeg_row(padded_row: np.ndarray, length: int) -> Optional[np.ndarray]:
    if length <= 0:
        return None
    buf = np.frombuffer(padded_row[: int(length)].tobytes(), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def resize_fit(image: np.ndarray, width: int, height: int) -> np.ndarray:
    if image is None or image.size == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)
    h, w = image.shape[:2]
    scale = min(width / max(w, 1), height / max(h, 1))
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    x0 = (width - nw) // 2
    y0 = (height - nh) // 2
    canvas[y0 : y0 + nh, x0 : x0 + nw] = resized
    return canvas


def put_text(
    image: np.ndarray,
    text: str,
    org: Tuple[int, int],
    scale: float = 0.55,
    color=(240, 240, 240),
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


class EpisodeViewer:
    def __init__(self, path: Path, demo_index: int = 0):
        self.path = path
        self.demo_index = demo_index
        self.frame = 0
        self.playing = False
        self.show_joints = True
        self._load()

    def _load(self) -> None:
        with h5py.File(self.path, "r") as h5:
            demo_names = list(h5["data"].keys()) if "data" in h5 else []
            if not demo_names:
                raise RuntimeError(f"No data/* demos in {self.path}")
            if self.demo_index < 0 or self.demo_index >= len(demo_names):
                self.demo_index = 0
            demo_key = demo_names[self.demo_index]
            demo = h5[f"data/{demo_key}"]
            obs = demo["observations"]
            images = obs["images"]

            self.demo_key = demo_key
            self.timestamp = np.asarray(demo["timestamp"][:], dtype=np.float64)
            self.qpos = np.asarray(obs["qpos"][:], dtype=np.float64)
            self.qvel = np.asarray(obs["qvel"][:], dtype=np.float64) if "qvel" in obs else None
            self.eef = np.asarray(obs["eef"][:], dtype=np.float64) if "eef" in obs else None
            self.odom = np.asarray(obs["odom"][:], dtype=np.float64) if "odom" in obs else None
            self.cmd_vel = (
                np.asarray(obs["cmd_vel"][:], dtype=np.float64) if "cmd_vel" in obs else None
            )

            self.head = np.asarray(images["head"][:])
            self.head_len = np.asarray(images["head_lengths"][:], dtype=np.int32)
            self.left = np.asarray(images["left_wrist"][:])
            self.left_len = np.asarray(images["left_wrist_lengths"][:], dtype=np.int32)
            self.right = np.asarray(images["right_wrist"][:])
            self.right_len = np.asarray(images["right_wrist_lengths"][:], dtype=np.int32)

            if "meta" in h5 and "joint_names" in h5["meta"]:
                self.joint_names = [_as_str(n) for n in h5["meta/joint_names"][:]]
            else:
                self.joint_names = list(DEFAULT_JOINT_NAMES)

        self.n = int(self.timestamp.shape[0])
        if self.n <= 0:
            raise RuntimeError(f"Empty episode: {self.path}")
        dt = np.diff(self.timestamp) if self.n > 1 else np.array([1.0 / 30.0])
        self.hz = float(1.0 / max(float(np.median(dt)), 1e-3))
        self.duration = float(self.timestamp[-1] - self.timestamp[0]) if self.n > 1 else 0.0
        self.frame = min(self.frame, self.n - 1)

        qmin = self.qpos.min(axis=0)
        qmax = self.qpos.max(axis=0)
        span = np.maximum(qmax - qmin, 0.05)
        self.q_lo = qmin - 0.05 * span
        self.q_hi = qmax + 0.05 * span

    @property
    def t_rel(self) -> float:
        return float(self.timestamp[self.frame] - self.timestamp[0])

    def step(self, delta: int) -> None:
        self.frame = int(np.clip(self.frame + delta, 0, self.n - 1))
        if self.frame >= self.n - 1:
            self.playing = False

    def render(self, speed: float) -> np.ndarray:
        idx = self.frame
        head = decode_jpeg_row(self.head[idx], int(self.head_len[idx]))
        left = decode_jpeg_row(self.left[idx], int(self.left_len[idx]))
        right = decode_jpeg_row(self.right[idx], int(self.right_len[idx]))

        canvas_w, canvas_h = 1280, 900
        head_h = 420
        wrist_h = 300
        panel_h = canvas_h - head_h - wrist_h

        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[0:head_h, :, :] = resize_fit(head, canvas_w, head_h)
        put_text(canvas, "head", (16, 28), 0.7, (80, 220, 255), 2)

        half = canvas_w // 2
        canvas[head_h : head_h + wrist_h, 0:half, :] = resize_fit(left, half, wrist_h)
        canvas[head_h : head_h + wrist_h, half:canvas_w, :] = resize_fit(right, half, wrist_h)
        put_text(canvas, "left_wrist", (16, head_h + 28), 0.65, (80, 255, 160), 2)
        put_text(canvas, "right_wrist", (half + 16, head_h + 28), 0.65, (80, 255, 160), 2)

        panel = canvas[head_h + wrist_h : canvas_h, :, :]
        panel[:] = (28, 28, 28)
        self._draw_panel(panel, speed)

        status = "PLAY" if self.playing else "PAUSE"
        put_text(
            canvas,
            f"{self.path.name}  {self.demo_key}  [{status}]  "
            f"frame {idx + 1}/{self.n}  t={self.t_rel:.2f}s / {self.duration:.2f}s  "
            f"~{self.hz:.1f} Hz  x{speed:.2f}",
            (16, canvas_h - 16),
            0.55,
            (255, 255, 255),
            1,
        )
        return canvas

    def _draw_panel(self, panel: np.ndarray, speed: float) -> None:
        q = self.qpos[self.frame]
        names = self.joint_names[: len(q)]
        h, w = panel.shape[:2]
        y = 22
        put_text(panel, "SPACE play/pause  A/D frame  W/S ±30  N/P file  ,/. speed  G joints  Q quit", (12, y), 0.45)
        y += 26

        eef_txt = ""
        if self.eef is not None and self.eef.shape[1] >= 14:
            e = self.eef[self.frame]
            eef_txt = (
                f"  eef L xyz=({e[0]:.3f},{e[1]:.3f},{e[2]:.3f})"
                f"  R xyz=({e[7]:.3f},{e[8]:.3f},{e[9]:.3f})"
            )
        cmd_txt = ""
        if self.cmd_vel is not None and self.cmd_vel.shape[1] >= 1:
            c = self.cmd_vel[self.frame]
            cmd_txt = f"  cmd_vel=({c[0]:.3f}, ang={c[5] if c.size > 5 else 0.0:.3f})"
        odom_txt = ""
        if self.odom is not None and self.odom.shape[1] >= 3:
            o = self.odom[self.frame]
            odom_txt = f"  odom xy=({o[0]:.3f},{o[1]:.3f})"
        put_text(panel, eef_txt.strip() + cmd_txt + odom_txt, (12, y), 0.45, (200, 220, 255))
        y += 18

        if not self.show_joints:
            put_text(panel, "joint overlay off (press G)", (12, y + 16), 0.5)
            return

        n_bars = min(16, len(q))
        left_w = 70
        bar_area = w - 24
        gap = 6
        bar_w = max(8, (bar_area - left_w) // max(n_bars, 1) - gap)
        base_y = h - 28
        bar_h = h - y - 40
        for i in range(n_bars):
            x = 12 + i * (bar_w + gap)
            lo = float(self.q_lo[i]) if i < len(self.q_lo) else -3.2
            hi = float(self.q_hi[i]) if i < len(self.q_hi) else 3.2
            frac = (float(q[i]) - lo) / max(hi - lo, 1e-6)
            frac = float(np.clip(frac, 0.0, 1.0))
            bh = int(bar_h * frac)
            color = (80, 180, 255) if i < 8 else (80, 255, 180)
            cv2.rectangle(panel, (x, base_y - bar_h), (x + bar_w, base_y), (50, 50, 50), 1)
            cv2.rectangle(
                panel,
                (x, base_y - bh),
                (x + bar_w, base_y),
                color,
                -1,
            )
            label = names[i].replace("openarm_", "").replace("finger_joint1", "grip")
            if len(label) > 8:
                label = label[-8:]
            put_text(panel, label, (x, base_y + 16), 0.32, (210, 210, 210))
            put_text(panel, f"{q[i]:+.2f}", (x, base_y - bar_h - 4), 0.32, color)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Browse OpenArm HDF5 episodes")
    parser.add_argument(
        "hdf5",
        nargs="?",
        default="",
        help="HDF5 file or directory. Default: this script's directory.",
    )
    parser.add_argument("--demo", type=int, default=0, help="demo index inside the file")
    parser.add_argument("--start", type=int, default=0, help="start frame")
    return parser.parse_args()


def resolve_files(arg: str) -> Tuple[List[Path], int]:
    if not arg:
        files = find_episodes(SCRIPT_DIR)
        return files, max(0, len(files) - 1)
    path = Path(arg).expanduser()
    if not path.is_absolute():
        cand = (Path.cwd() / path).resolve()
        path = cand if cand.exists() else (SCRIPT_DIR / path).resolve()
    if path.is_dir():
        files = find_episodes(path)
        return files, 0
    if path.is_file():
        files = find_episodes(path.parent) or [path]
        idx = files.index(path) if path in files else 0
        if path not in files:
            files = [path]
            idx = 0
        return files, idx
    raise FileNotFoundError(arg)


def main() -> int:
    args = parse_args()
    try:
        files, file_idx = resolve_files(args.hdf5)
    except FileNotFoundError as exc:
        print(f"File not found: {exc}", file=sys.stderr)
        return 1
    if not files:
        print(f"No episode_*.hdf5 under {SCRIPT_DIR}", file=sys.stderr)
        return 1

    viewer = EpisodeViewer(files[file_idx], demo_index=args.demo)
    viewer.frame = int(np.clip(args.start, 0, viewer.n - 1))
    speed = 1.0
    print(f"Loaded {viewer.path}  frames={viewer.n}  duration={viewer.duration:.2f}s")
    print("SPACE play/pause  A/D frame  N/P file  Q quit")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 900)

    last_tick = cv2.getTickCount()
    while True:
        canvas = viewer.render(speed)
        cv2.imshow(WINDOW_NAME, canvas)
        delay_ms = max(1, int(1000.0 / max(viewer.hz * speed, 1.0)))
        key = cv2.waitKey(delay_ms if viewer.playing else 30) & 0xFF

        if key in (27, ord("q"), ord("Q")):
            break
        if key in (ord(" "),):
            viewer.playing = not viewer.playing
        elif key in (ord("d"), ord("D"), 83):  # Right in some backends
            viewer.playing = False
            viewer.step(1)
        elif key in (ord("a"), ord("A"), 81):
            viewer.playing = False
            viewer.step(-1)
        elif key in (ord("w"), ord("W")):
            viewer.playing = False
            viewer.step(30)
        elif key in (ord("s"), ord("S")):
            viewer.playing = False
            viewer.step(-30)
        elif key in (ord("n"), ord("N")):
            file_idx = (file_idx + 1) % len(files)
            viewer = EpisodeViewer(files[file_idx], demo_index=args.demo)
            print(f"Loaded {viewer.path}  frames={viewer.n}")
        elif key in (ord("p"), ord("P")):
            file_idx = (file_idx - 1) % len(files)
            viewer = EpisodeViewer(files[file_idx], demo_index=args.demo)
            print(f"Loaded {viewer.path}  frames={viewer.n}")
        elif key in (ord("."),):
            speed = min(4.0, speed * 1.25)
        elif key in (ord(","),):
            speed = max(0.25, speed / 1.25)
        elif key in (ord("g"), ord("G")):
            viewer.show_joints = not viewer.show_joints

        if viewer.playing:
            now = cv2.getTickCount()
            elapsed = (now - last_tick) / cv2.getTickFrequency()
            step = max(1, int(round(elapsed * viewer.hz * speed)))
            if step > 0:
                viewer.step(step)
                last_tick = now
        else:
            last_tick = cv2.getTickCount()

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
