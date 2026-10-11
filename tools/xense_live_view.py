"""Display every available Xense output in one live dashboard."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
from xensesdk import Sensor

TITLE = "Xense tactile live view"
TILES = ("Raw", "Rectify", "Difference", "AugDifference", "Depth",
         "FineDepth", "CoarseDepth", "Force", "ForceNorm", "Mesh3DFlow",
         "Marker2D", "Flow", "ImgFloat", "ImgMarkerFree",
         "ImgMarkerEnhance", "ImgObjEnhance")
TW, TH = 280, 215
BG = (24, 27, 32)


def text(image, line, xy, scale=.45, color=(225, 230, 235)):
    cv2.putText(image, line, xy, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


def summary(value):
    if value is None:
        return {"status": "missing"}
    a = np.asarray(value)
    if not a.size or a.dtype.kind not in "fibu" or not np.isfinite(a).all():
        return {"status": "invalid", "shape": list(a.shape)}
    info = {"status": "ok", "shape": list(a.shape),
            "min": float(a.min()), "max": float(a.max())}
    if a.size <= 6:
        info["values"] = a.tolist()
    return info


def heatmap(value, limit):
    return cv2.applyColorMap(
        np.uint8(np.clip(value / limit, 0, 1) * 255), cv2.COLORMAP_TURBO)


def tile(name, data, stats, args):
    panel = np.full((TH, TW, 3), BG, np.uint8)
    text(panel, name, (8, 19), .52)
    if stats.get("status") != "ok":
        text(panel, stats.get("status", "unavailable"), (10, 100))
        return panel
    a = np.asarray(data[name])
    if name in ("Depth", "FineDepth", "CoarseDepth"):
        image = heatmap(a, args.depth_range_mm)
    elif name in ("Force", "ForceNorm", "Mesh3DFlow"):
        limit = args.depth_range_mm if name == "Mesh3DFlow" else args.force_range
        image = heatmap(np.linalg.norm(a, axis=-1), limit)
    elif name in ("Marker2D", "Flow"):
        image = np.zeros((700, 400, 3), np.uint8)
        if name == "Marker2D":
            if summary(data.get("Rectify"))["status"] == "ok":
                image = data["Rectify"].copy()
            initial = data.get("Marker2DInit")
            for i, point in enumerate(a.reshape(-1, 2)):
                end = tuple(np.rint(point).astype(int))
                cv2.circle(image, end, 3, (0, 255, 255), -1)
                if initial is not None and np.shape(initial) == a.shape:
                    start = tuple(np.rint(initial.reshape(-1, 2)[i]).astype(int))
                    cv2.arrowedLine(image, start, end, (0, 255, 0), 1)
        else:
            h, w = a.shape[:2]
            for y in range(0, h, 4):
                for x in range(0, w, 4):
                    start = (int((x + .5) * 400 / w), int((y + .5) * 700 / h))
                    end = (int(start[0] + a[y, x, 0] * 10),
                           int(start[1] + a[y, x, 1] * 10))
                    cv2.circle(image, start, 2, (80, 120, 120), -1)
                    cv2.arrowedLine(image, start, end, (0, 255, 255), 1)
    else:
        image = a if a.dtype == np.uint8 else np.uint8(np.clip(a, 0, 1) * 255)
        if image.ndim == 2 or image.shape[-1] == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    scale = min(TW / image.shape[1], (TH - 49) / image.shape[0])
    image = cv2.resize(image, (max(1, int(image.shape[1] * scale)),
                               max(1, int(image.shape[0] * scale))))
    y, x = 27, (TW - image.shape[1]) // 2
    panel[y:y + image.shape[0], x:x + image.shape[1]] = image
    text(panel, f"min {stats['min']:.3g}  max {stats['max']:.3g}", (8, TH - 8), .4)
    return panel


def dashboard(data, stats, names, history, args, frame, fps):
    canvas = np.full((940, 1800, 3), BG, np.uint8)
    text(canvas, f"Xense {args.serial} | {fps:.1f} FPS | Frame {frame} | Q / Esc: exit",
         (15, 28), .7)
    text(canvas, f"Depth: fixed 0..{args.depth_range_mm:g} mm | Force-map magnitude: "
         f"0..{args.force_range:g} SDK units | Blue=low, red=high", (15, 53), .5)
    for index, name in enumerate(TILES):
        y, x = 65 + index // 4 * TH, index % 4 * TW
        canvas[y:y + TH, x:x + TW] = tile(
            name, data, stats.get(name, {"status": "unavailable"}), args)
    x = 1135
    text(canvas, "Six-axis force / torque (SDK values; units unverified)", (x, 86), .51)
    for row, name in enumerate(("ForceResultant", "Force6DTuned")):
        text(canvas, name, (x, 110 + row * 60), .46)
        a = data.get(name)
        if summary(a)["status"] == "ok" and np.size(a) == 6:
            labels = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
            values = np.asarray(a).reshape(-1)
            for col in range(3):
                text(canvas, f"{labels[col]}: {values[col]:+.6g}",
                     (x + col * 216, 132 + row * 60), .46)
                text(canvas, f"{labels[col+3]}: {values[col+3]:+.6g}",
                     (x + col * 216, 150 + row * 60), .46)
        else:
            text(canvas, "Unavailable", (x, 132 + row * 60))
    chart_y, chart_h, chart_w = 217, 110, 630
    cv2.rectangle(canvas, (x, chart_y), (x + chart_w, chart_y + chart_h), (70, 75, 80), 1)
    cv2.line(canvas, (x, chart_y + chart_h // 2),
             (x + chart_w, chart_y + chart_h // 2), (65, 65, 65), 1)
    colors = ((100, 150, 255), (100, 255, 150), (255, 180, 100))
    if len(history) > 1:
        points = np.asarray(history)
        for axis, color in enumerate(colors):
            yy = chart_y + chart_h / 2 - np.clip(
                points[:, axis] / args.trace_range, -1, 1) * (chart_h / 2 - 2)
            xx = np.linspace(x, x + chart_w - 1, len(yy))
            cv2.polylines(canvas, [np.column_stack((xx, yy)).astype(np.int32)],
                          False, color, 1)
    text(canvas, f"Fx / Fy / Fz | +/-{args.trace_range:g} SDK units | last 300 frames",
         (x, 348), .46)
    text(canvas, "All outputs: shape, current min..max / timestamp", (x, 375), .49)
    for index, name in enumerate(names):
        info = stats.get(name, {"status": "unavailable"})
        if info["status"] == "ok":
            shape = "x".join(str(i) for i in info["shape"]) or "scalar"
            values = f"{info['min']:.5g} .. {info['max']:.5g}"
            if not info["shape"]:
                values = f"{info['min']:.6f}"
            line = f"{name:18s} {shape:11s} {values}"
        else:
            line = f"{name:18s} {info['status']}"
        text(canvas, line, (x, 398 + index * 17), .41,
             (225, 230, 235) if info["status"] == "ok" else (90, 170, 245))
    text(canvas, "Touch gently to compare image, depth and force. No automatic recalibration.",
         (15, 935), .48)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True, help="USB sensor serial number")
    parser.add_argument("--depth-range-mm", type=float, default=.2)
    parser.add_argument("--force-range", type=float, default=.02,
                        help="Fixed force-map range in SDK units")
    parser.add_argument("--trace-range", type=float, default=1,
                        help="Fixed +/- range for force trace in SDK units")
    parser.add_argument("--log", type=Path, default=Path("/tmp/vtla_xense_dashboard.jsonl"))
    args = parser.parse_args()
    for name in ("depth_range_mm", "force_range", "trace_range"):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"{name} must be finite and positive")
    devices = Sensor.scanSerialNumber()
    if args.serial not in devices:
        parser.error(f"Sensor {args.serial!r} absent; available devices: {devices}")
    sensor = Sensor.create(args.serial)
    try:
        with args.log.open("a", buffering=1) as log:
            def record(event, **fields):
                log.write(json.dumps({"event": event, "time": time.time(),
                                      "serial": args.serial, **fields}) + "\n")

            names = list(Sensor.OutputType.__members__)
            supported, unavailable = {}, {}
            for name in names:
                try:
                    value = sensor.selectSensorInfo(Sensor.OutputType[name])
                    info = summary(value)
                    if info["status"] == "ok":
                        supported[name] = Sensor.OutputType[name]
                    else:
                        unavailable[name] = info["status"]
                    record("probe", output=name, **info)
                except (KeyError, ValueError, RuntimeError) as error:
                    unavailable[name] = str(error)
                    record("probe", output=name, status="unavailable", error=str(error))
            for name in ("Rectify", "Depth"):
                if name not in supported:
                    raise RuntimeError(f"Required output unavailable: {name}")
            record("start", supported=list(supported), unavailable=unavailable,
                   depth_range_mm=args.depth_range_mm, force_range=args.force_range,
                   trace_range=args.trace_range)
            cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(TITLE, 1800, 940)
            history = deque(maxlen=300)
            frames, started = 0, time.monotonic()
            while True:
                requested = time.monotonic()
                outputs = sensor.selectSensorInfo(*supported.values())
                if len(outputs) != len(supported):
                    raise RuntimeError("SDK output count differs from request")
                data = dict(zip(supported, outputs))
                stats = {name: summary(value) for name, value in data.items()}
                for name in unavailable:
                    stats[name] = {"status": "unavailable"}
                for name in ("Rectify", "Depth"):
                    if stats[name]["status"] != "ok":
                        raise RuntimeError(f"Invalid live output: {name}")
                result = data.get("ForceResultant")
                if summary(result)["status"] == "ok" and np.size(result) == 6:
                    history.append(np.asarray(result).reshape(-1)[:3].copy())
                frames += 1
                fps = frames / (time.monotonic() - started)
                read_ms = (time.monotonic() - requested) * 1000
                canvas = dashboard(data, stats, names, history, args, frames, fps)
                cv2.imshow(TITLE, canvas)
                key = cv2.waitKey(1) & 0xFF
                record("frame", frame=frames, read_ms=read_ms, fps=fps, outputs=stats)
                if key in (ord("q"), ord("Q"), 27):
                    record("stop", reason="key", frames=frames)
                    break
                try:
                    visible = cv2.getWindowProperty(TITLE, cv2.WND_PROP_VISIBLE)
                except cv2.error:
                    visible = 0
                if visible < 1:
                    record("stop", reason="window_closed", frames=frames)
                    break
    finally:
        cv2.destroyAllWindows()
        sensor.release()


if __name__ == "__main__":
    main()
