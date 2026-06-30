#!/usr/bin/env python3
"""Plot the notable data from a truckbot rosbag2 (.mcap) file with Plotly.

Reads the bag with the pure-Python ``mcap`` reader (no ROS environment
required) and writes a single self-contained HTML report containing:

  * 2D trajectory map  - odometry path vs. particle-filter estimate
  * Pose vs. time      - x / y / yaw for odom vs. heaviest particle
  * Velocities         - measured (odom) vs. commanded (cmd_vel)
  * IMU                - angular velocity and linear acceleration
  * Wheel encoders     - all four wheels
  * LiDAR              - range statistics over time
  * Landmark slider    - heaviest-particle landmarks animated over time

Usage:
    python3 viz/plot_bag.py [BAG_DIR_OR_MCAP] [-o report.html] [--show]

If no bag is given it defaults to the bag referenced when this module was
written.  A bag directory or a direct ``*.mcap`` path both work.
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import webbrowser
from collections import defaultdict

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError as exc:  # pragma: no cover - dependency hint
    raise SystemExit(
        "Missing dependencies. Install with:\n"
        "    pip install mcap mcap-ros2-support\n"
        f"(import error: {exc})"
    )

DEFAULT_BAG = "/home/emory/repos/bags/rosbag2_2026_06_16-21_02_50"

# Topics we actually pull data from.
ENCODER_TOPICS = [
    "/encoder/left_front",
    "/encoder/right_front",
    "/encoder/left_rear",
    "/encoder/right_rear",
]


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) from a quaternion, in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def resolve_mcap(path: str) -> str:
    """Accept either a bag directory or a direct .mcap file."""
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not files:
            raise SystemExit(f"No .mcap file found in {path!r}")
        return files[0]
    if not os.path.isfile(path):
        raise SystemExit(f"Bag not found: {path!r}")
    return path


def load_bag(mcap_path: str) -> dict:
    """Read the bag once and collect arrays for the topics we plot.

    All time stamps use the bag receive time (``log_time_ns``) so every
    series shares one consistent clock, normalised to seconds from start.
    """
    odom = defaultdict(list)          # t, x, y, yaw, v, w
    pf = defaultdict(list)            # heaviest particle pose: t, x, y, yaw
    cmd = defaultdict(list)           # commanded: t, v, w
    imu = defaultdict(list)           # t, wx, wy, wz, ax, ay, az
    enc = {topic: defaultdict(list) for topic in ENCODER_TOPICS}
    scan = defaultdict(list)          # t, rmin, rmean, rmax, valid
    landmarks = []                    # per-frame: {t, x[], y[]} for the slider

    for m in read_ros2_messages(mcap_path):
        topic = m.channel.topic
        t = m.log_time_ns * 1e-9
        msg = m.ros_msg

        if topic == "/odom":
            p = msg.pose.pose
            tw = msg.twist.twist
            odom["t"].append(t)
            odom["x"].append(p.position.x)
            odom["y"].append(p.position.y)
            odom["yaw"].append(
                quat_to_yaw(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
            )
            odom["v"].append(tw.linear.x)
            odom["w"].append(tw.angular.z)

        elif topic == "/heaviest_particle_pose":
            p = msg.pose
            pf["t"].append(t)
            pf["x"].append(p.position.x)
            pf["y"].append(p.position.y)
            pf["yaw"].append(
                quat_to_yaw(p.orientation.x, p.orientation.y,
                            p.orientation.z, p.orientation.w)
            )

        elif topic == "/cmd_vel":
            cmd["t"].append(t)
            cmd["v"].append(msg.linear.x)
            cmd["w"].append(msg.angular.z)

        elif topic == "/imu":
            imu["t"].append(t)
            imu["wx"].append(msg.angular_velocity.x)
            imu["wy"].append(msg.angular_velocity.y)
            imu["wz"].append(msg.angular_velocity.z)
            imu["ax"].append(msg.linear_acceleration.x)
            imu["ay"].append(msg.linear_acceleration.y)
            imu["az"].append(msg.linear_acceleration.z)

        elif topic in enc:
            enc[topic]["t"].append(t)
            enc[topic]["v"].append(msg.data)

        elif topic == "/scan":
            r = np.asarray(msg.ranges, dtype=float)
            valid = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
            scan["t"].append(t)
            scan["rmin"].append(float(np.min(r[valid])) if valid.any() else np.nan)
            scan["rmean"].append(float(np.mean(r[valid])) if valid.any() else np.nan)
            scan["rmax"].append(float(np.max(r[valid])) if valid.any() else np.nan)
            scan["valid"].append(int(valid.sum()))

        elif topic == "/heaviest_particle_landmarks":
            xs = np.array([mk.pose.position.x for mk in msg.markers], dtype=float)
            ys = np.array([mk.pose.position.y for mk in msg.markers], dtype=float)
            landmarks.append({"t": t, "x": xs, "y": ys})

    # Normalise all timelines to seconds from the earliest stamp.
    starts = [s["t"][0] for s in (odom, pf, cmd, imu, scan) if s.get("t")]
    starts += [e["t"][0] for e in enc.values() if e.get("t")]
    starts += [landmarks[0]["t"]] if landmarks else []
    t0 = min(starts) if starts else 0.0

    def to_np(d: dict) -> dict:
        out = {k: np.asarray(v, dtype=float) for k, v in d.items()}
        if "t" in out:
            out["t"] = out["t"] - t0
        return out

    for fr in landmarks:
        fr["t"] -= t0

    return {
        "odom": to_np(odom),
        "pf": to_np(pf),
        "cmd": to_np(cmd),
        "imu": to_np(imu),
        "enc": {k: to_np(v) for k, v in enc.items()},
        "scan": to_np(scan),
        "landmarks": landmarks,
    }


# --------------------------------------------------------------------------- #
# Figure builders
# --------------------------------------------------------------------------- #
# Right-column time-series panels, top to bottom. Each is one subplot row whose
# x-axis (time) is shared/linked with every other row in the column.
DYNAMICS_ROWS = [
    "x [m]",
    "y [m]",
    "yaw [rad]",
    "lin. vel [m/s]",
    "ang. vel [rad/s]",
    "IMU ang. vel [rad/s]",
    "IMU lin. acc [m/s^2]",
    "encoders",
    "LiDAR range [m]",
]


def fig_dashboard(data: dict) -> go.Figure:
    """Trajectory map (left) beside the stacked, x-linked dynamics (right)."""
    odom, pf, cmd = data["odom"], data["pf"], data["cmd"]
    imu, enc, scan = data["imu"], data["enc"], data["scan"]
    nrows = len(DYNAMICS_ROWS)

    # Left column: a single trajectory cell spanning every row. Right column:
    # one panel per dynamics row. shared_xaxes links the right column's x-axes.
    specs = [[{"type": "xy", "rowspan": nrows}, {"type": "xy"}]]
    specs += [[None, {"type": "xy"}] for _ in range(nrows - 1)]
    fig = make_subplots(
        rows=nrows, cols=2,
        column_widths=[0.5, 0.5],
        shared_xaxes=True,
        horizontal_spacing=0.08,
        vertical_spacing=0.012,
        specs=specs,
        subplot_titles=["2D Trajectory: odom vs. PF"] + DYNAMICS_ROWS,
    )

    # --- left: trajectory ---
    if len(odom.get("x", [])):
        fig.add_trace(go.Scatter(x=odom["x"], y=odom["y"], mode="lines",
                                 name="odom", legendgroup="odom",
                                 line=dict(color="#1f77b4")), row=1, col=1)
        fig.add_trace(go.Scatter(x=[odom["x"][0]], y=[odom["y"][0]], mode="markers",
                                 name="start", marker=dict(color="green", size=10),
                                 showlegend=True), row=1, col=1)
    if len(pf.get("x", [])):
        fig.add_trace(go.Scatter(x=pf["x"], y=pf["y"], mode="lines",
                                 name="PF", legendgroup="pf",
                                 line=dict(color="#d62728")), row=1, col=1)
    fig.update_xaxes(title_text="x [m]", row=1, col=1)
    fig.update_yaxes(title_text="y [m]", scaleanchor="x", scaleratio=1,
                     row=1, col=1)

    # --- right: dynamics, one row each ---
    def add(row, x, y, **kw):
        fig.add_trace(go.Scatter(x=x, y=y, **kw), row=row, col=2)

    have_odom = len(odom.get("t", []))
    have_cmd = len(cmd.get("t", []))
    if have_odom:
        add(1, odom["t"], odom["x"], name="odom", legendgroup="odom",
            line=dict(color="#1f77b4"), showlegend=False)
        add(2, odom["t"], odom["y"], legendgroup="odom",
            line=dict(color="#1f77b4"), showlegend=False)
        add(3, odom["t"], odom["yaw"], legendgroup="odom",
            line=dict(color="#1f77b4"), showlegend=False)
        add(4, odom["t"], odom["v"], legendgroup="odom",
            line=dict(color="#1f77b4"), showlegend=False)
        add(5, odom["t"], odom["w"], legendgroup="odom",
            line=dict(color="#1f77b4"), showlegend=False)
    if len(pf.get("t", [])):
        add(1, pf["t"], pf["x"], name="PF", legendgroup="pf",
            line=dict(color="#d62728"), showlegend=False)
        add(2, pf["t"], pf["y"], legendgroup="pf",
            line=dict(color="#d62728"), showlegend=False)
        add(3, pf["t"], pf["yaw"], legendgroup="pf",
            line=dict(color="#d62728"), showlegend=False)
    if have_cmd:
        add(4, cmd["t"], cmd["v"], name="cmd_vel", legendgroup="cmd",
            line=dict(color="#ff7f0e"))
        add(5, cmd["t"], cmd["w"], legendgroup="cmd",
            line=dict(color="#ff7f0e"), showlegend=False)
    if len(imu.get("t", [])):
        for axis, color in (("wx", "#1f77b4"), ("wy", "#2ca02c"), ("wz", "#9467bd")):
            add(6, imu["t"], imu[axis], name=axis, line=dict(color=color))
        for axis, color in (("ax", "#1f77b4"), ("ay", "#2ca02c"), ("az", "#9467bd")):
            add(7, imu["t"], imu[axis], name=axis, line=dict(color=color))
    enc_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    for (topic, series), color in zip(enc.items(), enc_colors):
        if len(series.get("t", [])):
            add(8, series["t"], series["v"],
                name=topic.replace("/encoder/", ""), line=dict(color=color))
    if len(scan.get("t", [])):
        for key, color in (("rmin", "#2ca02c"), ("rmean", "#1f77b4"),
                           ("rmax", "#d62728")):
            add(9, scan["t"], scan[key], name=key, line=dict(color=color))

    fig.update_xaxes(title_text="time [s]", row=nrows, col=2)
    fig.update_layout(
        title="truckbot dynamics",
        height=1300,
        legend=dict(orientation="v", yanchor="top", y=1.0, xanchor="left", x=1.02),
        margin=dict(t=80, r=140),
    )
    return fig


def fig_landmarks_slider(data: dict) -> go.Figure:
    """Heaviest-particle landmark map with a time slider / play button.

    Each animation frame is one ``/heaviest_particle_landmarks`` message; the
    slider scrubs through the run while the axes stay fixed so the landmark
    layout can be compared frame to frame.
    """
    frames_data = data["landmarks"]
    pf = data["pf"]
    fig = go.Figure()
    if not frames_data:
        fig.update_layout(title="Heaviest-particle landmarks (no data)")
        return fig

    # Pose for each landmark frame, matched by nearest time stamp.
    pf_t = pf.get("t")
    has_pose = pf_t is not None and len(pf_t)

    def pose_at(t):
        if not has_pose:
            return None
        i = int(np.argmin(np.abs(pf_t - t)))
        return pf["x"][i], pf["y"][i], pf["yaw"][i]

    # Fixed, padded axis range across all frames so nothing jumps around.
    all_x = list(np.concatenate([f["x"] for f in frames_data if len(f["x"])]))
    all_y = list(np.concatenate([f["y"] for f in frames_data if len(f["y"])]))
    if has_pose:
        all_x += list(pf["x"])
        all_y += list(pf["y"])
    all_x, all_y = np.asarray(all_x), np.asarray(all_y)
    span = max(all_x.max() - all_x.min(), all_y.max() - all_y.min()) or 1.0
    pad_x = 0.05 * (all_x.max() - all_x.min() or 1.0)
    pad_y = 0.05 * (all_y.max() - all_y.min() or 1.0)
    xr = [all_x.min() - pad_x, all_x.max() + pad_x]
    yr = [all_y.min() - pad_y, all_y.max() + pad_y]
    head_len = 0.06 * span  # length of the heading whisker

    def landmark_trace(fr):
        return go.Scatter(
            x=fr["x"], y=fr["y"], mode="markers",
            marker=dict(size=9, color="#2ca02c", line=dict(width=1, color="#145214")),
            name="landmarks")

    def pose_trace(fr):
        p = pose_at(fr["t"])
        if p is None:
            return go.Scatter(x=[], y=[], mode="markers", name="heaviest pose")
        px, py, yaw = p
        return go.Scatter(
            x=[px, px + head_len * math.cos(yaw)],
            y=[py, py + head_len * math.sin(yaw)],
            mode="lines+markers",
            line=dict(color="#d62728", width=3),
            marker=dict(size=[14, 0], color="#d62728", symbol="circle"),
            name="heaviest pose")

    def frame_traces(fr):
        traces = [landmark_trace(fr)]
        if has_pose:
            traces.append(pose_trace(fr))
        return traces

    for tr in frame_traces(frames_data[0]):
        fig.add_trace(tr)
    fig.frames = [
        go.Frame(data=frame_traces(fr), name=f"{i}")
        for i, fr in enumerate(frames_data)
    ]

    steps = [
        dict(method="animate", label=f"{fr['t']:.1f}",
             args=[[f"{i}"], dict(mode="immediate",
                                  frame=dict(duration=0, redraw=True),
                                  transition=dict(duration=0))])
        for i, fr in enumerate(frames_data)
    ]
    play = dict(
        type="buttons", x=0.0, y=1.12, xanchor="left", yanchor="top",
        buttons=[
            dict(label="▶ Play", method="animate",
                 args=[None, dict(fromcurrent=True,
                                  frame=dict(duration=120, redraw=True),
                                  transition=dict(duration=0))]),
            dict(label="❚❚ Pause", method="animate",
                 args=[[None], dict(mode="immediate",
                                    frame=dict(duration=0, redraw=True))]),
        ])

    fig.update_layout(
        title="Heaviest-particle landmarks + pose over time",
        height=650,
        xaxis=dict(title="x [m]", range=xr),
        yaxis=dict(title="y [m]", range=yr, scaleanchor="x", scaleratio=1),
        updatemenus=[play],
        sliders=[dict(active=0, x=0.0, len=1.0, y=0.0, xanchor="left",
                      yanchor="top", pad=dict(t=40),
                      currentvalue=dict(prefix="t = ", suffix=" s"),
                      steps=steps)],
    )
    return fig


def build_figures(data: dict) -> list[go.Figure]:
    return [
        fig_dashboard(data),
        fig_landmarks_slider(data),
    ]


def write_report(figs: list[go.Figure], out_path: str, bag_name: str) -> None:
    parts = [
        "<html><head><meta charset='utf-8'>",
        f"<title>truckbot bag report - {bag_name}</title>",
        "<style>body{font-family:sans-serif;margin:0;background:#fafafa}"
        "h1{padding:16px 24px;margin:0;background:#222;color:#fff}</style></head><body>",
        f"<h1>truckbot bag report &mdash; {bag_name}</h1>",
    ]
    for i, fig in enumerate(figs):
        # Only embed plotly.js once (first figure) via CDN.
        parts.append(fig.to_html(
            full_html=False,
            include_plotlyjs="cdn" if i == 0 else False))
    parts.append("</body></html>")
    with open(out_path, "w") as f:
        f.write("\n".join(parts))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?", default=DEFAULT_BAG,
                    help="bag directory or .mcap file")
    ap.add_argument("-o", "--out", default=None,
                    help="output HTML path (default: <bag>_report.html in CWD)")
    ap.add_argument("--no-show", dest="show", action="store_false",
                    help="do not open the report in a browser (opens by default)")
    args = ap.parse_args()

    mcap_path = resolve_mcap(args.bag)
    bag_name = os.path.basename(os.path.dirname(mcap_path)) or os.path.basename(mcap_path)
    out_path = args.out or f"{bag_name}_report.html"

    print(f"Reading {mcap_path} ...")
    data = load_bag(mcap_path)

    counts = {
        "odom": len(data["odom"].get("t", [])),
        "pf": len(data["pf"].get("t", [])),
        "cmd_vel": len(data["cmd"].get("t", [])),
        "imu": len(data["imu"].get("t", [])),
        "scan": len(data["scan"].get("t", [])),
    }
    print("Messages plotted:", ", ".join(f"{k}={v}" for k, v in counts.items()))

    figs = build_figures(data)
    write_report(figs, out_path, bag_name)
    print(f"Wrote {out_path}")
    if args.show:
        webbrowser.open(f"file://{os.path.abspath(out_path)}")


if __name__ == "__main__":
    main()
