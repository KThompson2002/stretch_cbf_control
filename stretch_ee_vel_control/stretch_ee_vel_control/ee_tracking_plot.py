"""
Live plot of expected vs actual end-effector position. Runs anywhere on the ROS graph
(e.g. a laptop) and only uses topics/TF, so it works the same for sim and hardware.

  actual   = TF  odom_frame -> ee_frame   (odom->base_link from ee_velocity,
                                            base_link->ee from robot_state_publisher)
  expected = open-loop integral of the twist on cmd_topic, rotated into odom using the
             twist's header.frame_id, re-seeded from actual at each motion onset

Mirrors ee_velocity's behaviour: a twist older than cmd_timeout counts as zero.
On close, writes a CSV and PNG to log_dir.

ros2 run stretch_ee_vel_control ee_tracking_plot --ros-args -p window_s:=30.0
"""
import csv
import os
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

ACTUAL_COLOR = "#2a78d6"
EXPECTED_COLOR = "#eb6834"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class EETrackingPlot(Node):
    def __init__(self):
        super().__init__("ee_tracking_plot")
        self.declare_parameter("cmd_topic", "/ee_velocity/ee_cmd_vel")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("ee_frame", "link_grasp_center")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("reset_on_motion", True)
        self.declare_parameter("window_s", 30.0)
        self.declare_parameter("log_dir", "~/.ros/ee_tracking")  # "" disables saving
        self.declare_parameter("plane", "xy")  # right panel axes (horizontal, vertical): xy, yz, ...

        p = lambda n: self.get_parameter(n).value
        self.plane = p("plane").lower()
        if len(self.plane) != 2 or len(set(self.plane)) != 2 or not set(self.plane) <= set("xyz"):
            raise ValueError(f"plane must be two distinct axes from 'xyz', got '{self.plane}'")
        self.odom_frame, self.ee_frame = p("odom_frame"), p("ee_frame")
        self.timeout = p("cmd_timeout")
        self.reset_on_motion = p("reset_on_motion")
        self.window = p("window_s")
        self.log_dir = p("log_dir")

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_subscription(TwistStamped, p("cmd_topic"), self.on_twist, 10)

        self.lock = threading.Lock()
        self.twist = np.zeros(6)
        self.twist_frame = "base_link"
        self.last_cmd = -np.inf
        self.expected = None
        self.was_moving = False
        self.t0 = time.monotonic()
        self.last_tick = None
        # full history for the saved CSV; plot reads a trailing window of it
        self.rows = []

        self.create_timer(1.0 / p("rate_hz"), self.tick)
        self.get_logger().info(
            f"Tracking {self.odom_frame}->{self.ee_frame}, commands on {p('cmd_topic')}")

    def on_twist(self, msg):
        t = msg.twist
        with self.lock:
            self.twist = np.array([t.linear.x, t.linear.y, t.linear.z,
                                   t.angular.x, t.angular.y, t.angular.z])
            self.twist_frame = msg.header.frame_id or "base_link"
            self.last_cmd = time.monotonic()

    def lookup(self, child):
        tf = self.tf_buf.lookup_transform(self.odom_frame, child, Time())
        tr = tf.transform.translation
        return np.array([tr.x, tr.y, tr.z]), quat_to_rot(tf.transform.rotation)

    def tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        try:
            actual, _ = self.lookup(self.ee_frame)
        except TransformException as e:
            self.get_logger().warn(f"TF: {e}", throttle_duration_sec=2.0)
            return

        with self.lock:
            twist, frame, last_cmd = self.twist.copy(), self.twist_frame, self.last_cmd
        moving = (now - last_cmd) <= self.timeout and np.any(twist)

        if self.expected is None or (moving and not self.was_moving and self.reset_on_motion):
            self.expected = actual.copy()
        if moving:
            try:
                _, R = self.lookup(frame)
                self.expected = self.expected + R @ twist[:3] * dt
            except TransformException as e:
                self.get_logger().warn(f"TF for twist frame '{frame}': {e}",
                                       throttle_duration_sec=2.0)
        self.was_moving = moving

        with self.lock:
            self.rows.append((now - self.t0, int(moving), *actual, *self.expected))

    def snapshot(self):
        with self.lock:
            if not self.rows:
                return None
            a = np.array(self.rows)
        return a[a[:, 0] >= a[-1, 0] - self.window] if self.window > 0 else a

    def save(self, fig):
        if not self.log_dir or not self.rows:
            return
        d = os.path.expanduser(self.log_dir)
        os.makedirs(d, exist_ok=True)
        stem = os.path.join(d, f"ee_track_{time.strftime('%Y%m%d_%H%M%S')}")
        with self.lock, open(stem + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "moving", "act_x", "act_y", "act_z", "exp_x", "exp_y", "exp_z"])
            w.writerows(self.rows)
        # re-render with the full run, not just the live window
        self.window = 0
        draw(self, fig.axes)
        fig.savefig(stem + ".png", dpi=150)
        self.get_logger().info(f"Saved {stem}.csv / .png")


def style(ax, ylabel):
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=MUTED, labelsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)


def shade_moving(ax, t, moving):
    edges = np.flatnonzero(np.diff(np.r_[0, moving, 0]))
    for start, stop in zip(edges[::2], edges[1::2]):
        ax.axvspan(t[start], t[stop - 1], color=MUTED, alpha=0.08, linewidth=0)


def draw(node, axes):
    a = node.snapshot()
    if a is None:
        return
    t, moving = a[:, 0], a[:, 1].astype(bool)
    act, exp = a[:, 2:5], a[:, 5:8]
    err = np.linalg.norm(act - exp, axis=1) * 1000.0
    for ax in axes:
        ax.clear()

    for i, name in enumerate("xyz"):
        ax = axes[i]
        shade_moving(ax, t, moving)
        ax.plot(t, act[:, i], color=ACTUAL_COLOR, lw=2, label="actual")
        ax.plot(t, exp[:, i], color=EXPECTED_COLOR, lw=2, ls="--", label="expected")
        style(ax, f"{name} [m]")
    axes[0].legend(loc="upper left", frameon=False, fontsize=8, ncol=2)
    axes[0].set_title(f"EE position in {node.odom_frame} (shaded = commanding)",
                      color=INK, fontsize=10, loc="left")

    ax = axes[3]
    shade_moving(ax, t, moving)
    ax.plot(t, err, color=ACTUAL_COLOR, lw=2)
    ax.annotate(f"{err[-1]:.1f} mm", (t[-1], err[-1]), xytext=(4, 0),
                textcoords="offset points", color=INK, fontsize=8, va="center")
    style(ax, "|error| [mm]")
    ax.set_xlabel("time [s]", color=MUTED)

    h, v = ("xyz".index(c) for c in node.plane)
    ax = axes[4]
    ax.plot(act[:, h], act[:, v], color=ACTUAL_COLOR, lw=2, label="actual")
    ax.plot(exp[:, h], exp[:, v], color=EXPECTED_COLOR, lw=2, ls="--", label="expected")
    ax.plot(act[-1, h], act[-1, v], "o", color=ACTUAL_COLOR, ms=8, mec="white", mew=2)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"{node.plane[0]}-{node.plane[1]} plane", color=INK, fontsize=10, loc="left")
    style(ax, f"{node.plane[1]} [m]")
    ax.set_xlabel(f"{node.plane[0]} [m]", color=MUTED)


def main(args=None):
    rclpy.init(args=args)
    node = EETrackingPlot()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.6, 1])
    axes = [fig.add_subplot(gs[0, 0])]
    axes += [fig.add_subplot(gs[i, 0], sharex=axes[0]) for i in (1, 2, 3)]
    axes.append(fig.add_subplot(gs[:, 1]))
    fig.tight_layout()

    _anim = FuncAnimation(fig, lambda _: draw(node, axes), interval=150,
                          cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.save(fig)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
