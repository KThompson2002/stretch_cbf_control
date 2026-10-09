#!/usr/bin/env python3
"""
CBF safety filter for Stretch EE velocity control.

  RelativeMove (input_topic) -> filter -> TwistStamped (output_topic, consumed by ee_velocity)

The EE is constrained to a plane in reference_frame:
  plane:=xy  -> moves in x/y inside a box, z held at the latched plane
  plane:=yz  -> moves in y/z inside a box, x held at the latched plane
In-plane velocities are clamped by a box CBF; the out-of-plane command is dropped and
replaced by a PI correction that holds the EE on the plane (enable_plane_hold).
dtheta is passed through as angular velocity about the reference z axis.

gripper is not filtered: OPEN/CLOSED is forwarded to gripper_topic (std_msgs/Bool,
true = closed) only when it changes the commanded state; HOLD leaves the gripper alone.

~/home (std_srvs/Trigger) drives the EE to the home corner of the box (home_<axis> picks
min/max/center per in-plane axis, pulled home_margin inside since the CBF never quite
reaches an edge) and returns once it arrives or times out. While homing, incoming
RelativeMove commands are dropped, so homing takes priority.
"""
import threading
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Point, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from stretch_ee_vel_ctrl_interfaces.msg import RelativeMove
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

AXES = 'xyz'


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CBFFilter(Node):

    def __init__(self):
        super().__init__('cbf_filter')

        # Plane the EE is limited to: 'xy' or 'yz' (axes of reference_frame)
        self.declare_parameter('plane', 'yz')

        # Workspace box bounds in reference_frame [metres]. Only the two in-plane
        # axes are used. In base_link the arm extends along -y and the lift is +z.
        self.declare_parameter('x_min', -0.2)
        self.declare_parameter('x_max', 0.2)
        self.declare_parameter('y_min', -0.9)
        self.declare_parameter('y_max', -0.4)
        self.declare_parameter('z_min', 0.825)
        self.declare_parameter('z_max', 1.2)

        # CBF decay rate α: higher values brake harder near boundaries
        self.declare_parameter('cbf_alpha', 1.1)

        # Frames. Positions and RelativeMove velocities are in reference_frame; the
        # output twist is rotated into base_link, the frame ee_velocity accepts.
        self.declare_parameter('ee_frame', 'link_grasp_center')
        self.declare_parameter('reference_frame', 'base_link')

        # Topics
        self.declare_parameter('input_topic', '/velocity_pub/vel_command')
        self.declare_parameter('output_topic', '/ee_velocity/ee_cmd_vel')
        self.declare_parameter('gripper_topic', '/ee_velocity/gripper_cmd')
        self.declare_parameter('enable_plane_hold', True)
        self.declare_parameter('hold_kp', 2.0)
        self.declare_parameter('hold_ki', 0.1)
        self.declare_parameter('hold_integral_clamp', 0.05)
        self.declare_parameter('hold_correction_max', 0.05)
        self.declare_parameter('hold_relatch_s', 0.5)

        # Homing
        #   home_x/y/z:     'min', 'max' or 'center' of the box (in-plane axes only)
        #   home_margin:    m, how far inside the box edge the home target sits
        #   home_kp:        1/s, proportional gain toward the target
        #   home_speed_max: m/s, cap on homing speed (straight line, direction kept)
        #   home_tolerance: m, in-plane distance counted as arrived
        self.declare_parameter('home_x', 'center')
        self.declare_parameter('home_y', 'max')
        self.declare_parameter('home_z', 'min')
        self.declare_parameter('home_margin', 0.02)
        self.declare_parameter('home_kp', 1.5)
        self.declare_parameter('home_speed_max', 0.05)
        self.declare_parameter('home_tolerance', 0.01)
        self.declare_parameter('home_timeout_s', 30.0)
        self.declare_parameter('home_rate_hz', 30.0)

        plane = self.get_parameter('plane').value.lower()
        if plane not in ('xy', 'yz'):
            raise ValueError(f"plane must be 'xy' or 'yz', got '{plane}'")
        self._plane = [AXES.index(c) for c in plane]
        self._normal = ({0, 1, 2} - set(self._plane)).pop()

        self._hold_target = None
        self._hold_integral = 0.0
        self._last_cmd_time = None

        self._homing = False
        self._home_target = None
        self._home_deadline = 0.0
        self._home_done = threading.Event()
        self._home_result = (False, '')

        self._gripper_closed = None  # last state forwarded; None until the first command

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 10)
        self._gripper_pub = self.create_publisher(
            Bool, self.get_parameter('gripper_topic').value, 10)
        self._cmd_sub = self.create_subscription(
            RelativeMove, self.get_parameter('input_topic').value, self._cmd_cb, 10)

        self._marker_pub = self.create_publisher(MarkerArray, '~/bounds_marker', 1)
        self._marker_timer = self.create_timer(1.0, self._publish_bounds_marker)

        # The service blocks until homing finishes, so it gets its own callback group;
        # the homing timer, command subscription and TF keep running in the default one.
        self._home_timer = self.create_timer(
            1.0 / self.get_parameter('home_rate_hz').value, self._home_step)
        self.create_service(Trigger, '~/home', self._home_cb,
                            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f'CBF filter ready: {plane} plane, holding {AXES[self._normal]}')

    def _bounds(self, axis):
        c = AXES[axis]
        return (self.get_parameter(f'{c}_min').value,
                self.get_parameter(f'{c}_max').value)

    def _lookup(self, target, source):
        try:
            return self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None

    def _get_ee_position(self):
        """Return EE position in reference_frame as an array, or None on failure."""
        t = self._lookup(self.get_parameter('reference_frame').value,
                         self.get_parameter('ee_frame').value)
        if t is None:
            return None
        p = t.transform.translation
        return np.array([p.x, p.y, p.z])

    def _apply_cbf(self, v, pos):
        a = self.get_parameter('cbf_alpha').value
        for i in self._plane:
            lo, hi = self._bounds(i)
            v[i] = _clamp(v[i], -a * (pos[i] - lo), a * (hi - pos[i]))
        return v

    def _apply_plane_hold(self, v, pos):
        n = self._normal
        v[n] = 0.0
        if not self.get_parameter('enable_plane_hold').value:
            return v

        kp = self.get_parameter('hold_kp').value
        ki = self.get_parameter('hold_ki').value
        i_clamp = self.get_parameter('hold_integral_clamp').value
        corr_max = self.get_parameter('hold_correction_max').value

        now = self.get_clock().now()
        dt = None
        if self._last_cmd_time is not None:
            dt = (now - self._last_cmd_time).nanoseconds * 1e-9
        self._last_cmd_time = now

        # Fresh start or long gap: latch the plane where the EE is now.
        if self._hold_target is None or dt is None or dt > self.get_parameter('hold_relatch_s').value:
            self._hold_target = pos[n]
            self._hold_integral = 0.0

        error = self._hold_target - pos[n]
        # Reject stale dt (long gap) — don't integrate over it.
        if ki != 0.0 and dt is not None and 0.0 < dt < 0.2:
            self._hold_integral = _clamp(
                self._hold_integral + error * dt, -i_clamp, i_clamp)

        v[n] = _clamp(kp * error + ki * self._hold_integral, -corr_max, corr_max)
        return v

    def _home_position(self, pos):
        """Home target in reference_frame; the plane-normal axis stays where it is."""
        m = self.get_parameter('home_margin').value
        target = pos.copy()
        for i in self._plane:
            lo, hi = self._bounds(i)
            side = self.get_parameter(f'home_{AXES[i]}').value
            if side == 'min':
                target[i] = lo + m
            elif side == 'max':
                target[i] = hi - m
            elif side == 'center':
                target[i] = 0.5 * (lo + hi)
            else:
                raise ValueError(f"home_{AXES[i]} must be 'min', 'max' or 'center', got '{side}'")
        return target

    def _finish_homing(self, success, message):
        self._publish_cmd(np.zeros(3), np.zeros(3))
        self._homing = False
        self._home_result = (success, message)
        self._home_done.set()
        log = self.get_logger().info if success else self.get_logger().warn
        log(f'Homing: {message}')

    def _home_step(self):
        if not self._homing:
            return
        if time.monotonic() > self._home_deadline:
            self._finish_homing(False, 'timed out')
            return
        pos = self._get_ee_position()
        if pos is None:
            return

        err = np.zeros(3)
        err[self._plane] = (self._home_target - pos)[self._plane]
        dist = np.linalg.norm(err)
        if dist < self.get_parameter('home_tolerance').value:
            self._finish_homing(True, f'reached home ({dist * 1000:.1f} mm)')
            return

        v = self.get_parameter('home_kp').value * err
        speed, vmax = np.linalg.norm(v), self.get_parameter('home_speed_max').value
        if speed > vmax:
            v *= vmax / speed
        v = self._apply_cbf(v, pos)
        v = self._apply_plane_hold(v, pos)
        self._publish_cmd(v, np.zeros(3))

    def _home_cb(self, _req, res):
        if self._homing:
            res.success, res.message = False, 'already homing'
            return res
        pos = self._get_ee_position()
        if pos is None:
            res.success, res.message = False, 'no TF for end effector'
            return res
        try:
            target = self._home_position(pos)
        except ValueError as e:
            res.success, res.message = False, str(e)
            return res

        timeout = self.get_parameter('home_timeout_s').value
        self._home_target = target
        self._home_deadline = time.monotonic() + timeout
        self._home_done.clear()
        self._homing = True
        self.get_logger().info(f'Homing to {np.round(target, 3).tolist()}')

        if not self._home_done.wait(timeout + 2.0):
            self._homing = False
            self._home_result = (False, 'homing timer stalled')
        res.success, res.message = self._home_result
        return res

    def _publish_bounds_marker(self):
        frame = self.get_parameter('reference_frame').value
        i, j = self._plane
        (i_min, i_max), (j_min, j_max) = self._bounds(i), self._bounds(j)

        # Draw the rectangle at the latched plane if available; otherwise at the
        # current EE position; otherwise at 0 as a last resort.
        if self._hold_target is not None:
            offset = self._hold_target
        else:
            pos = self._get_ee_position()
            offset = pos[self._normal] if pos is not None else 0.0

        def point(a, b):
            p = [0.0, 0.0, 0.0]
            p[i], p[j], p[self._normal] = a, b, offset
            return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

        now = self.get_clock().now().to_msg()

        outline = Marker()
        outline.header.frame_id = frame
        outline.header.stamp = now
        outline.ns = 'cbf_bounds'
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.005
        outline.color.r = 0.1
        outline.color.g = 0.9
        outline.color.b = 0.2
        outline.color.a = 1.0
        outline.pose.orientation.w = 1.0
        corners = [
            (i_min, j_min), (i_max, j_min),
            (i_max, j_max), (i_min, j_max),
            (i_min, j_min),
        ]
        outline.points = [point(a, b) for a, b in corners]

        plane = Marker()
        plane.header.frame_id = frame
        plane.header.stamp = now
        plane.ns = 'cbf_bounds'
        plane.id = 1
        plane.type = Marker.CUBE
        plane.action = Marker.ADD
        plane.pose.position = point(0.5 * (i_min + i_max), 0.5 * (j_min + j_max))
        plane.pose.orientation.w = 1.0
        scale = [0.001, 0.001, 0.001]
        scale[i] = max(1e-3, i_max - i_min)
        scale[j] = max(1e-3, j_max - j_min)
        plane.scale.x, plane.scale.y, plane.scale.z = scale
        plane.color.r = 0.1
        plane.color.g = 0.6
        plane.color.b = 1.0
        plane.color.a = 0.15

        self._marker_pub.publish(MarkerArray(markers=[outline, plane]))

    def _forward_gripper(self, gripper):
        if gripper == RelativeMove.GRIPPER_HOLD:
            return
        if gripper not in (RelativeMove.GRIPPER_OPEN, RelativeMove.GRIPPER_CLOSED):
            self.get_logger().warn(f'Unknown gripper command {gripper}', throttle_duration_sec=2.0)
            return
        closed = gripper == RelativeMove.GRIPPER_CLOSED
        if closed != self._gripper_closed:
            self._gripper_closed = closed
            self._gripper_pub.publish(Bool(data=closed))
            self.get_logger().info(f'Gripper -> {"closed" if closed else "open"}')

    def _cmd_cb(self, msg: RelativeMove):
        # Gripper goes first so it still works while homing or without EE TF.
        self._forward_gripper(msg.gripper)
        if self._homing:
            return
        pos = self._get_ee_position()
        if pos is None:
            return

        v = np.array([msg.dx, msg.dy, msg.dz])
        v = self._apply_cbf(v, pos)
        v = self._apply_plane_hold(v, pos)
        w = np.array([0.0, 0.0, msg.dtheta])
        self._publish_cmd(v, w)

    def _publish_cmd(self, v, w):
        ref = self.get_parameter('reference_frame').value
        if ref != 'base_link':
            t = self._lookup('base_link', ref)
            if t is None:
                return
            R = _quat_to_rot(t.transform.rotation)
            v, w = R @ v, R @ w

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = map(float, v)
        out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = map(float, w)
        self._cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CBFFilter()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
