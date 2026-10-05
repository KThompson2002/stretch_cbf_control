import math
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

ARM = ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
WRIST = {"joint_wrist_yaw": "wrist_yaw",
         "joint_wrist_pitch": "wrist_pitch",
         "joint_wrist_roll": "wrist_roll"}
HEAD = {"joint_head_pan": "head_pan", "joint_head_tilt": "head_tilt"}


class StretchKinematics:
    def __init__(self, urdf_xml, ee_frame):
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(ee_frame)
        self.joint_names = list(self.model.names)[1:]
        if self.fid >= self.model.nframes:
            raise ValueError(f"Frame '{ee_frame}' not in URDF")

    def _j(self, name):
        return self.model.joints[self.model.getJointId(name)]

    def limits(self, name):
        j = self._j(name)
        if j.nq == 2:
            return -math.inf, math.inf
        return (float(self.model.lowerPositionLimit[j.idx_q]),
                float(self.model.upperPositionLimit[j.idx_q]))

    def _set(self, q, name, val):
        j = self._j(name)
        if j.nq == 2:  # continuous joint stored as (cos, sin)
            q[j.idx_q:j.idx_q + 2] = [math.cos(val), math.sin(val)]
        else:
            q[j.idx_q] = val

    def update(self, lift, arm, wrist):
        """Returns (J 6x7 in base_link axes about the EE point, EE pose in base_link as pin.SE3)."""
        q = pin.neutral(self.model)
        self._set(q, "joint_lift", lift)
        for n in ARM:
            self._set(q, n, arm / 4.0)
        for n, v in wrist.items():
            self._set(q, n, v)

        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J6 = pin.getFrameJacobian(self.model, self.data, self.fid, pin.LOCAL_WORLD_ALIGNED)
        oMf = self.data.oMf[self.fid]
        p = oMf.translation
        col = lambda n: J6[:, self._j(n).idx_v]
        J = np.column_stack([
            [1, 0, 0, 0, 0, 0],                  # base forward
            [-p[1], p[0], 0, 0, 0, 1],           # base rotation about base_link z
            col("joint_lift"),
            sum(col(n) for n in ARM) / 4.0,      # 4 telescoping segments -> 1 DOF
            *[col(n) for n in WRIST],
        ])
        return J, oMf


class SimBackend:
    """Perfect velocity tracking with URDF joint limits. Good for checking the math, not dynamics."""

    def __init__(self, kin, lift0, arm0):
        self.kin = kin
        self.q = {"lift": lift0, "arm": arm0, **{n: 0.0 for n in WRIST}}
        self.base = [0.0, 0.0, 0.0]  # x, y, theta in odom
        self.qd = np.zeros(7)
        lo, hi = kin.limits(ARM[0])
        self.lim = {"lift": kin.limits("joint_lift"), "arm": (4 * lo, 4 * hi),
                    **{n: kin.limits(n) for n in WRIST}}

    def tick(self, dt):
        v, w = self.qd[0], self.qd[1]
        th = self.base[2]
        self.base[0] += v * math.cos(th) * dt
        self.base[1] += v * math.sin(th) * dt
        self.base[2] += w * dt
        for key, vel in zip(["lift", "arm", *WRIST], self.qd[2:]):
            lo, hi = self.lim[key]
            self.q[key] = min(max(self.q[key] + vel * dt, lo), hi)

    def read(self):
        return {"lift": self.q["lift"], "arm": self.q["arm"],
                "wrist": {n: self.q[n] for n in WRIST},
                "head": {n: 0.0 for n in HEAD},
                "base": tuple(self.base)}

    def send(self, qd):
        self.qd = np.array(qd, dtype=float)

    def runstopped(self):
        return False

    def stop(self):
        self.qd[:] = 0.0

class HardwareBackend:
    def __init__(self):
        import stretch_body.robot
        self.r = stretch_body.robot.Robot()
        if not self.r.startup():
            raise RuntimeError("Stretch Body startup failed (is another process using the robot?)")
        if not self.r.is_calibrated():
            self.r.stop()
            raise RuntimeError("Robot is not homed. Run stretch_robot_home.py first.")

    def tick(self, dt):
        pass

    def read(self):
        r = self.r
        b = r.base.status
        return {"lift": r.lift.status["pos"], "arm": r.arm.status["pos"],
                "wrist": {n: r.end_of_arm.get_joint(s).status["pos"] for n, s in WRIST.items()},
                "head": {n: r.head.get_joint(s).status["pos"] for n, s in HEAD.items()},
                "base": (b["x"], b["y"], b["theta"])}

    def send(self, qd):
        r = self.r
        r.base.set_velocity(qd[0], qd[1])
        r.lift.set_velocity(qd[2])
        r.arm.set_velocity(qd[3])
        for v, s in zip(qd[4:], WRIST.values()):
            r.end_of_arm.get_joint(s).set_velocity(v)
        r.push_command()

    def runstopped(self):
        return bool(self.r.pimu.status.get("runstop_event", False))

    def stop(self):
        try:
            self.send(np.zeros(7))
        finally:
            self.r.stop()


class EEVelocityNode(Node):
    def __init__(self):
        super().__init__("ee_velocity")
        self.declare_parameter("sim", True)
        self.declare_parameter("ee_frame", "link_grasp_center")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("cmd_timeout", 0.25)
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("use_base", True)
        self.declare_parameter("joint_weights", [4.0, 4.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("vel_limits", [0.10, 0.30, 0.10, 0.10, 1.0, 1.0, 1.0])
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("sim_initial_lift", 0.6)
        self.declare_parameter("sim_initial_arm", 0.1)
        self.declare_parameter("urdf_path", "")
        # self.declare_parameter("kin", None)
        # self.declare_parameter("backend", None)

        self.sim = self.get_parameter('sim').get_parameter_value().bool_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.rate = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.lam = self.get_parameter('damping').get_parameter_value().double_value
        self.use_base = self.get_parameter('use_base').get_parameter_value().bool_value
        self.weights = np.array(self.get_parameter('joint_weights').value, dtype=float)
        self.vmax = np.array(self.get_parameter('vel_limits').value, dtype=float)
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value
        self.sim_initial_lift = self.get_parameter('sim_initial_lift').get_parameter_value().double_value
        self.sim_initial_arm = self.get_parameter('sim_initial_arm').get_parameter_value().double_value
        self.sim_init = (self.sim_initial_lift, self.sim_initial_arm)
        self.kin = None
        self.backend = None
        self.urdf = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.dt = 1.0 / self.rate

        self.twist = np.zeros(6)
        self.twist_frame = "base_link"
        self.last_cmd = self.get_clock().now()
        self.moving = False

        self.create_subscription(TwistStamped, "~/ee_cmd_vel", self.on_twist, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.path_pub = self.create_publisher(Path, "~/ee_path", 10)

        self.tf = TransformBroadcaster(self)

        self.create_service(Trigger, "~/stop", self.on_stop)
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.create_timer(self.dt, self.step)

        # robot_state_publisher publishes /robot_description latched (transient local)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, qos)
        self.get_logger().info("Waiting for /robot_description ...")

    def on_description(self, msg):
        if self.kin is None:
            self.init_model(msg.data, "/robot_description")

    def init_model(self, xml, source):
        self.kin = StretchKinematics(xml, self.ee_frame)
        self.backend = SimBackend(self.kin, *self.sim_init) if self.sim else HardwareBackend()
        self.get_logger().info(f"Kinematics loaded from {source} ({len(self.kin.joint_names)} joints)")

    # ---------------- callbacks ----------------
    def on_twist(self, msg):
        t = msg.twist
        self.twist = np.array([t.linear.x, t.linear.y, t.linear.z,
                                t.angular.x, t.angular.y, t.angular.z])
        self.twist_frame = msg.header.frame_id or "base_link"
        self.last_cmd = self.get_clock().now()

    def on_stop(self, _req, res):
        self.twist[:] = 0.0
        if self.backend:
            self.backend.send(np.zeros(7))
        self.moving = False
        self.path.poses.clear()
        res.success, res.message = True, "stopped, trail cleared"
        return res

    # ---------------- control loop ----------------
    def step(self):
        if self.backend is None:
            return
        self.backend.tick(self.dt)
        s = self.backend.read()
        J, oMf = self.kin.update(s["lift"], s["arm"], s["wrist"])
        now = self.get_clock().now()
        self.publish_state(s, oMf, now)

        stale = (now - self.last_cmd).nanoseconds * 1e-9 > self.timeout
        if stale or self.backend.runstopped() or not np.any(self.twist):
            if self.moving:
                self.backend.send(np.zeros(7))
                self.moving = False
            return

        x = self.twist.copy()
        if self.twist_frame == self.ee_frame:          # gripper-frame twist -> base axes
            R = oMf.rotation
            x[:3], x[3:] = R @ x[:3], R @ x[3:]
        elif self.twist_frame != "base_link":
            self.get_logger().warn(f"Unsupported frame '{self.twist_frame}'", throttle_duration_sec=2.0)
            return

        if not self.use_base:
            J[:, 0:2] = 0.0

        Winv = np.diag(1.0 / self.weights)               # weighted damped least squares
        qd = Winv @ J.T @ np.linalg.solve(J @ Winv @ J.T + self.lam**2 * np.eye(6), x)

        scale = np.max(np.abs(qd) / self.vmax)            # uniform scaling keeps direction
        if scale > 1.0:
            qd /= scale

        self.backend.send(qd)
        self.moving = True


    # ---------------- publishing ----------------
    def publish_state(self, s, oMf, now):
        stamp = now.to_msg()

        js = JointState()
        js.header.stamp = stamp
        # Publish EVERY movable URDF joint (wheels, fingers, ...), or robot_state_publisher
        # can't compute TF for those links and RViz shows them as missing.
        known = {"joint_lift": s["lift"], **{n: s["arm"] / 4.0 for n in ARM},
                 **s["wrist"], **s["head"]}
        js.name = self.kin.joint_names
        js.position = [float(known.get(n, 0.0)) for n in js.name]
        self.js_pub.publish(js)

        bx, by, bth = s["base"]
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = "base_link"
            t.transform.translation.x, t.transform.translation.y = bx, by
            t.transform.rotation.z, t.transform.rotation.w = math.sin(bth / 2), math.cos(bth / 2)
            self.tf.sendTransform(t)

        # EE trail in odom: lets you see whether a "straight line" command really is straight
        odom_T_base = pin.SE3(pin.utils.rpyToMatrix(0, 0, bth), np.array([bx, by, 0.0]))
        ee = odom_T_base * oMf
        pt = ee.translation
        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last is None or np.linalg.norm(pt - [last.x, last.y, last.z]) > 0.002:
            ps = PoseStamped()
            ps.header.frame_id = self.odom_frame
            ps.header.stamp = stamp
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pt)
            qx, qy, qz, qw = pin.Quaternion(ee.rotation).coeffs()
            ps.pose.orientation.x, ps.pose.orientation.y = float(qx), float(qy)
            ps.pose.orientation.z, ps.pose.orientation.w = float(qz), float(qw)
            self.path.poses.append(ps)
            del self.path.poses[:-3000]
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)

    def shutdown(self):
        if self.backend:
            self.backend.stop()


def main(args=None):
    """Entry point for the ball_track node."""
    rclpy.init(args=args)
    node = EEVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()          # zero velocities + release Stretch Body on hardware
        node.destroy_node()
        rclpy.try_shutdown()



if __name__ == '__main__':
    import sys
    main(sys.argv)
