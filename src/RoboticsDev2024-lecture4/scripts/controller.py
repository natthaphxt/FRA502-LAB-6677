#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from std_msgs.msg import String, Header
from r_interfaces.srv import Scheduler, Random, Controller
import numpy as np

# Inverse Kinematics
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Frequency settings
        self.declare_parameter("frequency", 50.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)

        # Services
        self.controller_server = self.create_service(
            Controller, "controller_server", self.controller_server_callback
        )
        self.controller_state = "IDLE"

        # State management
        self.current_state = "IDLE"
        self.create_subscription(
            String, "/current_state", self.current_state_callback, 10
        )
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")

        # Teleop subscriber
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.tele_x = 0.0
        self.tele_y = 0.0
        self.tele_z = 0.0

        # TF & Visualization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "end_effector"
        self.source_frame = "link_0"

        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.endeff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)

        # Robot state
        self.kp = 1.0
        self.q = np.array([0.0, np.pi / 4, np.pi / 6])  # Safe init pose

        # Workspace limits
        self.r_max = 0.53
        self.r_min = 0.03
        self.l = 0.2

        self.ik_setpoint = [0, 0, 0]
        self.random_setpoint = [0, 0, 0]
        self.auto_start_time = None
        self.auto_timeout = 10.0

        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        self.joint_state = JointState()
        self.joint_state.header.frame_id = ""
        self.joint_state.name = ["joint_1", "joint_2", "joint_3"]
        self.joint_state.position = [0.0, 0.0, 0.0]

        # Robot Model
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )
        self.publish_joint_state(self.q)
        self.singularity_pub = self.create_publisher(String, "/singularity_warning", 10)
        self.get_logger().info("controller_node started (Multi-Seed IK).")
        init_pos, _ = self.get_current_fk()
        self.rviz_pub(init_pos)

    def check_robot_safety(self, q):
        """
        Check if configuration is safe.
        Ensures Shoulder and Elbow stay above ground (Z > 0.05).
        """
        try:
            min_safe_z = 0.05
            T2 = self.robot.fkine([q[0], q[1], 0])
            if T2.t[2] < min_safe_z:
                return False  # Shoulder/Elbow is dipping too low

            # Check End Effector Height
            T3 = self.robot.fkine(q)
            if T3.t[2] < min_safe_z:
                return False  # Hand is hitting floor

            return True
        except:
            return False

    def solve_ik_robust(self, x, y, z):
        """
        Try to solve IK using multiple seeds to find a SAFE solution.
        """
        T_target = SE3(x, y, z)
        seeds = [
            self.q,  # Current
            [0.0, 1.0, -1.0],  # High Shoulder, Bent Elbow (Very Safe)
            [0.0, 0.5, 0.5],  # Mild Elbow Up
            [0.0, 0.0, 0.0],  # Zero
            [np.arctan2(y, x), 0, 0],  # Facing target
        ]

        for i, seed in enumerate(seeds):
            ik = self.robot.ikine_LM(T_target, mask=[1, 1, 1, 0, 0, 0], q0=seed)

            if ik.success and self.check_robot_safety(ik.q):
                if i > 0:  # Log if we had to use a backup seed
                    self.get_logger().info(f"IK found using seed #{i}")
                return ik.q

        return None  # No safe solution found

    def cmd_vel_callback(self, msg: Twist):
        self.tele_x = msg.linear.x
        self.tele_y = msg.linear.y
        self.tele_z = msg.linear.z

    def req_scheduler(self, state):
        state_request = Scheduler.Request()
        state_request.state.data = str(state)
        self.scheduler_client.call_async(state_request)

    def controller_server_callback(
        self, request: Controller.Request, response: Controller.Response
    ):
        self.get_logger().info(
            f"Mode change: {self.controller_state} -> {request.mode.data}"
        )

        self.tele_x = 0.0
        self.tele_y = 0.0
        self.tele_z = 0.0

        self.controller_state = request.mode.data

        if self.controller_state == "AUTO":
            self.random_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]
            self.auto_start_time = time.time()
            response.inprogress = True

        elif self.controller_state == "IK":
            self.ik_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]

            # Check immediately
            q_sol = self.solve_ik_robust(*self.ik_setpoint)
            if q_sol is not None:
                response.inprogress = True
                self.get_logger().info(f"IK Target accepted: {self.ik_setpoint}")
            else:
                response.inprogress = False
                self.get_logger().warn(
                    f"IK Target {self.ik_setpoint} unreachable or unsafe."
                )

        elif "TELEOP" in self.controller_state:
            response.inprogress = True

        return response

    def current_state_callback(self, msg: String):
        self.current_state = msg.data

    def publish_joint_state(self, positions):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = (
            positions.tolist() if hasattr(positions, "tolist") else positions
        )
        self.joint_state_publisher.publish(self.joint_state)

    def get_current_fk(self):
        T = self.robot.fkine(self.q)
        return T.t, T.R

    def control_vel(self, mode):
        try:
            p_now, r_e = self.get_current_fk()

            if mode == "TELEOP_G":
                p_dot = np.array([self.tele_x, self.tele_y, self.tele_z])
            elif mode == "TELEOP_F":
                p_dot = r_e @ np.array([self.tele_x, self.tele_y, self.tele_z])
            else:
                return

            if np.linalg.norm(p_dot) < 0.001:
                return

            J = self.robot.jacob0(self.q)
            J_pos = J[0:3, :]

            if abs(np.linalg.det(J_pos)) < 1e-3:
                self.singularity_pub.publish(String(data="Singularity Risk"))

            q_dot = np.linalg.pinv(J_pos) @ p_dot
            new_q = self.q + q_dot * (1.0 / self.frequency)

            if self.check_robot_safety(new_q):
                self.q = new_q
                self.publish_joint_state(self.q)
            else:
                self.get_logger().warn("Teleop unsafe move", throttle_duration_sec=1.0)
                self.tele_x, self.tele_y, self.tele_z = 0, 0, 0

        except Exception as e:
            self.get_logger().error(f"Control error: {e}")

    def control_to_pos(self, p_set):
        try:
            p_now, _ = self.get_current_fk()
            p_setpoint = np.array(p_set)

            error = p_setpoint - p_now
            if np.linalg.norm(error) <= 0.001:
                self.get_logger().info(
                    f"Target reached. Error: {np.linalg.norm(error):.4f}"
                )
                # --- [IMPORTANT FIX] Send FINISHED, not IDLE ---
                self.req_scheduler("FINISHED")
                self.controller_state = "IDLE"
                return False

            q_target = self.solve_ik_robust(p_setpoint[0], p_setpoint[1], p_setpoint[2])

            if q_target is None:
                self.get_logger().warn("Target became unreachable during move")
                # If target is bad, ask for a new one (FINISHED), don't stop (IDLE)
                self.req_scheduler("FINISHED")
                self.controller_state = "IDLE"
                return False

            self.q = self.q + 0.2 * (q_target - self.q)
            self.publish_joint_state(self.q)
            return True

        except Exception as e:
            self.get_logger().error(f"Auto control error: {e}")
            return False

    def inverse_kinematic(self, x, y, z):
        dist_sq = x**2 + y**2 + (z - 0.2) ** 2
        if not (self.r_min**2 <= dist_sq <= self.r_max**2):
            self.get_logger().warn(
                f"Target out of workspace (dist={np.sqrt(dist_sq):.2f})"
            )
            return None

        return self.solve_ik_robust(x, y, z)

    def rviz_pub(self, pos):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "link_0"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            float(pos[0]),
            float(pos[1]),
            float(pos[2]),
        )
        self.target_pub.publish(target)

        p_now, _ = self.get_current_fk()
        endeff = PoseStamped()
        endeff.header.stamp = self.get_clock().now().to_msg()
        endeff.header.frame_id = "link_0"
        endeff.pose.position.x, endeff.pose.position.y, endeff.pose.position.z = (
            float(p_now[0]),
            float(p_now[1]),
            float(p_now[2]),
        )
        self.endeff_pub.publish(endeff)

    def timer_callback(self):
        if not hasattr(self, "initialized"):
            self.initialized = False
            self.init_time = time.time()
        if not self.initialized:
            if time.time() - self.init_time > 2.0:
                self.initialized = True
            else:
                return

        curr_pos, _ = self.get_current_fk()

        if self.controller_state == "AUTO":
            if self.auto_start_time and (
                time.time() - self.auto_start_time > self.auto_timeout
            ):
                self.get_logger().warn("Auto move timed out, requesting next.")
                # Timeout -> ask for next target
                self.req_scheduler("FINISHED")
                self.controller_state = "IDLE"
                return
            if not self.control_to_pos(self.random_setpoint):
                self.auto_start_time = None
            self.rviz_pub(self.random_setpoint)

        elif self.controller_state == "IK":
            if not self.control_to_pos(self.ik_setpoint):
                pass
            self.rviz_pub(self.ik_setpoint)

        elif "TELEOP" in self.controller_state:
            self.control_vel(self.controller_state)
            self.rviz_pub(curr_pos)

        else:
            self.rviz_pub(curr_pos)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
