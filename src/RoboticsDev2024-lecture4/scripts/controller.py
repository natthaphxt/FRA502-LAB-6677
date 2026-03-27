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
        """Request state change from scheduler"""
        if not self.scheduler_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("Scheduler service not available")
            return
        state_request = Scheduler.Request()
        state_request.state.data = str(state)
        self.scheduler_client.call_async(state_request)

    def controller_server_callback(
        self, request: Controller.Request, response: Controller.Response
    ):
        new_mode = request.mode.data
        self.get_logger().info(
            f"Controller mode: {self.controller_state} -> {new_mode}"
        )

        # Reset teleop velocities on mode change
        self.tele_x = 0.0
        self.tele_y = 0.0
        self.tele_z = 0.0

        self.controller_state = new_mode

        if new_mode == "AUTO":
            self.random_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]
            self.auto_start_time = time.time()
            response.inprogress = True
            self.get_logger().info(f"AUTO target: {self.random_setpoint}")

        elif new_mode == "IK":
            self.ik_setpoint = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]

            # Validate IK target
            q_sol = self.solve_ik_robust(*self.ik_setpoint)
            if q_sol is not None:
                response.inprogress = True
                self.get_logger().info(f"IK target accepted: {self.ik_setpoint}")
            else:
                response.inprogress = False
                self.get_logger().warn(
                    f"IK target {self.ik_setpoint} unreachable or unsafe"
                )

        elif new_mode in ["TELEOP_G", "TELEOP_F"]:
            response.inprogress = True
            self.get_logger().info(f"TELEOP mode: {new_mode}")

        elif new_mode == "IDLE":
            response.inprogress = True
            self.auto_start_time = None
            self.get_logger().info("Entering IDLE mode")

        else:
            response.inprogress = False
            self.get_logger().warning(f"Unknown mode: {new_mode}")

        return response

    def current_state_callback(self, msg: String):
        """Sync with scheduler state"""
        if self.current_state != msg.data:
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

    def get_end_effector_from_tf(self):
        """Get actual end effector position and orientation from TF tree (matches RViz)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "link_0",
                "end_effector",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            pos = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            orientation = transform.transform.rotation  # Quaternion
            return pos, orientation
        except Exception as e:
            # Fallback to FK if TF not available
            p, _ = self.get_current_fk()
            return p, None

    def control_vel(self, mode):
        """Velocity control for TELEOP modes"""
        try:
            p_now, r_e = self.get_current_fk()

            if mode == "TELEOP_G":
                # Velocity in global frame
                p_dot = np.array([self.tele_x, self.tele_y, self.tele_z])
            elif mode == "TELEOP_F":
                # Velocity in end-effector frame, transform to global
                p_dot = r_e @ np.array([self.tele_x, self.tele_y, self.tele_z])
            else:
                return

            # No movement if velocity is near zero
            if np.linalg.norm(p_dot) < 0.001:
                return

            # Compute Jacobian
            J = self.robot.jacob0(self.q)
            J_pos = J[0:3, :]

            # Check for singularity
            det_J = abs(np.linalg.det(J_pos))

            # SEVERE singularity - must stop and notify scheduler
            if det_J < 1e-6:
                self.singularity_pub.publish(String(data=f"SEVERE:det={det_J:.8f}"))
                self.get_logger().error(
                    f"SEVERE singularity! det(J)={det_J:.8f} - Stopping!"
                )
                return  # Hard stop

            # Warning zone - just log, don't stop or publish
            elif det_J < 1e-4:
                self.get_logger().warning(
                    f"Near singularity, det(J)={det_J:.6f}", throttle_duration_sec=1.0
                )

            # Compute joint velocities using pseudo-inverse
            q_dot = np.linalg.pinv(J_pos) @ p_dot
            new_q = self.q + q_dot * (1.0 / self.frequency)

            # Safety check before applying
            if self.check_robot_safety(new_q):
                self.q = new_q
                self.publish_joint_state(self.q)
            else:
                self.get_logger().warn(
                    "TELEOP: Unsafe move blocked", throttle_duration_sec=1.0
                )

        except Exception as e:
            self.get_logger().error(f"TELEOP control error: {e}")

    def control_to_pos(self, p_set):
        """Position control for IK and AUTO modes"""
        try:
            p_now, _ = self.get_current_fk()
            p_setpoint = np.array(p_set)

            error = p_setpoint - p_now
            error_norm = np.linalg.norm(error)

            # Check if target reached
            if error_norm <= 0.001:
                self.get_logger().info(f"Target reached! Error: {error_norm:.4f}")
                return False  # Target reached

            # Solve IK for target
            q_target = self.solve_ik_robust(p_setpoint[0], p_setpoint[1], p_setpoint[2])

            if q_target is None:
                self.get_logger().warn("Target unreachable during motion")
                return False  # Can't reach target

            # Smooth interpolation towards target
            alpha = min(0.2, error_norm)  # Slower when close
            self.q = self.q + alpha * (q_target - self.q)
            self.publish_joint_state(self.q)
            return True  # Still moving

        except Exception as e:
            self.get_logger().error(f"Position control error: {e}")
            return False

    def inverse_kinematic(self, x, y, z):
        """Check if position is in workspace and solve IK"""
        dist_sq = x**2 + y**2 + (z - 0.2) ** 2
        if not (self.r_min**2 <= dist_sq <= self.r_max**2):
            self.get_logger().warn(
                f"Target out of workspace (dist={np.sqrt(dist_sq):.2f})"
            )
            return None
        return self.solve_ik_robust(x, y, z)

    def rviz_pub(self, pos):
        """Publish target and end-effector for RViz visualization"""
        # Target position
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "link_0"
        target.pose.position.x = float(pos[0])
        target.pose.position.y = float(pos[1])
        target.pose.position.z = float(pos[2])
        self.target_pub.publish(target)

        # Current end-effector position and orientation (from TF - matches RViz exactly)
        p_now, orientation = self.get_end_effector_from_tf()
        endeff = PoseStamped()
        endeff.header.stamp = self.get_clock().now().to_msg()
        endeff.header.frame_id = "link_0"
        endeff.pose.position.x = float(p_now[0])
        endeff.pose.position.y = float(p_now[1])
        endeff.pose.position.z = float(p_now[2])

        # Set orientation from TF
        if orientation is not None:
            endeff.pose.orientation = orientation
        else:
            endeff.pose.orientation.w = 1.0  # Default identity quaternion

        self.endeff_pub.publish(endeff)

    def timer_callback(self):
        """Main control loop"""
        # Initialization delay
        if not hasattr(self, "initialized"):
            self.initialized = False
            self.init_time = time.time()
        if not self.initialized:
            if time.time() - self.init_time > 2.0:
                self.initialized = True
            else:
                return

        # Get current position from TF (matches RViz)
        curr_pos, _ = self.get_end_effector_from_tf()

        if self.controller_state == "AUTO":
            # Check timeout
            if self.auto_start_time and (
                time.time() - self.auto_start_time > self.auto_timeout
            ):
                self.get_logger().warn("AUTO move timed out")
                self.req_scheduler("FINISHED")
                self.controller_state = "IDLE"
                self.auto_start_time = None
                return

            # Move towards target
            still_moving = self.control_to_pos(self.random_setpoint)
            if not still_moving:
                # Target reached or unreachable, request next
                self.req_scheduler("FINISHED")
                self.controller_state = "IDLE"
                self.auto_start_time = None

            self.rviz_pub(self.random_setpoint)

        elif self.controller_state == "IK":
            still_moving = self.control_to_pos(self.ik_setpoint)
            if not still_moving:
                # Stay in IK mode but stop moving
                pass
            self.rviz_pub(self.ik_setpoint)

        elif self.controller_state in ["TELEOP_G", "TELEOP_F"]:
            self.control_vel(self.controller_state)
            self.rviz_pub(curr_pos)

        else:  # IDLE or unknown
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
