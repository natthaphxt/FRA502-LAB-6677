#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Header
from std_msgs.msg import String
from r_interfaces.srv import Scheduler, Random, Controller
import random
import numpy as np
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3


class RandomNode(Node):
    def __init__(self):
        super().__init__("random_node")

        self.declare_parameter("frequency", 10.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.create_timer(1 / self.frequency, self.timer_callback)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.create_subscription(
            String, "/current_state", self.current_state_callback, 10
        )
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")
        self.controller_client = self.create_client(Controller, "controller_server")
        self.random_server = self.create_service(
            Random, "random_pose", self.random_server_callback
        )

        self.r_max = 0.55  # Adjusted for new model
        self.r_min = 0.10
        self.l = 0.2
        self.z_min = 0.25
        self.z_max = 0.55
        self.ground_clearance = 0.05

        self.current_state = "IDLE"
        self.auto_mode_active = False

        # === FIX: Updated DH Parameters to match Xacro offsets ===
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=-0.12, offset=0.0),
                rtb.RevoluteMDH(alpha=0, a=0.25, d=-0.1, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )
        # ==========================================================

        self.get_logger().info("Random Node Started (Corrected Model).")

    def check_singularity(self, q):
        try:
            J = self.robot.jacob0(q)
            det_J = np.linalg.det(J[0:3, :])
            return abs(det_J) >= 3e-3
        except:
            return False

    def check_full_robot_clearance(self, q):
        try:
            min_safe_z = self.ground_clearance
            T2 = self.robot.fkine([q[0], q[1], 0])
            if T2.t[2] < min_safe_z:
                return (False, 0, "link2")
            T3 = self.robot.fkine(q)
            if T3.t[2] < min_safe_z:
                return (False, 0, "end")
            return (True, 1.0, None)
        except:
            return (False, 0.0, "error")

    def verify_ik_solution(self, x, y, z):
        try:
            T = SE3(x, y, z)
            # Try safe seed
            ik = self.robot.ikine_LM(T, mask=[1, 1, 1, 0, 0, 0], q0=[0, 0.5, 0.5])
            if ik.success:
                is_safe, _, _ = self.check_full_robot_clearance(ik.q)
                if is_safe and self.check_singularity(ik.q):
                    return (ik.q, 1.0)
            return None
        except:
            return None

    def generate_random_position(self):
        for _ in range(200):
            theta = random.uniform(0, 2 * pi)
            phi = random.uniform(0, pi / 2)  # Upper hemisphere
            r = random.uniform(self.r_min, self.r_max)

            x = r * np.cos(phi) * np.cos(theta)
            y = r * np.cos(phi) * np.sin(theta)
            z = self.l + r * np.sin(phi)

            if z < self.z_min or z > self.z_max:
                continue

            if self.verify_ik_solution(x, y, z) is not None:
                self.get_logger().info(f"Generated: ({x:.2f}, {y:.2f}, {z:.2f})")
                return [x, y, z]

        return [0.2, 0.0, 0.45]  # Safe fallback

    def publish_target(self, position):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def random_server_callback(
        self, request: Random.Request, response: Random.Response
    ):
        if request.mode.data == "AUTO":
            position = self.generate_random_position()
            response.position.x = float(position[0])
            response.position.y = float(position[1])
            response.position.z = float(position[2])
            response.inprogress = True
            self.publish_target(position)
            self.send_auto_command(position)
            self.auto_mode_active = True
        return response

    def send_auto_command(self, position):
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            return
        req = Controller.Request()
        req.mode.data = "AUTO"
        req.position.x, req.position.y, req.position.z = (
            position[0],
            position[1],
            position[2],
        )
        self.controller_client.call_async(req)

    def current_state_callback(self, msg):
        old_state = self.current_state
        self.current_state = msg.data
        if (
            self.auto_mode_active
            and old_state == "AUTO"
            and self.current_state == "IDLE"
        ):
            pos = self.generate_random_position()
            self.publish_target(pos)
            self.send_auto_command(pos)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
