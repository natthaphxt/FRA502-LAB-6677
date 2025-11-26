#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header  
from geometry_msgs.msg import PoseStamped  
from r_interfaces.srv import Random  

# Logic imports
import random
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from math import pi


class RandomServiceNode(Node):
    def __init__(self):
        super().__init__("random_node")  

        self.srv = self.create_service(Random, "random_pose", self.random_callback)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)

        # --- ROBOT DEFINITION ---
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
                rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
                rtb.RevoluteMDH(alpha=0, a=0.25, d=0.0, offset=0.0),
            ],
            tool=SE3.Tx(0.28),
            name="RRR_Robot",
        )

        self.get_logger().info("Random Node Ready.")

    def random_callback(self, request, response):
        if request.mode.data != "AUTO":
            response.success = False
            return response

        # --- LOGIC สุ่มแบบ FK ---
        valid_pose = False
        limit_check_count = 0

        while not valid_pose:
            limit_check_count += 1
            if limit_check_count > 1000:
                break

            q_rand = [
                random.uniform(-np.pi, np.pi),
                random.uniform(-np.pi, np.pi),
                random.uniform(-np.pi, np.pi),
            ]

            T_rand = self.robot.fkine(q_rand)
            x, y, z = T_rand.t[0], T_rand.t[1], T_rand.t[2]

            dist_sq = x**2 + y**2 + (z - 0.2) ** 2
            if 0.15**2 <= dist_sq <= 0.55**2:
                if z >= 0.10:
                    valid_pose = True

        # --- PUBLISH TARGET ---
        msg = PoseStamped()
        msg.header = Header() 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_0"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        quat = UnitQuaternion(T_rand.R)
        msg.pose.orientation.w = float(quat.s)
        msg.pose.orientation.x = float(quat.v[0])
        msg.pose.orientation.y = float(quat.v[1])
        msg.pose.orientation.z = float(quat.v[2])

        self.target_pub.publish(msg)

        # --- SERVICE RESPONSE ---
        response.position.x = float(x)
        response.position.y = float(y)
        response.position.z = float(z)
        response.inprogress = True
        # response.success = True

        self.get_logger().info(f"Generated Target: [{x:.3f}, {y:.3f}, {z:.3f}]")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
