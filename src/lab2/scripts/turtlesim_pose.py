#!/usr/bin/python3

from turtle_pkg.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, TransformStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import numpy as np

class DummyNode(Node):
    def __init__(self):
        super().__init__("dummy_node")
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.publisher_1 = self.create_publisher(Pose, "/turtle1/pose_passes", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.pose1_callback, 10)

        self.publisher_2 = self.create_publisher(Pose, "/turtle2/pose_passes", 10)
        self.create_subscription(Pose, "/turtle2/pose", self.pose2_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turtle1_pose = np.array([0.0, 0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pos = np.array([0.0, 0.0])

    def pose1_callback(self, msg):
        self.turtle1_pose = np.array([msg.x, msg.y, msg.theta])

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = 'turtle1'
        odom_msg.pose.pose.position.x = self.turtle1_pose[0]
        odom_msg.pose.pose.position.y = self.turtle1_pose[1]

        q = quaternion_from_euler(0, 0, self.turtle1_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_publisher.publish(odom_msg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "turtle1"
        t.transform.translation.x = self.turtle1_pose[0]-5.44
        t.transform.translation.y = self.turtle1_pose[1]-5.44
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.publisher_1.publish(msg)

    def pose2_callback(self, msg):
        self.turtle2_pose = np.array([msg.x, msg.y, msg.theta])

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "turtle2"
        odom_msg.pose.pose.position.x = self.turtle2_pose[0]
        odom_msg.pose.pose.position.y = self.turtle2_pose[1]

        q = quaternion_from_euler(0, 0, self.turtle2_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_publisher.publish(odom_msg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "turtle2"
        t.transform.translation.x = self.turtle2_pose[0]-5.44
        t.transform.translation.y = self.turtle2_pose[1]-5.44
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.publisher_2.publish(msg)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
