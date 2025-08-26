#!/usr/bin/python3

from turtle_pkg.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


import numpy as np
import math


class eater(Node):
    def __init__(self):
        super().__init__("eater")
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.publisher_text = self.create_publisher(Bool, "/finished", 10)
        self.create_subscription(Pose, "/turtle1/pose_passes", self.pose_callback, 10)
        self.create_subscription(Point, "/mouse_position", self.mouse_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.create_subscription(Pose, 'turtle1_pose_passes', self.pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "/turtle1/eat")
        

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pos = np.array([0.0, 0.0])
        self.pizza_queue = []
        self.count_pizza = []
        self.pizza_count = 0
        self.current_target = None
        self.target_dis = 0.3
        self.pizza = 0
        self.count = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal = 0
        self.kp_d = 5
        self.kp_theta = 5
        self.x = 0
        
    
    def eat_pizza(self):
        self.eat_pizza_callback()

    def eat_pizza_callback(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def spawn_pizza_callback(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_client.call_async(position_request)

    def mouse_callback(self, msg):
        self.mouse_pos[0] = msg.x
        self.mouse_pos[1] = msg.y
        if self.pizza <= 4:
            self.spawn_pizza_callback(self.mouse_pos[0], self.mouse_pos[1])
            self.pizza += 1
            self.pizza_queue.append([self.mouse_pos[0], self.mouse_pos[1]])
            self.pizza_count = len(self.pizza_queue)
        self.get_logger().info(f"Received mouse position: {self.pizza}")

    def pose_callback(self, msg):
        self.turtle_pose = np.array([msg.x, msg.y, msg.theta])

    def goal_pose_callback(self, msg):
        self.goal = 1
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def cmdvel_callback(self, x, w):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = w
        self.publisher.publish(msg)

    def controller(self, x, y):
        if self.count < 5:
            d_x = x - self.turtle_pose[0]
            d_y = y - self.turtle_pose[1]
            d = math.sqrt(((d_x**2) + (d_y**2))) 

            tan = math.atan2(d_y, d_x)
            theta = tan - self.turtle_pose[2]
            error = math.atan2(math.sin(theta), math.cos(theta))

            v = self.kp_d * d
            w = self.kp_theta * error
            self.cmdvel_callback(v, w)

            if self.goal == 1:
                if abs(d_x) < 0.2 and abs(d_y) < 0.2:
                    self.goal = 0
            else:
                if abs(d_x) < 0.2 and abs(d_y) < 0.2:
                    self.eat_pizza()
                    self.count += 1
        else:
            d_x = x - self.turtle_pose[0]
            d_y = y - self.turtle_pose[1]
            d = math.sqrt(((d_x**2) + (d_y**2)))

            target = math.atan2(d_y, d_x)
            theta = target - self.turtle_pose[2]
            w = math.atan2(math.sin(theta), math.cos(theta))

            v = self.kp_d * d
            wz = self.kp_theta * w
            self.cmdvel_callback(v, wz)
            
    def timer_callback(self):
        if self.count < 5:
            if self.goal == 1:
                self.controller(self.goal_x, self.goal_y)
            else:
                if self.count < self.pizza_count:
                    if len(self.pizza_queue) == 0:
                        return
                    else:
                        self.controller(
                            self.pizza_queue[self.count][0], self.pizza_queue[self.count][1]
                        )
                elif self.count == self.pizza_count:
                    self.get_logger().info("All pizza eaten")
                    self.cmdvel_callback(0.0, 0.0)
                    self.x += 1
        else:
            msg = Bool()
            msg.data = True
            self.publisher_text.publish(msg)
            self.controller(self.mouse_pos[0], self.mouse_pos[1])

def main(args=None):
    rclpy.init(args=args)
    node = eater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
