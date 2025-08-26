#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.srv import Spawn
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import numpy as np
from turtlesim.srv import Kill
from std_msgs.msg import Bool

class MinimalTurtleSpawner(Node):
    def __init__(self):
        super().__init__("minimal_turtle_spawner")

        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle2/pose", self.turtle2_pose_callback, 10)
        self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback, 10)
        self.create_timer(2.0, self.spawn_turtle_once)
        self.create_timer(0.1, self.timer_callback)
        self.create_subscription(Bool, "/finished", self.finished_callback, 10)
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.spawned = False
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.turtle1_position = [5.5, 5.5]  
        self.kp_d = 1.0  
        self.kp_theta = 3.0  
        self.stop = 0
        self.finished = False

    def spawn_turtle_once(self):
        if self.spawned:
            return
        if not self.spawn_turtle_client.service_is_ready():
            return
        self.spawn_turtle()
        self.spawned = True

    def kill_turtle(self):
        kill_request = Kill.Request()
        kill_request.name = "turtle1"
        self.kill_turtle_client.call_async(kill_request)

    def turtle2_pose_callback(self, msg):
        self.turtle_pose = np.array([msg.x, msg.y, msg.theta])

    def turtle1_pose_callback(self, msg):
        self.turtle1_position = [msg.x, msg.y]

    def finished_callback(self, msg):
        self.finished = msg.data

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = 3.0
        request.y = 8.0
        request.theta = 0.0
        request.name = "turtle2"
        self.spawn_turtle_client.call_async(request)

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def control(self, target_x, target_y):
        d_x = target_x - self.turtle_pose[0]
        d_y = target_y - self.turtle_pose[1]
        d = math.sqrt(((d_x**2) + (d_y**2)))

        target = math.atan2(d_y, d_x)
        theta = target - self.turtle_pose[2]
        w = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_d * d
        wz = self.kp_theta * w
        self.cmdvel(v, wz)

        if d < 0.1 and abs(w) < 0.1:
            self.kill_turtle()
            return

    def timer_callback(self):
        if not self.spawned:
            return
        if self.finished == True:
            self.control(self.turtle1_position[0], self.turtle1_position[1])
        else :
            self.cmdvel(0.0, 0.0)
        if not self.spawned:
            return


def main():
    rclpy.init()
    node = MinimalTurtleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
