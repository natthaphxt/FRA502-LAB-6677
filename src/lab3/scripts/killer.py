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
from std_msgs.msg import Bool,Int64,Float64
from controller_interfaces.srv import SetParam

class MinimalTurtleSpawner(Node):
    def __init__(self):
        super().__init__("minimal_turtle_spawner")
        self.declare_parameter("sampling_frequency", 5.0)
        self.rates = (self.get_parameter("sampling_frequency").get_parameter_value().double_value)

        self.name2 = self.get_namespace()
        self.declare_parameter("target_turtle_name")
        self.name1 = (
            self.get_parameter("target_turtle_name").get_parameter_value().string_value
        )

        self.kp_linear = 1.0
        self.kp_angular = 3.0
        # create service sever for set cmd
        self.set_kp_sever = self.create_service(SetParam, self.name2+"/set_kp_kill", self.set_kp_callback)

        self.declare_parameter("spawn_x", 2.0)
        self.spawn_x = self.get_parameter("spawn_x").get_parameter_value().double_value
        self.declare_parameter("spawn_y",2.0)
        self.spawn_y = self.get_parameter("spawn_y").get_parameter_value().double_value
        self.declare_parameter("spawn_theta",0.0)
        self.spawn_theta = self.get_parameter("spawn_theta").get_parameter_value().double_value
        self.declare_parameter("turtle_name", "turtle2")
        self.turtle_name = (
            self.get_parameter("turtle_name").get_parameter_value().string_value
        )
        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.cmd_vel_pub = self.create_publisher(Twist, self.name2 +"/cmd_vel", 10)
        self.create_subscription(Pose, self.name2+"/pose", self.turtle2_pose_callback, 10)
        self.create_subscription(Pose, "/"+self.name1+"/pose", self.turtle1_pose_callback, 10)
        self.create_timer(2.0, self.spawn_turtle_once)
        self.create_timer(1.0/self.rates, self.timer_callback)
        self.get_logger().info(f'Starting {self.get_namespace}')
        self.create_subscription(Bool, "/"+self.name1+"/finished", self.finished_callback, 10)
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.spawned = False
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])
        self.turtle1_position = [5.5, 5.5]  
        self.kp_d = 1.0  
        self.kp_theta = 3.0  
        self.stop = 0
        self.finished = False
        self.start = 0

    def finished_callback(self,msg):
        self.start = msg.data

    def set_kp_callback(self, request: SetParam.Request, response: SetParam.Response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        return response

    def spawn_turtle_once(self):
        if self.spawned:
            return
        if not self.spawn_turtle_client.service_is_ready():
            return
        self.spawn_turtle()
        self.spawned = True

    def kill_turtle(self):
        kill_request = Kill.Request()
        kill_request.name = "/"+self.name1
        self.kill_turtle_client.call_async(kill_request)

    def turtle2_pose_callback(self, msg):
        self.turtle_pose = np.array([msg.x, msg.y, msg.theta])

    def turtle1_pose_callback(self, msg):
        self.turtle1_position = [msg.x, msg.y]

    def finished_callback(self, msg):
        self.finished = msg.data

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = self.spawn_x
        request.y = self.spawn_y
        request.theta = self.spawn_theta
        request.name = self.name2
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

        v = self.kp_linear * d
        wz = self.kp_angular * w
        self.cmdvel(v, wz)

        if d < 0.1 and abs(w) < 0.1:
            self.kill_turtle()
            return

    def timer_callback(self):
        # self.get_logger().info(self.name2)
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
