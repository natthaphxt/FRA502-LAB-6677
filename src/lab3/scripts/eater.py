#!/usr/bin/python3

import sys
from turtle_pkg.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, TransformStamped, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Bool,Int64,String,Float64
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from controller_interfaces.srv import SetParam 
from controller_interfaces.srv import SetMaxPizza
import numpy as np
import math
from turtlesim.srv import Kill
from turtlesim.srv import Spawn

class eater(Node):
    def __init__(self):
        super().__init__("eater")
        self.names = self.get_namespace()
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Twist, self.names + "/cmd_vel", 10)
        self.publisher_text = self.create_publisher(Bool, self.names + "/finished", 10)
        self.create_subscription(Pose, self.names + "/pose", self.pose_callback, 10)
        self.create_subscription(Point, "/mouse_position", self.mouse_callback, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, self.names+"/eat")
        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.create_timer(2.0, self.kill_turtle_once)
        self.create_timer(2.0, self.spawn_turtle_once)
        self.declare_parameter("sampling_frequency", 5.0)
        self.rate = (
            self.get_parameter("sampling_frequency").get_parameter_value().double_value
        )
        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.kp_linear = 3.0
        self.kp_angular = 4.0
        # create service sever for set cmd
        self.set_kp_sever = self.create_service(SetParam, '/set_kp',self.set_kp_callback)
        self.names = self.get_namespace()
        self.set_pizza_sever = self.create_service(SetMaxPizza, '/set_maxpizza',self.set_maxpizza_callback)
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)
        self.get_logger().info(f"node has been started {self.get_name()} with default param.v: {self.kp_linear}, param.w: {self.kp_angular}")
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
        self.kp_d = 3.0
        self.kp_theta = 4.0
        self.x = 0
        self.max_pizza = 0
        self.spawn = False
        self.spawned = False
        self.kill = False

    def spawn_turtle_once(self):
        if self.spawned:
            return
        if not self.spawn_turtle_client.service_is_ready():
            return
        self.spawn_turtle()
        self.spawned = True

    def kill_turtle(self):
        kill = Kill.Request()
        kill.name = 'turtle1'
        self.kill_turtle_client.call_async(kill)

    def spawn_turtle(self):
        spawn_request = Spawn.Request()
        spawn_request.x = 5.5  
        spawn_request.y = 5.5  
        spawn_request.theta = 0.0  
        spawn_request.name = self.names
        self.spawn_turtle_client.call_async(spawn_request)

    def kill_turtle_once(self):
        if self.kill:
            return
        if not self.kill_turtle_client.service_is_ready():
            return
        self.kill_turtle()
        self.kill = True

    def set_maxpizza_callback(self, request:SetMaxPizza.Request, response:SetMaxPizza.Response):
        msg = Bool()
        msg.data = False
        self.publisher_text.publish(msg)
        self.max_pizza = request.max_pizza.data
        response.log.data = f"set pizza to {self.max_pizza}"
        return response

    def set_kp_callback(self, request: SetParam.Request, response: SetParam.Response):
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        return response

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
        if self.pizza <= self.max_pizza- 1:
            self.spawn_pizza_callback(self.mouse_pos[0], self.mouse_pos[1])
            self.pizza += 1
            self.pizza_queue.append([self.mouse_pos[0], self.mouse_pos[1]])
            self.pizza_count = len(self.pizza_queue)
        self.get_logger().info(f"Received mouse position: {self.pizza}")
        self.get_logger().info(f"Pizza All cleared: {self.pizza}")

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
        if self.count < self.max_pizza:
            d_x = x - self.turtle_pose[0]
            d_y = y - self.turtle_pose[1]
            d = math.sqrt(((d_x**2) + (d_y**2))) 

            tan = math.atan2(d_y, d_x)
            theta = tan - self.turtle_pose[2]
            error = math.atan2(math.sin(theta), math.cos(theta))

            self.v = self.kp_linear * d
            self.w = self.kp_angular * error
            self.cmdvel_callback(self.v, self.w)

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

            v = self.kp_linear * d
            wz = self.kp_angular * w
            self.cmdvel_callback(v, wz)

    def timer_callback(self):
        if self.count < self.max_pizza:
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
                    # self.get_logger().info("All pizza eaten")
                    self.cmdvel_callback(0.0, 0.0)
                    self.x += 1
        elif(self.count != 0 and self.count >= self.max_pizza):
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
