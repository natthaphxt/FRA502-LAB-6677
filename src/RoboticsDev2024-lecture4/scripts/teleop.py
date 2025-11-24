#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from r_interfaces.srv import Controller
import sys, select, termios, tty
import threading

msg = """
---------------------------
3R Robot Teleop Keyboard Control
---------------------------
Control Modes:
  1: Inverse Kinematics Mode
  2: Teleoperation Global Frame
  3: Teleoperation End-Effector Frame
  4: Auto Mode
  0: IDLE/Stop

Movement keys (for Teleoperation):
        w
   a    s    d     (x-y plane)

   q: up (+z)
   e: down (-z)

Speed Control:
   t/g: increase/decrease linear speed by 10%
   
Enter coordinates for IK mode:
   i: Input target position for IK mode
   
CTRL-C to quit
---------------------------
Current Mode: IDLE
Current Speed: 0.80 m/s
"""


class TeleopJogKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_jog_keyboard")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.controller_client = self.create_client(Controller, "controller_server")
        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'controller_server' service...")

        self.linear_speed = 0.8  # m/s
        self.speed_increment = 0.01
        self.current_mode = "IDLE"
        self.movement_keys = {
            "w": (1, 0, 0),  # +x
            "s": (-1, 0, 0),  # -x
            "a": (0, 1, 0),  # +y
            "d": (0, -1, 0),  # -y
            "q": (0, 0, 1),  # +z
            "e": (0, 0, -1),  # -z
        }

        self.mode_keys = {
            "1": "IK",
            "2": "TELEOP_G",
            "3": "TELEOP_F",
            "4": "AUTO",
            "0": "IDLE",
        }

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Teleop Jog Keyboard started. Press keys to control robot."
        )
        print(msg)
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.start()

    def get_key(self):
        """Get keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_controller_mode(self, mode, position=None):
        """Send mode change request to controller"""
        request = Controller.Request()
        request.mode.data = mode

        if position:
            request.position.x = position[0]
            request.position.y = position[1]
            request.position.z = position[2]
        else:
            request.position.x = 0.0
            request.position.y = 0.0
            request.position.z = 0.0

        future = self.controller_client.call_async(request)
        future.add_done_callback(self.controller_response_callback)

    def controller_response_callback(self, future):
        """Handle controller response"""
        try:
            response = future.result()
            if response.inprogress:
                self.get_logger().info(
                    f"Controller mode changed successfully to {self.current_mode}"
                )
            else:
                self.get_logger().warning(f"Controller mode change failed")
        except Exception as e:
            self.get_logger().error(f"Controller service call failed: {e}")

    def get_ik_target(self):
        """Get target position for IK mode from user input"""
        print("\n--- Enter Target Position for IK Mode ---")
        try:
            x = float(input("Enter X coordinate (m): "))
            y = float(input("Enter Y coordinate (m): "))
            z = float(input("Enter Z coordinate (m): "))
            return [x, y, z]
        except ValueError:
            print("Invalid input! Using default position (0.3, 0.0, 0.3)")
            return [0.3, 0.0, 0.3]

    def get_auto_target(self):
        """Generate random target for AUTO mode"""
        import random
        import numpy as np

        r_max = 0.53
        r_min = 0.03
        l = 0.2

        while True:
            x = random.uniform(-r_max, r_max)
            y = random.uniform(-r_max, r_max)
            z = random.uniform(0, r_max + l)

            distance_squared = x**2 + y**2 + (z - l) ** 2

            if r_min**2 < distance_squared < r_max**2:
                return [x, y, z]

    def keyboard_loop(self):
        """Main keyboard control loop"""
        try:
            while self.running:
                key = self.get_key()

                if key == "\x03":  # Ctrl-C
                    self.running = False
                    break
                if key in self.mode_keys:
                    new_mode = self.mode_keys[key]
                    self.current_mode = new_mode

                    if new_mode == "IK":
                        target = self.get_ik_target()
                        self.send_controller_mode(new_mode, target)
                        print(
                            f"\nMode: {new_mode} - Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
                        )
                    elif new_mode == "AUTO":
                        target = self.get_auto_target()
                        self.send_controller_mode(new_mode, target)
                        print(
                            f"\nMode: {new_mode} - Random Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
                        )
                    else:
                        self.send_controller_mode(new_mode)
                        print(f"\nMode changed to: {new_mode}")

                elif key in self.movement_keys and self.current_mode in [
                    "TELEOP_G",
                    "TELEOP_F",
                ]:
                    twist = Twist()
                    direction = self.movement_keys[key]
                    twist.linear.x = self.linear_speed * direction[0]
                    twist.linear.y = self.linear_speed * direction[1]
                    twist.linear.z = self.linear_speed * direction[2]
                    self.cmd_vel_pub.publish(twist)

                # Speed control
                elif key == "t":
                    self.linear_speed = min(
                        0.5, self.linear_speed + self.speed_increment
                    )
                    print(f"Linear speed: {self.linear_speed:.2f} m/s")
                elif key == "g":
                    self.linear_speed = max(
                        0.01, self.linear_speed - self.speed_increment
                    )
                    print(f"Linear speed: {self.linear_speed:.2f} m/s")

                # Input IK coordinates
                elif key == "i":
                    if self.current_mode != "IK":
                        self.current_mode = "IK"
                    target = self.get_ik_target()
                    self.send_controller_mode("IK", target)
                    print(
                        f"\nIK Mode - Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
                    )

                # Stop movement when key is released in TELEOP modes
                if self.current_mode in ["TELEOP_G", "TELEOP_F"]:
                    # Send zero velocity after a short delay if no new key is pressed
                    if not select.select([sys.stdin], [], [], 0)[0]:
                        twist = Twist()
                        self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Keyboard loop error: {e}")
        finally:
            # Stop robot on exit
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)

        # Reset terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJogKeyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
