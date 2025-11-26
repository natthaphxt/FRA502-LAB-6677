#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from r_interfaces.srv import Controller, Scheduler
import sys, select, termios, tty
import threading
import time

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
Current Speed: 0.50 m/s
"""


class TeleopJogKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_jog_keyboard")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Client for scheduler (for mode changes)
        self.scheduler_client = self.create_client(Scheduler, "robot_state_server")
        while not self.scheduler_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'robot_state_server' service...")

        # Client for controller (for IK target positions)
        self.controller_client = self.create_client(Controller, "controller_server")
        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'controller_server' service...")

        # Subscribe to current state from scheduler
        self.create_subscription(String, "/current_state", self.state_callback, 10)

        self.linear_speed = 0.5  # m/s
        self.speed_increment = 0.1
        self.current_mode = "IDLE"

        # Current velocity command
        self.current_twist = Twist()
        self.last_key_time = 0.0
        self.key_timeout = 0.15  # Stop if no key pressed for 150ms

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

        # Timer to continuously publish velocity (50Hz)
        self.create_timer(0.02, self.publish_velocity_callback)

        self.get_logger().info(
            "Teleop Jog Keyboard started. Press keys to control robot."
        )
        print(msg)
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.start()

    def state_callback(self, msg: String):
        """Sync with scheduler's current state"""
        if self.current_mode != msg.data:
            self.current_mode = msg.data

    def publish_velocity_callback(self):
        """Timer callback to continuously publish velocity"""
        if self.current_mode not in ["TELEOP_G", "TELEOP_F"]:
            return

        # Check if key timed out (no key pressed recently)
        if time.time() - self.last_key_time > self.key_timeout:
            # Stop movement
            if (
                self.current_twist.linear.x != 0.0
                or self.current_twist.linear.y != 0.0
                or self.current_twist.linear.z != 0.0
            ):
                self.current_twist = Twist()

        # Always publish current velocity (either movement or zero)
        self.cmd_vel_pub.publish(self.current_twist)

    def get_key(self):
        """Get keyboard input with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 50ms timeout
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_scheduler_mode(self, mode):
        """Send mode change request to scheduler"""
        request = Scheduler.Request()
        request.state.data = mode
        future = self.scheduler_client.call_async(request)
        future.add_done_callback(self.scheduler_response_callback)

    def scheduler_response_callback(self, future):
        """Handle scheduler response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Mode changed successfully to {self.current_mode}"
                )
            else:
                self.get_logger().warning(
                    f"Mode change to {self.current_mode} failed (invalid transition)"
                )
        except Exception as e:
            self.get_logger().error(f"Scheduler service call failed: {e}")

    def send_ik_target(self, position):
        """Send IK target position to controller"""
        request = Controller.Request()
        request.mode.data = "IK"
        request.position.x = position[0]
        request.position.y = position[1]
        request.position.z = position[2]
        future = self.controller_client.call_async(request)
        future.add_done_callback(self.controller_response_callback)

    def controller_response_callback(self, future):
        """Handle controller response for IK"""
        try:
            response = future.result()
            if response.inprogress:
                self.get_logger().info("IK target accepted")
            else:
                self.get_logger().warning("IK target rejected (unreachable or unsafe)")
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

    def keyboard_loop(self):
        """Main keyboard control loop"""
        try:
            while self.running:
                key = self.get_key()

                if key == "\x03":  # Ctrl-C
                    self.running = False
                    break

                if key == "":
                    continue  # No key pressed, continue loop

                # Mode change keys
                if key in self.mode_keys:
                    new_mode = self.mode_keys[key]

                    # Stop any current movement
                    self.current_twist = Twist()

                    if new_mode == "IK":
                        self.send_scheduler_mode(new_mode)
                        self.current_mode = new_mode
                        target = self.get_ik_target()
                        self.send_ik_target(target)
                        print(
                            f"\nMode: {new_mode} - Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
                        )

                    elif new_mode == "AUTO":
                        self.send_scheduler_mode(new_mode)
                        self.current_mode = new_mode
                        print(
                            f"\nMode changed to: {new_mode} (scheduler will generate targets)"
                        )

                    else:
                        self.send_scheduler_mode(new_mode)
                        self.current_mode = new_mode
                        print(f"\nMode changed to: {new_mode}")

                # Movement keys (only in TELEOP modes)
                elif key in self.movement_keys and self.current_mode in [
                    "TELEOP_G",
                    "TELEOP_F",
                ]:
                    direction = self.movement_keys[key]
                    self.current_twist.linear.x = self.linear_speed * direction[0]
                    self.current_twist.linear.y = self.linear_speed * direction[1]
                    self.current_twist.linear.z = self.linear_speed * direction[2]
                    self.last_key_time = time.time()

                # Speed control
                elif key == "t":
                    self.linear_speed = min(2.0, self.linear_speed * 1.1)
                    print(f"Linear speed: {self.linear_speed:.2f} m/s")
                elif key == "g":
                    self.linear_speed = max(0.01, self.linear_speed * 0.9)
                    print(f"Linear speed: {self.linear_speed:.2f} m/s")

                # Input IK coordinates (shortcut)
                elif key == "i":
                    self.current_twist = Twist()  # Stop movement
                    if self.current_mode != "IK":
                        self.send_scheduler_mode("IK")
                        self.current_mode = "IK"
                    target = self.get_ik_target()
                    self.send_ik_target(target)
                    print(
                        f"\nIK Mode - Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})"
                    )

        except Exception as e:
            self.get_logger().error(f"Keyboard loop error: {e}")
        finally:
            # Stop robot on exit
            self.current_twist = Twist()
            self.cmd_vel_pub.publish(self.current_twist)
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
