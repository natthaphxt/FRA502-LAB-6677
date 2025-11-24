#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from r_interfaces.srv import Scheduler, Controller, Random
from enum import Enum
import threading
import time


class RobotState(Enum):
    """Robot state enumeration"""

    IDLE = "IDLE"
    IK = "IK"
    TELEOP_F = "TELEOP_F"
    TELEOP_G = "TELEOP_G"
    AUTO = "AUTO"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class RobotSchedulerNode(Node):
    def __init__(self):
        super().__init__("robot_scheduler_node")

        # State management
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        self.state_lock = threading.Lock()
        self.transition_in_progress = False

        # State history for debugging
        self.state_history = []
        self.max_history_size = 100

        # AUTO mode management
        self.auto_mode_active = False
        self.auto_cycle_count = 0
        self.max_auto_cycles = 100  # Prevent infinite loops

        # Safety parameters
        self.emergency_stop_active = False
        self.state_timeout = 30.0  # Maximum time in any state except IDLE
        self.state_start_time = time.time()

        # Publishers
        self.state_publisher = self.create_publisher(String, "/current_state", 10)
        self.status_publisher = self.create_publisher(String, "/robot_status", 10)

        # Service server
        self.scheduler_service = self.create_service(
            Scheduler, "robot_state_server", self.scheduler_service_callback
        )

        # Service clients for coordination
        self.controller_client = self.create_client(Controller, "controller_server")
        self.random_client = self.create_client(Random, "random_pose")

        # Timers
        self.create_timer(0.1, self.state_publish_callback)  # Publish state at 10Hz
        self.create_timer(
            1.0, self.state_monitor_callback
        )  # Monitor state health at 1Hz
        self.create_timer(5.0, self.status_report_callback)  # Status report at 0.2Hz

        # Subscribers for safety monitoring
        self.create_subscription(
            String, "/singularity_warning", self.singularity_warning_callback, 10
        )

        # Initialize state
        self.get_logger().info("Robot Scheduler Node initialized")
        self.get_logger().info(f"Initial state: {self.current_state.value}")
        self.publish_state()

    def scheduler_service_callback(
        self, request: Scheduler.Request, response: Scheduler.Response
    ):
        """Handle state change requests"""
        requested_state = request.state.data.upper()

        self.get_logger().info(
            f"State change request: {self.current_state.value} -> {requested_state}"
        )

        # Check if state transition is valid
        if self.validate_state_transition(requested_state):
            success = self.change_state(requested_state)
            response.success = success

            if success:
                self.get_logger().info(f"State changed to: {self.current_state.value}")
            else:
                self.get_logger().warning(
                    f"Failed to change state to: {requested_state}"
                )
        else:
            response.success = False
            self.get_logger().warning(
                f"Invalid state transition: {self.current_state.value} -> {requested_state}"
            )

        return response

    def validate_state_transition(self, new_state: str) -> bool:
        """Validate if state transition is allowed"""
        # Convert string to RobotState
        try:
            new_state_enum = RobotState(new_state)
        except ValueError:
            self.get_logger().error(f"Unknown state: {new_state}")
            return False

        # Emergency stop can be activated from any state
        if new_state_enum == RobotState.EMERGENCY_STOP:
            return True

        # Cannot transition if emergency stop is active
        if self.emergency_stop_active:
            self.get_logger().warning(
                "Cannot change state while emergency stop is active"
            )
            return False

        # Cannot transition if another transition is in progress
        if self.transition_in_progress:
            self.get_logger().warning("Another state transition is in progress")
            return False

        # Define valid transitions
        valid_transitions = {
            RobotState.IDLE: [
                RobotState.IK,
                RobotState.TELEOP_F,
                RobotState.TELEOP_G,
                RobotState.AUTO,
            ],
            RobotState.IK: [RobotState.IDLE, RobotState.EMERGENCY_STOP],
            RobotState.TELEOP_F: [
                RobotState.IDLE,
                RobotState.TELEOP_G,
                RobotState.EMERGENCY_STOP,
            ],
            RobotState.TELEOP_G: [
                RobotState.IDLE,
                RobotState.TELEOP_F,
                RobotState.EMERGENCY_STOP,
            ],
            RobotState.AUTO: [
                RobotState.IDLE,
                RobotState.EMERGENCY_STOP,
                RobotState.AUTO,  # Allow AUTO to AUTO for continuous operation
            ],
            RobotState.EMERGENCY_STOP: [RobotState.IDLE],
        }

        # Check if transition is valid
        if new_state_enum in valid_transitions.get(self.current_state, []):
            return True

        # Allow transition to same state (refresh)
        if new_state_enum == self.current_state:
            return True

        return False

    def change_state(self, new_state: str) -> bool:
        """Execute state change"""
        try:
            with self.state_lock:
                self.transition_in_progress = True

                # Store previous state
                self.previous_state = self.current_state

                # Convert string to enum
                new_state_enum = RobotState(new_state)

                # Execute state exit actions
                self.on_state_exit(self.current_state)

                # Change state
                self.current_state = new_state_enum
                self.state_start_time = time.time()

                # Add to history
                self.add_to_history(new_state_enum)

                # Execute state entry actions
                self.on_state_enter(new_state_enum)

                # Publish new state
                self.publish_state()

                self.transition_in_progress = False
                return True

        except Exception as e:
            self.get_logger().error(f"Error during state change: {e}")
            self.transition_in_progress = False
            return False

    def on_state_exit(self, state: RobotState):
        """Execute actions when exiting a state"""
        self.get_logger().debug(f"Exiting state: {state.value}")

        if state == RobotState.AUTO:
            self.auto_mode_active = False
            self.auto_cycle_count = 0
        elif state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_active = False

    def on_state_enter(self, state: RobotState):
        """Execute actions when entering a state"""
        self.get_logger().debug(f"Entering state: {state.value}")

        if state == RobotState.AUTO:
            self.auto_mode_active = True
            self.auto_cycle_count = 0
            # Request first random position
            self.request_random_position()
        elif state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_active = True
            self.send_stop_command()
        elif state == RobotState.IDLE:
            # Reset any active modes
            self.auto_mode_active = False
            self.emergency_stop_active = False

    def request_random_position(self):
        """Request a random position for AUTO mode"""
        if not self.random_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Random pose service not available")
            return

        request = Random.Request()
        request.mode.data = "AUTO"

        future = self.random_client.call_async(request)
        future.add_done_callback(self.random_position_callback)

    def random_position_callback(self, future):
        """Handle random position response"""
        try:
            response = future.result()
            if response.inprogress:
                self.auto_cycle_count += 1
                self.get_logger().info(
                    f"AUTO mode cycle {self.auto_cycle_count}: "
                    f"Target ({response.position.x:.3f}, "
                    f"{response.position.y:.3f}, {response.position.z:.3f})"
                )

                # Check if we've reached max cycles
                if self.auto_cycle_count >= self.max_auto_cycles:
                    self.get_logger().warning(
                        "Maximum AUTO cycles reached, switching to IDLE"
                    )
                    self.change_state(RobotState.IDLE.value)
            else:
                self.get_logger().warning("Failed to get random position")

        except Exception as e:
            self.get_logger().error(f"Random position callback error: {e}")

    def send_stop_command(self):
        """Send stop command to controller"""
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                "Controller service not available for stop command"
            )
            return

        request = Controller.Request()
        request.mode.data = "IDLE"
        request.position.x = 0.0
        request.position.y = 0.0
        request.position.z = 0.0

        future = self.controller_client.call_async(request)
        self.get_logger().info("Emergency stop command sent to controller")

    def singularity_warning_callback(self, msg: String):
        """Handle singularity warnings"""
        self.get_logger().warning(f"Singularity warning received: {msg.data}")

        # If in a movement mode, consider switching to IDLE for safety
        if self.current_state in [
            RobotState.TELEOP_F,
            RobotState.TELEOP_G,
            RobotState.AUTO,
        ]:
            self.get_logger().warning(
                "Singularity detected, switching to IDLE for safety"
            )
            self.change_state(RobotState.IDLE.value)

    def state_publish_callback(self):
        """Periodically publish current state"""
        self.publish_state()

    def state_monitor_callback(self):
        """Monitor state health and timeouts"""
        if self.current_state == RobotState.IDLE:
            return  # No timeout for IDLE

        # Check for state timeout
        time_in_state = time.time() - self.state_start_time

        if time_in_state > self.state_timeout and self.current_state != RobotState.IDLE:
            self.get_logger().warning(
                f"State timeout: {self.current_state.value} "
                f"({time_in_state:.1f}s > {self.state_timeout}s)"
            )

            # AUTO mode gets special handling for continuous operation
            if self.current_state == RobotState.AUTO and self.auto_mode_active:
                # Request new position instead of switching to IDLE
                self.get_logger().info("AUTO mode timeout, requesting new position")
                self.request_random_position()
                self.state_start_time = time.time()  # Reset timer
            else:
                self.change_state(RobotState.IDLE.value)

    def status_report_callback(self):
        """Publish detailed status report"""
        status_msg = String()

        time_in_state = time.time() - self.state_start_time

        status_data = {
            "current_state": self.current_state.value,
            "previous_state": self.previous_state.value,
            "time_in_state": f"{time_in_state:.1f}s",
            "auto_cycles": self.auto_cycle_count if self.auto_mode_active else 0,
            "emergency_stop": self.emergency_stop_active,
            "transition_in_progress": self.transition_in_progress,
        }

        status_msg.data = str(status_data)
        self.status_publisher.publish(status_msg)

    def publish_state(self):
        """Publish current state"""
        msg = String()
        msg.data = self.current_state.value
        self.state_publisher.publish(msg)

    def add_to_history(self, state: RobotState):
        """Add state to history"""
        timestamp = time.time()
        self.state_history.append(
            {
                "state": state.value,
                "timestamp": timestamp,
                "time_str": time.strftime("%H:%M:%S", time.localtime(timestamp)),
            }
        )

        # Limit history size
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)

    def get_state_history(self):
        """Get state history for debugging"""
        return self.state_history

    def emergency_stop(self):
        """Trigger emergency stop"""
        self.get_logger().warning("EMERGENCY STOP ACTIVATED")
        self.change_state(RobotState.EMERGENCY_STOP.value)

    def reset(self):
        """Reset scheduler to initial state"""
        self.get_logger().info("Resetting scheduler to IDLE")
        with self.state_lock:
            self.current_state = RobotState.IDLE
            self.previous_state = RobotState.IDLE
            self.auto_mode_active = False
            self.auto_cycle_count = 0
            self.emergency_stop_active = False
            self.transition_in_progress = False
            self.state_start_time = time.time()
            self.state_history.clear()

        self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSchedulerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Scheduler shutdown requested")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
