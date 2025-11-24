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
        self.max_auto_cycles = 100

        # Safety parameters
        self.emergency_stop_active = False
        self.state_timeout = 30.0
        self.state_start_time = time.time()
        self.cycle_timer = None  # Timer for next cycle

        # Publishers
        self.state_publisher = self.create_publisher(String, "/current_state", 10)
        self.status_publisher = self.create_publisher(String, "/robot_status", 10)

        # Service server
        self.scheduler_service = self.create_service(
            Scheduler, "robot_state_server", self.scheduler_service_callback
        )

        # Service clients
        self.controller_client = self.create_client(Controller, "controller_server")
        self.random_client = self.create_client(Random, "random_pose")

        # Timers
        self.create_timer(0.1, self.state_publish_callback)
        self.create_timer(1.0, self.state_monitor_callback)
        self.create_timer(5.0, self.status_report_callback)

        self.create_subscription(
            String, "/singularity_warning", self.singularity_warning_callback, 10
        )

        self.get_logger().info("Robot Scheduler Node initialized (Fixed Loop Logic)")
        self.publish_state()

    def scheduler_service_callback(self, request, response):
        requested_state = request.state.data.upper()

        # [FIX] Handle FINISHED signal from controller
        if requested_state == "FINISHED":
            if self.current_state == RobotState.AUTO and self.auto_mode_active:
                self.get_logger().info("Task finished. Next cycle in 1s...")
                # ใช้ Timer รอ 1 วินาทีแล้วเริ่มรอบใหม่ (ไม่เปลี่ยนเป็น IDLE เพื่อรักษา auto_mode_active)
                if self.cycle_timer:
                    self.cycle_timer.cancel()
                self.cycle_timer = self.create_timer(1.0, self.trigger_next_cycle)
            else:
                self.change_state("IDLE")

            response.success = True
            return response

        self.get_logger().info(
            f"State change request: {self.current_state.value} -> {requested_state}"
        )

        if self.validate_state_transition(requested_state):
            success = self.change_state(requested_state)
            response.success = success
        else:
            response.success = False
            self.get_logger().warning(f"Invalid transition: {requested_state}")

        return response

    def trigger_next_cycle(self):
        """Trigger next auto cycle"""
        if self.cycle_timer:
            self.cycle_timer.cancel()
            self.cycle_timer = None

        if self.current_state == RobotState.AUTO and self.auto_mode_active:
            self.request_random_position()

    def validate_state_transition(self, new_state: str) -> bool:
        try:
            new_state_enum = RobotState(new_state)
        except ValueError:
            return False

        if new_state_enum == RobotState.EMERGENCY_STOP:
            return True
        if self.emergency_stop_active:
            return False
        if self.transition_in_progress:
            return False

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
                RobotState.AUTO,
            ],
            RobotState.EMERGENCY_STOP: [RobotState.IDLE],
        }

        if new_state_enum in valid_transitions.get(self.current_state, []):
            return True
        if new_state_enum == self.current_state:
            return True
        return False

    def change_state(self, new_state: str) -> bool:
        try:
            with self.state_lock:
                self.transition_in_progress = True
                self.previous_state = self.current_state
                new_state_enum = RobotState(new_state)

                self.on_state_exit(self.current_state)
                self.current_state = new_state_enum
                self.state_start_time = time.time()
                self.add_to_history(new_state_enum)
                self.on_state_enter(new_state_enum)
                self.publish_state()

                self.transition_in_progress = False
                return True
        except Exception as e:
            self.get_logger().error(f"Error during state change: {e}")
            self.transition_in_progress = False
            return False

    def on_state_exit(self, state: RobotState):
        if state == RobotState.AUTO:
            self.auto_mode_active = False
            self.auto_cycle_count = 0
            if self.cycle_timer:
                self.cycle_timer.cancel()
        elif state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_active = False

    def on_state_enter(self, state: RobotState):
        if state == RobotState.AUTO:
            self.auto_mode_active = True
            self.auto_cycle_count = 0
            self.request_random_position()
        elif state == RobotState.EMERGENCY_STOP:
            self.emergency_stop_active = True
            self.send_stop_command()
        elif state == RobotState.IDLE:
            self.auto_mode_active = False
            self.emergency_stop_active = False

    def request_random_position(self):
        if not self.random_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Random pose service not available")
            return
        request = Random.Request()
        request.mode.data = "AUTO"
        future = self.random_client.call_async(request)
        future.add_done_callback(self.random_position_callback)

    def random_position_callback(self, future):
        try:
            response = future.result()
            if response.inprogress:
                self.auto_cycle_count += 1
                self.get_logger().info(
                    f"AUTO mode cycle {self.auto_cycle_count}: Target ({response.position.x:.3f}, {response.position.y:.3f}, {response.position.z:.3f})"
                )

                # [FIX] ส่งคำสั่งไป Controller (ของเดิมหายไป)
                self.send_to_controller(response.position)

                if self.auto_cycle_count >= self.max_auto_cycles:
                    self.change_state(RobotState.IDLE.value)
            else:
                self.get_logger().warning("Failed to get random position")
        except Exception as e:
            self.get_logger().error(f"Random position callback error: {e}")

    # [ADD] ฟังก์ชันส่งคำสั่งที่หายไป
    def send_to_controller(self, pos):
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Controller service not available")
            return
        req = Controller.Request()
        req.mode.data = "AUTO"
        req.position = pos
        self.controller_client.call_async(req)

    def send_stop_command(self):
        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            return
        request = Controller.Request()
        request.mode.data = "IDLE"
        self.controller_client.call_async(request)

    def singularity_warning_callback(self, msg):
        self.get_logger().warning(f"Singularity warning: {msg.data}")
        if self.current_state in [
            RobotState.TELEOP_F,
            RobotState.TELEOP_G,
            RobotState.AUTO,
        ]:
            self.change_state(RobotState.IDLE.value)

    def state_publish_callback(self):
        self.publish_state()

    def state_monitor_callback(self):
        if self.current_state == RobotState.IDLE:
            return
        time_in_state = time.time() - self.state_start_time
        if time_in_state > self.state_timeout:
            self.get_logger().warning(f"State timeout: {self.current_state.value}")
            if self.current_state == RobotState.AUTO and self.auto_mode_active:
                self.request_random_position()
                self.state_start_time = time.time()
            else:
                self.change_state(RobotState.IDLE.value)

    def status_report_callback(self):
        pass

    def publish_state(self):
        msg = String()
        msg.data = self.current_state.value
        self.state_publisher.publish(msg)

    def add_to_history(self, state: RobotState):
        timestamp = time.time()
        self.state_history.append({"state": state.value, "timestamp": timestamp})
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
