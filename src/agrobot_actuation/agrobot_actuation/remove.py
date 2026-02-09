import time
import threading

import rclpy
from rclpy.node import Node

import pigpio
from agrobot_interfaces.srv import RemoveServo


def angle_to_pulse_us(angle_deg: float) -> int:
    # Typical servo mapping: 0..180 -> 500..2500 microseconds
    return int(500 + (angle_deg / 180.0) * 2000)


class RemoveServoServer(Node):
    def __init__(self):
        super().__init__('remove_servo_server') #name of node ros2 node list

        self.srv = self.create_service(RemoveServo, 'remove_servo', self.handle) #name of service ros2 service list

        # --- Servo config ---
        self.gpio_pin = 18
        self.angle_a = 67
        self.angle_b = 113
        self.hold_s = 0.5  # how long to hold each position

        # --- Concurrency guard ---
        self._motion_lock = threading.Lock()

        # --- pigpio connection ---
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running (pigpiod).")

        # Stop any previous pulses
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)

        self.get_logger().info("RemoveServo service ready; servo initialized.")

    def handle(self, request, response):
        # Ignore request.request intentionally
        if not self._motion_lock.acquire(blocking=False):
            response.response = "BUSY: motion already in progress"
            return response

        try:
            self.get_logger().info("Service called: executing fixed servo motion")

            # Move to angle A
            pulse_a = angle_to_pulse_us(self.angle_a)
            self.pi.set_servo_pulsewidth(self.gpio_pin, pulse_a)
            time.sleep(self.hold_s)

            # Move to angle B
            pulse_b = angle_to_pulse_us(self.angle_b)
            self.pi.set_servo_pulsewidth(self.gpio_pin, pulse_b)
            time.sleep(self.hold_s)

            # Optional: stop sending pulses (depends on whether you want it to hold)
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)

            response.response = f"OK: moved {self.angle_a}° then {self.angle_b}°"
            return response

        except Exception as e:
            response.response = f"ERROR: {e}"
            return response

        finally:
            self._motion_lock.release()

    def destroy_node(self):
        try:
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RemoveServoServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
