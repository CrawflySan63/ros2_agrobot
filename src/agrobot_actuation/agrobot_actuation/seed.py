import time
import threading

import rclpy
from rclpy.node import Node

from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory

from agrobot_interfaces.srv import RemoveServo


def angle_to_pulse_us(angle_deg: float) -> int:
    # Typical servo mapping: 0..180 -> 500..2500 microseconds
    return int(500 + (angle_deg / 180.0) * 2000)


class SeedServoServer(Node):
    def __init__(self):
        super().__init__('seed_servo_server') #name of node ros2 node list

        self.srv = self.create_service(RemoveServo, 'seed_servo', self.handle_seed_servo) # name of service ros2 service list
        # --- Servo config ---
        self.gpio_pin = 12
        self.angle_a = 67
        self.angle_b = 113
        self.hold_s = 0.5  # how long to hold each position

        # --- Concurrency guard ---
        self._motion_lock = threading.Lock()

        # --- gpiozero / lgpio setup ---
        factory = LGPIOFactory()
        self.servo = AngularServo(
            self.gpio_pin,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,   # 0.5 ms
            max_pulse_width=0.0025,   # 2.5 ms
            pin_factory=factory
        )

        # Optional: start detached (not driving)
        self.servo.detach()

        self.get_logger().info("SeedServo service ready; servo initialized.")   

    def handle_seed_servo(self, request, response):
        # Ignore request.request intentionally
        if not self._motion_lock.acquire(blocking=False):
            response.response = "BUSY: motion already in progress"
            return response

        try:
            self.get_logger().info("Service called: executing fixed servo motion")

            # Move to angle A
            self.servo.angle = float(self.angle_a)
            time.sleep(self.hold_s)

            # Move to angle B
            self.servo.angle = float(self.angle_b)
            time.sleep(self.hold_s)

            # Optional: stop sending pulses (depends on whether you want it to hold)
            self.servo.detach()

            response.response = f"OK: moved {self.angle_a}° then {self.angle_b}°"
            return response

        except Exception as e:
            response.response = f"ERROR: {e}"
            return response

        finally:
            self._motion_lock.release()

    def destroy_node(self):
        try:
            self.servo.detach()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SeedServoServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
