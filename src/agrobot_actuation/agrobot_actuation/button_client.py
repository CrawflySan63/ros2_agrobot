#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import Button
from agrobot_interfaces.srv import MoveServo


class ButtonClient(Node):
    def __init__(self):
        super().__init__('button_client')

        # Service clients
        self.remove_client = self.create_client(MoveServo, 'remove_servo')
        self.seed_client = self.create_client(MoveServo, 'seed_servo')

        self.get_logger().info('Waiting for remove_servo service...')
        self.remove_client.wait_for_service()

        self.get_logger().info('Waiting for seed_servo service...')
        self.seed_client.wait_for_service()

        self.get_logger().info('Both services are available.')

        # Buttons
        self.remove_button = Button(17, pull_up=True, bounce_time=0.1)
        self.seed_button = Button(27, pull_up=True, bounce_time=0.1)

        self.remove_button.when_pressed = self.call_remove
        self.seed_button.when_pressed = self.call_seed

        self.get_logger().info('Button client ready.')

    def call_remove(self):
        self.get_logger().info('Remove button pressed')
        req = MoveServo.Request()
        req.request = 'remove'
        future = self.remove_client.call_async(req)
        future.add_done_callback(self.remove_callback)

    def call_seed(self):
        self.get_logger().info('Seed button pressed')
        req = MoveServo.Request()
        req.request = 'seed'
        future = self.seed_client.call_async(req)
        future.add_done_callback(self.seed_callback)

    def remove_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Remove response: {response.response}")
        except Exception as e:
            self.get_logger().error(f"Remove service call failed: {e}")

    def seed_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Seed response: {response.response}")
        except Exception as e:
            self.get_logger().error(f"Seed service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ButtonClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
