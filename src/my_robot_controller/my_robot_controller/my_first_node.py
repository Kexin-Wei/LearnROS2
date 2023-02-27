#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")  # node name
        """ use colcon build --symlink-install to avoid repeating build
        self.get_logger().info("Hello from ROS2 in my_first_node")
        self.get_logger().info("Hello again from ROS2 in my_first_node")
        self.get_logger().info("Hello third time from ROS2 in my_first_node")
        """
        self.timer = 0
        self.create_timer(
            1.0, self.timer_callback
        )  # call timer_callback every 1 second

    def timer_callback(self):
        self.get_logger().info(f"Hello, {self.timer}")
        self.timer += 1


def main(args=None):
    rclpy.init(args=args)

    # where you code
    node = MyNode()
    rclpy.spin(
        node=node
    )  # all the callbacks will keep executing, until ctrl-c to kill it

    rclpy.shutdown()


if __name__ == "__main__":
    main()
