#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial
import random


class TurtleController(Node):
    def __init__(self):
        super().__init__("teturtle_controller")
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Turtle controller has been started")
        self.time_counter_ = 0

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 0.0
        if pose.x > 9 or pose.x < 3 or pose.y > 9 or pose.y < 3:
            cmd.linear.x = 1.2
            cmd.angular.z = 0.8
        self.cmd_vel_publisher_.publish(cmd)

        self.time_counter_ += 1
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        # r = (self.time_counter_ + 0) % 255
        # g = (self.time_counter_ + 50) % 255
        # b = (self.time_counter_ + 100) % 255
        if self.time_counter_ % 30 == 0:
            self.call_set_pen_service(r, g, b, 3, 0)
        self.get_logger().info(f"time is {self.time_counter_}")
        # self.get_logger().info(f"( {pose.x:.2f}, {pose.y:.2f})")

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request=request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()
