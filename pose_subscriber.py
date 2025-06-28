#!/usr/bin/env pythjon3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubsciberNode(Node):

    def __init__(self):
        super().__init__("pose_subsciber")
        self.pose_subscriber = self.create_subscription(
         Pose, "/turtle1/pose",    self.pose_callback, 10
        )

    def pose_callback(self, msg: Pose):

        self.get_logger().info("(" + str(msg.x)+"," + str(msg.y)+")")

def main(args = None):
    rclpy.init(args=args)
    node = PoseSubsciberNode()
    rclpy.spin(node)
    rclpy.shutdown()