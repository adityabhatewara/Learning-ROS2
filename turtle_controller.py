#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pi
class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose",    self.pose_callback, 10)
        
        self.get_logger().info("Turtle controller has started")
        self.state = "Forward"
        self.initial_pose = None

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if self.initial_pose == None:
            self.initial_pose = (pose.x,pose.y)
        if self.state == "Forward":
            cmd.linear.x = 3.0
            cmd.angular.z = 0.0
            if pose.x > (self.initial_pose[0] +  3):
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.state = "Turn"
        elif self.state == "Turn":
            cmd.angular.z = 2.0
            cmd.linear.x = 0.0
            if abs(pose.theta - (pi/2.0)) < 0.1:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.state = "Circle"

        elif self.state == "Circle":
            cmd.angular.z = 0.65
            cmd.linear.x = 1.0
            if abs(pose.x - self.initial_pose[0]) + abs(pose.y - self.initial_pose[1]) < 0.1:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                self.state = "Stop"

              
           



        self.cmd_vel_pub.publish(cmd)

def main(args = None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

