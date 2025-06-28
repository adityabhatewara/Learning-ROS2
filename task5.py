#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pi
import numpy as np
from turtlesim.srv import Spawn  # import the Spawn service

X = 0.0
Y = 0.0

class reflections(Node):
    def __init__(self):
        super().__init__("reflector")
        self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber1 = self.create_subscription(
            Pose, "/turtle1/pose",    self.pose_turtle1, 10)
        self.reached = False
        

    def pose_turtle1(self, pose: Pose):
        cmd = Twist()
        dist = abs_dist(pose)
        cmd.linear.x = 0.4 * (dist)
        angle_to_goal = np.arctan2(Y - pose.y, X - pose.x)
        angle_diff = angle_to_goal - pose.theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff)) 
        cmd.angular.z = angle_diff

        if dist < 0.025:
            twist = Twist()
            self.turtle1_pub.publish(twist) 
            if not self.reached:
                print(dist)
                self.reached = True
            return
        print(dist)
        

        self.turtle1_pub.publish(cmd) 


def abs_dist(pose):
    return np.sqrt((X- pose.x)**2 + (Y - pose.y)**2)
# Inline function to spawn turtle2
def spawn_turtle2(node):
    global X,Y
    client = node.create_client(Spawn, '/spawn')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn service...')
    request = Spawn.Request()
    X = float(input("Enter x coordinate of new turtle: "))
    Y = float(input("Enter y coordinate of new turtle: "))
    request.x = X
    request.y = Y
    request.theta = 0.0
    request.name = 'turtle2'
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Spawned turtle: {future.result().name}")
    else:
        node.get_logger().error("Failed to spawn turtle2")

def main(args = None):
    rclpy.init(args=args)
    node = reflections()
    spawn_turtle2(node)
    
    rclpy.spin(node)
    rclpy.shutdown()
