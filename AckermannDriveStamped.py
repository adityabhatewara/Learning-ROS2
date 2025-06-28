#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannTalker(Node):
    def __init__(self):
        super().__init__("talker")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        timer_period = 0.0
        # Your variables
        self.v = 2.0  # speed in m/s
        self.d = 0.3  # steering angle in radians
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):  
        msg = AckermannDriveStamped()
        
        
        msg.drive.speed = self.v
        msg.drive.steering_angle = self.d
        self.publisher_.publish(msg)
        self.get_logger().info(f"v: {msg.drive.speed}, d: {msg.drive.steering_angle}")

class AckermannListener(Node):
    def __init__(self):
        super().__init__("Listener")
        self.pose_subscriber = self.create_subscription(
            AckermannDriveStamped, "/ackermann_cmd",   self.listener_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/drive_relay",10)

    def listener_callback(self, msg: AckermannDriveStamped):
        msg1 = AckermannDriveStamped()
        msg1.drive.speed = msg.drive.speed
        msg1.drive.steering_angle = msg.drive.steering_angle
        self.get_logger().info(f"3v: {3*msg1.drive.speed}, d: {3*msg1.drive.steering_angle}")
        self.publisher_.publish(msg1)  # Publish the modified messag


    

def main(args=None):
    rclpy.init(args=args)
    talker_node = AckermannTalker()
    listener_node = AckermannListener()
    # Executor to handle both nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(talker_node)
    executor.add_node(listener_node)

    executor.spin()
    executor.shutdown()
    talker_node.destroy_node()
    listener_node.destroy_node()
    rclpy.shutdown()
