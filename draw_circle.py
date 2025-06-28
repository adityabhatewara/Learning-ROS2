#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DrawDShape(Node):
    def __init__(self):
        super().__init__("draw_d_shape")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Draw D shape node has been started!")
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Call every 0.1 sec

    def timer_callback(self):
        current_time = time.time() - self.start_time

        msg = Twist()

        if current_time < 3.0:
            # Move straight for 3 seconds
            msg.linear.x = 1.3
            msg.angular.z = 0.0
        
        elif current_time < 4.57:
            msg.angular.z = 1.0
             
        elif current_time < 7.7:
            # Then draw a half circle for 5 seconds
            msg.linear.x = 2.0
            msg.angular.z = 1.0
        else:
            # Stop after drawing
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Finished drawing D shape!")
            self.timer.cancel()

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawDShape()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
