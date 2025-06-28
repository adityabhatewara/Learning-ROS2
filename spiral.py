#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
from math import pi
class Spiral(Node):

    def __init__(self):
        super().__init__("spiral")
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose",    self.pose_callback, 10)
        
        self.get_logger().info("Turtle spiral has started")
        self.state = "spiral"
        self.initial_pose = None
        self.dv = 0.1
        self.start_time = time.time()
        self.turning = False
        self.angle_marked = False

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if self.state == "start":
            cmd.linear.x = 1.0
            self.state = "spiral"
        elif self.state == "spiral":
            if not near_wall(pose):
                cmd.angular.z = 1.0
                cmd.linear.x = 1.0  + self.dv * (time.time() - self.start_time) 

            else:
                self.state = "Square"

        elif self.state == "Square":
            if self.turning == False and self.angle_marked == False:
                self.theta_before_turning = pose.theta
                self.angle_marked = True

      
            if near_wall(pose):
                wall_hit = which_wall(pose)
                self.turning = True
                if wall_hit == "R" or wall_hit == "L":
                    reflected_angle = pi-self.theta_before_turning
                    reflected_angle = (reflected_angle + pi) % (2 * pi) - pi
                elif wall_hit == "U" or wall_hit == "D":
                    reflected_angle = -self.theta_before_turning 
                    reflected_angle = (reflected_angle + pi) % (2 * pi) - pi

                cmd.angular.z = 1.0 if angle_diff(reflected_angle,pose.theta)> 0 else -1.0
                cmd.linear.x = 0.0
              #  cmd.angular.z = 1.0 if angle_diff(reflected_angle,pose.theta) > 0 else -1.0
                #cmd.angular.z = 1.0
                '''if abs(angle_diff(pose.theta, self.theta_before_turning + pi/2)) < 0.1:
                    self.turning = False   
                    self.angle_marked = True
                    cmd.linear.x = 1.0
                    cmd.angular.z = 0.0'''
                if wall_hit == "R" or wall_hit == "L":
                    reflected_angle = pi-self.theta_before_turning
                    reflected_angle = (reflected_angle + pi) % (2 * pi) - pi
                    if abs(angle_diff(reflected_angle,pose.theta)) < 0.1:
                        self.turning = False   
                        self.angle_marked = True
                        cmd.linear.x = 1.0
                        cmd.angular.z = 0.0
                elif wall_hit == "U" or wall_hit == "D":
                    reflected_angle = -self.theta_before_turning 
                    reflected_angle = (reflected_angle + pi) % (2 * pi) - pi
                    if abs(angle_diff(reflected_angle,pose.theta)) < 0.1:
                        self.turning = False   
                        self.angle_marked = True
                        cmd.linear.x = 1.0
                        cmd.angular.z = 0.0

            else:
                self.angle_marked = False
                cmd.linear.x = 3.0
                cmd.angular.z = 0.0

            


        self.cmd_vel_pub.publish(cmd)
        




def main(args = None):
    rclpy.init(args=args)
    node = Spiral()
    rclpy.spin(node)
    rclpy.shutdown()

def angle_diff(a, b):
    return (a - b + pi) % (2 * pi) - pi

def near_wall(pose):
    bool = pose.x <= 0.5 or pose.x >= 10.5 or pose.y <= 0.5 or pose.y >= 10.5

    return bool
def which_wall(pose):
    if pose.x <= 0.5:
        return "L"
    elif pose.y <= 0.5:
        return "D"
    elif pose.x >= 10.5:
        return "R"
    elif pose.y >= 10.5:
        return "U"
    else:
        return None
