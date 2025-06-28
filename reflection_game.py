#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
from math import pi
import numpy as np
from turtlesim.srv import Spawn  # import the Spawn service

class reflections(Node):
    def __init__(self):
        super().__init__("reflector")
        self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle2_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pose_subscriber1 = self.create_subscription(
            Pose, "/turtle1/pose",    self.pose_turtle1, 10)
        self.pose_subscriber2 = self.create_subscription(
            Pose, "/turtle2/pose",    self.pose_turtle2, 10)

        self.get_logger().info("Turtle spiral has started")
           
        self.turning = False
        self.angle_marked = False
        self.dv = 0.1
        self.turtle_pose1 = None
        self.turtle_time1 = None
        self.start = 1
        self.init_rotation_done = False
        self.last_pose1 = None
        self.last_time1 = None


    def pose_turtle1(self, pose: Pose):
        if not self.init_rotation_done:
            if not self.initial_pose(pose):
                return 
        cmd = Twist()
        if self.start == 0:
            if self.initial_pose(pose):
                self.start = 1
                
            

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

        

        self.last_pose1 = pose
        self.last_time1 = time.time()
        self.turtle1_pub.publish(cmd) 

    def pose_turtle2(self,pose: Pose):

        if self.last_pose1 is None or self.last_time1 is None:
            return  # Not ready yet
            # Access position
        cmd2 = Twist()
        
        x1 = self.last_pose1.x
        y1 = self.last_pose1.y
        theta1 = self.last_pose1.theta

        if y1 > pose.y:
            '''
            t = (y1 - pose.y)/vy1
            x_final = x1 + t*vx1

            x2 = pose.x
            cmd2.linear.x = (x_final - x2)/t

            self.turtle2_pub.publish(cmd2)
            '''
            dt = 0.1
            error_x = x1 - pose.x
            error_y = y1 - pose.y
            if abs(error_y) + abs(error_x) < 0.2 :
                cmd2.linear.x = 0.0

            else:
                cmd2.linear.x = error_x/dt
            
            self.turtle2_pub.publish(cmd2)

    
    def initial_pose(self, pose: Pose):
        cmd = Twist()

        if not self.init_rotation_done:
            angle_error = angle_diff(pi*0.4, pose.theta)
            if abs(angle_error) > 0.05:
                cmd.angular.z = 1.0 if angle_error > 0 else -1.0
                cmd.linear.x = 0.0
                self.turtle1_pub.publish(cmd)
                return  False# Don't proceed to game logic yet
            else:
                self.init_rotation_done = True
                self.get_logger().info("Initial rotation complete.")
                return  True # Let next callback handle starting game logic
            




# Inline function to spawn turtle2
def spawn_turtle2(node):
    client = node.create_client(Spawn, '/spawn')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn service...')
    request = Spawn.Request()
    request.x = 2.0
    request.y = 0.5
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
