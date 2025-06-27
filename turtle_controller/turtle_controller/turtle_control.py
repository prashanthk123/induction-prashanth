#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class drawCircle(Node):

    def __init__(self):
        super().__init__("turtle_control")
        self.cmd_vel_pub_=self.create_publisher(Twist,"/cmd_vel",10)
        self.timer_=self.create_timer(0.5,self.send_vel)
    
    def send_vel(self):
        # self.get_logger().info("hi")
        msg=Twist()
        msg.linear.x=0.1
        msg.angular.z=0.5
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node1=drawCircle()
    rclpy.spin(node1)
    rclpy.shutdown()
