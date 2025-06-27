#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
class avoid_obstacle(Node):
    
    def __init__(self):
        super().__init__("avoid_obstacle")

        self.start_angle=math.radians(-30) 
        self.end_angle = math.radians(30)
        self.threshold_distance=0.5
        self.scan_sub_=self.create_subscription(LaserScan,"/scan",self.callback,10)
        self.cmd_vel_pub_=self.create_publisher(Twist,"/cmd_vel",10)
    
    def callback(self,msg: LaserScan):

        cmd_vel=Twist()

        #start=int(self.start_angle/msg.angle_increment)
        start=int((self.start_angle-msg.angle_min)/msg.angle_increment)
        end=int((self.end_angle-msg.angle_min)/msg.angle_increment)
        flag=True
        for i in range(start,end):
            # self.get_logger().info(str(msg.ranges[i]))
            if msg.ranges[i]<self.threshold_distance:
                flag=False
                break

        if flag:
            cmd_vel.linear.x=0.1
            cmd_vel.angular.z=0.0
        else:
            cmd_vel.linear.x=0.0
            cmd_vel.angular.z=-0.5
        
        self.cmd_vel_pub_.publish(cmd_vel)
        # self.get_logger().info(str(msg.angle_increment))
        # self.get_logger().info(str(msg.ranges[0]))
        # self.get_logger().info(str(msg.angle_max))

        return
  

def main(args=None):
    rclpy.init(args=args)
    avoid=avoid_obstacle()
    rclpy.spin(avoid)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


