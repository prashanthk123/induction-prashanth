#!/usr/bin/env python3

import time
import math
import asyncio
import rclpy

from rclpy.node import Node
from coordinate_follower.action import CoordinateTraverse
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


subscriber_callback_group=ReentrantCallbackGroup()
class CoordinateFollowerServerNode(Node):
    
    def __init__(self):
        
        super().__init__("coordinate_follower_server")
        
        self.currentx=0.0
        self.currenty=0.0
        self.targetx=0.0
        self.targety=0.0
        self.yaw=0        
        self.callback_group=ReentrantCallbackGroup()

        self.cmd_vel_pub_=self.create_publisher(Twist,"/cmd_vel",10)

        self.pos_subscriber=self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=subscriber_callback_group)

        self.coordinate_follower_server_=ActionServer(
            self,   
            CoordinateTraverse,
            "coordinate_follow",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group)
        
        self.get_logger().info("Action Server started")



    def odom_callback(self,msg:Odometry):
        
        self.currentx=msg.pose.pose.position.x
        self.currenty=msg.pose.pose.position.y
        q=msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    async def execute_callback(self,goal_handle:ServerGoalHandle):
        #get request from goal
        self.targetx=goal_handle.request.x
        self.targety=goal_handle.request.y
        cmd_vel=Twist()

        self.get_logger().info("Received goal request with target coords: "+str((self.targetx,self.targety)))


        #execution
        #self.get_logger().info("executing the goal")

        while rclpy.ok():

            feedback_msg=CoordinateTraverse.Feedback()
            dx=self.targetx-self.currentx
            dy=self.targety-self.currenty
            distance=math.sqrt(dx**2+dy**2)
            angle=math.atan2(dy,dx)

            # rclpy.spin_once(self,timeout_sec=0.1)

            # self.get_logger().info("the target is: "+str((self.targetx,self.targety,angle)))
            # self.get_logger().info("the current is: "+str((self.currentx,self.currenty,self.yaw)))

            if distance<0.01: break
            if abs(angle-self.yaw)>0.1:
                cmd_vel.linear.x=0.0
                cmd_vel.angular.z=0.2*(angle-self.yaw)
            else:   
                cmd_vel.linear.x=min(0.5,10*distance)
                cmd_vel.angular.z=0.0
            
            feedback_msg.currentx=self.currentx
            feedback_msg.currenty=self.currenty
            feedback_msg.distance=distance
            goal_handle.publish_feedback(feedback_msg)

            self.cmd_vel_pub_.publish(cmd_vel)
            #rclpy.spin_once(self,timeout_sec=0.1)

        #once done
        cmd_vel.linear.x=0.0
        cmd_vel.angular.z=0.0
        self.cmd_vel_pub_.publish(cmd_vel)
        goal_handle.succeed()
        result=CoordinateTraverse.Result()
        result.success=True
        self.get_logger().info("success")
        return result   
    

def main(args=None):
    rclpy.init(args=args)
    node=CoordinateFollowerServerNode()
    executor=MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()


