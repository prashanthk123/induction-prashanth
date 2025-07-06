#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from coordinate_follower.action import CoordinateTraverse
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor

class CoordinateFollowerClientNode(Node):

    def __init__(self):
        super().__init__("coordinate_follower_client")
        self.get_logger().info("node started")
        self.index=0
        self.coordinate_follower_client_=ActionClient(
            self,
            CoordinateTraverse,
            "coordinate_follow")
        
        path=r"/home/prashanth/Desktop/ros2/src/coordinate_follower/input.txt"

        with open(path,'r') as file:
            temp=[]
            for line in file:
                x=float((line.strip().split())[0])
                y=float((line.strip().split())[1])
                temp.append((x,y))

        self.coordinates=temp
        # self.get_logger().info(str(temp))
        self.send_goal()

    def send_goal(self):
        self.coordinate_follower_client_.wait_for_server()
        goal=CoordinateTraverse.Goal()
        goal.x=self.coordinates[self.index][0]
        goal.y=self.coordinates[self.index][1]

        self.get_logger().info("Sending next goal:")

        self.coordinate_follower_client_\
            .send_goal_async(goal,feedback_callback=self.feedback_callback)\
                .add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        # self.get_logger().info("response_callback")
        self.goal_handle_:ClientGoalHandle=future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("goal rejected????")
        

    def goal_result_callback(self,future):
        result=future.result().result
        self.get_logger().info("result of coordinate " +str(self.index+1)+
                                " is "+ "success" if(result.success) else "failure")
        
        self.index+=1
        if self.index<len(self.coordinates):
            time.sleep(3)
            self.send_goal() 
        else:
            self.get_logger().info("finished")

        

    def feedback_callback(self,feedback):
        feedback_msg=feedback.feedback
        self.get_logger().info("Current coords: "+str((feedback_msg.currentx,feedback_msg.currenty))+", distance: "+str(feedback_msg.distance))


def main(args=None):
    rclpy.init(args=args)
    node=CoordinateFollowerClientNode()
    executor=MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()