#!/usr/bin/env python3
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan

cam_info_callback_group=MutuallyExclusiveCallbackGroup()
odom_callback_group=MutuallyExclusiveCallbackGroup()
camera_callback_group=MutuallyExclusiveCallbackGroup()

#H:1080*W:1920
class ArucoNavigator(Node):

    def __init__(self):
        super().__init__("aruco_navigator")
        self.camera_width=1920
        self.markerSize=0.1778
        self.state="searching"
        self.count=0

        self.camera_matrix=np.array([
                                    [1696.80,0.0,960.5],
                                    [0.0,1696.80,540.5],
                                    [   0.0,0.0,1.0   ]
                                                     ],dtype=float)
        self.distortion_coeff=np.zeros((5,1),dtype=float)        
        
        self.bridge=CvBridge()
        self.arucoDict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParam=aruco.DetectorParameters()

        self.cmd_vel_pub=self.create_publisher(Twist,"/cmd_vel",10)
        self.camera_sub=self.create_subscription(Image,"/camera/image_raw",self.image_process,10,callback_group=camera_callback_group)
        self.odometry_sub=self.create_subscription(Odometry,"/odom",self.yaw_calc,10,callback_group=odom_callback_group)      



    def yaw_calc(self,msg:Odometry):

        q=msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)    

    def image_process(self,img:Image):  

        
        cv_img=self.bridge.imgmsg_to_cv2(img,desired_encoding='passthrough')
        corners, id, _ = aruco.detectMarkers(cv_img, self.arucoDict,
                                                            parameters=self.arucoParam)
        
        yaw=self.yaw

        if id is not None:
            
            rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners[0],self.markerSize,self.camera_matrix,self.distortion_coeff)
            distance = np.linalg.norm(tvec[0][0]) 
            
            midframe=self.camera_width/2

            #find the center x position of the marker
            x=0
            for i in range(4): x+=corners[0][0][i][0]
            markerCenter=x/4
            
            if(distance<=1):

                #detected tag and within 1m so turn 90 deg
                if id[0]==1:
                    while(self.yaw<=yaw+1.36):
                        self.publish_velocity(angularz=0.5,linearx=0.0)
                elif id[0]==0:
                    while(self.yaw>=yaw-1.36):   

                        self.publish_velocity(angularz=-0.5,linearx=0.0)
                self.count+=1
                self.state="searching"
            else:
                #detected the tag but not within 1m so adjusts trajectory to seek marker
                self.state="approaching"
                self.publish_velocity(angularz=(midframe-markerCenter)*0.0002,linearx=0.3)

        if (self.count==5):
            #if 5 tags have been detected, stop the robot
            self.publish_velocity(angularz=0.0,linearx=0.0)
            return

        if self.state=="searching":
            #no tag detected
            self.publish_velocity(angularz=0.0,linearx=0.3)
        
    def publish_velocity(self,angularz,linearx):
        msg=Twist()
        msg.angular.z=angularz
        msg.linear.x=linearx
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=ArucoNavigator()
    executor=MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__=="__main__":
    main()

