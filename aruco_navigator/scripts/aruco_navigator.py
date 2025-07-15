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
from geometry_msgs.msg import Twist,Point
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
        self.get_logger().info("node started")
        self.camera_width=1920
        self.markerSize=0.1778
        self.state="initialise"
        self.count=0
        self.lastPoint=Point()
        self.searchRevolution=False
        self.initial_yaw=0.0

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
        self.point=msg.pose.pose.position 

    def distance_between(self,point1:Point,point2:Point):
        return (math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2))


    def min_angle_diff(self,angle1,angle2):
        '''returns the minimum angular difference between 2 angles in a -pi to pi 
        ranging coordinate system'''
        diff=angle1-angle2
        if (diff>3.14):delta=diff-6.28
        elif diff<-3.14:delta=diff+6.28
        else: delta=diff
        return delta


    def image_process(self,img:Image):  

        cv_img=self.bridge.imgmsg_to_cv2(img,desired_encoding='passthrough')
        corners, id, _ = aruco.detectMarkers(cv_img, self.arucoDict,
                                                            parameters=self.arucoParam)
        
        yaw=self.yaw

        if (self.count==5):
            #if 5 tags have been detected, stop the robot
            self.publish_velocity(angularz=0.0,linearx=0.0)
            return

        if id is not None:
            #check to avoid looking for tags in the sector behind the robot
            #to prevent it reading the tag it has just read
            delta=self.min_angle_diff(self.yaw,self.initial_yaw)
            if self.searchRevolution==True and abs(delta)> math.radians(120):return

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
                    while(abs(self.min_angle_diff(self.yaw,yaw))<=1.36):
                        self.publish_velocity(angularz=0.5,linearx=0.0)
                elif id[0]==0:
                    while(abs(self.min_angle_diff(self.yaw,yaw))<=1.36):   
                        self.publish_velocity(angularz=-0.5,linearx=0.0)
                self.count+=1

                self.lastPoint=self.point
                self.searchRevolution=False
                self.state="searching"

            else:
                #detected the tag but not within 1m so adjusts trajectory to seek marker
                self.state="approaching"
                self.publish_velocity(angularz=(midframe-markerCenter)*0.0002,linearx=0.3)

        #no tag detected
        if self.state=="searching":
            #does 1 revolution every 3.5 metres
            if self.distance_between(point1=self.point,point2=self.lastPoint)>=3.5:
                if self.searchRevolution == False:
                    self.publish_velocity(angularz=0.2,linearx=0.0)
                    self.searchRevolution=True
                    self.initial_yaw=self.yaw
                    time.sleep(1)

                elif abs(self.yaw-self.initial_yaw)<=0.1:
                    self.searchRevolution=False
                    self.lastPoint=self.point
        
            else:
                self.publish_velocity(angularz=0.0,linearx=0.3)
        
        if self.state=="initialise":
            self.lastPoint=self.point
            self.state="searching"
        
        return
   
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

