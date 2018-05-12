#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pennair2 import core, autopilot
import cv2
import numpy as np
import roslib
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from pennair2.localization import NavstatTransformNode, LocalizationNode
# from pennair2.launch import launch, LaunchFile
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Quaternion
import rospkg


class colorDetect:

    def __init__(self,myQuad):
        self.image_pub = rospy.Publisher('/general_contours/image',Image,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/uav_1_camera_down/image_raw',Image,self.callback)#What is the image topic for uav?
        self.quad=myQuad #passed in the quad 



    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        #Guassian Blur if needed
        #cv_image = cv2.GaussianBlur(cv_image,(3,3),0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_mag=np.array([130,150,170])#CHANGE TO MAGENTA
        upper_mag=np.array([170,200,200])
        # [[[150 174 183]]]


        # Threshold the HSV image to get only white color

        mag_mask = cv2.inRange(hsv, lower_mag, upper_mag)

        mask=mag_mask
        # mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)


        # Bitwise-AND mask and original image to give color as well to mask
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        

        try: #publishing
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))#convert back to img message
        except CvBridgeError as e:
            print(e)

        
        height, width, channels= cv_image.shape
        # print("height: " + str(height))
        # print("width: " + str(width))
        # print("channels: " +channels)
        # cv2.imshow("Image window", cv_image)


        #find centroids of "blobs"
        m=cv2.moments(mask)
        try: 
            cx= int(m['m10']/m['m00'])
            cy= int(m['m01']/m['m00'])#gets coords of mask

        except ZeroDivisionError: #if no coords found
            cy= int(height/2)
            cx= int(width/2)
            #continue on navigation

        print(cx,cy)


        # vel_msg = quad.get_velocity()

        #Enter zeroing in mode once color is found
        # while cx<(width/2)-30 or cx>(width/2)+30 or cy>(height/2)-10: #while out of bounds
        #     # if quad.get_position().position.z >=20:
        #     #     vel_msg.twist.linear.z =-1 
        #     #WANT TO MAINTAIN CERTAIN HEIGHT 
            
        #     if cx<280:
        #         vel_msg.twist.angular.z=0.3
                
        #     else:
        #         vel_msg.twist.angular.z=-0.3
                
        #     quad.set_velocity(vel_msg) #swivel until target is in front

        #move toward target


        # vel_msg.twist.linear.x=10
            
        # quad.set_position([2, 2, 5], blocking=True)


def spawn_square(x, y, z):
        
        request = SpawnModelRequest()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.model_name = "square"

        with open(rospkg.RosPack().get_path("dragonfly") + "/models/square/model.sdf", 'r') as f:
            model_xml = f.read()
        request.model_xml = model_xml

        # print(model_xml)

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        service_proxy = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        try:
            response = service_proxy(request)  # type: SpawnModelResponse
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":
    
    rospy.init_node('color_detector', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()

    # mavrosnew = autopilot.Mavros(mavros_prefix="/mavros2")
    # quadnew = core.Multirotor(mavrosnew, frequency=10)
    # quadnew.takeoff()
    # quadnew.set_position([-5, 5, 2], blocking=True)
    # # THIS IS THE UAV 2, USE IT FOR TESTING
    # rospy.sleep(3)
    # try:
    #     rospy.spin()
        
    # except KeyboardInterrupt:
    #     print("Shutting down")
        
    # quadnew.land()
    spawn_square(0,0,5)

    # magenta = np.uint8([[[183,58,181 ]]]) 
    # magHSV = cv2.cvtColor(magenta, cv2.COLOR_BGR2HSV)
    # print(magHSV)


    #Actual drone
    mavros = autopilot.Mavros(mavros_prefix="/mavros1")
    quad = core.Multirotor(mavros, frequency=10)

    quad.takeoff()
    cd= colorDetect(quad)

    quad.set_position([0, 0, 15], blocking=True)
    rospy.sleep(3)
    

    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

    quad.land()


