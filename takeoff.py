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

# mavrosnew = autopilot.Mavros(mavros_prefix="/mavros2")
# quadnew = core.Multirotor(mavrosnew, frequency=10)
# quadnew.takeoff()
# quadnew.set_position([100, 10, 10], blocking=True)
#THIS IS THE UAV 2, USE IT FOR TESTING

# rospy.sleep(3)
# quadnew.land()


class colorDetect:

    def __init__(self):
        self.image_pub = rospy.Publisher('/general_contours/image',Image,queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/uav_1_camera_front/image_raw',Image,self.callback)#What is the image topic for uav?

    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        # # circle drawing code
        # (rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #   cv2.circle(cv_image, (50,50), 10, 255)


        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


        # lower_blue = np.array([90,150,150])
        # upper_blue = np.array([130,255,255])

        ##    lower_orange=np.array([20, 80, 70])#30,92,81
        ##    upper_orange=np.array([40,100,90])
        #purple=np.uint8([[[128,0,128]]])

        ##    print cv2.cvtColor(purple,cv2.COLOR_BGR2HSV)

        # define range of purple color in HSV
        ##    lower_purple=np.array([140, 200, 120])#150,255,128
        ##    upper_purple=np.array([160,255,140])


        lower_green=np.array([40,150,150])#hsv=60,255,255
        upper_green=np.array([90,255,255])

        # Threshold the HSV image to get only green colors
        # blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # mask=blue_mask+green_mask
        mask=green_mask

        # mask = cv2.erode(mask, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)


        # Bitwise-AND mask and original image to give color as well to mask
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        height, width, channels= cv_image.shape

        #find centroid of "blobs"?
        m=cv2.moments(mask, False)
        try: 
            cx, cy= m['m10']/m['m00'], m['01']/m['m00']#gets coords of mask

        except ZeroDivisionError: #if no coords found
            cx, cy=height/2, width/2


        # print(cx, cy)

        # cv2.imshow('frame',frame)
        # cv2.imshow('mask',mask)#SHOW THIS IN GENERAL COUNTOURS IMAGE
        # cv2.imshow('res',res)
        # cv2.waitKey(0)

        # cv2.imshow("Image window", cv_image)
        

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask, "bgr8"))#convert back to img message
        except CvBridgeError as e:
            print(e)

    

if __name__ == "__main__":
    # rospy.init_node("node")
    rospy.init_node('color_detector', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()

    mavros = autopilot.Mavros(mavros_prefix="/mavros1")
    quad = core.Multirotor(mavros, frequency=10)

    quad.takeoff()
    quad.set_position([25, 10, 10], blocking=True)
    rospy.sleep(3)

    # analyze

    cd= colorDetect()
    

    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

    quad.land()






# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     #analyze image here
    
# def analyze():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # node are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('analyzer', anonymous=True)

#     rospy.Subscriber("chatter", String, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()


# cap = cv2.VideoCapture(0)#how do I subscribe to image node?

# while(1):

#     # Take each frame
#     _, frame = cap.read()

#     # Convert BGR to HSV
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    
#     lower_blue = np.array([90,150,150])
#     upper_blue = np.array([130,255,255])

# ##    lower_orange=np.array([20, 80, 70])#30,92,81
# ##    upper_orange=np.array([40,100,90])
#     #purple=np.uint8([[[128,0,128]]])
    
# ##    print cv2.cvtColor(purple,cv2.COLOR_BGR2HSV)

#     # define range of purple color in HSV
# ##    lower_purple=np.array([140, 200, 120])#150,255,128
# ##    upper_purple=np.array([160,255,140])

    
#     lower_green=np.array([40,150,150])#hsv=60,255,255
#     upper_green=np.array([90,255,255])

#     # Threshold the HSV image to get only blue colors
#     blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
#     green_mask = cv2.inRange(hsv, lower_green, upper_green)
    
#     mask=blue_mask+green_mask
#     mask = cv2.erode(mask, None, iterations=2)
#     mask = cv2.dilate(mask, None, iterations=2)
    
#     coords= cv2.findNonZero(mask)
# ##    print coords

#     # Bitwise-AND mask and original image to give color as well to mask
#     res = cv2.bitwise_and(frame,frame, mask= mask)

#     cv2.imshow('frame',frame)
#     cv2.imshow('mask',mask)
#     cv2.imshow('res',res)
#     k = cv2.waitKey(5) & 0xFF
#     if k == 27:
#         break

# cv2.destroyAllWindows()