import os
import sys
import cv2
import cv_bridge
import argparse
import random
import numpy as np

import rospy
import tf
import time

import baxter_interface

from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import(Image,)
#Valid groups: Standard, Large_Sclera_Outlined
image_group = "Large_Sclera_Outlined"

def send_image(path):
    global face_pub
    img = cv2.imread(path)
    print path
    print type(img)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    face_pub.publish(msg)
    rospy.sleep(1)

def main():
    global face_pub
    print("Initializing node... ")
    rospy.init_node("rsdk_head_wobbler")

    print("Wobbling... ")
    rospy.sleep(1)

    face_pub = rospy.Publisher('/robot/xdisplay',Image,latch=True,queue_size=1)
    
    for i in range(180):
        try:
            send_image("../images/" + image_group + "/baxter_face_"+str(i)+".png")
            print i
        except:
            print "i hate life"



if __name__ == '__main__':
    main()
