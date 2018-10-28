#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import cv2
import cv_bridge
import argparse
import random
import numpy as np

import rospy
import tf

import baxter_interface

from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import(Image,)
from std_msgs.msg import String


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
        self._done = False
        self._head = baxter_interface.Head()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # Start transform listener
        self.tf_listener = tf.TransformListener()
        self.face_pub = rospy.Publisher('/robot/xdisplay',Image,latch=True,queue_size=1)
        self.face_sub = rospy.Subscriber('/follow_vr_head', String, self.move_face)
    def send_image(self,path):
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.face_pub.publish(msg)
        rospy.sleep(1)

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self._done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def get_transform(self,link):
        try:
            t = self.tf_listener.getLatestCommonTime("base",link)
            (trans, rot) = self.tf_listener.lookupTransform("base", link, t)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            print e
            return
        return trans, rot

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)
        rospy.sleep(5)
        print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])
 
    def custom(self):  
        self._head.set_pan(0.0)
        rospy.sleep(1)
        print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])
        rospy.sleep(1) 
        self._head.set_pan(-0.9)
        rospy.sleep(5)
        print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])
        print self.get_transform("head")
        rospy.sleep(1) 
        self._head.set_pan(0.3)
        rospy.sleep(5)
        print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])
        print self.get_transform("head")
        rospy.sleep(1) 

    def move_face(self,data):
        print data
        split_data = data.data.split(" ")
        self.look_at([float(split_data[0]),float(split_data[1]),float(split_data[2])])

    def look_at(self,position):
        head_pose = self.get_transform("head")
        head_position = head_pose[0]
        head_quat = head_pose[1]

        xd = position[0] - head_position[0]
        yd = position[1] - head_position[1]
        zd = position[2] - head_position[2]
        hor_ang = np.arctan(float(yd)/xd) #horizontal angle
        ver_ang = np.arctan(float(zd)/xd) #horizontal angle
        print "vertical angle:",ver_ang
        if np.absolute(hor_ang) <= 1.5:    
            self._head.set_pan(hor_ang)
            if ver_ang <= -1.108:
                self.send_image("../images/gaze30012.tif")
            elif ver_ang <= -.950:
                self.send_image("../images/gaze30010.tif")
            elif ver_ang <= -.785:
                self.send_image("../images/gaze30008.tif")
            elif ver_ang <= -.635:
                self.send_image("../images/gaze30007.tif")
            else:
                self.send_image("../images/gaze30006.tif")
            """
            if ver_ang <= -.5:
                self.send_image("../images/gaze30012.tif")
            else:
                self.send_image("../images/gaze30006.tif")
            """
        else:
            print "Can not face that position, head angle change too large"
        #print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])

def main():
    """RSDK Head Example: Wobbler

    Nods the head and pans side-to-side towards random angles.
    Demonstrates the use of the baxter_interface.Head class.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_head_wobbler")

    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    print("Wobbling... ")
    rospy.sleep(3)
    #wobbler.custom()
    #wobbler.look_at([.61,-0.43,0.516])
    #wobbler.look_at([.640,.315,.115])
    #wobbler.look_at([.405,-.526,.631])
    #wobbler.look_at([.594,-.308,-.022])
    #wobbler.look_at([.769,-.427,0])
    #wobbler.look_at([.47,.01,0])
    wobbler.look_at([1.02,.01,0])

    print("Done.")
    rospy.spin()

if __name__ == '__main__':
    main()
