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
import subprocess
import os
import sys
import time
import cv2
import cv_bridge
import argparse
import random
import numpy as np
import blender_interface

from std_msgs.msg import String

import rospy
import tf

import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_interface import settings  #settings.HEAD_PAN_ANGLE_TOLERANCE used in shake head

from sensor_msgs.msg import(Image,)

blender_output_directory = "../images/"
look_precision = 4

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
            pass
            # print("Disabling robot...")
            # self._rs.disable()

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
        image_path = blender_output_directory + "neutral.png0001"
        if not os.path.isfile(image_path):
            blender_interface.generate_neutral_image()
        self.send_image(image_path)
        # rospy.sleep(5)
        # print tf.transformations.euler_from_quaternion(self.get_transform("head")[1])
 
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
    def shake_head(self, angle = 0.2):
        # See http://sdk.rethinkrobotics.com/wiki/Head_Wobbler_-_Code_Walkthrough
        print("shaking head " + str(angle))
        control_rate = rospy.Rate(100)
        self._head.set_pan(-angle)
        while(not rospy.is_shutdown() and abs(self._head.pan() - (-angle))) > settings.HEAD_PAN_ANGLE_TOLERANCE:
            control_rate.sleep()
        self._head.set_pan(angle)
        while(not rospy.is_shutdown() and abs(self._head.pan() - angle)) > settings.HEAD_PAN_ANGLE_TOLERANCE:
            control_rate.sleep()
        self._head.set_pan(0.0)

    def look_at(self,position, pan_ratio = 0.5):
        """
        param pan_ratio: portion of xy angle to account for via pan_ratio
        """
        #TODO panning takes time. We should be creating eye image while this is happening, which would require us to predict final head pose.
        reuse_images = True
        head_pose = self.get_transform("head")
        head_position = head_pose[0]
        head_quat = head_pose[1]
        print("head orientation: " + str(tf.transformations.euler_from_quaternion(head_quat)))
        xd = float(position[0]) - float(head_position[0])
        yd = float(position[1]) - float(head_position[1])
        xy_ang = np.arctan(yd/xd)
        always_pan = True
        if always_pan or pan_ratio > 0:
            pan_angle = pan_ratio * xy_ang
            pan_euler = (0,0,pan_angle)
            head_quat_predicted = tf.transformations.quaternion_from_euler(*pan_euler)
            head_pose_predicted = [head_position,head_quat_predicted]
            self._head.set_pan(pan_angle)
            #Update head_pose. May need to add a delay.
            head_pose = self.get_transform("head")
            head_position = head_pose[0]
            head_quat = head_pose[1]
        else:
            head_quat_predicted = head_quat
        #Round head pose and target location for the sake of naming. This will allow us to reuse images for similar target positions and head poses. 
        head_position_round = [round(i,look_precision) for i in head_position]
        head_quat_round = [round(i,look_precision) for i in head_quat_predicted]
        position_round = [round(i,look_precision) for i in position]
        image_name = str(position_round) + str(head_position_round) + str(head_quat_round) + ".png"
        # image_name = "beyes.png"
        image_name_output = image_name + "0001"
        image_path = blender_output_directory + image_name_output
        #Generate the image if a comparable one does not exist. The speed gains seem relatively insignificant.
        if not (reuse_images and os.path.isfile(image_path)):
            blender_interface.generate_image(position,head_pose_predicted,image_name)
            print("Generated image")
        else:
            print("Reused image: ",image_path)
        self.send_image(blender_output_directory + image_name_output)
        time.sleep(1)
        head_pose_final = self.get_transform("head")
        head_position_final = head_pose_final[0]
        head_quat_final = head_pose_final[1]
        head_position_delta = abs(np.sum(np.subtract(head_position_final,head_position)))
        head_quat_delta = abs(np.sum(np.subtract(head_quat_final,head_quat)))
        # print("head_position_delta",head_position_delta)
        # print("head_quat_delta",head_quat_delta)
        # print("xy_ang",xy_ang)
        print("final head quat: ",head_quat_final)
        print("predicted head quat: ",head_quat_predicted)
        angle_between_predicted_and_final_quat = np.arccos(2 * np.dot(head_quat_final,head_quat_predicted) - 1.0)
        print("angle between predicted and final head quat: ",angle_between_predicted_and_final_quat)
    def eyes_callback(self, data):
        # print "eyes_callback!"
        msg = data.data
        print("received message: " + msg)
        split_msg = msg.split(" ")
        if msg == "goHome":
            self.set_neutral()
        elif split_msg[0] == "shake":
            if len(split_msg) == 0:
                self.shake_head()
            else:
                self.shake_head(angle = float(split_msg[1]))

        elif split_msg[0] == "lookAt":
            look_args = {}
            look_args["position"] = [float(split_msg[i]) for i in range(1,4)]
            if len(split_msg) >= 6 and split_msg[4] == "pan_ratio":
                pan_ratio = float(split_msg[5])
                look_args["pan_ratio"] = pan_ratio
            # msg2 = data.data.split(" ")
            #msg2 had '' as first element for some reason. Find source later, apply hack for now
            # if len(msg2) == 4:
            #     msg2 = [msg2[1],msg2[2],msg2[3]]
            # try:
            self.look_at(**look_args)
            # except Exception as e:
            #     print("Broke on the following msg:")
            #     print(msg)
            #     # print("Converted msg to:")
            #     # print(msg2)
            #     print(type(e))
            #     raise ValueError("Eyes callback busted")
    # def test_gaze(self,n):
    #     #finish
    #     start = [
    #             0.74, 
    #             -0.278, 
    #             0
    #         ]
    #     for i in range(n):
    #         ang = i * delta
    #         self.look_at(ang)

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
    rospy.init_node("baxter_eyes_node")

    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    print("Wobbling... ")
    rospy.sleep(1)
    # for i in range(10):
    #     wobbler._head.command_nod()
    #     rospy.sleep(0.2)
    # print("shake test")
    # for i in range(0):
    #     wobbler.shake_head(angle = .2)
    #     rospy.sleep(0.5)
    #wobbler.custom()
    # wobbler.look_at([.61,0.43,0.516])
    wobbler.set_neutral()
    # wobbler.look_at([.73,-0.15,0.2])
    # wobbler.set_neutral()
    print("Done.")
    rospy.Subscriber("baxter_eyes", String, wobbler.eyes_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
