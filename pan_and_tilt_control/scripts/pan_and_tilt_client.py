#!/usr/bin/python
import sys
import rospy
import math
import time
from pan_and_tilt_control.msg import PanAndTilt



class PanAndTiltClient(object):

    def __init__(self):
        rospy.loginfo("Starting PanAndTiltClient...")
        self._pan_and_tilt_pub = rospy.Publisher('/pan_and_tilt', PanAndTilt, queue_size=1)
        

    def pan_and_tilt_move(self, pan, tilt):
        """
        Topic Publisher
        """
        pan_and_tilt_msg = PanAndTilt()
        pan_and_tilt_msg.pan = pan
        pan_and_tilt_msg.tilt = tilt
        rospy.loginfo("Sending PandAndTilt New msg: " + str(pan_and_tilt_msg))
        self._pan_and_tilt_pub.publish(pan_and_tilt_msg)

def raw_input_pan_and_tilt_test():
    pan_and_tilt_client_obj = PanAndTiltClient()

    while not rospy.is_shutdown():
        print(">>>>>>New Input>>>>>>>>>>\n")
        pan = float(raw_input("Input Pan=>"))
        tilt = float(raw_input("Input Tilt=>"))
        pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan, tilt=tilt)
        print(">>>>>>@@@@@@@@@>>>>>>>>>>\n")


if __name__ == "__main__":
    rospy.init_node('pan_and_tilt_client')
    raw_input_pan_and_tilt_test()
    
