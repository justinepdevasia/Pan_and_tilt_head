#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from pan_and_tilt_client import PanAndTiltClient
from pan_and_tilt_control.msg import Deltaface
from std_msgs.msg import Float32

class FaceTracker(object):
    def __init__(self):
        self.pan_and_tilt_client_obj = PanAndTiltClient()
        self.faces_msg = Deltaface()
        self.image_sub = rospy.Subscriber("/delta_face",Deltaface, self.faces_detection_callback)


    def face_follow(self):
        """
        It first sets the PanAndTilt to degrees P&T= [0,0]
        Range of Real Pand and Tilt: TILT = [0,110], PAN = [0,180]
        :return:
        """
        RANGE_PAN = [0.0,180.0]
        RANGE_TILT = [0.0,110.0]

        # We Set an initial position where that maximises the range of follow faces
        pan_tilt_pose = [RANGE_PAN[1]/3.0, RANGE_TILT[1]/2.0]
        #self.pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan_tilt_pose[0],tilt=pan_tilt_pose[1])

        # We now read the Face that should be infront
        # We get the position of the person face ( that shouldnt move )

        # Start moving incrementaly pan and tilt until the deltas are practicaly zero
        pan_and_tilt_increment = 1.0
        # This is due to the fact that the image coordinates might not be the same as rotations.
        pan_rotation_sign = -1.0
        tilt_rotation_sign = 1.0
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            delta_width, delta_height = self.faces_msg.x,self.faces_msg.y
            sign_delta_width = np.sign(delta_width)
            sign_delta_height = np.sign(delta_height)
            pan_tilt_pose[0] += sign_delta_width * pan_rotation_sign * pan_and_tilt_increment
            pan_tilt_pose[1] += sign_delta_height * tilt_rotation_sign * pan_and_tilt_increment
            self.pan_and_tilt_client_obj.pan_and_tilt_move(pan=pan_tilt_pose[0],tilt=pan_tilt_pose[1])
            rate.sleep()
    
    def faces_detection_callback(self, data):
        self.faces_msg = data
        
def main():
    rospy.init_node('face_tracker_node', anonymous=True, log_level=rospy.DEBUG)
    face_tracker_obj = FaceTracker()
    face_tracker_obj.face_follow()
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
