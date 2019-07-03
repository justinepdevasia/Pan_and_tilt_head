#!/usr/bin/python
import rospy
from pan_and_tilt_control.msg import PanAndTilt
#from pan_and_tilt_move import PanAndTiltMove


class PanAndTiltServer(object):
    def __init__(self, ):
        rospy.loginfo("Starting PanAndTiltServer...")
	#self._pan_and_tilt_move_object = PanAndTiltMove()
        rospy.loginfo("PanAndTiltServer REAL___STARTED")
        pan_and_tilt_sub = rospy.Subscriber('/pan_and_tilt', PanAndTilt, self.pan_and_tilt_callback)
        rospy.loginfo("PanAndTiltServer...READY")

    def pan_and_tilt_callback(self, msg):
        """
        Topic Subscriber callback
        """
	rospy.logdebug("Received PandAndTilt New msg: " + str(msg))
        print("PanAndTiltServer receiving",str(msg))
	#self._pan_and_tilt_move_object.move_to_pitch_yaw(yaw=msg.pan, pitch=msg.tilt)

if __name__ == "__main__":
    rospy.init_node('pan_and_tilt_server')
    machine = PanAndTiltServer()
    rospy.spin()

