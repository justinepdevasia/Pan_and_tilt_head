#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import dlib
from sensor_msgs.msg import CompressedImage
from pan_and_tilt_control.msg import Deltaface

class Facefinder:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.subscriber = rospy.Subscriber("/camera/image_raw/compressed",
        CompressedImage, self.callback,  queue_size = 1)
        self.face_pub = rospy.Publisher("delta_face",Deltaface)
        self.final=None
        self.faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')
        self.trackingQuality=0
        self.trackingFace = 0
        self.screen_size_x=640
        self.screen_size_y=480
        self.tracker = dlib.correlation_tracker()
        
            
    def string_to_cv_array(self,image_string):
        self.np_arr = np.fromstring(image_string, np.uint8)
        self.image_np = cv2.imdecode(self.np_arr,cv2.IMREAD_COLOR)
        return self.image_np

    def callback(self,ros_data):
        self.final=self.string_to_cv_array(ros_data.data)
        self.detectAndTrackLargestFace()
    def calculate_delta_face(self,x_corner,y_corner,width,height):

        face_width = width
        face_height = height
        face_top = y_corner
        face_left = x_corner
        face_width_center = face_left + (face_width / 2.0)
        face_height_center = face_top + (face_height / 2.0)

        center_width = self.screen_size_x / 2.0
        center_height = self.screen_size_y / 2.0
        delta_width = face_width_center - center_width
        delta_height = face_height_center - center_height

        return delta_width, delta_height
    def detectAndTrackLargestFace(self):
        self.baseImage = cv2.resize(self.final,(self.screen_size_x,self.screen_size_y))
        self.resultImage = self.baseImage.copy()
        self.rectangleColor = (0,165,255)
        self.rectangleColor1=(0,0,0)
        self.rectangleColor2=(255,255,255)
        
        if not self.trackingFace:
            gray = cv2.cvtColor(self.baseImage, cv2.COLOR_BGR2GRAY)
            self.faces = self.faceCascade.detectMultiScale(gray, 1.3, 5)
            print("Using the cascade detector to detect face")
            maxArea = 0
            x = 0
            y = 0
            w = 0
            h = 0
            for (_x,_y,_w,_h) in self.faces:
                if  _w*_h > maxArea:
                    x = int(_x)
                    y = int(_y)
                    w = int(_w)
                    h = int(_h)
                    maxArea = w*h

            if maxArea > 0 :
                self.tracker.start_track(self.baseImage,dlib.rectangle( x-10,y-20,x+w+10,y+h+20))
                self.trackingFace = 1
        if self.trackingFace:
            self.trackingQuality = self.tracker.update(self.baseImage)
            print("trackingQuality",self.trackingQuality)
            if self.trackingQuality >= 8.75:
                tracked_position = self.tracker.get_position()
                t_x = int(tracked_position.left())
                t_y = int(tracked_position.top())
                t_w = int(tracked_position.width())
                t_h = int(tracked_position.height())
                cv2.rectangle(self.resultImage,(t_x, t_y),(t_x + t_w , t_y + t_h),self.rectangleColor ,2)
                print("no cascade")
            else:
                self.trackingFace = 0

            
        cv2.imshow("result-image",self.resultImage)
        try:
            dx,dy=self.calculate_delta_face(t_x,t_y,t_w,t_h)
            self.face_pub.publish(dx,dy)
            print(t_x,t_y,t_w,t_h)
            print(t_x+t_w/2,t_y+t_h/2)
            print("screen center",self.screen_size_x/2,self.screen_size_y/2)
            print(dx,dy)
        except:
            dx,dy=999.0,999.0
            self.face_pub.publish(dx,dy)
        if cv2.waitKey(2) == ord('q'):
            cv2.destroyAllWindows()
            exit(0)

def main():
    ic = Facefinder()
    rospy.init_node('usb_cam_to_face_track', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
