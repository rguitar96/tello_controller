'''
REFERENCES:
https://towardsdatascience.com/face-detection-in-2-minutes-using-opencv-python-90f89d7c0f81
https://towardsdatascience.com/computer-vision-detecting-objects-using-haar-cascade-classifier-4585472829a9
https://answers.opencv.org/question/57/how-to-reduce-false-positives-for-face-detection/
https://github.com/murtazahassan/Tello-Object-Tracking/blob/master/ObjectTrackingTello.py
https://github.com/geaxgx/tello-openpose/blob/master/tello_openpose.py
'''


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

IMG_WIDTH = 2592
IMG_HEIGHT = 1936

class Node(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(15)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/drone0/sensor_measurement/camera",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg, "bgr8")


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            #br = CvBridge()
            if self.image is not None:
                rospy.loginfo('publishing image')
                self.image = cv2.resize(self.image, (IMG_WIDTH, IMG_HEIGHT))

                face_cascade = cv2.CascadeClassifier('/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/haarcascade_frontalface_default.xml')

                gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.1, 4)
                for (x, y, w, h) in faces:
                    cv2.rectangle(self.image, (x, y), (x+w, y+h), (255, 0, 0), 2)

                    cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
                    cy = int(y + (h / 2))  # CENTER Y OF THE OBJECT

                    deadZone = 0
                    cv2.putText(self.image, str(cx)+","+str(cy), (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                    '''
                    if (cx <int(IMG_WIDTH/2)-deadZone):
                        cv2.putText(self.image, " GO LEFT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                        cv2.rectangle(self.image,(0,int(IMG_HEIGHT/2-deadZone)),(int(IMG_WIDTH/2)-deadZone,int(IMG_HEIGHT/2)+deadZone),(0,0,255),cv2.FILLED)
                        dir = 1
                    elif (cx > int(IMG_WIDTH / 2) + deadZone):
                        cv2.putText(self.image, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                        cv2.rectangle(self.image,(int(IMG_WIDTH/2+deadZone),int(IMG_HEIGHT/2-deadZone)),(IMG_WIDTH,int(IMG_HEIGHT/2)+deadZone),(0,0,255),cv2.FILLED)
                        dir = 2
                    elif (cy < int(IMG_HEIGHT / 2) - deadZone):
                        cv2.putText(self.image, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                        cv2.rectangle(self.image,(int(IMG_WIDTH/2-deadZone),0),(int(IMG_WIDTH/2+deadZone),int(IMG_HEIGHT/2)-deadZone),(0,0,255),cv2.FILLED)
                        dir = 3
                    elif (cy > int(IMG_HEIGHT / 2) + deadZone):
                        cv2.putText(self.image, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
                        cv2.rectangle(self.image,(int(IMG_WIDTH/2-deadZone),int(IMG_HEIGHT/2)+deadZone),(int(IMG_WIDTH/2+deadZone),IMG_HEIGHT),(0,0,255),cv2.FILLED)
                        dir = 4
                    else: dir=0
                    '''

                self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Node()
    my_node.start()