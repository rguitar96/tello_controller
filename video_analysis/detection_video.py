'''
REFERENCES:
https://www.pyimagesearch.com/2018/09/24/opencv-face-recognition/
https://www.pyimagesearch.com/2018/02/26/face-detection-with-opencv-and-deep-learning/
'''


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

IMG_WIDTH = 2592
IMG_HEIGHT = 1936

net = cv2.dnn.readNetFromCaffe("/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/detection_deploy.prototxt.txt", 
        "/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/detection_res10_300x300_ssd_iter_140000.caffemodel")

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
                #self.image = cv2.resize(self.image, (IMG_WIDTH, IMG_HEIGHT))

                (h, w) = self.image.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(self.image, (300, 300)), 1.0,
                    (300, 300), (104.0, 177.0, 123.0))
                net.setInput(blob)
                detections = net.forward()

                for i in range(0, detections.shape[2]):
                    # extract the confidence (i.e., probability) associated with the
                    # prediction
                    confidence = detections[0, 0, i, 2]

                    # filter out weak detections by ensuring the `confidence` is
                    # greater than the minimum confidence
                    if confidence > 0.4:
                        # compute the (x, y)-coordinates of the bounding box for the
                        # object
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")
                
                        # draw the bounding box of the face along with the associated
                        # probability
                        text = "{:.2f}%".format(confidence * 100)
                        y = startY - 10 if startY - 10 > 10 else startY + 10
                        cv2.rectangle(self.image, (startX, startY), (endX, endY),
                            (0, 0, 255), 2)
                        cv2.putText(self.image, text, (startX, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

                self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Node()
    my_node.start()