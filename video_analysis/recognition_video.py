'''
REFERENCES:
https://www.pyimagesearch.com/2018/09/24/opencv-face-recognition/
https://www.pyimagesearch.com/2018/02/26/face-detection-with-opencv-and-deep-learning/
'''

'''
TO TRAIN:
python3 extract_embeddings.py --dataset dataset --embeddings output/embeddings.pickle --detector model --embedding-model openface_nn4.small2.v1.t7
python3 train_model.py --embeddings output/embeddings.pickle --recognizer output/recognizer.pickle --le output/le.pickle
'''


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import pickle
import imutils

IMG_WIDTH = 2592
IMG_HEIGHT = 1936

protoPath = os.path.sep.join(["/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/recognition/model", "deploy.prototxt"])
modelPath = os.path.sep.join(["/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/recognition/model",
	"res10_300x300_ssd_iter_140000.caffemodel"])
detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)


embedder = cv2.dnn.readNetFromTorch("/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/recognition/openface_nn4.small2.v1.t7")

recognizer = pickle.loads(open("/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/recognition/output/recognizer.pickle", "rb").read())
le = pickle.loads(open("/home/rodrigo/catkin_ws/src/tello_controller/video_analysis/recognition/output/le.pickle", "rb").read())


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

                #self.image = imutils.resize(self.image, width=600)
                (h, w) = self.image.shape[:2]

                # construct a blob from the image
                imageBlob = cv2.dnn.blobFromImage(
                    cv2.resize(self.image, (300, 300)), 1.0, (300, 300),
                    (104.0, 177.0, 123.0), swapRB=False, crop=False)

                # apply OpenCV's deep learning-based face detector to localize
                # faces in the input image
                detector.setInput(imageBlob)
                detections = detector.forward()

                # loop over the detections
                for i in range(0, detections.shape[2]):
                    # extract the confidence (i.e., probability) associated with the
                    # prediction
                    confidence = detections[0, 0, i, 2]

                    # filter out weak detections
                    if confidence > 0.5:
                        # compute the (x, y)-coordinates of the bounding box for the
                        # face
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        (startX, startY, endX, endY) = box.astype("int")

                        # extract the face ROI
                        face = self.image[startY:endY, startX:endX]
                        (fH, fW) = face.shape[:2]

                        # ensure the face width and height are sufficiently large
                        if fW < 20 or fH < 20:
                            continue

                        # construct a blob for the face ROI, then pass the blob
                        # through our face embedding model to obtain the 128-d
                        # quantification of the face
                        faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255, (96, 96),
                            (0, 0, 0), swapRB=True, crop=False)
                        embedder.setInput(faceBlob)
                        vec = embedder.forward()

                        # perform classification to recognize the face
                        preds = recognizer.predict_proba(vec)[0]
                        j = np.argmax(preds)
                        proba = preds[j]
                        name = le.classes_[j]

                        # draw the bounding box of the face along with the associated
                        # probability
                        text = "{}: {:.2f}%".format(name, proba * 100)
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