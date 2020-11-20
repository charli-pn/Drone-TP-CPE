#! /usr/bin/python3
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy
import socket

ADRESSE = '192.168.1.35'
PORT = 6790
running = True
# Instantiate CvBridge
bridge = CvBridge()

IGNORE_COUNT = 1

print(cv2.__version__)


class CameraFeedSender:

    def image_callback(self,msg):
        self.counter = self.counter + 1
        #print("Received an image!")
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2_img = cv2.resize(cv2_img,(400,200))
        height, width, channels = cv2_img.shape
        #print(width, " ", height)
        cv2.imshow("cv bridge img show", cv2_img)
        cv2.waitKey(1)
        result, imgencode = cv2.imencode('.jpg',cv2_img)
        stringData = numpy.array(imgencode)
        
        if(self.counter >= IGNORE_COUNT):
            self.counter = 0
            print("sending " + str(len(stringData)))
            try:
                self.client.sendall(stringData)
            except:
                self.client.close()
                rospy.signal_shutdown("connexion error")
        
        

    def __init__(self):
        self.serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serveur.bind((ADRESSE, PORT))
        self.serveur.listen(1)
        self.client, self.adresseClient = self.serveur.accept()
        self.counter = 0
        print('Connexion de ', self.adresseClient)
        rospy.init_node('image_listener')
        # Define your image topic
        image_topic = "/bebop2/camera_base/image_raw"
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)
        # Spin until ctrl + c
        rospy.spin()
        self.client.close()
        

if __name__ == '__main__':
    cameraFeed = CameraFeedSender()