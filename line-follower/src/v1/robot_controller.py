#!/usr/bin/env python

#import modules
import numpy as np
from PIL import Image
from scipy.ndimage import zoom
import rospy
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image

from keras.models import Sequential
from keras.layers.convolutional import *
from keras.layers.core import Flatten, Dense
from keras.optimizers import Adam

# from matplotlib import pyplot as plt
# import seaborn as sns

class RobotController(object):
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber(image_topic, Image, self.process_image)
        self.moves = Twist(linear=Vector3(x = 0.0), angular=Vector3(z = 0.0))
        cv2.namedWindow('video_window')
        self.last_img = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        def process_image(self,msg):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
