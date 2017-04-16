#!/usr/bin/env python

'''Bag processor.
    Inputs: a rosbag filename
    Outputs:    - many 640x480 .jpg images, named sequentially from 0000.jpg, in the respective run directory's "raw" folder
                - cmd_vel.csv, which has three columns: an index counting from zero (corresponding to the image), a value in m/s from cmd_vel.linear.x and a value in m/s from cmd_vel.angular.z
                - possibly a "small" folder with compressed images (similarly structured to the first line, above)
'''

from scipy import misc
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np
import rosbag

class BagProcessor(object):

    def __init__(self):
        self.all_imgs_array = None
		self.all_vel_array = Non
        self.latest_vel = np.matrix([0.5,0])

    def img_msg_to_jpg(self, img_msg):
		'''
		Input: an image name within the directory
		Output: image saved as .jpg
		'''
		np_arr = np.fromstring(img_msg.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		gray_image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

		reimg = misc.imresize(gray_image_np, 25) # Resizes image to 25% the original

	 	_, thresh_img = cv2.threshold(reimg,230,255,cv2.THRESH_BINARY)
		return thresh_img
