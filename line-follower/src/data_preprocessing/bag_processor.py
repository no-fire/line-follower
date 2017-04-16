#!/usr/bin/env python

'''Bag processor.
	Inputs: a directory name (e.g. 'run1'), which has a rosbag named source.bag in it
	Outputs:    - many 640x480 .jpg images, named sequentially from 0000.jpg, in the respective run directory's "raw" folder
				- cmd_vel.csv, which has three columns: an index counting from zero (corresponding to the image), a value in m/s from cmd_vel.linear.x and a value in m/s from cmd_vel.angular.z
				- possibly a "small" folder with compressed images (similarly structured to the first line, above)
'''

from scipy import misc
import numpy as np
import rosbag
import os
import cv2

class BagProcessor(object):

	def __init__(self, dir_name):
		self.all_vel_array = None
		self.latest_vel = np.matrix([0.5,0])
		self.dir_name = str(dir_name)
		self.dir_name_raw = self.dir_name+'/raw/'
		self.index=0
		self.padded_index = '%04d' % self.index
		if not os.path.exists(self.dir_name_raw):
			os.makedirs(self.dir_name_raw)

	def img_msg_to_jpg(self, img_msg):
		'''
		Input: an image message from ros
		Saves that image as a .jpg file in the raw folder
		'''
		np_arr = np.fromstring(img_msg.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		filename = self.dir_name_raw+self.padded_index+'.jpg'
		misc.imsave(filename, image_np) #uses scipy imsave

	def test_image(self):
		'''Can be run as a unit test to see where your images will end up!'''
		rgb = np.zeros((255, 255, 3), dtype=np.uint8)
		rgb[..., 0] = np.arange(255)
		rgb[..., 1] = 55
		rgb[..., 2] = 1 - np.arange(255)
		misc.imsave(self.dir_name_raw+'/TESTIMG_gradient.jpg', rgb)

	def add_vel(self):
		'''Is called when an image is saved. Adds respective velocity to vel_array.'''
		if self.index == 0:
			self.all_vel_array = np.matrix(self.latest_vel)
		else:
			self.all_vel_array = np.concatenate((self.all_vel_array, np.array(self.latest_vel)), axis=0)


	def get_imgs(self):
		'''
		Input: Bag file with images and velocities
		Goes through each msg to pull out images and cmd_vel
		Calls function to processes the images
		'''
		bag = rosbag.Bag(self.dir_name+'/source.bag')
		for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/camera/image_raw/compressed']):
			if (topic=='/cmd_vel'):
				self.latest_vel = np.matrix([msg.linear.x, msg.angular.z])
			if (topic=='/camera/image_raw/compressed'):
				self.img_msg_to_jpg(msg)
				self.add_vel()
				self.index+=1
				self.padded_index = '%04d' % self.index
		bag.close()


if __name__ == '__main__':
	dir_name = raw_input("Name of run directory in which to find source.bag:\n")

	if dir_name=='':
		dir_name='data/run1'

	bp = BagProcessor(dir_name)
	#Check to see the images end up in the right place with a unit test before the full run
	# bp.test_image()

	bp.get_imgs()
