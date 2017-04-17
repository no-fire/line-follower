#!/usr/bin/env python

'''Bag processor.
	Inputs: - a directory name (e.g. 'run1'), which has a rosbag named source.bag in it.
			- Optional second input: 'test' for if you want the program to stop after 20 images. Good for diagnostic runs.
	SAMPLE RUN COMMAND: [run within line-follower package] `python src/data_preprocessing/bag_processor.py data/run1 test`
	Outputs:    - many 640x480 .jpg images, named sequentially from 0000.jpg, in the respective run directory's "raw" folder
				- cmd_vel.csv, which has two columns and each row corresponding to the images in order (from first line above). The columns are values for linear cmd_vel and angular cmd_vel.
				- possibly a "small" folder with compressed images (similarly structured to the first line, above)

'''

from scipy import misc
import numpy as np
import rosbag
import os
import cv2
import sys

class BagProcessor(object):

	def __init__(self, dir_name):
		self.all_vel_array = None
		self.latest_vel = np.matrix([0.5,0])
		self.latest_img = None
		self.dir_name = str(dir_name)
		self.dir_name_raw = self.dir_name+'/raw/'
		self.index=0
		self.padded_index = '%04d' % self.index
		if not os.path.exists(self.dir_name_raw):
			os.makedirs(self.dir_name_raw)
		self.test=False #if test is true, it'll only create 20 images and then stop.

	def save_latest_img(self, img_msg):
		'''
		Input: an image message from ros
		Temporarily saves that image to self.latest_img
		Temp image will be used if a cmd_vel comes in before the next image overwrites it.
		'''
		self.latest_img = np.fromstring(img_msg.data, np.uint8)

	def latest_img_to_jpg(self):
		'''
		Saves the latest image as a .jpg file in the raw folder.
		Activates when a cmd_vel command is received.
		'''
		image_np = cv2.imdecode(self.latest_img, cv2.IMREAD_COLOR)
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
				#For each incoming cmd_vel, save the cmd_vel and the last image to their respective places.
				self.latest_vel = np.matrix([msg.linear.x, msg.angular.z])
				if (self.latest_img!=None):
					self.add_vel()
					self.latest_img_to_jpg()
					self.index+=1
					self.padded_index = '%04d' % self.index
			if (topic=='/camera/image_raw/compressed'):
				#For each image coming in, save it temporarily. The latest image will be saved when a new cmd_vel comes in.
				self.save_latest_img(msg)
			if self.test==True and self.index > 20:
				break
		bag.close()


if __name__ == '__main__':
	# dir_name = raw_input("Name of run directory in which to find source.bag:\n")
	# if dir_name=='':
	# 	dir_name='data/run1'

	if len(sys.argv)>1:
		dir_name = sys.argv[1]
	else:
		dir_name = 'data/sun_apr_16_office_full_line_1'
	bp = BagProcessor(dir_name)

	#is this a test?
	if len(sys.argv)>2:
		bp.test = bool(sys.argv[2]=='test' or sys.argv[2]=='true')
	#Check to see the images end up in the right place with a unit test before the full run
	# bp.test_image()

	bp.get_imgs()
	np.savetxt(dir_name+'/cmd_vel.csv', bp.all_vel_array, delimiter=',') #save csv of all velocities
