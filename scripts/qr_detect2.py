#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from vitarana_drone import*
from sensor_msgs.msg import*
from cv_bridge import CvBridge, CvBridgeError
from pyzbar import pyzbar
import cv2
import numpy as np
import rospy

class image_proc():

	
	# Initialise everything
	def __init__(self):
		self.image_sub = rospy.Subscriber("/edrone/location", NavSatFix, queue_size = 1) #Subscribing to the camera topic
		self.location = NavSatFix()		

	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			return
		
		# show the output image
			cv2.imshow("Image", image)
			cv2.waitKey(0)


if __name__ == '__main__':
    image_proc_obj = image_proc()

if (len(image_proc_obj.x) > 0 ):
	print("decode()")
	image_proc_obj.decode()
	rospy.spin()
