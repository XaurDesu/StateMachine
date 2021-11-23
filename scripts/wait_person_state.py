#!/usr/bin/env python
import cv2
import matplotlib.pyplot as plt
import rospy
import rospkg
import smach
import time
import smach_ros
import timestamp
from robot_toolkit_msgs.msg import *
from datetime import datetime
from robot_toolkit_msgs.srv import *
from std_msgs.msg import * 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
cv_bridge = CvBridge()

class WaitPerson(smach.State):

	def __init__(self):

		smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])   #outcome1 no hay persona outcome2 personas mayor a 0
		self.compressedImage_pub = rospy.Publisher('peppersImage', CompressedImage, queue_size=100) 
		rospy.Subscriber('/robot_toolkit_node/camera/front/image_raw', Image, self.cameraCallback)

		rospack = rospkg.RosPack()
		rospack.list()
		self.path = rospack.get_path('where_is_my_office')

		self.cascPath = self.path + "/resources/haarcascade_frontalface_alt.xml"
		self.cascPath1 = self.path + "/resources/haarcascade_frontalface_alt2.xml"

		self.faceCascade = cv2.CascadeClassifier(self.cascPath)
		self.faceCascade1 = cv2.CascadeClassifier(self.cascPath1)

		self.foto = []
		self.request = None
		self.gray = []
		self.anyImage = False
		self.faces = []
		self.pub_number_faces = None
		self.message = None
		self.personas = 0
		self.compressedImage_pub = None
		self.numberFacesPub = None
		self.numeroPersonas = 0
		self.compressedImage_pub = None
		self.personasTotales = 0
		self.msg = vision_tools_msg()
		self.msg.camera_name = "front_camera"
		self.msg.command = "custom"
		self.msg.frame_rate = 1
		self.res = 2
		self.msg.resolution = self.res
		self.msg.color_space = 11
		self.image_flag = False 
		self.resolution = [120, 240, 480, 960, 1920, 0, 0, 0, 60, 30]

		try:
			camera = rospy.ServiceProxy('/robot_toolkit/vision_tools_srv', vision_tools_srv)  #Se crea el objeto del servicio
			service = camera(self.msg)    #Se activa el sevicio     #return True

		except rospy.ServiceException, e:
			print("Service call failed: %s"%e) #return False

		self.msg.command = "set_parameters"
		self.msg.camera_parameters.brightness = 10 ### original en 0
		self.msg.camera_parameters.contrast = 30
		self.msg.camera_parameters.saturation = 64
		self.msg.camera_parameters.hue = 0
		self.msg.camera_parameters.horizontal_flip = 0
		self.msg.camera_parameters.vertical_flip = 0
		self.msg.camera_parameters.auto_exposition = 1
		self.msg.camera_parameters.auto_white_balance = 1
		self.msg.camera_parameters.auto_gain = 1
		self.msg.camera_parameters.reset_camera_registers = 0
		self.msg.camera_parameters.auto_focus = 1
		self.msg.camera_parameters.compress = True
		self.msg.camera_parameters.compression_factor = 30
		try:
			camera = rospy.ServiceProxy('/robot_toolkit/vision_tools_srv', vision_tools_srv)  #Se crea el objeto del servicio
			service = camera(self.msg)    #Se activa el sevicio    
		except rospy.ServiceException, e:
			print("Service call failed: %s"%e) #return False

		rospy.wait_for_service('/robot_toolkit/audio_tools_srv')
		
		rospy.Subscriber('/robot_toolkit_node/camera/front/image_raw', Image, self.cameraCallback)

		while self.image_flag == False:
		  self.reconocimiento_por_foto(self.foto)	

		if self.image_flag:
			imageFlag = False
		
	def execute(self, userdata):
		
		while True:
			if self.image_flag:
				self.reconocimiento_por_foto(self.foto)	
				image_flag = False
				cv2.imshow('PepperImage', self.gray)
				key = cv2.waitKey(100) & 0xFF
				if key == ord("q"):
					break
			if self.numeroPersonas > 0:
				return 'outcome2'
				break
		cv2.destroyAllWindows()


	def reconocimiento_por_foto(self, imagenOpera):
		#self.gray = cv2.cvtColor(imagenOpera, cv2.COLOR_BGR2GRAY)
		self.gray = imagenOpera

		if len(self.gray) != 0:
			self.faces = self.faceCascade.detectMultiScale(self.gray, 1.2,5)   #parametros 1.3,5
			self.faces1 = self.faceCascade1.detectMultiScale(self.gray, 1.2,5)  #parametros 1.3,5
			for (x,y,w,h) in self.faces:
				if w>70 and h>70:
					cv2.rectangle(self.gray, (x,y), (x+w,y+h), (255,0,0),2)
					roi_gray = self.gray[y:y+h,x:x+w]
					roi_color = imagenOpera[y:y+h,x:x+w]
					time.sleep(1)
					self.anyImage = True
					time.sleep(1)

			for(x,y,w,h) in self.faces1:
				if w>70 and h>70:
					cv2.rectangle(self.gray, (x,y),(x+w, y+h), (255,0,0),3)
					roi_gray2 = self.gray[y:y+h,x:x+w]
					roi_color2 = imagenOpera[y:y+h,x:x+w]
					time.sleep(2)
					self.anyImage = True
					time.sleep(2)

			if len(self.faces1) == len(self.faces):
				self.personasTotales = len(self.faces1)
			elif len(self.faces1) > len(self.faces):
				self.personasTotales = len(self.faces1)
			elif len(self.faces1) > len(self.faces):
				self.personasTotales = len(self.faces1)
			elif len(self.faces) > len(self.faces1):
				self.personasTotales = len(self.faces)
			print(self.personasTotales)

			if self.anyImage:
				time.sleep(1)
				cv2.imwrite(self.path + '/resources/imagen.jpg', self.gray[y:y+h,x:x+w])
				self.anyImage = False
			self.numeroPersonas = self.personasTotales

	def cameraCallback(self, frontData):
		image = None
		imageFrontCamera = None
		if (frontData.encoding == 'compressed bgr8'):
			frame = np.frombuffer(frontData.data, dtype='uint8')
			image = cv2.imdecode(frame, 1)
		else:
			image = cv_bridge.imgmsg_to_cv2(frontData, "bgr8")

		if (image is not None):
			timestamp = datetime.now()
			self.foto = image
			self.image_flag = True

