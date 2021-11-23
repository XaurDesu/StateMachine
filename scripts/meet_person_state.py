#!/usr/bin/env python

import time
import timestamp
import rospy
import rospkg
import smach
import cv2
import numpy as np 
import matplotlib.pyplot as plt 
import os
import random
import pickle
from datetime import datetime
import timestamp
from robot_toolkit_msgs.msg import *
from robot_toolkit_msgs.srv import *
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
cv_bridge = CvBridge()

class MeetPerson(smach.State):

    def __init__(self):

        smach.State.__init__(self, outcomes=['outcome5'])

        self.image_flag = False 
        self.foto = []

        # Speech publisher
    	self.speech_pub = rospy.Publisher("/speech", speech_msg, queue_size=100)

    	self.t2s_msg = speech_msg()
    	self.t2s_msg.language = "English"
        self.t2s_msg.animated = False

        self.speech_recog = rospy.ServiceProxy('/robot_toolkit/speech_recognition_srv', speech_recognition_srv)

        rospack = rospkg.RosPack()
        rospack.list()
        self.path = rospack.get_path('where_is_my_office')

        self.cascPath = self.path + "/resources/haarcascade_frontalface_alt.xml"
        self.cascPath1 = self.path + "/resources/haarcascade_frontalface_alt2.xml"
        self.cascProfile = self.path + "/resources/haarcascade_profileface.xml"

        self.faceCascade = cv2.CascadeClassifier(self.cascPath)
        self.profileCascade = cv2.CascadeClassifier(self.cascProfile)
        self.faceCascade1 = cv2.CascadeClassifier(self.cascPath1)


        self.name = ""
        self.string_list1 = ["yes", "no"]
        self.names_list = ["Cesar", "Caroline", "George", "Juan", "Nicolas", "Fernando", "Maria Paula", "Andres", "Claudia", "Elian", "Angela", "Sandra", "Micael", "Mauricio", "Juan Jose"]
        self.treshold = 0.4

        rospy.Subscriber('/robot_toolkit_node/camera/front/image_raw', Image, self.cameraCallback)

    def execute(self, userdata):
        rospy.loginfo("Executing meet person state")

        # Person recognition
        finished = False
        while not finished:
            # Ask for the professor
            found = False
            while not found:
                self.speak("Could you please tell me your name?", p_sleep = 3)
                self.treshold = 0.4
                # Professor recognition
                result = self.speech_recog(self.names_list, self.treshold)
                if not result.result == 'NONE':
                    found = True
                    self.name = result.result
                else:
                    self.speak("I'm sorry. I was not able to recognize what you said", p_sleep = 3)

            self.treshold = 0.5

            # Result confirmation
            self.speak("I understood that you are %s" % result.result)
            self.speak("Is that correct?", p_sleep=2.0)
            result = self.speech_recog(self.string_list1, self.treshold)

            if result.result == self.string_list1[0]:
                finished = True
            else:
                finished = False

        self.speak("I will try to remember your face")
        self.speak("I will take some pictures of you. Smile")

        self.speak("Mean while I am going to tell you about the purpose of the challenge we are developing, which we call Where is my office?. Right now we are working on the classification for Robocup 2020, which is going to be in France. I will participate for the second time with Sinfoneea in this competition, but this time I am going to have a better behaviour. It is going be an amazing experience I am really excited!I am glad to have you here today, please wait a few more seconds until i finish taking all the pictures i need to remember you. You look really nice! ")

        if self.image_flag:
            self.image_flag = False
            self.sampleNum = 0
            while True:
                self.meetPerson(self.foto)   
                cv2.imshow('frame',self.gray) #wait for 100 miliseconds 
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break # break if the sample number is morethan 20   
                elif self.sampleNum>30:
                    break

        self.speak("Thanks " + str(self.name) + ", I will try to remember you from now on", p_sleep = 3)
        return 'outcome5'

    def speak(self, p_text, p_sleep=2.0):
        # Method for speech to text purposes
        self.t2s_msg.text = p_text
        self.speech_pub.publish(self.t2s_msg)
        time.sleep(p_sleep)

    def meetPerson(self, imagenOpera):
        
        self.gray = imagenOpera
        self.faces = self.faceCascade.detectMultiScale(self.gray, 1.3, 5)
        for (x,y,w,h) in self.faces:
            if len(self.gray) != 0:
                self.faces = self.faceCascade.detectMultiScale(self.gray, 1.3,5)   #parametros 1.3,5
                self.faces1 = self.faceCascade1.detectMultiScale(self.gray, 1.3,5)  #parametros 1.3,5
                self.profile = self.profileCascade.detectMultiScale(self.gray, 1.1,5)  #parametros 1.1,5
                for (x,y,w,h) in self.faces:
                    cv2.rectangle(self.gray, (x,y), (x+w,y+h), (255,0,0),2)
                    roi_gray = self.gray[y:y+h,x:x+w]
                    roi_color = imagenOpera[y:y+h,x:x+w]

                for(x,y,w,h) in self.faces1:    
                    cv2.rectangle(self.gray, (x,y),(x+w, y+h), (255,0,0),3)
                    roi_gray2 = self.gray[y:y+h,x:x+w]
                    roi_color2 = imagenOpera[y:y+h,x:x+w]

            #incrementing sample number 
            self.sampleNum=self.sampleNum+1
            print("foto" + str(self.sampleNum)) 
            #saving the captured face in the dataset folder
            time.sleep(1)
            cv2.imwrite(self.path+"/resources/dataset/NewPerson/" + str(self.name)+ str(self.sampleNum) + ".jpg", self.gray[y:y+h,x:x+w])
            img = self.gray[y:y+h,x:x+w]

        #return 'outcome5' 

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
