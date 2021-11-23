#!/usr/bin/env python

import time
import rospy
import smach

from robot_toolkit_msgs.msg import speech_msg


class StartState(smach.State):

    def __init__(self):

    	# State initialization
        smach.State.__init__(self, outcomes=['outcome0'])

    	# Waits for audio tools to be available
    	rospy.wait_for_service('/robot_toolkit/audio_tools_srv')

    	# Speech publisher
    	self.speech_pub = rospy.Publisher("/speech", speech_msg, queue_size=100)

        # Speech topic message
    	self.t2s_msg = speech_msg()
    	self.t2s_msg.language = "English"
        self.t2s_msg.animated = True

    def execute(self, userdata):
        # Keeps track of the state execution
        rospy.loginfo("Executing start state")
        
        # Says its intentions to the audience
        self.speak("Hi everybody!", 1.0)
        self.speak("I will do the where is my office challenge, beeeware")

        # Returns the expected output
        return 'outcome0'

    def speak(self, p_text, p_sleep=2.0):
        # Method for speech to text purposes
        self.t2s_msg.text = p_text
        self.speech_pub.publish(self.t2s_msg)
        time.sleep(p_sleep)
