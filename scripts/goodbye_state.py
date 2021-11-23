#!/usr/bin/env python

import time

import rospy
import smach

from std_msgs.msg import String
from robot_toolkit_msgs.msg import speech_msg
from simple_navigation_msgs.srv import go_to_place_srv


class Goodbye(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome11', 'outcome12'], input_keys=['nav_in'])

        # Speech publisher
    	self.speech_pub = rospy.Publisher("/speech", speech_msg, queue_size=100)
        self.goal_reached_sub = rospy.Subscriber("reachedGoal", String, self.goal_reached_callback)

    	# Text to speech message configuration
    	self.t2s_msg = speech_msg()
    	self.t2s_msg.language = "English"
        self.t2s_msg.animated = True

        # Variable containing the number of processed persons
        self.processed_persons = 0

        self.goal_reached_flag = False

        self.navigate_srv = rospy.ServiceProxy('/go_to_place_srv', go_to_place_srv)

    def execute(self, userdata):
        rospy.loginfo("Executing goodbye state")
        
        # Says goodbye to the human being
        self.speak("Well, it was a pleasure helping you to reach your destination")
        self.speak("I'll see you next time, bye")
        self.processed_persons += 1

        if userdata.nav_in:
            # Navigating service
            isNavigating = self.navigate_srv("Person")

            # Waits for the goal reached flag
            i = 0
            while not self.goal_reached_flag:
                # Prints console message every 5 seconds
                if i % 10 == 0:
                    print("Waiting for the goal reached flag")

                # Sleep for stability
                time.sleep(0.5)
                i += 1

            #self.goal_reached_flag = False

        if self.goal_reached_flag:
		self.goal_reached_flag = False
        	return 'outcome11'
        #else:
        	#return 'outcome12'

    def speak(self, p_text, p_sleep=2.0):
        # Method for speech to text purposes
        self.t2s_msg.text = p_text
        self.speech_pub.publish(self.t2s_msg)
        time.sleep(p_sleep)

    def goal_reached_callback(self, data):
        if data.data == 'Goal reached.':
            self.goal_reached_flag = True
