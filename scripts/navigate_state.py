#!/usr/bin/env python

import time
import rospy
import smach

from robot_toolkit_msgs.msg import speech_msg, motion_tools_msg, animation_msg
from robot_toolkit_msgs.srv import motion_tools_srv
from std_msgs.msg import String
from geometry_msgs.msg import *
from simple_navigation_msgs.srv import go_to_place_srv


class Navigate(smach.State):

    def __init__(self):
        smach.State.__init__(self, 	outcomes=['outcome10'],
        							input_keys=["n_name_in"])

        self.speech_pub = rospy.Publisher("/speech", speech_msg, queue_size=100)
        self.spinPub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        self.navigate_srv = rospy.ServiceProxy('/go_to_place_srv', go_to_place_srv)
        self.goal_reached_sub = rospy.Subscriber("reachedGoal", String, self.goal_reached_callback)

        self.t2s_msg = speech_msg()
        self.t2s_msg.language = "English"
        self.t2s_msg.animated = True

        self.goal_reached_flag = False

        self.motion_msg = motion_tools_msg()
        self.motion_msg.command = "custom"
        self.motion_msg.animation = "Enable"

        self.left_rooms = ["Mauricio", "Sanchez", "Luis"]

    def execute(self, userdata):
        rospy.loginfo("Executing navigate state")

        # Alerting navigation
        self.speak("I will start navigating to %s office" % userdata.n_name_in)
        if userdata.n_name_in in self.left_rooms:
            self.spin()
        self.speak("Please walk behind me, but keep your distance")
        
        # Navigating service
        isNavigating = self.navigate_srv(userdata.n_name_in)

        # Waits for the goal reached flag
        i = 0
        while not self.goal_reached_flag:
            # Prints console message every 5 seconds
            if i % 10 == 0:
                print("Waiting for the goal reached flag")
            
            # Sleep for stability
            time.sleep(0.5)
            i += 1
        
        # Enable motion tools topic to set standard position
        motion_srv = rospy.ServiceProxy("/robot_toolkit/motion_tools_srv", motion_tools_srv)
        motion_srv(self.motion_msg)
        time.sleep(0.1)

        # Sends standard position animation
        motion_pub = rospy.Publisher("/animations", animation_msg)
        anim_msg = animation_msg()
        anim_msg.family = "animations"
        anim_msg.animation_name = "Gestures/Maybe_1"
        motion_pub.publish(anim_msg)

        self.speak("I have reached our final destination", p_sleep=1.5)

        # Reset flag after navigating
        self.goal_reached_flag = False

        self.motion_msg.animation = "Disable"
        motion_srv(self.motion_msg)

        # Move to next state
        return 'outcome10'

    def goal_reached_callback(self, data):
        if data.data == 'Goal reached.':
            self.goal_reached_flag = True

    def speak(self, p_text, p_sleep=2.0):
        # Method for speech to text purposes
        self.t2s_msg.text = p_text
        self.speech_pub.publish(self.t2s_msg)
        time.sleep(p_sleep)

    def spin(self):
        velMsg = Twist()
        velMsg.angular.z = 0.6
        print("Pepper is going to spin...")
        self.spinPub.publish(velMsg)
        time.sleep(5)
        velMsg.angular.z = 0.0
        self.spinPub.publish(velMsg)

