import time
import rospy
import smach

from threading import Timer
from meet_person_state import meet_person
from navigate_state import Navigate
from start_state import start
from wait_person_state import wait
from recognize_object import RecognizeP
from recognize_person import RecognizeO
from give_answer import GiveAnsw

def standard_position_thread_function():
  # Enable animation topic
  motion_msg = motion_tools_msg()
  motion_msg.command = "custom"
  motion_msg.animation = "Enable"
  motion_srv = rospy.ServiceProxy("/robot_toolkit/motion_tools_srv", motion_tools_srv)
  motion_srv(motion_msg)
  time.sleep(0.1)

  # Send standard postion
  motion_pub = rospy.Publisher("/animations", animation_msg)
  anim_msg = animation_msg()
  anim_msg.family = "animations"
  anim_msg.animation_name = "Gestures/Maybe_1"
  motion_pub.publish(anim_msg)

  # Disable animation topic
  motion_msg.animation = "Disable"
  motion_srv(motion_msg)

def send_audio_tools_service(self, command_val="enable_tts"):
  """
  Method that sends an audio tools service request.

  command_val: String containing the request command.
  """
  enable_msg = audio_tools_msg()
  enable_msg.command = command_val
  enable_msg.frequency = 48000
  enable_msg.channels = 0
  try:
    speech = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
    service = speech(enable_msg)
    time.sleep(0.1)
    print("Service call with result %s" % service)
  except rospy.ServiceException, e:
    print("Service call failed: %s" % e)

def main():

    # Initialize the state machine ROS node
    rospy.init_node('where_is_my_lab_state_machine', anonymous=False)

    # Enabling tts as it is requiered for every step
    send_audio_tools_service("enable")

    # Run the standard position thread
    std_pos_thread = Timer(10.0, standard_position_thread_function)
    std_pos_thread.start()

    # Initialize smach state machine
    state_machine = smach.StateMachine(outcomes=['outcome_final'])
    state_machine.userdata.prof_name = None
    state_machine.userdata.navigating = False

# Adds the nine states that are required
    with state_machine:
        smach.StateMachine.add('start', start(),
                               transitions={
                                   'initialize': 'wait'
                               })
        smach.StateMachine.add('wait', wait(),
                               transitions={
                                   'keep_waiting': 'wait',
                                   'meet':'meet_person'
                               })
        smach.StateMachine.add('meet_person', meet_person(),
                               transitions={
                                   'Navigate': 'Navigate'
                               })
        smach.StateMachine.add('Navigate', Navigate(),
                               transitions={
                                   'recognize': 'RecognizeP',
                                   'recognizeo': 'RecognizeO',
                                   'GiveAnsw': 'GiveAnsw'
                               })
        smach.StateMachine.add('meet_person', meet_person(),
                               transitions={
                                   'Navigate': 'Navigate',
                               })
        smach.StateMachine.add('RecognizeP', RecognizeP(),
                               transitions={
                                   'Navigate': 'Navigate'
                               })
        smach.StateMachine.add('RecognizeO', RecognizeO(),
                               transitions={ 
                                   'Navigate': 'Navigate'
                               })
        smach.StateMachine.add('GiveAnsw', GiveAnsw(),
                               transitions={
                                   'Goodbye': 'Goodbye'
                               })

        smach.StateMachine.add('Goodbye', Goodbye(),
                               transitions={
                                   'go_wait': 'wait'
                               })