#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros

from std_msgs.msg import Int32


IR0 = 0
IR1 = 0
IR2 = 0
IR3 = 0
IR4 = 0
IR5 = 0
IR6 = 0
IR7 = 0
UIR0 = 0
UIR1 = 0
UIR2 = 0
pub_topic_LED6 = rospy.Publisher('topic/LED6', Int32)
pub_topic_LED1 = rospy.Publisher('topic/LED1', Int32)
pub_topic_LED0 = rospy.Publisher('topic/LED0', Int32)
pub_topic_beep = rospy.Publisher('topic/beep', Int32)
pub_topic_LED2 = rospy.Publisher('topic/LED2', Int32)
pub_topic_LED4 = rospy.Publisher('topic/LED4', Int32)
pub_topic_LED7 = rospy.Publisher('topic/LED7', Int32)
pub_topic_LED8 = rospy.Publisher('topic/LED8', Int32)
pub_topic_LED5 = rospy.Publisher('topic/LED5', Int32)
pub_topic_LED3 = rospy.Publisher('topic/LED3', Int32)
pub_topic_motors = rospy.Publisher('topic/motors', Int32)


class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 1')
		global IR0
		global pub_topic_motors
		global pub_topic_motors
		global pub_topic_LED1
		global pub_topic_LED2
		global pub_topic_LED3
		global pub_topic_LED4
		global pub_topic_LED5
		global pub_topic_LED6
		global pub_topic_LED7
		global pub_topic_LED0
		global pub_topic_LED8
		global pub_topic_beep

		while not rospy.is_shutdown():
			if(IR0>1000):
				return 'T1'
			rospy.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T2','T3'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')
		global IR0
		global IR0
		global pub_topic_motors
		global pub_topic_motors
		global pub_topic_LED1
		global pub_topic_LED2
		global pub_topic_LED3
		global pub_topic_LED4
		global pub_topic_LED5
		global pub_topic_LED6
		global pub_topic_LED7
		global pub_topic_LED0
		global pub_topic_LED8
		global pub_topic_beep

		while not rospy.is_shutdown():
			if(IR0<750):
				return 'T2'
			if(IR0>3500):
				return 'T3'
			rospy.sleep(0.01)

class State3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 3')
		global pub_topic_motors
		global pub_topic_motors
		global pub_topic_LED1
		global pub_topic_LED2
		global pub_topic_LED3
		global pub_topic_LED4
		global pub_topic_LED5
		global pub_topic_LED6
		global pub_topic_LED7
		global pub_topic_LED0
		global pub_topic_LED8
		global pub_topic_beep

		while not rospy.is_shutdown():
			rospy.sleep(0.01)

def callback_topic_IR1(msg):
	global IR1
	IR1 = msg.data

def callback_topic_IR2(msg):
	global IR2
	IR2 = msg.data

def callback_topic_UIR0(msg):
	global UIR0
	UIR0 = msg.data

def callback_topic_IR3(msg):
	global IR3
	IR3 = msg.data

def callback_topic_UIR1(msg):
	global UIR1
	UIR1 = msg.data

def callback_topic_IR4(msg):
	global IR4
	IR4 = msg.data

def callback_topic_UIR2(msg):
	global UIR2
	UIR2 = msg.data

def callback_topic_IR5(msg):
	global IR5
	IR5 = msg.data

def callback_topic_IR6(msg):
	global IR6
	IR6 = msg.data

def callback_topic_IR7(msg):
	global IR7
	IR7 = msg.data

def callback_topic_IR0(msg):
	global IR0
	IR0 = msg.data

if __name__ == '__main__':
	rospy.init_node('zusmoro_state_machine')
	rospy.Subscriber('topic/IR3', Int32, callback_topic_IR3)
	rospy.Subscriber('topic/UIR1', Int32, callback_topic_UIR1)
	rospy.Subscriber('topic/IR0', Int32, callback_topic_IR0)
	rospy.Subscriber('topic/UIR0', Int32, callback_topic_UIR0)
	rospy.Subscriber('topic/IR7', Int32, callback_topic_IR7)
	rospy.Subscriber('topic/IR4', Int32, callback_topic_IR4)
	rospy.Subscriber('topic/IR1', Int32, callback_topic_IR1)
	rospy.Subscriber('topic/IR5', Int32, callback_topic_IR5)
	rospy.Subscriber('topic/IR2', Int32, callback_topic_IR2)
	rospy.Subscriber('topic/UIR2', Int32, callback_topic_UIR2)
	rospy.Subscriber('topic/IR6', Int32, callback_topic_IR6)
	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('State1', State1(), transitions={'T1':'State2'})
		smach.StateMachine.add('State2', State2(), transitions={'T2':'State1','T3':'State3'})
		smach.StateMachine.add('State3', State3(), transitions={})
	sis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()