#!/usr/bin/env python

import roslib; roslib.load_manifest('xxxxxx')
import rospy
import smach
import smach_ros

from beep.msg import Motors
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8


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
imu_x = 0
imu_y = 0
imu_z = 0
timer = 0
pub_topic_LED1 = rospy.Publisher('topic/LED1', Int8)
pub_topic_beep = rospy.Publisher('topic/beep', Int8)
pub_topic_LED0 = rospy.Publisher('topic/LED0', Int8)
pub_topic_LED2 = rospy.Publisher('topic/LED2', Int8)
pub_topic_LED8 = rospy.Publisher('topic/LED8', Int8)
pub_topic_LED5 = rospy.Publisher('topic/LED5', Int8)
pub_topic_LED3 = rospy.Publisher('topic/LED3', Int8)
pub_topic_LED4 = rospy.Publisher('topic/LED4', Int8)
pub_topic_LED6 = rospy.Publisher('topic/LED6', Int8)
pub_topic_LED7 = rospy.Publisher('topic/LED7', Int8)
pub_topic_motors = rospy.Publisher('topic/motors', Motors)


class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 1')
		global timer
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
			if(timer>5):
				return 'T1'
			rospy.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')
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

def callback_topic_imu(msg):
	global imu_x
	imu_x = msg.linear_acceleration.x
	global imu_y
	imu_y = msg.linear_acceleration.y
	global imu_z
	imu_z = msg.linear_acceleration.z

def callback_topic_IR2(msg):
	global IR2
	IR2 = msg.data

def callback_topic_UIR0(msg):
	global UIR0
	UIR0 = msg.data

def callback_topic_IR3(msg):
	global IR3
	IR3 = msg.data

def callback_topic_IR4(msg):
	global IR4
	IR4 = msg.data

def callback_topic_UIR2(msg):
	global UIR1
	UIR1 = msg.data

def callback_topic_IR5(msg):
	global IR5
	IR5 = msg.data

def callback_topic_UIR3(msg):
	global UIR2
	UIR2 = msg.data

def callback_topic_IR6(msg):
	global IR6
	IR6 = msg.data

def callback_topic_IR7(msg):
	global IR7
	IR7 = msg.data

def callback_topic_Timer(msg):
	global timer
	timer = msg.data

def callback_topic_IR0(msg):
	global IR0
	IR0 = msg.data

if __name__ == '__main__':
	rospy.init_node('zusmoro_state_machine')
	rospy.Subscriber('topic/IR6', Int8, callback_topic_IR6)
	rospy.Subscriber('topic/imu', Imu, callback_topic_imu)
	rospy.Subscriber('topic/IR7', Int8, callback_topic_IR7)
	rospy.Subscriber('topic/IR0', Int8, callback_topic_IR0)
	rospy.Subscriber('topic/IR3', Int8, callback_topic_IR3)
	rospy.Subscriber('topic/IR1', Int8, callback_topic_IR1)
	rospy.Subscriber('topic/IR5', Int8, callback_topic_IR5)
	rospy.Subscriber('topic/IR2', Int8, callback_topic_IR2)
	rospy.Subscriber('topic/UIR2', Int8, callback_topic_UIR2)
	rospy.Subscriber('topic/UIR0', Int8, callback_topic_UIR0)
	rospy.Subscriber('topic/Timer', Int8, callback_topic_Timer)
	rospy.Subscriber('topic/UIR3', Int8, callback_topic_UIR3)
	rospy.Subscriber('topic/IR4', Int8, callback_topic_IR4)
	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('State1', State1(), transitions={'T1':'State2'})
		smach.StateMachine.add('State2', State2(), transitions={})
	sis = smach.ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()