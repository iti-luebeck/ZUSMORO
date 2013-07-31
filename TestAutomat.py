#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros

from std_msgs.msg import Int32
from std_msgs.msg import Int8
from beep_msgs.msg import Color
from beep_msgs.msg import Led


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
pub_BEEP = rospy.Publisher('/beep', Int8)
pub_MOTOR2 = rospy.Publisher('/motor_r', Int8)
pub_led = rospy.Publisher('/leds', Led)
pub_MOTOR1 = rospy.Publisher('/motor_l', Int8)


class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 1')
		c11 = Color()
		c1.r = 204
		c1.g = 51
		c1.b = 0
		led1= Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
		c66 = Color()
		c6.r = 0
		c6.g = 102
		c6.b = 102
		led6= Led()
		led6.header.frame_id = 'led'
		led6.header.stamp = rospy.get_rostime()
		led6.col = c6
		led6.led = 6
		pub_led.publish(led6)
		c77 = Color()
		c7.r = 153
		c7.g = 255
		c7.b = 255
		led7= Led()
		led7.header.frame_id = 'led'
		led7.header.stamp = rospy.get_rostime()
		led7.col = c7
		led7.led = 7
		pub_led.publish(led7)
		MOTOR1 = Int8()
		MOTOR1.data = 100
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = -100
		pub_MOTOR2.publish(MOTOR2)
		BEEP = Int8()
		BEEP.data = 0
		pub_BEEP.publish(BEEP)

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
		smach.StateMachine.add('State1', State1(), transitions={})
	sis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()