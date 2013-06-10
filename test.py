#!/usr/bin/env python

import roslib; roslib.load_manifest('xxxxxx')
import rospy
import smach
import smach_ros

from std_msgs.msg import int
from sensor_msgs.msg import Imu


IR0
IR1
IR2
IR3
imu_x
imu_y
imu_z
class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 1')
		while True:
			if(IR2>5):
				return 'T1'
			time.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')
		while True:
			time.sleep(0.01)

def callback_topic_IR(msg):
	IR0 = msg.distance0
	IR1 = msg.distance1
	IR2 = msg.distance2
	IR3 = msg.distance3

def callback_topic_imu(msg):
	imu_x = msg.linear.x
	imu_y = msg.linear.y
	imu_z = msg.linear.z

if __name__ == '__main__':
	rospy.init_node('zusmoro_state_machine')
	rospy.Subscriber('topic/IR', int, callback_topic_IR)
	rospy.Subscriber('topic/imu', Imu, callback_topic_imu)
	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('State1', State1(), transitions={'T1':'State2'})
		smach.StateMachine.add('State2', State2(), transitions={})
	sm.execute()