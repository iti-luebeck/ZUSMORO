#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros

import colorsys
from beep_msgs.msg import Color_sensors
from std_msgs.msg import Int32
from std_msgs.msg import Int8
from beep_msgs.msg import Color
from beep_msgs.msg import Led


t_timer = rospy.get_time()
colorSensor = array([0,0,0])
ir = array([0,0,0,0,0,0,0,0])
pub_BEEP = rospy.Publisher('/beep', Int8)
pub_MOTOR2 = rospy.Publisher('/motor_r', Int8)
pub_led = rospy.Publisher('/leds', Led)
pub_MOTOR1 = rospy.Publisher('/motor_l', Int8)


class State1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T1'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 1')
		global ir
		global t_timer
		t_timer = rospy.get_time()
		global colorSensor
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP

		while not rospy.is_shutdown():
			if(rospy.get_time()-t_timer>544):
				return 'T1'
			rospy.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')
		global ir
		global t_timer
		t_timer = rospy.get_time()
		global colorSensor
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP

		while not rospy.is_shutdown():
			rospy.sleep(0.01)


def color_cb(msg):
	global colorSensor
	for (i, sensor) in enumerate(msg.sensors)
		groundColor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]

def ir_cb(msg):
	global ir
	ir = msg.ir

if __name__ == '__main__':
	rospy.init_node('zusmoro_state_machine')
	
	rospy.Subscriber('/IR_filtered', Int32, ir_cb)

	rospy.Subscriber('/ground_Color', Color_sensors, color_cb)

	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('State1', State1(), transitions={'T1':'State2'})
		smach.StateMachine.add('State2', State2(), transitions={})
	sis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()