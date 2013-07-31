#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros

from beep_msgs.msg.msg import Color_sensors
import colorsys

from std_msgs.msg import Int32
from std_msgs.msg import Int8
from beep_msgs.msg import Color
from beep_msgs.msg import Led


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
		global IR0
		c1 = Color()
		c1.r = 255
		c1.g = 0
		c1.b = 102
		led1 = Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
		c6 = Color()
		c6.r = 0
		c6.g = 51
		c6.b = 153
		led6 = Led()
		led6.header.frame_id = 'led'
		led6.header.stamp = rospy.get_rostime()
		led6.col = c6
		led6.led = 6
		pub_led.publish(led6)
		MOTOR1 = Int8()
		MOTOR1.data = 0
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = 0
		pub_MOTOR2.publish(MOTOR2)
		BEEP = Int8()
		BEEP.data = 0
		pub_BEEP.publish(BEEP)

		while not rospy.is_shutdown():
			if(ir[0]>5):
				return 'T1'
			rospy.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')

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