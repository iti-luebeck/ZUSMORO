#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros

import colorsys
from beep_msgs.msg import Color_sensors
import time
from std_msgs.msg import Int8
from beep_msgs.msg import Color
from beep_msgs.msg import Led
from beep_msgs.msg import IR


t_timer = time.time()
ir = [0, 0, 0, 0, 0, 0, 0, 0]
colorSensor = [0, 0, 0]
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
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c1 = Color()
		c1.r = 51
		c1.g = 255
		c1.b = 51
		led1 = Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
		c2 = Color()
		c2.r = 0
		c2.g = 153
		c2.b = 153
		led2 = Led()
		led2.header.frame_id = 'led'
		led2.header.stamp = rospy.get_rostime()
		led2.col = c2
		led2.led = 2
		pub_led.publish(led2)
		c5 = Color()
		c5.r = 204
		c5.g = 0
		c5.b = 102
		led5 = Led()
		led5.header.frame_id = 'led'
		led5.header.stamp = rospy.get_rostime()
		led5.col = c5
		led5.led = 5
		pub_led.publish(led5)
		c6 = Color()
		c6.r = 0
		c6.g = 0
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
			if(time.time()-t_timer>1.0):
				return 'T1'
			rospy.sleep(0.01)

class State2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T2'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 2')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c1 = Color()
		c1.r = 0
		c1.g = 0
		c1.b = 0
		led1 = Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
		c2 = Color()
		c2.r = 0
		c2.g = 0
		c2.b = 0
		led2 = Led()
		led2.header.frame_id = 'led'
		led2.header.stamp = rospy.get_rostime()
		led2.col = c2
		led2.led = 2
		pub_led.publish(led2)
		c5 = Color()
		c5.r = 0
		c5.g = 0
		c5.b = 0
		led5 = Led()
		led5.header.frame_id = 'led'
		led5.header.stamp = rospy.get_rostime()
		led5.col = c5
		led5.led = 5
		pub_led.publish(led5)
		c6 = Color()
		c6.r = 0
		c6.g = 0
		c6.b = 0
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
			if(ir[1]>100):
				return 'T2'
			rospy.sleep(0.01)


def color_cb(msg):
	global colorSensor
	for (i, sensor) in enumerate(msg.sensors):
		groundColor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]

def ir_cb(msg):
	global ir
	ir = msg.ir

if __name__ == '__main__':
	rospy.init_node('zusmoro_state_machine')
	
	rospy.Subscriber('/IR_filtered', IR, ir_cb)

	rospy.Subscriber('/ground_Color', Color_sensors, color_cb)

	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('State1', State1(), transitions={'T1':'State2'})
		smach.StateMachine.add('State2', State2(), transitions={'T2':'State1'})
	sis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.spin()
	sis.stop()