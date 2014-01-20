#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros
import atexit

import colorsys
from beep_msgs.msg import Color_sensors
from beep_msgs.msg import MyColor
import time
from std_msgs.msg import Int8
from beep_msgs.msg import Led
from beep_msgs.msg import IR


t_timer = time.time()
ir = [0, 0, 0, 0, 0, 0, 0, 0]
colorSensor = [0, 0, 0]
pub_BEEP = rospy.Publisher('/beep', Int8)
pub_MOTOR2 = rospy.Publisher('/motor_r', Int8)
pub_led = rospy.Publisher('/leds', Led)
pub_MOTOR1 = rospy.Publisher('/motor_l', Int8)


class LinksDrift(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T1','T4','T7','T12'])

	def execute(self, userdata):
		rospy.loginfo('Executing state LinksDrift')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		MOTOR1 = Int8()
		MOTOR1.data = 45
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = 65
		pub_MOTOR2.publish(MOTOR2)
		BEEP = Int8()
		BEEP.data = 0
		pub_BEEP.publish(BEEP)

		while not rospy.is_shutdown():
			if(ir[7]<210):
				return 'T1'
			if(ir[6]>180):
				return 'T4'
			if(ir[7]<160):
				return 'T7'
			if(ir[5]>165):
				return 'T12'
			rospy.sleep(0.01)

class RechtsDrift(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T2','T31','T6','T11'])

	def execute(self, userdata):
		rospy.loginfo('Executing state RechtsDrift')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		MOTOR1 = Int8()
		MOTOR1.data = 70
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = 45
		pub_MOTOR2.publish(MOTOR2)

		while not rospy.is_shutdown():
			if(ir[7]>210):
				return 'T2'
			if(ir[6]>180):
				return 'T31'
			if(ir[7]<160):
				return 'T6'
			if(ir[5]>165):
				return 'T11'
			rospy.sleep(0.01)

class LinksKurve(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T5'])

	def execute(self, userdata):
		rospy.loginfo('Executing state LinksKurve')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		MOTOR1 = Int8()
		MOTOR1.data = -40
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = 40
		pub_MOTOR2.publish(MOTOR2)
		BEEP = Int8()
		BEEP.data = 0
		pub_BEEP.publish(BEEP)

		while not rospy.is_shutdown():
			if(ir[6]<180):
				return 'T5'
			rospy.sleep(0.01)

class RechtsKurve(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T8','T9','T10'])

	def execute(self, userdata):
		rospy.loginfo('Executing state RechtsKurve')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		MOTOR1 = Int8()
		MOTOR1.data = 60
		pub_MOTOR1.publish(MOTOR1)
		MOTOR2 = Int8()
		MOTOR2.data = 15
		pub_MOTOR2.publish(MOTOR2)

		while not rospy.is_shutdown():
			if(ir[7]>160):
				return 'T8'
			if(ir[6]>180):
				return 'T9'
			if(ir[5]>165):
				return 'T10'
			rospy.sleep(0.01)


def ir_cb(msg):
	global ir
	ir = msg.ir

def color_cb(msg):
	global colorSensor
	for (i, sensor) in enumerate(msg.sensors):
		colorSensor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]

if __name__ == '__main__':
	try:
		rospy.init_node('zusmoro_state_machine', disable_signals=True)
		
		rospy.Subscriber('/IR_filtered', IR, ir_cb)

		rospy.Subscriber('/ground_colors', Color_sensors, color_cb)

		sm = smach.StateMachine(outcomes=[])
		with sm:
			smach.StateMachine.add('LinksDrift', LinksDrift(), transitions={'T1':'RechtsDrift','T4':'LinksKurve','T7':'RechtsKurve','T12':'LinksKurve'})
			smach.StateMachine.add('RechtsDrift', RechtsDrift(), transitions={'T2':'LinksDrift','T31':'LinksKurve','T6':'RechtsKurve','T11':'LinksKurve'})
			smach.StateMachine.add('LinksKurve', LinksKurve(), transitions={'T5':'LinksDrift'})
			smach.StateMachine.add('RechtsKurve', RechtsKurve(), transitions={'T8':'LinksDrift','T9':'LinksKurve','T10':'LinksKurve'})
		sis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')
		sis.start()
		sm.execute()
		rospy.spin()
		sis.stop()
	finally:
		rospy.loginfo('zusmoro_state_machine is shutting down')
		pub_MOTOR1.publish(0)
		pub_MOTOR2.publish(0)
		pub_BEEP.publish(0)
		c0 = MyColor()
		c0.r = 0
		c0.g = 0
		c0.b = 0
		led0 = Led()
		led0.header.frame_id = 'led'
		led0.header.stamp = rospy.get_rostime()
		led0.col = c0
		led0.led = 0
		pub_led.publish(led0)
		c1 = MyColor()
		c1.r = 0
		c1.g = 0
		c1.b = 0
		led1 = Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
		c2 = MyColor()
		c2.r = 0
		c2.g = 0
		c2.b = 0
		led2 = Led()
		led2.header.frame_id = 'led'
		led2.header.stamp = rospy.get_rostime()
		led2.col = c2
		led2.led = 2
		pub_led.publish(led2)
		c3 = MyColor()
		c3.r = 0
		c3.g = 0
		c3.b = 0
		led3 = Led()
		led3.header.frame_id = 'led'
		led3.header.stamp = rospy.get_rostime()
		led3.col = c3
		led3.led = 3
		pub_led.publish(led3)
		c4 = MyColor()
		c4.r = 0
		c4.g = 0
		c4.b = 0
		led4 = Led()
		led4.header.frame_id = 'led'
		led4.header.stamp = rospy.get_rostime()
		led4.col = c4
		led4.led = 4
		pub_led.publish(led4)
		c5 = MyColor()
		c5.r = 0
		c5.g = 0
		c5.b = 0
		led5 = Led()
		led5.header.frame_id = 'led'
		led5.header.stamp = rospy.get_rostime()
		led5.col = c5
		led5.led = 5
		pub_led.publish(led5)
		c6 = MyColor()
		c6.r = 0
		c6.g = 0
		c6.b = 0
		led6 = Led()
		led6.header.frame_id = 'led'
		led6.header.stamp = rospy.get_rostime()
		led6.col = c6
		led6.led = 6
		pub_led.publish(led6)
		c7 = MyColor()
		c7.r = 0
		c7.g = 0
		c7.b = 0
		led7 = Led()
		led7.header.frame_id = 'led'
		led7.header.stamp = rospy.get_rostime()
		led7.col = c7
		led7.led = 7
		pub_led.publish(led7)
		rospy.signal_shutdown('zusmoro_state_machine was terminated by KeyBoard Interupt')
