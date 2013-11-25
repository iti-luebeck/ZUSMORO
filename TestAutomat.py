#!/usr/bin/env python

import roslib; roslib.load_manifest('zusmoro_state_machine')
import rospy
import smach
import smach_ros
import atexit

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
		smach.State.__init__(self, outcomes=['T1','T4','T5','T7','T9'])

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
		c0 = Color()
		c0.r = 0
		c0.g = 0
		c0.b = 0
		led0 = Led()
		led0.header.frame_id = 'led'
		led0.header.stamp = rospy.get_rostime()
		led0.col = c0
		led0.led = 0
		pub_led.publish(led0)
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
		c3 = Color()
		c3.r = 204
		c3.g = 255
		c3.b = 204
		led3 = Led()
		led3.header.frame_id = 'led'
		led3.header.stamp = rospy.get_rostime()
		led3.col = c3
		led3.led = 3
		pub_led.publish(led3)
		c4 = Color()
		c4.r = 0
		c4.g = 0
		c4.b = 0
		led4 = Led()
		led4.header.frame_id = 'led'
		led4.header.stamp = rospy.get_rostime()
		led4.col = c4
		led4.led = 4
		pub_led.publish(led4)
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
		c7 = Color()
		c7.r = 0
		c7.g = 0
		c7.b = 0
		led7 = Led()
		led7.header.frame_id = 'led'
		led7.header.stamp = rospy.get_rostime()
		led7.col = c7
		led7.led = 7
		pub_led.publish(led7)
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
			if(ir[2]>100):
				return 'T1'
			if(ir[1]>100):
				return 'T4'
			if(ir[4]>100):
				return 'T5'
			if(ir[0]>100):
				return 'T7'
			if(ir[5]>100):
				return 'T9'
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
		c2 = Color()
		c2.r = 255
		c2.g = 0
		c2.b = 51
		led2 = Led()
		led2.header.frame_id = 'led'
		led2.header.stamp = rospy.get_rostime()
		led2.col = c2
		led2.led = 2
		pub_led.publish(led2)
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
			if(ir[2]<100):
				return 'T2'
			rospy.sleep(0.01)

class State3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T3'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 3')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c1 = Color()
		c1.r = 204
		c1.g = 204
		c1.b = 0
		led1 = Led()
		led1.header.frame_id = 'led'
		led1.header.stamp = rospy.get_rostime()
		led1.col = c1
		led1.led = 1
		pub_led.publish(led1)
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
			if(ir[1]<100):
				return 'T3'
			rospy.sleep(0.01)

class State4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T6'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 4')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c4 = Color()
		c4.r = 102
		c4.g = 102
		c4.b = 255
		led4 = Led()
		led4.header.frame_id = 'led'
		led4.header.stamp = rospy.get_rostime()
		led4.col = c4
		led4.led = 4
		pub_led.publish(led4)
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
			if(ir[4]<100):
				return 'T6'
			rospy.sleep(0.01)

class State5(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T8'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 5')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c0 = Color()
		c0.r = 204
		c0.g = 0
		c0.b = 204
		led0 = Led()
		led0.header.frame_id = 'led'
		led0.header.stamp = rospy.get_rostime()
		led0.col = c0
		led0.led = 0
		pub_led.publish(led0)
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
			if(ir[0]<100):
				return 'T8'
			rospy.sleep(0.01)

class State6(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['T10'])

	def execute(self, userdata):
		rospy.loginfo('Executing state State 6')
		global ir
		global colorSensor
		global t_timer
		t_timer = time.time()
		global pub_led
		global pub_MOTOR1
		global pub_MOTOR2
		global pub_BEEP
		c5 = Color()
		c5.r = 255
		c5.g = 204
		c5.b = 51
		led5 = Led()
		led5.header.frame_id = 'led'
		led5.header.stamp = rospy.get_rostime()
		led5.col = c5
		led5.led = 5
		pub_led.publish(led5)
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
			if(ir[5]<100):
				return 'T10'
			rospy.sleep(0.01)


def color_cb(msg):
	global colorSensor
	for (i, sensor) in enumerate(msg.sensors):
		groundColor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]

def ir_cb(msg):
	global ir
	ir = msg.ir

if __name__ == '__main__':
	try:
		rospy.init_node('zusmoro_state_machine', disable_signals=True)
		
		rospy.Subscriber('/IR_filtered', IR, ir_cb)

		rospy.Subscriber('/ground_Color', Color_sensors, color_cb)

		sm = smach.StateMachine(outcomes=[])
		with sm:
			smach.StateMachine.add('State1', State1(), transitions={'T1':'State2','T4':'State3','T5':'State4','T7':'State5','T9':'State6'})
			smach.StateMachine.add('State2', State2(), transitions={'T2':'State1'})
			smach.StateMachine.add('State3', State3(), transitions={'T3':'State1'})
			smach.StateMachine.add('State4', State4(), transitions={'T6':'State1'})
			smach.StateMachine.add('State5', State5(), transitions={'T8':'State1'})
			smach.StateMachine.add('State6', State6(), transitions={'T10':'State1'})
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
		c0 = Color()
		c0.r = 0
		c0.g = 0
		c0.b = 0
		led0 = Led()
		led0.header.frame_id = 'led'
		led0.header.stamp = rospy.get_rostime()
		led0.col = c0
		led0.led = 0
		pub_led.publish(led0)
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
		c3 = Color()
		c3.r = 0
		c3.g = 0
		c3.b = 0
		led3 = Led()
		led3.header.frame_id = 'led'
		led3.header.stamp = rospy.get_rostime()
		led3.col = c3
		led3.led = 3
		pub_led.publish(led3)
		c4 = Color()
		c4.r = 0
		c4.g = 0
		c4.b = 0
		led4 = Led()
		led4.header.frame_id = 'led'
		led4.header.stamp = rospy.get_rostime()
		led4.col = c4
		led4.led = 4
		pub_led.publish(led4)
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
		c7 = Color()
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
