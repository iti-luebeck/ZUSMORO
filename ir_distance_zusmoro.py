#!/usr/bin/env python
from __future__ import division
from std_msgs.msg import Int32

import roslib; roslib.load_manifest('beep_imu')

import rospy

import smbus
import time

bus = smbus.SMBus(1)

irAddress = 0x55

#pub_mag_xsens = rospy.Publisher('imu/mag', Vector3Stamped)

def decFrom2Compl(val, bitlen):
	if val & (1 << bitlen - 1):
		val = val - (1 << bitlen)
	return val

#returns the val scaled from an old range into a new continouse range
def scaleToRange(val, oldMin, oldMax, newMin, newMax):
	val -= oldMin
	val /= oldMax - oldMin
	val *= newMax - newMin
	val += newMin
	return val

def init():
	bus.write_byte_data(irAddress, 0x80, 0x01) # start unit
	bus.write_byte_data(irAddress, 0x81, 0xD0) # pulses
	bus.write_byte_data(irAddress, 0x82, 0x20) # start messurement
	
# reading imu data and publishing Imu msg to topic Imu
def talker():

	rospy.init_node('IRNode')
	
	rospy.loginfo('starting IR_Node')

	while not rospy.is_shutdown():
		storeDistance()		
		rospy.sleep(0.01)
	
	rospy.loginfo('IRNode shut down')

def storeDistance():
	#read current linear accelerations
	#msg = decFrom2Compl(msg,12)

	#update acceleration in msg
	#msg = scaleToRange(dist, -512, 511, -19.6133, 19.6133)
	pub = rospy.Publisher('topic/IR3', Int32)

	msg = Int32()
	msg.data = bus.read_byte_data(irAddress, 0x85)
	msg.data += bus.read_byte_data(irAddress, 0x86) * 0x100

	pub.publish(msg)
	print msg


if __name__ == '__main__':
	init()
	talker()
	pass
	
	
	
