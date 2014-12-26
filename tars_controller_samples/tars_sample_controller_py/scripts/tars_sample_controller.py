#!/usr/bin/env python

import roslib; roslib.load_manifest('tars_control')

import math
import rospy

from level_helper import level_setup, get_model_position, setJoint1, setJoint2, setJoint3

def tars_loop(t):
	value = math.sin(t * 2) + t / 10
	
	model_position = get_model_position()
	log_str = "Sending Value: %.2f @ (%.2f, %.2f)" % (value, model_position.x, model_position.y)

	rospy.loginfo(log_str)
	setJoint1(value)
	# setJoint2(value)
	setJoint3(-value)


if __name__ == '__main__':
	level_setup('ExampleController', tars_loop)
