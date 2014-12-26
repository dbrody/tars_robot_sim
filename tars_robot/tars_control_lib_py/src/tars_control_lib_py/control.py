
import roslib; roslib.load_manifest('tars_control')

import rospy
from std_msgs.msg import Float64
from gazebo_helper import gazebo_get_model_state

pubs = dict()

def tars_setup(name, loop_func):
	rospy.init_node(name, anonymous=True)
	pubs['joint1'] = rospy.Publisher('/tars/joint1_position_controller/command', Float64, queue_size=10)
	pubs['joint2'] = rospy.Publisher('/tars/joint2_position_controller/command', Float64, queue_size=10)
	tars_run(loop_func)


# Runs the main loop
def tars_run(loop_func):
	try:	
		# Start publishing robot commands
		rate = rospy.Rate(30)
		base_time = None
		while not rospy.is_shutdown():
			if base_time is None or base_time < 1:
				base_time = rospy.get_time()
			time = (rospy.get_time() - base_time)
			loop_func(time)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass


# Functions to publish to robot joint positions
def setJoint1(value):
	_setJoint('joint1', value)
def setJoint2(value):
	_setJoint('joint2', value)
def setJoint3(value):
	_setJoint('joint3', value)

def _setJoint(name, value):
	if name in pubs:
		pubs[name].publish(value)


# Returns position of robot model
def get_model_position():
	return gazebo_get_model_state('tars').pose.position