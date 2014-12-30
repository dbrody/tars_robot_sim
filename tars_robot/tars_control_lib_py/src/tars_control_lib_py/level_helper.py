
import sys
from subprocess import Popen, PIPE
from os import path
import rospy

from std_msgs.msg import Float64, String

from gazebo_helper import gazebo_get_model_state, gazebo_set_model_state, gazebo_delete_all_objects, gazebo_spawn_robot, gazebo_spawn_object
from tars_control import tars_run

pubs = dict()

# Set up the level objects and topics
# Start the level running
def level_setup(name, loop_func):
	rospy.init_node(name, anonymous=True)
	gazebo_delete_all_objects()
	spawn_cones()
	if not spawn_tars():
		print "Unable to spawn new model."
		sys.exit()
	tars_run(loop_func)

# Make an instance of the robot
def spawn_tars():
	try:
		tars_root = Popen(['rospack', 'find', 'tars_description'], stdout=PIPE).stdout.read()
		tars_root = tars_root.rstrip()
	except Exception as er:
		return
	tars_file = tars_root+'/urdf/tars.xml'
	controllers = ['joint1_position_controller', 'joint2_position_controller', 'joint3_position_controller']
	return gazebo_spawn_robot(tars_file, 'tars', controllers)


# Spawn the rows of cones for the level
def spawn_cones():
	set_cone('cone_1', 0, 2)
	set_cone('cone_2', 0, -2)
	set_cone('cone_3', 2, 2)
	set_cone('cone_4', 2, -2)
	set_cone('cone_5', 4, 2)
	set_cone('cone_6', 4, -2)
	set_cone('cone_7', 6, 2)
	set_cone('cone_8', 6, -2)
	set_cone('cone_9', 8, 2)
	set_cone('cone_10', 8, -2)
	set_cone('cone_11', 10, 2)
	set_cone('cone_12', 10, -2)

def set_cone(name, x, y):
	cone = gazebo_get_model_state(name)
	if not cone:
		# Get path to cone model
		cone_file = path.expanduser("~/.gazebo/models/construction_cone/model.sdf")
		gazebo_spawn_object(name, cone_file, x, y)
	else:
		cone.pose.position.x = x
		cone.pose.position.y = y
		cone.pose.position.z = 0
		cone.pose.orientation.x = cone.pose.orientation.y = cone.pose.orientation.z = 0
		cone.twist.linear.x = cone.twist.linear.y = cone.twist.linear.z = 0
		cone.twist.angular.x = cone.twist.angular.y = cone.twist.angular.z = 0
		gazebo_set_model_state(name, cone.pose, cone.twist)
