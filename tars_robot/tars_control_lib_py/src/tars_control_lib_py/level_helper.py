
import sys
from subprocess import Popen, PIPE
from os import path
import rospy

from std_msgs.msg import Float64, String

from gazebo_helper import gazebo_delete_all_objects, gazebo_spawn_robot, gazebo_spawn_object
from control import tars_run

from ros_helper import setupService

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
	# Get path to cone model
	cone_file = path.expanduser("~/.gazebo/models/construction_cone/model.sdf")

	# Make cones
	gazebo_spawn_object('cone_1', cone_file, 0, 2)
	gazebo_spawn_object('cone_2', cone_file, 0, -2)
	gazebo_spawn_object('cone_3', cone_file, 2, 2)
	gazebo_spawn_object('cone_4', cone_file, 2, -2)
	gazebo_spawn_object('cone_5', cone_file, 4, 2)
	gazebo_spawn_object('cone_6', cone_file, 4, -2)
	gazebo_spawn_object('cone_7', cone_file, 6, 2)
	gazebo_spawn_object('cone_8', cone_file, 6, -2)
	gazebo_spawn_object('cone_9', cone_file, 8, 2)
	gazebo_spawn_object('cone_10', cone_file, 8, -2)
	gazebo_spawn_object('cone_11', cone_file, 10, 2)
	gazebo_spawn_object('cone_12', cone_file, 10, -2)