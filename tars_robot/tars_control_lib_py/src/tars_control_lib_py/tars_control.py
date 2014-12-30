
import roslib; roslib.load_manifest('tars_control')

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState

from gazebo_helper import gazebo_get_model_state

class TarsControl:

	def __init__(self, name, init_node=True, listen_only=False):

		self.name = name

		self.pubs = dict()
		self.subs = dict()
		self.model_joint_state = None
		self.joint_commands = [0, 0, 0]

		if init_node:
			rospy.init_node(name+"_node", anonymous=True)

		if listen_only:
			self.pubs['joint1'] = rospy.Publisher('/'+self.name+'/joint1_position_controller/command', Float64, queue_size=10)
			self.pubs['joint2'] = rospy.Publisher('/'+self.name+'/joint2_position_controller/command', Float64, queue_size=10)
			self.pubs['joint3'] = rospy.Publisher('/'+self.name+'/joint3_position_controller/command', Float64, queue_size=10)

		self.subs['joints'] = rospy.Subscriber('/'+self.name+'/joint_states', JointState, self.joint_state_callback)
		self.subs['joint1'] = rospy.Subscriber( '/'+self.name+'/joint1_position_controller/command', Float64, self.joint1_state_callback)
		self.subs['joint2'] = rospy.Subscriber( '/'+self.name+'/joint2_position_controller/command', Float64, self.joint2_state_callback)
		self.subs['joint3'] = rospy.Subscriber( '/'+self.name+'/joint3_position_controller/command', Float64, self.joint3_state_callback)


	def run(self, loop_func):
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


	def joint_state_callback(self, data):
		self.model_joint_state = data
	
	def joint1_state_callback(self, data):
		self.joint_commands[0] = data.data

	def joint2_state_callback(self, data):
		self.joint_commands[1] = data.data

	def joint3_state_callback(self, data):
		self.joint_commands[2] = data.data

	# Functions to publish to robot joint positions
	def setJoint1(self, value):
		self._setJoint('joint1', value)
	def setJoint2(self, value):
		self._setJoint('joint2', value)
	def setJoint3(self, value):
		self._setJoint('joint3', value)

	def _setJoint(self, name, value):
		if name in self.pubs:
			self.pubs[name].publish(value)

	# Get states of joints
	def get_joint_state(self):
		return self.model_joint_state

	def get_joint_commands(self):
		return self.joint_commands

	# Returns position of robot model
	def get_model_position(self):
		model_pose = self.get_model_pose()
		if model_pose == None:
			return None
		return model_pose.position

	# Returns position of robot model
	def get_model_pose(self):
		model_state = gazebo_get_model_state(self.name)
		if model_state == None:
			return None
		return model_state.pose