
import time
import roslaunch
from subprocess import Popen, PIPE
from tars_corepy.tars_control import TarsControl
from TarsHeuristicCalculator import TarsHeuristicCalculator


from tars_corepy.level_helper import spawn_tars
from tars_corepy.gazebo_helper import gazebo_delete_object

class TarsSimRobot:

	def __init__(self, name, x, y):
		self.name = name
		self.x = x
		self.y = y

		self.launcher = roslaunch.scriptapi.ROSLaunch()
		self.launcher.start()

		self.node_tars = None
		self.node_controller = None


	def spawn(self):
		tars_control_loc = Popen(['rospack', 'find', 'tars_world'], stdout=PIPE).stdout.read().rstrip()
		tars_control_yaml = tars_control_loc + "/config/tars_control.yaml"
		Popen(['rosparam', 'load', tars_control_yaml, self.name])

		# Start Tars Controller
		node_controller_def = roslaunch.core.Node('controller_manager', 'spawner',
			name='controller_spawner',
			respawn=False,
			output='screen',
			namespace=self.name,
			args='joint1_position_controller joint2_position_controller joint3_position_controller joint_state_controller')
		self.node_controller = self.launcher.launch(node_controller_def)
		# Class object to get/set TARS state
		self.Tars = TarsControl(self.name, init_node=False)
		gazebo_delete_object(self.name)
		spawn_tars(self.name, self.x, self.y)


	def start(self):
		# Start Tars Runner
		node_tars_def = roslaunch.core.Node('tars_controller_nn', 'tars_controller_nn_node',
			name='tars_controller_nn_node_'+self.name,
			args=self.name)
		self.node_tars = self.launcher.launch(node_tars_def)
		self.heur_tracker = TarsHeuristicCalculator(self.Tars)


	def stop(self):
		if self.node_tars is not None:
			self.node_tars.stop()
	
		if self.node_controller is not None:
			self.node_controller.stop()

		gazebo_delete_object(self.name)


	def update(self):
		self.heur_tracker.update(self.Tars)