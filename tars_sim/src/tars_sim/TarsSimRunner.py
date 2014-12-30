import rospy
import roslaunch
import time
from subprocess import Popen, PIPE

from tars_control_lib_py.tars_control import TarsControl
from tars_control_lib_py.gazebo_helper import gazebo_delete_object
from tars_control_lib_py.level_helper import spawn_tars, spawn_cones

from TarsHeuristicCalculator import TarsHeuristicCalculator
from TarsSimGui import TarsSimGui

class TarsSimRunner:
	def __init__(self):
		
		gui = TarsSimGui()
		gui2 = TarsSimGui()

		# Class object to get/set TARS state
		Tars = TarsControl('tars', init_node=False)


		rospy.init_node('TarsSim', anonymous=True)
		rospy.loginfo('TARS SIM Started.')

		time_per_sim = 10
		self.launcher = roslaunch.scriptapi.ROSLaunch()
		self.launcher.start()

		self.node_tars = None
		self.node_controller = None

		# gazebo_delete_all_objects()
		while not rospy.is_shutdown() and gui.isOpen():
			gui.reset()
			gui2.reset()

			gazebo_delete_object('tars')
			spawn_cones()
			spawn_tars()
			
			# Start TARS and controller
			self.startNodes()

			rate = rospy.Rate(30)
			heur_tracker = TarsHeuristicCalculator(Tars)

			while not rospy.is_shutdown() and gui.isOpen():
				heur_tracker.update(Tars)
				gui.update(heur_tracker)
				gui2.update(heur_tracker)

				if heur_tracker.duration() > time_per_sim:
					break

				rate.sleep()


			# Stop Running TARS
			self.stopNodes()

		gui.exit()

	# Start the TARS and TARS controller nodes
	def startNodes(self):
		# Start Tars Controller
		node_controller_def = roslaunch.core.Node('controller_manager', 'spawner',
			name='controller_spawner',
			respawn=False,
			output='screen',
			namespace='/tars',
			args='joint1_position_controller joint2_position_controller joint3_position_controller joint_state_controller')
		self.node_controller = self.launcher.launch(node_controller_def)

		# Wait for a bit
		time.sleep(0.5)

		# Start Tars Runner
		node_tars_def = roslaunch.core.Node('tars_controller_nn', 'tars_controller_nn_node',
			name='tars_controller_nn_node')
		self.node_tars = self.launcher.launch(node_tars_def)

		time.sleep(3)


	def stopNodes(self):
		if self.node_controller is not None:
			self.node_controller.stop()

		if self.node_tars is not None:
			self.node_tars.stop()

