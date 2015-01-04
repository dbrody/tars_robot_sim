import rospy
import roslaunch
import time

from tars_corepy.level_helper import spawn_tars, spawn_cones

from TarsSimRobot import TarsSimRobot
from TarsSimGui import TarsSimGui

class TarsSimRunner:
	def __init__(self):
		
		gui = TarsSimGui()

		rospy.init_node('TarsSim', anonymous=True)
		rospy.loginfo('TARS SIM Started.')

		time_per_sim = 10
		
		self.node_tars = None
		self.node_controller = None

		# gazebo_delete_all_objects()
		tars_obj_num = 0
		while not rospy.is_shutdown() and gui.isOpen():
			gui.reset()

			spawn_cones()

			# Create and start robot in gazebo
			robot = TarsSimRobot('tars1', 0, 0)
			robot.spawn()
			robot.start()
			time.sleep(2)
		
			# Start updating robot heuristics
			rate = rospy.Rate(30)
			while not rospy.is_shutdown() and gui.isOpen():
				robot.update()
				gui.update(robot.heur_tracker)
				if robot.heur_tracker.duration() > time_per_sim:
						break
				rate.sleep()

			# Remove robot
			robot.stop()

		gui.exit()
