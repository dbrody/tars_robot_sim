
import math
import rospy

class TarsHeuristicCalculator:

	def __init__(self, tars = None):
		if tars is not None:
			self.restart(tars)


	def restart(self, tars):
		self.last_pose = tars.get_model_pose()
		self.last_time = rospy.get_time()
		
		self.start_time = rospy.get_time()

		self.joint_state = None
		self.joint_commands = None

		self.heur_total = 0
		self.heur_step = 0
		self.pose = None
		self.vel = [0, 0, 0]
		

	def update(self, tars):
		this_time = rospy.get_time()
		time_delta = this_time - self.last_time

		self.joint_commands = tars.get_joint_commands()
		self.joint_state = tars.get_joint_state()
		self.pose = tars.get_model_pose()
		if self.last_pose is not None and self.pose is not None:
			self.vel = [self.pose.position.x - self.last_pose.position.x,
					self.pose.position.y - self.last_pose.position.y,
					self.pose.position.z - self.last_pose.position.z]
		else:
			self.vel = [0, 0, 0]
				
		if self.pose is not None: 
			if time_delta > 0:
				self.vel[0] /= time_delta
				self.vel[1] /= time_delta
				self.vel[2] /= time_delta

			rospy.loginfo('Tars @ (%.2f, %.2f, %.2f) / (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)',
				self.pose.position.x, self.pose.position.y, self.pose.position.z, 
				self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z,
				self.vel[0], self.vel[1], self.vel[2])

			self.last_pose = self.pose
			self.last_time = this_time

			target_vel = self.getDesiredVelocity()
			self.heur_step = 0

			# Error for velocity
			self.heur_step += 10 * abs(target_vel[0] - self.vel[0]) ** 2
			self.heur_step += 10 * abs(target_vel[1] - self.vel[1]) ** 2

			# Error for not staying upright
			self.heur_step += 30 * abs(1.25 - self.pose.position.z) ** 2

			# Error for sending commands that cant be achieved
			self.heur_step += 10 * abs(self.joint_commands[0] - self.joint_state.position[0]) ** 2
			self.heur_step += 10 * abs(self.joint_commands[1] - self.joint_state.position[1]) ** 2
			self.heur_step += 10 * abs(self.joint_commands[2] - self.joint_state.position[2]) ** 2

	def duration(self):
		return rospy.get_time() - self.start_time

	def getDesiredVelocity(self):
		return [2, 0, 0]

	def getPose(self):
		return self.pose

	def getVel(self):
		return self.vel

	def getJointState(self):
		return self.joint_state

	def getJointCommands(self):
		return self.joint_commands


	def getHeuristicStep(self):
		return self.heur_step

	def getHeuristicTotal(self):
		return self.heur_total
