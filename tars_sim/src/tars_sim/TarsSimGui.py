
import math
import pygame

class TarsSimGui:

	def __init__(self):
		pygame.init()
		self.size = (640, 480)

		try:
			pygame.display.set_caption("TARS Sim")
			self.screen = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
		
			self.bg = pygame.Surface(self.size)
			self.bg.fill((100, 100, 100))

			self.font = pygame.font.Font(None, 16)

			self.open = True
			self.heurSteps = []
			self.scores = []
		except Exception as err:
			self.open = False


	def reset(self):
		if len(self.heurSteps) > 0:
			heurTotal = 0
			for h in self.heurSteps:
				heurTotal += h
			self.scores.append(int(heurTotal))
		self.heurSteps = []
		self.update()


	def update(self, heur_tracker=None):
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				self.exit()
				return

		self.screen.fill((100, 100, 100))

		if heur_tracker is not None:
			self.heurSteps.append(heur_tracker.getHeuristicStep())

			vel = heur_tracker.getVel()
			velDesired = heur_tracker.getDesiredVelocity()

			jointState = heur_tracker.getJointState()
			if jointState is not None:
				joint1 = jointState.position[0]
				joint2 = jointState.position[1]
				joint3 = jointState.position[2]
			else:
				joint1 = joint2 = joint3 = 0

			jointCommands = heur_tracker.getJointCommands()
			if jointCommands is not None and len(jointCommands) >= 3:
				joint1g = jointCommands[0]
				joint2g = jointCommands[1]
				joint3g = jointCommands[2]
			else:
				joint1g = joint2g = joint3g = 0
		else:
			vel = [0, 0, 0]
			velDesired = [0, 0, 0]
			joint1 = joint2 = joint3 = 0
			joint1g = joint2g = joint3g = 0   

		# Global Velocity
		self.drawVelDisplay(25 * vel[0], 25 * vel[1], 25 * velDesired[0], 25 * velDesired[1])

		# Joint Positions
		self.drawJointDisplay("Joint1", 10, 140, joint1, joint1g)
		self.drawJointDisplay("Joint2", 140, 140, joint2, joint2g)
		self.drawJointDisplay("Joint3", 270, 140, joint3, joint3g)

		# Draw current run heuristics
		self.drawHeuristicGraph(10, 300, self.heurSteps)

		self.drawRunHeuristics(400, 10, self.scores)

		pygame.display.update()


	def drawRectDisplay(self, label, x, y, width=100, height=100):
		# self.screen.blit(self.bg, (0, 0))
		global_velocity_rect = pygame.Rect(x, y, width, height)
		self.drawText(label, global_velocity_rect.centerx, global_velocity_rect.y - 5)
		pygame.draw.rect(self.screen, (255, 255, 255), global_velocity_rect, 2)
		return global_velocity_rect
	
	def drawLineAtAngle(self, center, angle, lineLen, color = (255, 255, 255)):
		velLenX = lineLen * math.cos(angle)
		velLenY = lineLen * math.sin(angle)
		centerOff = center[0] + velLenX, center[1] + velLenY
		pygame.draw.line(self.screen, color, center, centerOff)


	def drawVelDisplay(self, dataDx, dataDy, dataDxDesired, dataDyDesired):
		rect = self.drawRectDisplay("Global Velocity", 10, 10)
		angle = math.atan2(dataDyDesired, dataDxDesired)
		velLen = math.sqrt(dataDxDesired * dataDxDesired + dataDyDesired * dataDyDesired)
		self.drawLineAtAngle(rect.center, angle, velLen, color=(0, 0, 255))

		angle = math.atan2(dataDy, dataDx)
		velLen = math.sqrt(dataDx * dataDx + dataDy * dataDy)
		self.drawLineAtAngle(rect.center, angle, velLen)


	def drawJointDisplay(self, label, x, y, dataActual, dataGoal):
		rect = self.drawRectDisplay(label, x, y)
		
		# Draw Goal
		self.drawLineAtAngle(rect.center, dataGoal, 40, (0, 0, 200))

		# Draw Measured
		self.drawLineAtAngle(rect.center, dataActual, 25, (255, 255, 255))


	def drawHeuristicGraph(self, x, y, data):
		rect = self.drawRectDisplay("Heuristic", x, y, 320, 100)
		lastx = x
		lasty = y
		for v in data:
			v += y
			pygame.draw.line(self.screen, (255, 255, 255), (lastx, lasty), (lastx+1, v))
			lasty = v
			lastx = lastx + 1


	def drawRunHeuristics(self, x, y, data):
		rect = self.drawRectDisplay("Heuristic", x, y, 100, 300)
		lasty = y
		for score in data:
			lasty += 15
			self.drawText(""+str(score), rect.centerx, lasty)


	def drawText(self, txtstr, x, y):
		strImg = self.font.render(txtstr, True, (255, 255, 255), (100, 100, 100))
		strRect = strImg.get_rect()
		strRect.center = x, y
		self.screen.blit(strImg, strRect)


	def isOpen(self):
		return self.open


	def exit(self):
		self.open = False
		pygame.quit()