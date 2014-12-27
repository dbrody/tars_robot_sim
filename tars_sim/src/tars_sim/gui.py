
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
			self.open = True
		except Exception as err:
			self.open = False


	def flip(self):
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				self.exit()
				return

		self.screen.fill(100)
		self.screen.blit(self.bg, (0, 0))
		pygame.display.update()

	def isOpen(self):
		return self.open


	def exit(self):
		self.open = False
		pygame.quit()