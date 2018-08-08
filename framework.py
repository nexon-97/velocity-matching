import sdl2
import sdl2.ext
import sys
import ctypes
from itertools import tee, izip
import math
from smath import *
from pathfinder import *

initialPosition = (float(sys.argv[1]), float(sys.argv[2]))
initialVelocity = (float(sys.argv[3]), float(sys.argv[4]))
targetPosition = (float(sys.argv[5]), float(sys.argv[6]))
targetVelocity = (float(sys.argv[7]), float(sys.argv[8]))

width = 1600
height = 900

# ==================================

CLEAR_COLOR = sdl2.ext.Color(0, 128, 255)
RED_COLOR = sdl2.ext.Color(255, 0, 0)
GREEN_COLOR = sdl2.ext.Color(0, 200, 0)
YELLOW_COLOR = sdl2.ext.Color(255, 255, 0)
YELLOW2_COLOR = sdl2.ext.Color(200, 200, 0)
RESOURCES = sdl2.ext.Resources(__file__, "resources")

# ==================================

def pairwise(iterable):
	a, b = tee(iterable)
	next(b, None)
	return izip(a, b)
		
# ==================================

class Transform(object):
	def __init__(self):
		super(Transform, self).__init__()
		self.position = vec2f()
		self.rotation = 0.0

# ==================================

class Player():
	def __init__(self, sprite):
		self.sprite = sprite
		self.transform = Transform()
		self.velocity = vec2f()
		self.speed = 30
		self.steeringCapability = 10
		
	def render(self, renderer):
		size = self.sprite.size
		halfSize = (size[0] / 2, size[1] / 2)
		position = self.transform.position
		drawPosition = (int(position.x - halfSize[0]), int(position.y - halfSize[1]))
		centerPos = position
		
		renderer.copy(self.sprite, (0, 0, size[0], size[1]),
			dstrect=(drawPosition[0], drawPosition[1], size[0], size[1]),
			angle=self.transform.rotation)
		
		renderer.fill((int(position.x) - 3, int(position.y) - 3, 6, 6), GREEN_COLOR)
		
# ==================================

class UIElement(sdl2.ext.Entity):
	def __init__(self, world, sprite, posx=0, posy=0):
		self.sprite = sprite
		self.sprite.position = posx, posy
		
# ==================================

def stepBtnHandler(sender, *args):
	if context.playbackStep + 1 < len(context.pathPoints):
		context.playbackStep += 1
		context.player.transform.position = context.pathPoints[context.playbackStep]
		context.player.transform.rotation = context.pathOrientations[context.playbackStep]
		
		context.player.velocity = angleToVelocity(context.player.transform.rotation)
		
def updatePath(sender, *args):
	position = context.player.transform.position
	velocity = context.player.velocity
	target = context.target.transform.position
	targetVelocity = context.target.velocity
	
	context.pathfinder.playerSpeed = context.player.speed
	path = context.pathfinder.getPath(position, velocity, target, targetVelocity)
	
	context.playbackStep = 0
	context.pathPoints = path["points"]
	context.pathOrientations = path["rotations"]

# ==================================
	
class Context():
	def __init__(self):
		sdl2.ext.init()
		
		self.window = sdl2.ext.Window("Velocity matching pathfinding", size=(width, height))
		self.renderer = sdl2.ext.Renderer(self.window, flags=sdl2.SDL_RENDERER_ACCELERATED)
		self.window.show()
		
		self.world = sdl2.ext.World()
		self.uiProcessor = sdl2.ext.UIProcessor()
		self.spriteFactory = sdl2.ext.SpriteFactory(sdl2.ext.TEXTURE, renderer=self.renderer)
		self.uiFactory = sdl2.ext.UIFactory(self.spriteFactory)
		
		self.lastTime = 0
		
		self.initUI()
		self.initPlayers()
		
		self.pathPoints = []
		self.pathOrientations = []
		self.playbackStep = 0
		
		self.pathfinder = PathFindingAlgorithm()
		
	def dispatchUI(self, event):
		self.uiProcessor.dispatch(self.world, event)
		
	def initUI(self):
		self.stepBtn = self.uiFactory.from_image(sdl2.ext.BUTTON, RESOURCES.get_path("StepBtn.png"))
		self.stepBtn.click += stepBtnHandler
		stepButtonObject = UIElement(self.world, self.stepBtn, 20, 20)
		
		self.createPathBtn = self.uiFactory.from_image(sdl2.ext.BUTTON, RESOURCES.get_path("CreatePathBtn.png"))
		self.createPathBtn.click += updatePath
		createPathObject = UIElement(self.world, self.createPathBtn, 140, 20)
		
	def initPlayers(self):
		playerSprite = self.spriteFactory.from_image(RESOURCES.get_path("arrow.png"))
		self.player = Player(playerSprite)
		
		self.player.transform.position.x = initialPosition[0]
		self.player.transform.position.y = initialPosition[1]
		self.player.velocity.x = initialVelocity[0]
		self.player.velocity.y = initialVelocity[1]
		self.player.velocity.normalize()
		self.player.transform.rotation = velocityToAngle(self.player.velocity)
		
		targetSprite = self.spriteFactory.from_image(RESOURCES.get_path("arrow_target.png"))
		self.target = Player(targetSprite)
		self.target.transform.position.x = targetPosition[0]
		self.target.transform.position.y = targetPosition[1]
		self.target.velocity.x = targetVelocity[0]
		self.target.velocity.y = targetVelocity[1]
		self.target.velocity.normalize()
		self.target.transform.rotation = velocityToAngle(self.target.velocity)
		
	def processWorld(self):
		self.world.process()
		
	def render(self):
		self.renderer.clear(CLEAR_COLOR)
		
		self.player.render(self.renderer)
		self.target.render(self.renderer)
		
		#Render target line
		targetLineFrom = vec2f(0, ((-self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		targetLineTo = vec2f(width, ((width - self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		adLineFrom1 = vec2f(0, ((-self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		adLineTo1 = vec2f(width, ((width - self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		adLineFrom2 = vec2f(0, ((-self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		adLineTo2 = vec2f(width, ((width - self.target.transform.position.x) / self.target.velocity.x) * self.target.velocity.y + self.target.transform.position.y)
		adLineFrom1 += vec2f(0, -attractionDistance)
		adLineFrom2 += vec2f(0, attractionDistance)
		adLineTo1 += vec2f(0, -attractionDistance)
		adLineTo2 += vec2f(0, attractionDistance)
		self.renderer.draw_line((int(targetLineFrom.x), int(targetLineFrom.y), int(targetLineTo.x), int(targetLineTo.y)), YELLOW_COLOR)
		self.renderer.draw_line((int(adLineFrom1.x), int(adLineFrom1.y), int(adLineTo1.x), int(adLineTo1.y)), YELLOW2_COLOR)
		self.renderer.draw_line((int(adLineFrom2.x), int(adLineFrom2.y), int(adLineTo2.x), int(adLineTo2.y)), YELLOW2_COLOR)
		
		# Render path
		for pfrom, pto in pairwise(self.pathPoints):
			self.renderer.draw_line((int(pfrom.x), int(pfrom.y), int(pto.x), int(pto.y)), RED_COLOR)
			self.renderer.fill((int(pto.x) - 1, int(pto.y) - 1, 3, 3), RED_COLOR)
		
		# Render players
		self.renderer.copy(self.stepBtn, srcrect=(0, 0, 100, 40), dstrect=(20, 20, 100, 40))
		self.renderer.copy(self.createPathBtn, srcrect=(0, 0, 100, 40), dstrect=(140, 20, 100, 40))
		
		self.renderer.present()
		
# ==================================

def run():
	running = True
	event = sdl2.SDL_Event()
	
	while running:
		while sdl2.SDL_PollEvent(ctypes.byref(event)) != 0:
			if event.type == sdl2.SDL_QUIT:
				running = False
				break
			
		context.dispatchUI(event)	
		context.processWorld()
		context.render()
		
	return 0
	
# ==================================

context = Context()
if __name__ == "__main__":
	sys.exit(run())
	
# ==================================
