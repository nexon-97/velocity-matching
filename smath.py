import math

# ==================================

def dot(a, b):
	return a.x * b.x + a.y * b.y
	
def radians(deg):
	return (deg / 180.0) * math.pi
	
def degrees(rad):
	return rad * 180.0 / math.pi
		
def rotate(v, deg):
	radAngle = radians(deg)
	result = vec2f()
	result.x = v.x * math.cos(radAngle) - v.y * math.sin(radAngle)
	result.y = v.x * math.sin(radAngle) + v.y * math.cos(radAngle)
	return result
	
def velocityToAngle(v):
	return math.atan2(v.y, v.x) * 180.0 / math.pi
	
def angleToVelocity(deg):
	radAngle = radians(deg)
	return vec2f(math.cos(radAngle), math.sin(radAngle))
	
def equals(a, b):
	return abs(a - b) < 0.001
	
def lerp(a, b, t):
	return (1 - t) * a + t * b
	
def sign(a):
	return -1 if a < 0 else 1
	
def clamp(a, min, max):
	if a < min:
		return min
	if a > max:
		return max
		
	return a
	
# ==================================

class vec2f(object):
	def __init__(self, x=0.0, y=0.0):
		super(vec2f, self).__init__()
		self.x = x
		self.y = y
		
	def normalize(self):
		length = self.length()
		invLength = 1 / length
		self.x *= invLength
		self.y *= invLength
		
	def length(self):
		return math.sqrt(self.lengthSq())
	
	def lengthSq(self):
		return self.x * self.x + self.y * self.y
		
	def __add__(self, other):
		result = vec2f()
		result.x = self.x + other.x
		result.y = self.y + other.y
		return result
		
	def __sub__(self, other):
		result = vec2f()
		result.x = self.x - other.x
		result.y = self.y - other.y
		return result
		
	def __mul__(self, other):
		result = vec2f()
		result.x = self.x * other
		result.y = self.y * other
		return result

	def __neg__(self):
		return vec2f(-self.x, -self.y)
		
	def __str__(self):
		return ("vec2f(%1.3f; %1.3f)") % (self.x, self.y)
		
# ==================================