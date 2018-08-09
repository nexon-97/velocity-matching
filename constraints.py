from smath import vec2f, rotate, degrees
import math

# ==================================

class Constraint(object):
	def __init__(self):
		pass

	def suggestPoint(self, point, velocity):
		return (point, velocity)

	def willViolate(self, point, velocity):
		return False

# ==================================

class CircleCollisionConstraint(Constraint):
	def __init__(self, center, radius):
		super(CircleCollisionConstraint, self).__init__()

		self.center = center
		self.radius = radius
		self.margin = 20

		self.violateDistance = self.radius + self.margin
		self.violateDistanceSq = self.violateDistance ** 2
		self.distanceSq = 0
	
	def suggestPoint(self, prevPoint, prevVelocity, point, velocity):
		# get tangent point
		toCenter = self.center - prevPoint
		print('toCenter: %s' % toCenter)
		print('prevPoint: %s' % prevPoint)
		distanceToCenter = toCenter.length()
		print('distanceToCenter: %1.3f' % distanceToCenter)
		sin = self.violateDistance / distanceToCenter
		print('sin: %1.3f' % sin)
		cos = math.sqrt(1.0 - sin ** 2)
		print('cos: %1.3f' % cos)
		angle = math.asin(sin)
		print('angle: %1.3f' % angle)
		
		distanceToLine = (point.x - self.center.x) * toCenter.y - (point.y - self.center.y) * toCenter.x
		print(distanceToLine)
		sideSign = -1 if distanceToLine > 0 else 1

		toCenter.normalize()
		rotatedVector = rotate(toCenter, (degrees(angle) + 2) * sideSign)
		point = prevPoint + rotatedVector * (distanceToCenter * cos)
		velocity = point - prevPoint
		velocity.normalize()

		return (point, velocity)

	def willViolate(self, point, velocity):
		self.targetOffset = point - self.center
		self.distanceSq = self.targetOffset.lengthSq()
		
		return self.distanceSq < self.violateDistanceSq

# ==================================

class PolylineCollisionConstraint(Constraint):
	def __init__(self, points):
		super(PolylineCollisionConstraint, self).__init__()

		self.points = points

	def suggestPoint(self, point, velocity):
		return (point, velocity)

	def willViolate(self, point, velocity):
		return False

# ==================================

class ConstraintFactory():
	def __init__(self):
		self.factoryMethods = {
			'circle': self.fromCircleShape,
			'polyline': self.fromPolylineShape,
		}

	def fromShapeDef(self, shapeDef):
		shapeType = shapeDef['type']
		factory = self.factoryMethods[shapeType]
		if factory:
			return factory(shapeDef)

	def fromCircleShape(self, shapeDef):
		center = vec2f(shapeDef['position']['x'], shapeDef['position']['y'])
		radius = shapeDef['radius']
		constraint = CircleCollisionConstraint(center, radius)
		return constraint

	def fromPolylineShape(self, shapeDef):
		points = shapeDef['points']
		constraint = PolylineCollisionConstraint(points)
		return constraint

# ==================================

# Tries to join 2 circle constraints. If succeeded, new constraint is placed inside a instance
def tryJoinCircleConstraints(a, b, maxSpacing):
	distanceVector = a.center - b.center
	distance = distanceVector.length()
	availableDistance = a.radius + b.radius + maxSpacing

	# Join constraints
	if distance < availableDistance:
		a.radius = distance / 2.0
		a.center = (a.center + b.center) / 2.0
		return True

	return False

# ==================================
