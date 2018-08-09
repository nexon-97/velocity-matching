import math
import constraints
from smath import *

attractionDistance = 60
		
class PathFindingAlgorithm():
	def __init__(self):
		self.pathPoints = []
		self.pathOrientations = []
		self.constraints = []
		self.executionTime = 0
		self.pathPointsCount = 40
		self.playerSpeed = 10
		self.maxSteeringAngle = radians(20)
		self.logEnabled = False
		self.pushAwayTreshold = 0.15
	
	def getPath(self, position, velocity, targetPosition, targetVelocity):
		self.pathPoints = []
		self.pathOrientations = []

		# Setup initial data for points generation
		targetFront = targetVelocity
		targetRight = rotate(targetVelocity, 90)
		playerPos = position
		targetPos = targetPosition
		playerVelocity = velocity
		playerSpeed = self.playerSpeed
		playerRotation = velocityToAngle(playerVelocity)
		oldVelocity = playerVelocity
		oldPos = playerPos

		# Add first trajectory point at current player state
		self.pathPoints.append(playerPos)
		self.pathOrientations.append(playerRotation)

		# Generate points of trajectory
		for i in xrange(self.pathPointsCount):
			self.log('====================')
			self.log('point [%d]' % i)

			frontCos = dot(playerVelocity, targetFront)
			rightCos = dot(playerVelocity, targetRight)
			self.log('frontCos: %1.3f' % frontCos)
			self.log('rightCos: %1.3f' % rightCos)
			
			distanceToTargetDirectionLine = (playerPos.x - targetPos.x) * targetVelocity.y - (playerPos.y - targetPos.y) * targetVelocity.x
			isAttractionMode = abs(distanceToTargetDirectionLine) > attractionDistance
			self.log('isAttractionMode: %s' % isAttractionMode)
			side = True if distanceToTargetDirectionLine > 0 else False
			
			self.log('distanceToTargetDirectionLine: %1.3f' % distanceToTargetDirectionLine)
			self.log('line side: %s' % ("left" if side else "right"))
			
			sideSign = 1 if side else -1
			subTargetVelocity = rotate(targetVelocity, 45 * sideSign)
			self.log('subTargetVelocity: %s' % subTargetVelocity)
			
			if isAttractionMode:
				playerVelocity += subTargetVelocity
			else:
				stabilizationFactor = 1 - (abs(distanceToTargetDirectionLine) / attractionDistance)
				
				shouldStickToTargetDirection = rightCos * sideSign < 0.1
				if rightCos * sideSign > self.pushAwayTreshold:
					self.log('Is pushing velocity away')
					
					# Predict velocity and orientation relative to the target direction line
					newVelocity = playerVelocity + (targetRight * -sideSign) * stabilizationFactor * 0.4
					newRightCos = dot(newVelocity, targetRight)
					
					oldSign = sign(rightCos)
					newSign = sign(newRightCos)
					
					if equals(oldSign, newSign):
						playerVelocity = newVelocity
					else:
						# if the new sign of relative velocity doesn't match the incoming -> then no longer need to push velocity away
						# so should stick to the target line as closer as possible
						shouldStickToTargetDirection = True
						
				if shouldStickToTargetDirection:
					self.log('Is sticking velocity to the target direction')
				
					# Find nearest position on track
					nearestPoint = playerPos + targetRight * distanceToTargetDirectionLine
					# Find point further along the track 2x player speed
					nearestPoint += targetFront * self.playerSpeed * 2
					# Get velocity in direction to the track next point
					followTargetVelocity = nearestPoint - playerPos
					followTargetVelocity.normalize()
					
					playerVelocity = followTargetVelocity
				
			# normalize output velocity
			playerVelocity.normalize()
			
			# filter velocity to match physic steering capabilities
			playerVelocity = self.filterSteering(oldVelocity, playerVelocity, self.maxSteeringAngle)

			# generate new trajectory point
			playerPos += playerVelocity * playerSpeed

			# check for constraints violation
			constraintsViolated = False
			constraintsChecked = False
			shouldFilterBackward = False

			while not constraintsChecked or constraintsViolated:
				constraintsViolated = False

				for constraint in self.constraints:
					if constraint.willViolate(playerPos, playerVelocity):
						constraintsViolated = True
						print('Constraint violated - %s! Suggesting a new point' % constraint)
						suggestion = constraint.suggestPoint(oldPos, oldVelocity, playerPos, playerVelocity)
						playerPos = suggestion[0]
						playerVelocity = suggestion[1]
						shouldFilterBackward = True

				constraintsChecked = True

			# store point info in output
			self.pathPoints.append(playerPos)
			playerRotation = velocityToAngle(playerVelocity)
			self.pathOrientations.append(playerRotation)

			# apply backward filtering
			if shouldFilterBackward:
				for j in xrange(i + 1, 2, -1):
					print('shouldFilterBackward step %d' % j)
					pFrom = (self.pathPoints[j], -angleToVelocity(self.pathOrientations[j]))
					#velocityTo = self.pathPoints[j - 2] - self.pathPoints[j - 1]
					#velocityTo.normalize()
					pTo = (self.pathPoints[j - 1], -angleToVelocity(self.pathOrientations[j - 1]))
					filterResult = self.filterSteeringAlt(pFrom, pTo, self.maxSteeringAngle)

					if filterResult[0]:
						self.pathPoints[j - 1] = filterResult[1]
						self.pathOrientations[j - 1] = velocityToAngle(-filterResult[2])
						if j >= 2:
							diff = self.pathPoints[j - 1] - self.pathPoints[j - 2]
							diff.normalize()
							self.pathOrientations[j - 2] = velocityToAngle(diff)
						print('filtering occured!')
					else:
						break

			oldPos = playerPos	
			oldVelocity = playerVelocity
			
		# Return result path
		return { "points": self.pathPoints, "rotations": self.pathOrientations }

	def filterSteering(self, currentVelocity, newVelocity, maxSteeringAngle):
		cos = clamp(dot(currentVelocity, newVelocity), -1, 1)
		angle = math.acos(cos)
		
		if angle > maxSteeringAngle:
			right = rotate(currentVelocity, 90)
			side = sign(dot(right, newVelocity))
			
			filtered = rotate(currentVelocity, degrees(maxSteeringAngle) * side)
			return filtered
			
		else:
			return newVelocity

	# Returns tuple, where the first element is the result, if the points were modified
	# If they were modified - new point is stored in tuple 2nd and 3rd positions
	def filterSteeringAlt(self, pointSrc, pointDest, maxSteeringAngle):
		srcVelocity = pointSrc[1]
		destVelocity = pointDest[1]
		posDelta = pointDest[0] - pointSrc[0]
		posDelta.normalize()
		#print('posDelta: %s' % posDelta)
		print('srcVelocity: %s' % srcVelocity)
		print('destVelocity: %s' % destVelocity)

		cos = clamp(dot(srcVelocity, destVelocity), -1, 1)
		angle = math.acos(cos)
		
		print('angle: %1.3f' % angle)
		if angle > maxSteeringAngle:
			right = rotate(srcVelocity, 90)
			side = sign(dot(right, destVelocity))
			
			filtered = rotate(srcVelocity, degrees(maxSteeringAngle) * side)
			print('filtered: %s' % filtered)
			newPos = pointSrc[0] + filtered * self.playerSpeed
			return (True, newPos, filtered)
		else:
			return (False,)
			
	def log(self, msg):
		if self.logEnabled:
			print(msg)
