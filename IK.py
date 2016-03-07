''' Simple IK model for the dVRK in OpenRave
	dVRK is modelled as a ball joint having 2 DOF and an extensible arm of 1 DOF
	3 DOF: Length, Theta (th) and Azimuth (az)
	th spans the XY plane and az spans the XZ plane

	Axis: X - towards you, Y - right, Z - upwards
	Reference is taken from the X axis with all angles th and az as 0

	User specified cartesian coordinates of end effector
	Jouint angles and length L is automatically calculated
	If L is beyond extension limits (Llimits), then the resultant L' = min(L , Llimit)
	th limits are defined by [th_low, th_high]. az limits are defined by [az_low, az_high]

	User parameters:
	[X,Y,Z] - End coordinates in cartesian space
'''

import math
import numpy as np

class dVRK_IK_simple:

	def __init__(self, endCoords):
		self.thLimits = [-math.pi/2, math.pi/2]
		self.azLimits = [-math.pi/2, math.pi/2]
		self.lLimits = 3								# 3 meters of extension length? 
		self.endEffector = endCoords					# desired end effector coordinates as list
		assert (type(self.endEffector) is list)

	def getDOF(self):
		# Gets the current position of the arm and the desired end effector position
		# Returns a list of [L, th, az]
		# For now assume that the arm starts off at (1,0,0) always
		basePos = np.array([1,0,0]); 
		desiredPos = np.array(self.endEffector);
		L = np.linalg.norm(desiredPos)								# Gets the desired length
		[th, az] = self.__checkSingularity(basePos, desiredPos)

		return self.__getLlimits(L, th, az)					# Returns to the user the final DOF 

	def get_endEffector_fromDOF(self):
		# Returns end effector pose from DOF input
		# Based on the relationship x^2 + y^2 + z^2 = L
		# y/x = tan(th)
		# z/x = -tan(az)
		# rtype = [x, y, z]	as list

		from math import tan
		


	def __getLlimits(self, L,th, az):
		L = min(L, self.lLimits)

		if th < self.thLimits[0]:
			th = self.thLimits[0]
		elif th > self.thLimits[1]:
			th = self.thLimits[1]

		if az < self.azLimits[0]:
			az = self.azLimits[0]
		elif az > self.azLimits[1]:
			az = self.azLimits[1]	

		return [L, th, az]	

	def __checkSingularity(self, basePos, desiredPos):
		projXY = np.array([desiredPos[0], desiredPos[1], 0]) 		# Vector projection on XY plane	
		projXZ = np.array([desiredPos[0], 0, desiredPos[2]])		# Vector projection on XZ plane

		if np.linalg.norm(projXZ) == 0:
			# Case of singularity on XY plane
			az = 0
		else:
			normXZ = projXZ /np.linalg.norm(projXZ)
			az0 = math.acos(np.dot(basePos, normXZ))			# Gets azimuth value
			az = self.__checkDir(basePos, normXZ)*az0

		if np.linalg.norm(projXY) == 0:
			# cartesianse of singularity on XZ plane
			th = 0
		else:
			normXY = projXY /np.linalg.norm(projXY)
			th0 = math.acos(np.dot(basePos, normXY))			# Gets theta value
			th = self.__checkDir(basePos, normXY)*th0

		return [th, az]

	def __checkDir(self, basePos, normPos):
		# Checks the direction of turn to be +ve or -ve
		# Turn in the +ve Z axis is +ve
		# Turn in the -ve Z axis is -ve
		# Cross product is done wrt to basePos first

		vect = np.cross(basePos, normPos)
		positiveCases = [np.array([0,-1,0]), np.array([0,0,1])]
		negativeCases = [np.array([0,1,0]), np.array([0,0,-1])]

		if np.linalg.norm(vect) == 0:
			return 1											# Catch case where vector is 0
		else:
			vect = vect /np.linalg.norm(vect)					# Normalizes vector

		if vect.tolist() in [i.tolist() for i in positiveCases]:
			return 1
		elif vect.tolist() in [ii.tolist() for ii in negativeCases]:
			return -1

if __name__ == "__main__":
	IK = dVRK_IK_simple([0,-1,0])
	print(IK.getDOF())


