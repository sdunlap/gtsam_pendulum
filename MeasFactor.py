import gtsam
import numpy as np
from typing import List
from dataclasses import dataclass



@dataclass
class MyCustomPriorFactor:
	
	expected : gtsam.Pose2
	
	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		key0 = this.keys()[0]
	   

		# Calculate non-linear error
		x = v.atPose2(key0)
		error = self.expected.localCoordinates(x)

		# If we need Jacobian
		if H is not None:
			# Fill the Jacobian arrays
			# Note we have two vars, so two entries
			H[0] = np.eye(3)
		
		# Return the error
		return error





class MyCustomFactor:

	def __init__(self, expected) -> None:
		self.expected = expected

	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		key0 = this.keys()[0]
		key1 = this.keys()[1]
		
		#print(f'Expected: {self.expected}')

		# Calculate non-linear error
		gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
		error = self.expected.localCoordinates(gT1.between(gT2))

		# If we need Jacobian
		if H is not None:
			# Fill the Jacobian arrays
			# Note we have two vars, so two entries
			result = gT1.between(gT2)
			H[0] = -result.inverse().AdjointMap()
			H[1] = np.eye(3)
		
		# Return the error
		return error




class MyCustomUnaryFactor:

	def __init__(self, x, y) -> None:
		self.mx = x
		self.my = y

	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		
		q = v.atPose2(this.keys()[0])
		
		error = np.array([ q.x() - self.mx, q.y() - self.my ])
		#print(error)

		# If we need Jacobian
		if H is not None:
			R = q.rotation()
			H[0] = np.array([	[R.c(), -R.s(), 0.0],
								[R.s(), R.c(), 0.0]])

		
		# Return the error
		return error
