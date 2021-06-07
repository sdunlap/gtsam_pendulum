#
# This file implements all the factors. 
# Each factor class must have:
#   keys		: List of unsigned integers
#	noise 		: gtsam.noiseModel
#	measurement : any type (used by the class)
#   other       : any thing the class needs to calculate error
#
#   error_func(...) - see gtsam custom factor docs
#
#
# To add a custom factor to the graph:
#   f = pdata.PenCenterMeasFactor(keys=[2], measurement=meas.center, noise=config.center_cov)
#   graph.add(gtsam.CustomFactor(f.noise, f.keys, f.error_func))
#

from dataclasses import dataclass

# Needed for config params
import pendulum_data as pdata
import pendulum_model as pmodel

import numpy as np
from math import pi, sqrt, isclose


import gtsam
from typing import List
from dataclasses import dataclass



@dataclass
class PenMeasFactor:
	keys		: List # size=1
	noise 		: gtsam.noiseModel
	measurement : List # size=3 (radius last)
	config		: pdata.ExpConfig
	
	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		state = v.atVector(this.keys()[0])

		pred_cen, H_cen = pmodel.h_center(state, self.config, need_H=True)
		pred_rad, H_rad = pmodel.h_radius(state, self.config, need_H=True)

		error = np.concatenate((pred_cen, [pred_rad])) - self.measurement

		if not H is None:
			H[0] = np.vstack((H_cen, H_rad))

		return error






@dataclass
class PenCenterMeasFactor:
	keys		: List # size=1
	noise 		: gtsam.noiseModel
	measurement : List # size=2
	config		: pdata.ExpConfig
	
	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		state = v.atVector(this.keys()[0])

		pred, myH = pmodel.h_center(state, self.config, need_H=True)

		error = pred - self.measurement

		if not H is None:
			H[0] = myH

		return error




@dataclass
class PenRadiusMeasFactor:
	keys		: List # size=1
	noise 		: gtsam.noiseModel
	measurement : List # size=1
	config		: pdata.ExpConfig
	
	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		state = v.atVector(this.keys()[0])

		pred, myH = pmodel.h_radius(state, self.config, need_H=True)

		error = pred - self.measurement

		if not H is None:
			H[0] = myH

		return np.array([error])






@dataclass
class PenDynFactor:
	keys		: List # size=2
	noise 		: gtsam.noiseModel
	# Change in time from current image (i) to the next image (i+1)
	dt			: float
	config		: pdata.ExpConfig
		

	def error_func(self, this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
		# Get the variable values from `v`
		x0   = v.atVector(this.keys()[0])
		x1   = v.atVector(this.keys()[1])
		hook = v.atVector(this.keys()[2])

		gravity = self.config.gravity
		air_res = self.config.air_res
		
		# Calculate non-linear error
		pred, F = pmodel.f(curr_x=x0, hook_point=hook, dt=self.dt, gravity=gravity, need_F=True, k_a=air_res)
		
		error = pred - x1

		# If we need Jacobian
		if H is not None:
			# Fill the Jacobian arrays
			# Note we have two vars, so two entries
			H[0] = F[0:6,0:6]
			H[1] = -np.eye(6, dtype=float)
			H[2] = F[0:6, 6:]
		
		# Return the error
		return error









