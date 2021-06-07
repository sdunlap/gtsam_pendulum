'''
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Simple robot motion example, with prior and two odometry measurements
Author: Frank Dellaert
'''
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import numpy as np

import gtsam

import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot
from typing import List, Optional
from functools import partial


def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: Optional[List[np.ndarray]], mx, my):
	q = v.atPose2(this.keys()[0])
	error = np.array([ q.x() - mx, q.y() - my ], dtype=float)
	# If we need Jacobian
	if H is not None:
		R = q.rotation()
		H[0] = np.array([	[R.c(), -R.s(), 0.0],
							[R.s(), R.c(), 0.0]], dtype=float)
	# Return the error
	return error

### 
### @file LocalizationExample.cpp
### @brief Simple robot localization example, with three 'GPS-like' measurements
### @author Frank Dellaert
### 

### 
### A simple 2D pose slam example with 'GPS' measurements
###  - The robot moves forward 2 meter each iteration
###  - The robot initially faces along the X axis (horizontal, to the right in 2D)
###  - We have full odometry between pose
###  - We have 'GPS-like' measurements implemented with a custom factor
### 

# 1. Create a factor graph container and add factors to it
graph = gtsam.NonlinearFactorGraph()

ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))

# Add odometry factors
odometry = gtsam.Pose2(2.0, 0.0, 0.0)
# For simplicity, we will use the same noise model for each odometry factor
# Create odometry (Between) factors between consecutive poses
graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
#print(f'\nFactor Graph:\n{graph}')


# 2b. Add 'GPS-like' measurements
# We will use our custom UnaryFactor for this.

UNARY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))

graph.add(gtsam.CustomFactor(UNARY_NOISE, gtsam.KeyVector([1]), partial(error_func, mx=0.0, my=0.0)))
graph.add(gtsam.CustomFactor(UNARY_NOISE, gtsam.KeyVector([2]), partial(error_func, mx=2.0, my=0.0)))
graph.add(gtsam.CustomFactor(UNARY_NOISE, gtsam.KeyVector([3]), partial(error_func, mx=4.0, my=0.0)))
print(f'\nFactor Graph:\n{graph}')

# 3. Create the data structure to hold the initialEstimate estimate to the solution
# For illustrative purposes, these have been deliberately set to incorrect values
initial = gtsam.Values()
initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))
print('\nInitial Estimate:\n{}'.format(initial))




#%%
# optimize using Levenberg-Marquardt optimization
params = gtsam.LevenbergMarquardtParams()

optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)

#%%

before = optimizer.values()

result = optimizer.optimize()

print(f'Initial values:\n{before}')
print(f'Final values:\n{result}')

# %%
