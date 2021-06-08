###
### 
### 
### 
### 
###

#%%
import math as m
import numpy as np
import gtsam
import pendulum_data as pdata
import pendulum_factors as pfactors
import pendulum_model as pmodel

from detectBall import intersect_over_union_2circs

import time

#%%

graph = gtsam.NonlinearFactorGraph()

use_combined_meas = True

num_test = 10

data_folder = 'data/collect1'
start_at = 30

# data_folder = 'data/collect2'
# start_at=45

# data_folder = 'data/collect3'
# start_at=10

# data_folder = 'data/collect4'
# start_at=15

config, measurements = pdata.get_data(data_folder=data_folder)


# include the last estimated measurement so we have dt for it
test_data = measurements[-num_test-1:]

# Only optimize on these values
measurements = measurements[start_at: -num_test]

########## SETUP FACTORS ###########

# Value Keys:
#
#	N			: hook position
#	0 to N-1	: ball positions through time
#
# Custom factors need a List of keys (indices), an optional measurement element, and a noise model

factors = []


hook_key = len(measurements)


for i, meas in enumerate(measurements):

	if not use_combined_meas:
		cen = pfactors.PenCenterMeasFactor(keys=[i], noise=config.center_cov, measurement=meas.center, config=config)
		rad = pfactors.PenRadiusMeasFactor(keys=[i], noise=config.radii_cov, measurement=meas.radius, config=config)

		factors.append(cen)
		factors.append(rad)
	else:
		m = np.concatenate((meas.center, [meas.radius]))
		mf = pfactors.PenMeasFactor(keys=[i], noise=config.combined_cov, measurement=m, config=config)

		factors.append(mf)

# N-1 time steps (dynamics)
for i in range(0, len(measurements)-1):
	
	# -1 is the key for the hook position
	mf = pfactors.PenDynFactor(keys=[i, i+1, hook_key], noise=config.Q, dt=measurements[i].dt, config=config)
	factors.append(mf)



for f in factors:
	graph.add(gtsam.CustomFactor(f.noise, f.keys, f.error_func))






########## INSERT VALUES ###########


### Load all Values
initial_estimates = gtsam.Values()

# Do I need a velocity??
hook = np.array([0,1,-2.], order='F')

# Insert the initial hook location
initial_estimates.insert(hook_key, hook)


for i in range(len(measurements)):
	
	initial = np.zeros(6, dtype=float, order='F')
	initial[:3] = hook + config.gravity/10

	# Values need a key and data (I think I can just use Numpy arrays)
	initial_estimates.insert(i, initial)




########## OPTIMIZE ###########

params = gtsam.LevenbergMarquardtParams()


print('Starting trial')

num_trials = 1

start_time = time.time()
last_time = start_time

for i in range(0, num_trials):
	#optimize using Levenberg-Marquardt optimization

	#params.setVerbosity('ERROR')
	optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimates, params)

	result = optimizer.optimize()

	print(f'Trial {i+1} complete after {time.time() - last_time} seconds')
	last_time = time.time()

elapsed = time.time() - start_time

print(f'Average time to optimize: {elapsed/num_trials :.03f} seconds')





res_list = []
for k in result.keys():
	res_list.append(result.atVector(k))

#print(f'results:\n{res_list}')










after_x = res_list[-2]
hook_loc = res_list[-1]


print(f'Final pendulum position:\n{after_x}')
print(f'Final hook position:\n{hook_loc}')

IOUs = []

for i in range(1, len(test_data)):
	meas = test_data[i]
	dt 	 = test_data[i-1].dt

	after_x = pmodel.f(after_x, hook_loc, dt, gravity=config.gravity, need_F=False, k_a=config.air_res)

	pred_circle = (pmodel.h_center(after_x, config, False), pmodel.h_radius(after_x, config, False))
	IOUs.append(intersect_over_union_2circs((meas.center, meas.radius), pred_circle))
	print(f'pred_circle is {pred_circle}, from im is {meas.center},{meas.radius}, IOU {IOUs[-1]}')
	
IOUs = np.array(IOUs)
print('average IOUs is ', np.average(IOUs) )



# %%
