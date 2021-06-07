#
# This file loads in the sample data from the given collect and organizes it
# into useful data structures
#
#
#

import numpy as np
from math import pi, sqrt, isclose
import json
import os

from dataclasses import dataclass

import gtsam
from typing import List
from dataclasses import dataclass



@dataclass
class ExpConfig:
	# Class for holding experimental data (camera info, pendulum radius, etc)
	
	kappa		: float			
	c_x			: float			
	c_y			: float			
	gravity 	: np.ndarray	
	
	# Softball is exactly 12 inches in circumference
	radius  	: float 		= 12*.0254/(2*pi)

	# Air resistance (determined by trial and error)
	air_res 	: float 		= .125

									# Variamce(dim(int), variance(float))
	radii_cov 	 : float 		= gtsam.noiseModel.Isotropic.Variance(1, 4.0)
	center_cov 	 : np.ndarray 	= gtsam.noiseModel.Diagonal.Variances([9.0, 9.0]) 
	
	combined_cov : np.ndarray 	= gtsam.noiseModel.Diagonal.Variances([9.0, 9.0, 4.0]) 

	#center_cov  : np.ndarray 	= gtsam.noiseModel.Isotropic.Variance(2, 9.0) 
	Q 			 : np.ndarray 	= gtsam.noiseModel.Isotropic.Variance(6, .0001 / 8)

	# Alternative noise setup
	#  auto priorNoise = noiseModel::Diagonal::Sigmas(
    #  Vector3(0.3, 0.3, 0.1));            // 30cm std on x,y, 0.1 rad on theta



@dataclass
class Measurement:
	# Class for holding the measurement data for each image
	
	# Change in time from current image to the next image
	dt 		: float
	# measured radius of the pendulum
	radius 	: float
	# measured image center of the pendulum (x, y)
	center	: np.ndarray 

#
#
# 	data_folder: folder containing: measurements.npz, params.json and accel.npy
#
def get_data( data_folder = 'data/collect1' ):
	
	# Read in parameters
	in_params = json.load(open(os.path.join(data_folder, 'params.json')))
	kappa = sqrt(in_params['color']['fx']*in_params['color']['fy'])
	c_x = in_params['color']['ppx']
	c_y = in_params['color']['ppy']    

	# Get the gravity value
	accel_array = np.load( os.path.join(data_folder, 'accel.npy') )
	avg_accel=np.average(accel_array[:,1:],axis=0)
	# Accels measure negative gravity when fixed, but y and z
	# are flipped from where they should be, so...
	gravity = np.array([-avg_accel[0],avg_accel[1],avg_accel[2]])


	config = ExpConfig(kappa=kappa, c_x=c_x, c_y=c_y, gravity=gravity)

	measurements = []

	# Read in the measurement data
	im_data = np.load(os.path.join(data_folder, 'measurements.npz'))
	
	dts 	= im_data['dts']
	radii 	= im_data['radii']
	centers = im_data['centers']

	# Dummy measurement for the last element (should be ignored)
	dts = np.concatenate((dts, [0.0]))

	assert len(dts) == len(radii) == len(centers)

	for i in range(len(radii)):
		measurements.append(Measurement(dts[i], radii[i], centers[i]))

	return config, measurements

