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

@dataclass
class ExpConfig:
	# Class for holding experimental data (camera info, pendulum radius, etc)
	
	kappa		: float			
	c_x			: float			
	c_y			: float			
	gravity 	: np.recarray	
	
	# Softball is exactly 12 inches in circumference
	radius  	: float 		= 12*.0254/(2*pi)

	# Air resistance (determined by trial and error)
	air_res 	: float 		= .125
	
	radii_cov 	: float 		= 4.0
	center_cov 	: np.recarray 	= 9*np.eye(2)
	Q 			: np.recarray 	= .0001 * np.eye(6)/8


@dataclass
class Measurement:
	# Class for holding the measurement data for each image
	
	# Change in time from current image to the next image
	dt 		: float
	# measured radius of the pendulum
	radius 	: float
	# measured image center of the pendulum (x, y)
	center	: np.recarray 

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


	config = ExpConfig(kappa, c_x, c_y, gravity)

	measurements = []

	# Read in the measurement data
	im_data = np.load(os.path.join(data_folder, 'measurements.npz'))
	
	dts 	= im_data['dts']
	radii 	= im_data['radii']
	centers = im_data['centers']

	assert len(dts) + 1 == len(radii) == len(centers)

	for i in range(len(dts)):
		measurements.append(Measurement(dts[i], radii[i], centers[i]))

	return config, measurements

