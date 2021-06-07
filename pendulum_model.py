import numpy as np
import copy


def mag_sqrd(a):
	return np.linalg.norm(a)**2

def mag(a):
	return np.linalg.norm(a)






def get_F(v, dt, d, g, k_a):

	magd2 = mag_sqrd(d)
	magv2 = mag_sqrd(v)
	
	ascg = (-1/magd2) * np.array([g[0]*d[0]*d[0] + g[1]*d[0]*d[1] + g[2]*d[0]*d[2],
								g[0]*d[0]*d[1] + g[1]*d[1]*d[1] + g[2]*d[1]*d[2],
								g[0]*d[0]*d[2] + g[1]*d[2]*d[1] + g[2]*d[2]*d[2]], dtype=float)
	

	d_ascg_x = (-ascg * (2*d[0]) / magd2) - (1 / magd2) * np.array([2*g[0]*d[0] + g[1]*d[1] + g[2]*d[2],
																g[0]*d[1],
																g[0]*d[2]], dtype=float)
								
	d_ascg_y = (-ascg * (2*d[1]) / magd2) - (1 / magd2) * np.array([g[1]*d[0],
																g[0]*d[0] + 2*g[1]*d[1] + g[2]*d[2],
																g[1]*d[2]], dtype=float)

	d_ascg_z = (-ascg * (2*d[2]) / magd2) - (1 / magd2) * np.array([g[2]*d[0],
																g[2]*d[1],
																g[0]*d[0] + g[1]*d[1] + 2*g[2]*d[2]], dtype=float)
	

	d_asc_x = (magv2 / magd2) * ( (2*d[0]/magd2)*d - np.array([1., 0, 0], dtype=float) )
	d_asc_y = (magv2 / magd2) * ( (2*d[1]/magd2)*d - np.array([0., 1, 0], dtype=float) )
	d_asc_z = (magv2 / magd2) * ( (2*d[2]/magd2)*d - np.array([0., 0, 1], dtype=float) )


	d_asc_vx = -(2.0*v[0]/magd2)*d
	d_asc_vy = -(2.0*v[1]/magd2)*d
	d_asc_vz = -(2.0*v[2]/magd2)*d

	d_a_air_vx = -k_a * np.array([1., 0, 0], dtype=float)
	d_a_air_vy = -k_a * np.array([0., 1, 0], dtype=float)
	d_a_air_vz = -k_a * np.array([0., 0, 1], dtype=float)


	tmp = np.zeros((9,9), dtype=float)
	tmp[0:3,3:6] = np.eye(3, dtype=float)

	F = np.zeros((9,9), dtype=float)
	
	F[0:3,0] = 0.5 * dt * dt * (d_ascg_x + d_asc_x)
	F[0:3,1] = 0.5 * dt * dt * (d_ascg_y + d_asc_y)
	F[0:3,2] = 0.5 * dt * dt * (d_ascg_z + d_asc_z)

	F[0:3,3] = 0.5 * dt * dt * (d_asc_vx + d_a_air_vx)
	F[0:3,4] = 0.5 * dt * dt * (d_asc_vy + d_a_air_vy)
	F[0:3,5] = 0.5 * dt * dt * (d_asc_vz + d_a_air_vz)
	
	F[3:6,0] = dt * (d_ascg_x + d_asc_x)
	F[3:6,1] = dt * (d_ascg_y + d_asc_y)
	F[3:6,2] = dt * (d_ascg_z + d_asc_z)

	F[3:6,3] = dt * (d_asc_vx + d_a_air_vx)
	F[3:6,4] = dt * (d_asc_vy + d_a_air_vy)
	F[3:6,5] = dt * (d_asc_vz + d_a_air_vz)

	F[0:6, 6:] = -1.0 * F[0:6,0:3]

	return F + np.eye(9, dtype=float) + dt * tmp



####### System Implementation Functions #########
### TODO:  Fill these functions in.  For the most part
### You can copy them from PendulumEKF2 (what you did
# in the final for last class).  The one modification
# that will be required is for f() to include the air
# resistance term

# f and h function for system
def f(curr_x, hook_point, dt, gravity = np.array([0,-9.8,0]), need_F=True, k_a =.125):
	#Copy, but modify to consider air resistance!
	'''
	Takes in the current state and propagates forward in time.
	Also returns the Jacobian matrix of this function w.r.t. curr_x

	Args: 
		curr_x: the current state.  
		dt: the time to step forward
		gravity: If you want to rotate the system, use this vector
		need_F:  If you don't want the computation for F, set this to False
			If False, returns just an x, not a tuple
		k_a:  The constant to be used to simulate air resistance
	
	Returns a tuple of the next x value and the F matrix
	'''
	divider=10
	my_dt = dt/divider
	
	next_x = copy.copy(curr_x)
	position = next_x[:3]
	velocity = next_x[3:6]
	
	F = np.eye(9, dtype=float)
	
	#F = get_F(velocity, dt, position-hook_loc, gravity, k_a)

	for _ in range(divider):
		
		u_s = (position - hook_point)
		d = u_s
		u_s = u_s / np.linalg.norm(u_s)
		
		F = get_F(velocity, my_dt, d, gravity, k_a) @ F

		U = np.outer(u_s, u_s)
		
		a_scg = -U @ gravity
		
		a_sc = -u_s * mag_sqrd(velocity) / mag(position - hook_point)
		
		a_air = - k_a * velocity

		accel = a_scg + a_sc + gravity + a_air
		
		position += velocity * my_dt + accel * 0.5 * my_dt*my_dt
		velocity += my_dt * accel

	next_x[0:3] = position
	next_x[3:6] = velocity
	#next_x[6:9] = hook_point # for clarity

	if need_F:
		return (next_x,F)   
	else:
		return next_x



def h_center(curr_x, config, need_H=True):
	'''
	Args:
		curr_x: The current state of the system
		config: A dictionary with kappa, c_x, and c_y in it.
		need_H:  If the derivative of the measurement function is needed

	Returns:
		Either a 2d array that is the predicted measurement (if need_H is False)
		or a tuple with a 2d array and a 2x6 Jacobian matrix (if need_H is True)
	'''
	x = curr_x[0]
	y = curr_x[1]
	z = curr_x[2]
	k = config.kappa
	c_x = config.c_x
	c_y = config.c_y


	#Your predicted measurement code goes here
	u = (-k * x / z) + c_x
	v = (k * y / z) + c_y

	pred_meas = np.array([u,v], dtype=float)

	if need_H:
		#You can compute H here

		H = np.zeros((2,6), dtype=float)
		H[0,0] = -k / z
		#H[0,1] = 0.
		H[0,2] = k * x / (z*z)
		#H[1,0] = 0.
		H[1,1] = k / z
		H[1,2] = -k * y / (z*z)

		return (pred_meas, H)

	return pred_meas



def h_radius(curr_x, config,  need_H=True):
	'''
	Args:
		curr_x: The current state of the system
		config: A dictionary with radius (the radius of the ball) and kappa in it
		need_H:  If the derivative of the measurement function is needed

	Returns:
		Either a scalar that is the predicted measurement (if need_H is False)
		or a tuple with a scalar array and a (6,) Jacobian matrix (if need_H is True)
	'''
	kappa = config.kappa
	r_ball = config.radius

	pred_meas = -kappa * r_ball / curr_x[2]

	#You predict the measurement here
	if need_H:

		H = np.zeros((1,6), dtype=float)
		H[0,2] = kappa * r_ball / (curr_x[2]**2)

		#You compute H here...
		return (pred_meas, H)


	return pred_meas

