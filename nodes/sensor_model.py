import math
import prob_utilities as prob_util

"""
Key assumptions:
	- Everyone needs to agree on Theta in robot's pose
	- Pose X_t is in grid coordinates
	- Will have access to a nearest neighbor funtion that returns the nearest (euclidean dist) occupied grid space given a grid space.
		- get_dist_to_nearest_occupied((x, y)) 
			- this function should also take a parameter that defines the local area around (x, y) to search 
	- Grid cells are either occuppied, free, or unknown (it is very important that unknown grid cells are marked as such)
	- Assume rangefinder shares x,y location of robot pose 
"""

class Simple_Laser_Sensor_Model():

	def __init__(self, stddev = 2.5):
		self.stddev = stddev

	'''
	measurement_model_map
		params:
		  - z_t: sensor readings (Laser_Scan())
		  - pose (x_t): robot pose
		  - m: map
		return: importance weight
		note: importance weight is a belief in the current sensor reading given the pose and the map
	'''
	def measurement_model_map(self, z_t = [], pose = None, m = None):
		return self.simple_laser_model(z_t, pose, m)

	'''
	simple_laser_model
		params:
		 - see above function
		returns the probability of receiving sensor reading z_t given the current pose and a map
		 - P(z|x_t, m_t-1)
	'''
	def simple_laser_model(self, z_t, pose, m):
		result = 1.0
		cur_angle = z_t.angle_min 				#store current angle in radians
		inc_angle = z_t.angle_increment 		#angle increment between scans in radians
		for i in xrange(0, len(z_t.ranges)):
			#get the expected distance to obstacle
			expected_distance = m.rayTrace(pose.loc, cur_angle + pose.theta)
			result *= self.getProbReadingGivenDistance(z_t.ranges[i], expected_distance)
			cur_angle += inc_angle
		return result

	def getProbReadingGivenDistance(self, sensor_distance = 0, expected_distance = 0):
		#alpha is 1.0 divided by the area of the normal curve that is to the right of 0.0
		#Basically, it is the renormalization constant.
		alpha = 1.0/(1.0 - prob_util.getCND(0.0, expected_distance, self.stddev))
		return alpha * prob_util.getProbND(sensor_distance, expected_distance, self.stddev)
