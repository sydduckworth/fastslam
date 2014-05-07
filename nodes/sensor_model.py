import math
import random
from fastslam_utilities import *
from scipy import stats
import numpy

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

class SensorModelSimple(object):

	def __init__(self, stddev = 2.5, object_thickness = 1.0):
		self.stddev = stddev
		self.object_thickness = object_thickness
	'''
	update
		params:
		  - z_t: sensor readings (Laser_Scan())
		  - pose (x_t): robot pose
		  - m: map
		return: importance weight
		note: importance weight is a belief in the current sensor reading given the pose and the map
	'''
	def update(self, z_t, pose, m):
		#TODO: probably don't want to use all range scans
		scan_step = 50
		result = 1.0
		cur_angle = z_t.angle_min 				#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step 		#angle increment between scans in radians
		range_max = z_t.range_max
		for i in xrange(0, len(z_t.ranges), scan_step):
			object_coords = m.rayTrace((pose.x, pose.y), cur_angle + pose.theta)
			#get the expected distance to obstacle
			if object_coords:
				expected_distance = euclidean_distance(object_coords, (pose.x, pose.y))
				result *= self.getProbReadingGivenDistance(z_t.ranges[i], expected_distance, range_max)
			cur_angle += inc_angle
		return result

	def getProbReadingGivenDistance(self, sensor_distance = 0, expected_distance = 0, max_distance = 0):
		#alpha is 1.0 divided by the area of the normal curve that is to the right of 0.0
		#Basically, it is the renormalization constant.
		alpha = 1.0/(1.0 - getCND(0.0, expected_distance, self.stddev))
		beta = 1.0 - getCND(max_distance, expected_distance, self.stddev) #probability of max range
		if (sensor_distance >= max_distance):
	 		return beta
		return alpha * getND(sensor_distance, expected_distance, self.stddev)

	def update_map(self, z_t, pose, m):
		scan_step = 1
		cur_angle = z_t.angle_min 				#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step		#angle increment between scans in radians
		range_max = z_t.range_max

		for r in xrange(0, len(z_t.ranges), scan_step):
			#Discard any infinite readings
			if not math.isnan(z_t.ranges[r]) and not math.isinf(z_t.ranges[r]):
				# For each reading, mark cell being sensed and all cells in an arc around it as occupied
				init_angle = cur_angle + pose.theta - inc_angle
				end_angle = cur_angle + pose.theta + inc_angle
				step = .5 * math.atan2(m.step, r) #This ensures that the width of each step is never greater than the grid size
				while init_angle <= end_angle:
					point_x = z_t.ranges[r] * math.cos(init_angle) + pose.x
					point_y = z_t.ranges[r] * math.sin(init_angle) + pose.y
					#Clear all points up to the sensed point, then set the sensed point to occupied
					m.clearTo((pose.x, pose.y), (point_x, point_y))
					if z_t.intensities[r] == 1.0:
						#m.fillRect((point_x, point_y), .6, .6)
						#m[point_x][point_y] = True
						obj_x = self.object_thickness * math.cos(init_angle) + point_x
						obj_y = self.object_thickness * math.sin(init_angle) + point_y
						m.fillTo((point_x, point_y), (obj_x, obj_y))
					init_angle += step

			cur_angle += inc_angle
		return m


class SensorModelNarrow(object):

	def __init__(self, stddev = 2.5, object_thickness = 1.0):
		self.stddev = stddev
		self.object_thickness = object_thickness
		self.scan_step = 30

	'''
	update
		params:
		  - z_t: sensor readings (Laser_Scan())
		  - pose (x_t): robot pose
		  - m: map
		return: importance weight
		note: importance weight is a belief in the current sensor reading given the pose and the map
	'''
	def update(self, z_t, pose, m):
		scan_step = self.scan_step
		result = 0
		cur_angle = z_t.angle_min 							#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step 		#angle increment between scans in radians
		range_max = z_t.range_max
		normalization_constant = 0
		for i in xrange(0, len(z_t.ranges), scan_step):
			object_coords = m.rayTrace((pose.x, pose.y), pose.theta - cur_angle) 
			#get the expected distance to obstacle
			if object_coords:
				expected_distance = euclidean_distance(object_coords, (pose.x, pose.y))

				#check if scan is nan or inf
				if math.isnan(z_t.ranges[i]) or math.isinf(z_t.ranges[i]):
					#if so, set scan to max range
					scan = range_max
				else:
					#else, scan reading is okay
					scan = z_t.ranges[i]
				#get probability of the reading given the expected reading and update result
				result += self.getProbReadingGivenDistance(scan, expected_distance, range_max)
				normalization_constant += 1 #max probability = 1, add max probability of scan to normalization constant
			cur_angle += inc_angle
		normalization_constant = 1 if normalization_constant == 0 else normalization_constant
		result = result/float(normalization_constant) #normalize the result
		return result

	def getProbReadingGivenDistance(self, sensor_distance = 0, expected_distance = 0, max_distance = 0):
		#alpha is 1.0 divided by the area of the normal curve that is to the right of 0.0
		#Basically, it is the renormalization constant.
		alpha = 1.0/(1.0 - getCND(0.0, expected_distance, self.stddev))
		beta = 1.0 - getCND(max_distance, expected_distance, self.stddev) #probability of max range
		if (sensor_distance >= max_distance):
	 		return beta
		return alpha * getND(sensor_distance, expected_distance, self.stddev)

	def update_map(self, z_t, pose, m):
		scan_step = 1
		cur_angle = z_t.angle_min + pose.theta 			#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step		#angle increment between scans in radians
		range_max = z_t.range_max

		for r in xrange(0, len(z_t.ranges), scan_step):
			#Discard any infinite readings
			if not math.isnan(z_t.ranges[r]) and not math.isinf(z_t.ranges[r]):
				point_x = z_t.ranges[r] * math.cos(cur_angle) + pose.x
				point_y = z_t.ranges[r] * math.sin(cur_angle) + pose.y
				#Clear all points up to the sensed point, then set the sensed point to occupied
				m.clearTo((pose.x, pose.y), (point_x, point_y))
				if z_t.intensities[r] == 1.0:
					#m.fillRect((point_x, point_y), .6, .6)
					#m[point_x][point_y] = True
					obj_x = self.object_thickness * math.cos(cur_angle) + point_x
					obj_y = self.object_thickness * math.sin(cur_angle) + point_y
					m.fillTo((point_x, point_y), (obj_x, obj_y))
			cur_angle += inc_angle
		return m


class SensorModelNarrowNoIntensity(object):

	def __init__(self, stddev = 2.5, object_thickness = 1.0):
		self.stddev = stddev
		self.object_thickness = object_thickness

	'''
	update
		params:
		  - z_t: sensor readings (Laser_Scan())
		  - pose (x_t): robot pose
		  - m: map
		return: importance weight
		note: importance weight is a belief in the current sensor reading given the pose and the map
	'''
	def update(self, z_t, pose, m):
		#TODO: probably don't want to use all range scans
		scan_step = 30
		result = 0
		cur_angle = z_t.angle_min 				#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step 		#angle increment between scans in radians
		range_max = z_t.range_max
		normalization_constant = 0
		for i in xrange(0, len(z_t.ranges), scan_step):
			object_coords = m.rayTrace((pose.x, pose.y), pose.theta - cur_angle) #TODO: correct angle??
			#get the expected distance to obstacle
			if object_coords:
				expected_distance = euclidean_distance(object_coords, (pose.x, pose.y))

				#check if scan is nan or inf
				if math.isnan(z_t.ranges[i]) or math.isinf(z_t.ranges[i]):
					#if so, set scan to max range
					scan = range_max
				else:
					#else, scan reading is okay
					scan = z_t.ranges[i]
				#get probability of the reading given the expected reading and update result
				result += self.getProbReadingGivenDistance(scan, expected_distance, range_max)
				normalization_constant += 1 #max probability = 1, add max probability of scan to normalization constant
			cur_angle += inc_angle
		normalization_constant = 1 if normalization_constant == 0 else normalization_constant
		result = result/float(normalization_constant) #normalize the result
		return result

	def getProbReadingGivenDistance(self, sensor_distance = 0, expected_distance = 0, max_distance = 0):
		#alpha is 1.0 divided by the area of the normal curve that is to the right of 0.0
		#Basically, it is the renormalization constant.
		alpha = 1.0/(1.0 - getCND(0.0, expected_distance, self.stddev))
		beta = 1.0 - getCND(max_distance, expected_distance, self.stddev) #probability of max range
		if (sensor_distance >= max_distance):
	 		return beta
		return alpha * getND(sensor_distance, expected_distance, self.stddev)

	def update_map(self, z_t, pose, m):
		scan_step = 1
		cur_angle = z_t.angle_min + pose.theta 			#store current angle in radians
		inc_angle = z_t.angle_increment * scan_step		#angle increment between scans in radians
		range_max = z_t.range_max

		for r in xrange(0, len(z_t.ranges), scan_step):
			#Discard any infinite readings
			if math.isnan(z_t.ranges[r]) or math.isinf(z_t.ranges[r]):
				scan = range_max
			else:
				scan = z_t.ranges[r]
			point_x = scan * math.cos(cur_angle) + pose.x
			point_y = scan * math.sin(cur_angle) + pose.y
			#Clear all points up to the sensed point, then set the sensed point to occupied
			m.clearTo((pose.x, pose.y), (point_x, point_y))
			if scan != range_max:
				#m.fillRect((point_x, point_y), .6, .6)
				#m[point_x][point_y] = True
				obj_x = self.object_thickness * math.cos(cur_angle) + point_x
				obj_y = self.object_thickness * math.sin(cur_angle) + point_y
				m.fillTo((point_x, point_y), (obj_x, obj_y))
			cur_angle += inc_angle
		return m
