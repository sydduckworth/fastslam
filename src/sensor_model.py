import math

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

#variables that alter sensor model
z_max = 0 		#maximum allowable range value from range sensor
#TODO: find out what z_hit, sigma_hit, and z_rand are!
z_hit = 0 		#??
sigma_hit = 0 	#??
z_rand = 0 		#??


'''
	params:
	  - z_t: sensor readings
	  - pose (x_t): robot pose
	  - m: map
	return: importance weight
	note: importance weight is a belief in the current sensor reading given the pose and the map
'''
def measurement_model_map(z_t = [], pose = None, m = None):
	#use likelihood_field_range_finder_model to determine weight
	return likelihood_field_range_finder_model(z_t, pose, m)

def likelihood_field_range_finder_model(z_t = [], pose = None, m = None):
	global z_max, z_hit, z_rand, sigma_hit
	
	w = 1 #importance weight
	#for each reading in z_t
	for k in xrange(0, len(z_t)):
		sens_theta = 0 #TODO: calculate sensor reading theta!
		sens_x = pose.x #Assume sensor x = robot pose x
		sens_y = pose.y #Assume sensor y = robot pose y

		if (z_t[k] < z_max):
			z_endpoint_x = pose.x + (sens_x * math.cos(pose.theta)) - (sens_y * math.sin(pose.theta)) + (z_t[k] * math.cos(pose.theta + sens_theta)) #x endpoint of sensor reading projected onto grid
			z_endpoint_y = pose.y + (sens_y * math.cos(pose.theta)) + (sens_x * math.sin(pose.theta)) + (z_t[k] * math.sin(pose.theta + sens_theta)) #y endpoint of sensor reading projected onto grid
			dist = m.get_dist_to_nearest_occupied((z_endpoint_x, z_endpoint_y)) #eucl dist to nearest object in grid
			#if the endpoint of sensor reading is uknown, we no nothing about it and we just use 1/float(z_max) as our belief for this particular reading
			if (not m[z_endpoint_x][z_endpoint_y].unknown):
				w = w * (z_hit * prob_dist(dist, sigma_hit) + (float(z_rand)/float(z_max)))
			else:
				w = w * (1.0/float(z_max))
	return w


