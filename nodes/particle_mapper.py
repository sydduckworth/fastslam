#!/usr/bin/python
import random
import atexit
import copy
import math
import roslib; roslib.load_manifest("fastslam")
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from robot_pose import *
from occupancygrid import *
from motion_model import *
from sensor_model import *
from particle import *
from robot import *
from drawing_tools import *
from sensor_msgs.msg import Joy
from fastslam_utilities import *

class Mapper(object):
	"""
		This class manages a set of particles and contains a run() function that is the main loop
		of the algorithm.
	"""
	iteration = 0 #Keeps track of the number of times that the particles have been resampled.
	pose_noise = 0.1
	percent_random = .1
	def __init__(self, dimensions, step):
		self.dimensions = dimensions
		self.step = step
		self.map = OccupancyGrid(self.dimensions, self.step) #this stores dead reckoning map

		self.particles = []
		self.num_particles = 10
		assert self.num_particles > 0, "num_particles is <= 0!"

		self.new_scan = LaserScan()
		self.new_pose = RobotPose() 
		self.current_scan = LaserScan()
		self.current_pose = RobotPose()
		self.pose_delta = (0, 0, 0)		

		self.sensor_model = SensorModelNarrowNoIntensity()
		self.motion_model = MotionModelSimple(stds = (0.1, 0.1, 0.01))
		self.got_scan = False
		self.first_odom = False
		self.start = False
		self.produce_dr_map = False
		
		rospy.init_node("mapper")
		rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		rospy.Subscriber("/joy", Joy, self.joy_callback)
		atexit.register(self.output_maps)

	def joy_callback(self, data):
		if data.buttons[1]:
			rospy.signal_shutdown("Joystick interrupt")
		if data.buttons[0]:
			self.start = True

	def laser_callback(self, data):
		self.new_scan = data

		self.got_scan = True
		#print("# of Scans: " + str(len(data.ranges))) #512 scans for our laser range finder!

	def odom_callback(self, data):
		[r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		self.new_pose.x = data.pose.pose.position.x
		self.new_pose.y = data.pose.pose.position.y
		self.new_pose.theta = yaw
		self.first_odom = True

	def run(self, event = None):
		rospy.loginfo("Waiting for odom and laser data...")
		#wait for first odometry and scan to be received
		while not (self.first_odom and self.got_scan):
			pass
		#update current scan, current pose, and previous pose
		self.update_state_info()
		#initialize particles
		self.initialize_particles(self.current_pose)
		rospy.loginfo("Done initializing particles")

		#begin run loop
		while not rospy.is_shutdown():
			if self.got_scan and self.first_odom and self.start:
				rospy.loginfo("Iteration: " + str(Mapper.iteration))
				#update current scan and current pose
				self.update_state_info()
				#update dead reckoning map
				if self.produce_dr_map:
					self.update_deadreckoning_map()
				#update particles
				self.update_particles()
				
				#self.print_particle_pose()
				self.got_scan = False
				Mapper.iteration += 1

	def update_deadreckoning_map(self):
		self.map = self.sensor_model.update_map(self.current_scan, self.current_pose, self.map)
		try:
			gridToNpyFile(self.map, self.current_pose, "./maps", "dr_map" + str(Mapper.iteration))
			#gridToNpyFile(self.map, self.current_pose, "./maps", "map_final")
		except:
			pass

	def print_particle_pose(self, index = None):
		i = random.randint(0, len(self.particles) - 1) if index == None else index
		x = self.particles[i].pose.x
		y = self.particles[i].pose.y
		theta = math.degrees(self.particles[i].pose.theta)
		weight = self.particles[i].weight
		print("Random Pose: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")
		print("Particle weight: " + str(weight))

	def update_particles(self):
		#For each particle:
		for i in xrange(0, len(self.particles)):
			#########################################################
			########## MOTION MODEL (UPDATE PARTICLE POSE) ##########	
			x, y, theta = self.motion_model.update(self.particles[i], self.pose_delta)
			self.particles[i].pose.x = x
			self.particles[i].pose.y = y
			self.particles[i].pose.theta = theta
			#########################################################
			######## GET PARTICLE WEIGHT USING SENSOR MODEL #########
			self.particles[i].weight = self.sensor_model.update(z_t = self.current_scan, pose = self.particles[i].pose, m = self.particles[i].grid)
			#########################################################
			################# UPDATE PARTICLE MAP ###################
			self.particles[i].grid = self.sensor_model.update_map(self.current_scan, self.particles[i].pose, self.particles[i].grid)

		self.resample_particles()

	def resample_particles(self):
		random_particles = int(self.num_particles * Mapper.percent_random)
		new_particles = []
		total_weight = 0
		max_particle = 0

		#search for maximum particle to save map, and sum all particle weights
		for i in xrange(0, len(self.particles)):
			total_weight += self.particles[i].weight
			if (self.particles[i].weight > self.particles[max_particle].weight):
				max_particle = i
		#save out max particle's map	
		self.print_particle_pose(index = max_particle)
		gridToNpyFile(self.particles[max_particle].grid, self.particles[max_particle].pose, "./maps", "mp_map" + str(Mapper.iteration))

		#sample based on weights
		for i in xrange(0, len(self.particles) - random_particles):
			r = random.uniform(0, total_weight)
			tw = 0
			k = 0
			while tw < r:
				tw = tw + self.particles[k].weight
				k += 1
			new_particle = copy.deepcopy(self.particles[k - 1])
			new_particles.append(new_particle)

		#sample randomly
		for i in xrange(0, random_particles):
			r = random.randint(0, len(self.particles) - 1)
			new_particle = copy.deepcopy(self.particles[i])
			new_particles.append(new_particle)

		self.particles = new_particles

	def update_state_info(self):
		"""
			This function updates current scan and pose with new information when called.
			It also updates the pose delta.
		"""
		#calculate deltas
		dx = self.new_pose.x - self.current_pose.x
		dy = self.new_pose.y - self.current_pose.y
		dtheta = self.new_pose.theta - self.current_pose.theta
		#update pose delta, current scan, and current pose
		self.pose_delta = (dx, dy, dtheta)
		self.current_scan = copy.deepcopy(self.new_scan)
		self.current_pose = copy.deepcopy(self.new_pose)


	def initialize_particles(self, seed_pose):
		"""
			Initializes a the set of particles.  All particles have a pose equal to the seed pose 
			with some random noise.
		"""
		for i in xrange(0, self.num_particles):
			#initialize new values for new particles
			new_x = random.gauss(seed_pose.x, Mapper.pose_noise)
			new_y = random.gauss(seed_pose.y, Mapper.pose_noise)
			new_theta = random.gauss(seed_pose.theta, Mapper.pose_noise)
			new_pose = RobotPose(new_x, new_y, new_theta)
			new_grid = OccupancyGrid(self.dimensions, self.step)
			new_particle = Particle(new_grid, new_pose, 1.0/self.num_particles)
			self.particles.append(new_particle)


	def output_maps(self):
		if self.produce_dr_map:
			for i in range(0, self.iteration, 5):
				try:
					print("Generating dr map " + str(i) + "...")
					npyToMapIMG("./maps/dr_map" + str(i) + ".npy", self.dimensions, self.step, 5)
				except:
					pass

		for i in range(0, self.iteration, 5):
			try:
				print("Generating max particle map " + str(i) + "...")
				npyToMapIMG("./maps/mp_map" + str(i) + ".npy", self.dimensions, self.step, 5)
			except:
				pass

		"""print("Generating final map...")
		npyToMapIMG("./maps/map_final.npy", self.dimensions, self.step, 3)"""

if __name__ == "__main__":
	rospy.loginfo("Starting")
	map_size = (20, 20) #Map dimensions [-m, m] x [-n, n] in meters
	step_size = .2 #Step size in meters. Must be <= 1
	try:
		mapper= Mapper(map_size, step_size)
	except Exception as er:
		rospy.logerr(er)
	finally:
		#rospy.Timer(rospy.Duration(.5), mapper.run)
		mapper.run()
		