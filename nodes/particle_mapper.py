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
		self.got_scan = False
		self.first_odom = True
		self.start = True
		
		rospy.init_node("mapper")
		rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		rospy.Subscriber("/joy", Joy, self.joy_callback)
		#atexit.register(self.output_maps)

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
				#update particles
				self.update_particles()
				
				self.print_random_particle_pose()
				self.got_scan = False
				Mapper.iteration += 1

	def print_random_particle_pose(self):
		i = random.randint(0, len(self.particles) - 1)
		i = 2
		x = self.particles[i].pose.x
		y = self.particles[i].pose.y
		theta = math.degrees(self.particles[i].pose.theta)
		print("Random Pose: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")

	def update_particles(self):
		#1. advance physics using motion model
		self.advance_particles()
		#2. get particle weights using sensor model
		#3. update the occupancy grid
		#4. resample the particles

	def advance_particles(self):
		"""
			Use motion model to advance the particles
		"""
		for i in xrange(0, len(self.particles)):
			#for now hardcode motion model
			#add random noise to pose delta (only if we've actually detected movement)
			dx = random.gauss(self.pose_delta[0], Mapper.pose_noise) if self.pose_delta[0] != 0 else 0
			dy = random.gauss(self.pose_delta[1], Mapper.pose_noise) if self.pose_delta[1] != 0 else 0
			dtheta = random.gauss(self.pose_delta[2], Mapper.pose_noise) if self.pose_delta[2] != 0 else 0
			#update particle pose
			self.particles[i].pose.x += dx
			self.particles[i].pose.y += dy
			self.particles[i].pose.theta = wrap_angle(self.particles[i].pose.theta + dtheta)


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
		for i in range(0, self.iteration, 10):
			try:
				print("Generating map " + str(i) + "...")
				npyToMapIMG("./maps/map" + str(i) + ".npy", self.dimensions, self.step, 5)
			except:
				pass

		"""print("Generating final map...")
		npyToMapIMG("./maps/map_final.npy", self.dimensions, self.step, 3)"""

if __name__ == "__main__":
	rospy.loginfo("Starting")
	map_size = (40, 40) #Map dimensions [-m, m] x [-n, n] in meters
	step_size = .2 #Step size in meters. Must be <= 1
	try:
		mapper= Mapper(map_size, step_size)
	except Exception as er:
		rospy.logerr(er)
	finally:
		#rospy.Timer(rospy.Duration(.5), mapper.run)
		mapper.run()
		