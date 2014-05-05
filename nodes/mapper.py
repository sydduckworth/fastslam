#!/usr/bin/python
import random
import atexit
import copy
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

class Mapper(object):
	"""
		This class manages a set of particles and contains a run() function that is the main loop
		of the algorithm.
	"""
	iteration = 0 #Keeps track of the number of times that the particles have been resampled.
	def __init__(self, dimensions, step):
		self.dimensions = dimensions
		self.step = step
		self.map = OccupancyGrid(self.dimensions, self.step)
		self.current_scan = LaserScan()
		self.current_pose = RobotPose()
		self.sensor_model = SensorModelNarrowNoIntensity()
		self.got_scan = False
		self.first_odom = True
		self.start = True
		
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
		self.current_scan = data

		self.got_scan = True
		#print("# of Scans: " + str(len(data.ranges))) #512 scans for our laser range finder!

	def odom_callback(self, data):
		[r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		self.current_pose.x = data.pose.pose.position.x
		self.current_pose.y = data.pose.pose.position.y
		self.current_pose.theta = yaw
		self.first_odom = True

	def run(self, event = None):
		while not rospy.is_shutdown():
			if self.got_scan and self.first_odom and self.start:
				#self.current_scan = self.new_scan
				self.map = self.sensor_model.update_map(self.current_scan, self.current_pose, self.map)
				try:
					gridToNpyFile(self.map, self.current_pose, "./maps", "map" + str(Mapper.iteration))
					#gridToNpyFile(self.map, self.current_pose, "./maps", "map_final")
				except:
					pass
				Mapper.iteration += 1
				rospy.loginfo(str(Mapper.iteration))
				self.got_scan = False

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
		