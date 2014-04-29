#!/usr/bin/python
import random
import atexit
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
		self.new_scan = LaserScan()
		self.new_pose = RobotPose()
		self.temp_pose = RobotPose()
		self.sensor_model = SensorModelNarrow()
		self.got_scan = False
		
		rospy.init_node("mapper")
		rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		atexit.register(self.output_maps)

	def laser_callback(self, data):
		self.new_scan = data
		self.new_pose = self.temp_pose
		self.got_scan = True
		#print("# of Scans: " + str(len(data.ranges))) #512 scans for our laser range finder!

	def odom_callback(self, data):
		[r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		self.temp_pose.x = data.pose.pose.position.x
		self.temp_pose.y = data.pose.pose.position.y
		self.temp_pose.theta = yaw


	def run(self):
		rospy.loginfo("Done initializing particles")
		while not rospy.is_shutdown():
			if self.got_scan:
				self.map = self.sensor_model.update_map(self.current_scan, self.current_pose, self.map)
				self.current_pose = self.new_pose
				self.current_scan = self.new_scan
				gridToNpyFile(self.map, self.current_pose, "./maps", "map" + str(Mapper.iteration))
				Mapper.iteration += 1
				rospy.loginfo(str(Mapper.iteration))
				self.got_scan = False


	def output_maps(self):
		for i in range(0, self.iteration, 10):
			print("Generating map " + str(i) + "...")
			npyToMapIMG("./maps/map" + str(i) + ".npy", self.dimensions, self.step, 5)

if __name__ == "__main__":
	rospy.loginfo("Starting")
	map_size = (20, 20) #Map dimensions [-m, m] x [-n, n] in meters
	step_size = .2 #Step size in meters. Must be <= 1
	try:
		mapper= Mapper(map_size, step_size)
	except Exception as er:
		rospy.logerr(er)
	finally:
		mapper.run()