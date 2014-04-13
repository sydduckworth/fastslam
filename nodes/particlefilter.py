import random
import roslib; roslib.load_manifest("fastslam")
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from robot_pose import *
from occupancygrid import *



class ParticleFilter(object):
	pose_noise = 1.0 #Std of initial pose noise
	percent_random = .1 #Number of random particles to add for each resampling
	def __init__(self, num_particles, robot, dimensions, step):
		self.num_particles = num_particles
		self.p_set = []
		self.dimensions = dimensions
		self.step = step
		self.current_scan = LaserScan()
		self.current_pose = RobotPose()
		self.odom_received = False
		rospy.init_node("filter")
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		rospy.Subscriber("/odom", Odometry, self.odom_callback)

	def laser_callback(self, data):
		self.current_scan = data

	def odom_callback(self, data):
		[r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    	self.current_pose.coords = (data.pose.pose.position.x, data.pose.pose.position.y)
    	self.current_pose.theta = yaw
    	self.odom_received = True

    def run(self):
    	while not self.odom_received:
    		pass

    	self.init_particles(self.num_particles)
    	while not rospy.is_shutdown():
    		if self.odom_received:
    			self.update_particles()
    			self.odom_received = False


    def resample_particles(self):
    	random_particles = int(self.num_particles * ParticleFilter.percent_random)
    	new_particles = []
    	total_weight = 0

    	for p in self.p_set:
        	total_weight += p.weight

        for i in range(0, len(self.p_set) - random_particles):
        	r = random.uniform(0,total_weight)
        	tw = 0
	        k = 0
	        while tw < r:
	            tw = tw + particles[k].w
	            k = k + 1
	        newP = Particle(self.p_set[k-1].grid, self.p_set[k-1].pose, self.p_set[k-1].weight)
        	new_particles.append(newP)

        for i in range(0, random_particles):
        	r = random.randint(0, len(self.p_set))
        	newP =  Particle(self.p_set[r].grid, self.p_set[r].pose, self.p_set[r].weight)
        	new_particles.append(newP)

        self.p_set = new_particles


    def update_particles(self):

    def init_particles(self, num_particles):
    	for i in range(0, num_particles):
    		new_x = random.gauss(self.current_pose.coords[0], ParticleFilter.pose_noise)
    		new_y = random.gauss(self.current_pose.coords[1], ParticleFilter.pose_noise)
    		new_theta = new_x = random.gauss(self.current_pose.theta, ParticleFilter.pose_noise/2.0)
    		new_pose = RobotPose((new_x, new_y), new_theta)
    		new_grid = OccupancyGrid(self.dimensions, self.step)

    		p_temp = Particle(new_grid, new_pose, 1.0/num_particles)
    		self.p_set.append(p_temp)

    