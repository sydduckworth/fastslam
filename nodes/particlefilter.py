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

class ParticleFilter(object):
    """
        This class manages a set of particles and contains a run() function that is the main loop
        of the algorithm.
    """
    pose_noise = 1.0 #Std of initial pose noise
    percent_random = .1 #Number of random particles to add for each resampling
    iteration = 0 #Keeps track of the number of times that the particles have been resampled.
    def __init__(self, num_particles, robot, dimensions, step):
        self.num_particles = num_particles
        self.dimensions = dimensions
        self.step = step
        self.odom_received = False
        self.scan_received = False

        self.p_set = []
        self.current_scan = LaserScan()
        self.current_pose = RobotPose()
        self.recent_scan = LaserScan()
        self.recent_pose = RobotPose()

        self.prev_pose = RobotPose()
        self.pose_delta = (0, 0, 0)
        self.robot = robot
        
        rospy.init_node("filter")
        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        atexit.register(self.output_maps)

    def laser_callback(self, data):
        self.recent_scan = data
        self.scan_received = True
        #print("# of Scans: " + str(len(data.ranges))) #512 scans for our laser range finder!

    def odom_callback(self, data):
        [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.recent_pose.x = data.pose.pose.position.x
        self.recent_pose.y = data.pose.pose.position.y
        self.recent_pose.theta = yaw
        self.odom_received = True


    def run(self):
        #Wait until the first odom data is received
        while not self.odom_received and not rospy.is_shutdown():
            pass
        rospy.loginfo("First odom message received")
        self.current_pose = self.recent_pose
        self.prev_pose = self.current_pose
        #Initialize particles around guessed starting location
        self.init_particles(self.num_particles)
        rospy.loginfo("Done initializing particles")
        while not rospy.is_shutdown():
            #Wait until odom data is received, then update all the particles
            if self.odom_received and self.scan_received:
                #update the current scan and pose
                self.current_pose = self.recent_pose
                self.current_scan = self.recent_scan
                self.update_particles()
                self.resample_particles()
                self.odom_received = False
                self.scan_received = False


    def resample_particles(self):
        random_particles = int(self.num_particles * ParticleFilter.percent_random)
        new_particles = []
        total_weight = 0
        max_particle = 0

        for p in range(0, len(self.p_set)):
            total_weight += self.p_set[p].weight
            if self.p_set[p].weight > self.p_set[max_particle].weight:
                max_particle = p
        print(self.p_set[max_particle].weight)
        gridToNpyFile(self.p_set[max_particle].grid, self.p_set[max_particle].pose, "./maps", "map" + str(ParticleFilter.iteration))
        ParticleFilter.iteration += 1
        rospy.loginfo(str(ParticleFilter.iteration))

        #This loop is a mapping between the particles weights and their indices.
        #This allows us to sample the particles with frequency proportional to weight.
        for i in xrange(0, len(self.p_set) - random_particles):
            r = random.uniform(0,total_weight)
            tw = 0
            k = 0
            while tw < r:
                tw = tw + self.p_set[k].weight
                k = k + 1
            newP = Particle(self.p_set[k-1].grid, self.p_set[k-1].pose, self.p_set[k-1].weight)
            new_particles.append(newP)

        #This loop samples a percentage of the particles totally at random
        #This is to attempt to avoid sample starvation.
        for i in xrange(0, random_particles):
            r = random.randint(0, len(self.p_set)-1)
            newP =  Particle(self.p_set[r].grid, self.p_set[r].pose, self.p_set[r].weight)
            new_particles.append(newP)

        self.p_set = new_particles


    def update_particles(self):
        self.get_move_delta()
        self.prev_pose = self.current_pose
        for p in self.p_set:
            #Update the particle's position using the motion model
            #rospy.loginfo("Running motion model")
            p.pose.x, p.pose.y, p.pose.theta = self.robot.motionModel(old_particle = p, delta = self.pose_delta)
            #Weight the particle based on how closely it matches the sensor data
            #rospy.loginfo("Running sensor model")
            p.weight = self.robot.sensorModel(z_t = self.current_scan, pose = p.pose, m = p.grid)
            #Update the particle's map using the sensor model
            #rospy.loginfo("Updating map")
            p.grid = self.robot.sensor_model.update_map(self.current_scan, p.pose, p.grid)

    def init_particles(self, num_particles):
        """
            Initializes a set of particles. All particles have a starting pose equal to the 
            current odom reading plus random noise
        """
        for i in xrange(0, num_particles):
            new_x = random.gauss(self.current_pose.x, ParticleFilter.pose_noise)
            new_y = random.gauss(self.current_pose.y, ParticleFilter.pose_noise)
            new_theta = random.gauss(self.current_pose.theta, ParticleFilter.pose_noise/2.0)
            new_pose = RobotPose(new_x, new_y, new_theta)
            new_grid = OccupancyGrid(self.dimensions, self.step)
            p_temp = Particle(new_grid, new_pose, 1.0/num_particles)
            self.p_set.append(p_temp)

    def get_move_delta(self):
        """
            Gets the difference between the current pose and the previous pose, 
            then updates the previous pose
        """
        dx = self.current_pose.x - self.prev_pose.x
        dy = self.current_pose.y - self.prev_pose.y
        dtheta = self.current_pose.theta - self.prev_pose.theta
        self.pose_delta = (dx, dy, dtheta)
        self.prev_pose = self.current_pose

    def output_maps(self):
        for i in range(0, self.iteration, 5):
            print("Generating map " + str(i) + "...")
            npyToMapIMG("./maps/map" + str(i) + ".npy", self.dimensions, self.step, 5)

        """for i in range(0, self.iteration, 1):
            for n in range(0, self.num_particles):
                print("Generating map " + str(i) + "-" + str(n) +  "...")
                npyToMapIMG("./maps/map" + str(i) + "-" + str(n) +".npy", self.dimensions, self.step, 5)"""

if __name__ == "__main__":
    rospy.loginfo("Starting")
    particles = 20 #Number of particles to maintain
    map_size = (20, 20) #Map dimensions [-m, m] x [-n, n] in meters
    step_size = .2 #Step size in meters. Must be <= 1
    try:
        turtlebot_model = GenericBot(SensorModelNarrow(stddev = 1.5), MotionModelSimple())
        pfilter = ParticleFilter(particles, turtlebot_model, map_size, step_size)
    except Exception as er:
        rospy.logerr(er)
    finally:
        pfilter.run()