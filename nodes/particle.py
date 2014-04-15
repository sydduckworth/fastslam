from occupancygrid import *
from robot_pose import *
import random


class Particle(object):

	def __init__(self, grid, pose = RobotPose(), weight = 0):
		self.grid = grid
		self.pose = pose
		self.weight = weight
