import math
import random

from fastslam_utilities import *

class MotionModelSimple(object):
	def __init__(self, stds = (.1, .1, .1)):
		self.stds = stds

	def update(self, old_particle, delta):
		#add random noise to pose delta (only if we've actually detected movement)
		dx = random.gauss(delta[0], self.stds[0]) if delta[0] != 0 else 0
		dy = random.gauss(delta[1], self.stds[1]) if delta[1] != 0 else 0
		dtheta = random.gauss(delta[2], self.stds[2]) if delta[2] != 0 else 0
		return (old_particle.pose.x + dx, old_particle.pose.y + dy, wrap_angle(old_particle.pose.theta + dtheta))