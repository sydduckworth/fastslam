import math
import numpy
from fastslam_utilities import *

class OccupancyGrid(object):
	def __init__(self, dimensions, step):
		"""
			Initializes a new grid with dimensions [-m, m], [-n, n]. Each index represents a cell with 
			size = step. Each index contains a boolean indicating occupancy.
		"""

		self.dimensions = dimensions
		self.origin = (int(dimensions[0]/step), int(dimensions[1]/step))
		if step > 1:
			raise TypeError("Step size cannot be greater than one.")
		self.step = step
		self.map = [GridList(self.origin, self.dimensions, self.step) for i in range(0, int(2*dimensions[0]/self.step) + 1)]

	def __getitem__(self, key):
		if abs(int(key)) <= self.dimensions[0]:
			index = int(key/self.step) + self.origin[0]
			return self.map[index]
		else:
			raise IndexError

	def __iter__(self):
		for item in self.map:
			yield item

	def __contains__(self, item):
		return (abs(item[0]) <= self.dimensions[0]) and (abs(item[1]) <= self.dimensions[1])

	def map2grid(self, coords):
		return (int(coords[0]/self.step) + self.origin[0], int(coords[1]/self.step) + self.origin[1])

	def grid2map(self, coords):
		return ((coords[0] - self.origin[0]) * self.step + self.step/2.0, (coords[1] - self.origin[1]) * self.step + self.step/2.0)

	def rayTrace(self, coords, angle):
		"""
			Given a starting position and angle, recursively checks each index along the path.
			Returns the first occupied index.
		"""
		if coords not in self:
			return None
		elif self[coords[0]][coords[1]]:
			return coords
		else:
			next_coords = (coords[0] + math.cos(angle)*self.step, coords[1] + math.sin(angle)*self.step)
			return self.rayTrace(next_coords, angle)

	def rayTraceTo(self, coords1, coords2):
		"""
			Given two sets of coordinates, finds the angle between the coordinates and checks the path
			Returns true if the path is clear, false if blocked.
		"""
		angle = math.atan2((coords2[1] - coords1[1]), (coords2[0] - coords1[0]))
		return self.map2grid(self.rayTrace(coords1, angle)) == self.map2grid(coords2)

	def clearTo(self, coords1, coords2):
		"""
			Given two sets of coordinates, sets all cells between the points to empty
		"""
		if coords1 not in self or coords2 not in self:
			return False
		else:
			angle = math.atan2((coords2[1] - coords1[1]), (coords2[0] - coords1[0]))
			return self._clearToLoop(coords1, coords2, angle)

	def _clearToLoop(self, coords1, coords2, angle):
		cur_coords = coords1
		x_step = math.cos(angle)*self.step
		y_step = math.sin(angle)*self.step
		dist = euclidean_distance(coords1, coords2)
		while euclidean_distance(cur_coords, coords1) < dist:
			if coords1 not in self:
				return False
			else:
				self[cur_coords[0]][cur_coords[1]] = False
				cur_coords = (cur_coords[0] + x_step, cur_coords[1] + y_step)
		return True

	def _clearToRecursive(self, coords1, coords2, angle):
		if coords1 not in self:
			return False
		elif self.map2grid(coords1) == self.map2grid(coords2) or self[coords1[0]][coords1[1]] == True:
			return True
		else:
			if self[coords1[0]][coords1[1]] != True:
				self[coords1[0]][coords1[1]] = False
			next_coords1 = (coords1[0] + math.cos(angle)*self.step, coords1[1] + math.sin(angle)*self.step)
			return self._clearToRecursive(next_coords1, coords2, angle)

	def fillTo(self, coords1, coords2):
		"""
			Given two sets of coordinates, sets all cells between the points to occupied
		"""
		if coords1 not in self or coords2 not in self:
			return False
		else:
			angle = math.atan2((coords2[1] - coords1[1]), (coords2[0] - coords1[0]))
			return self._fillToLoop(coords1, coords2, angle)

	def _fillToLoop(self, coords1, coords2, angle):
		cur_coords = coords1
		x_step = math.cos(angle)*self.step
		y_step = math.sin(angle)*self.step
		dist = euclidean_distance(coords1, coords2)
		while euclidean_distance(cur_coords, coords1) < dist:
			if coords1 not in self:
				return False
			else:
				self[cur_coords[0]][cur_coords[1]] = True
				cur_coords = (cur_coords[0] + x_step, cur_coords[1] + y_step)
		return True

	def _fillToRecursive(self, coords1, coords2, angle):
		if coords1 not in self:
			return False
		elif self.map2grid(coords1) == self.map2grid(coords2):
			return True
		else:
			self[coords1[0]][coords1[1]] = True
			next_coords1 = (coords1[0] + math.cos(angle)*self.step, coords1[1] + math.sin(angle)*self.step)
			return self._fillToRecursive(next_coords1, coords2, angle)

	def fillRect(self, coords, width, height):
		for x in numpy.linspace(coords[0] - .5 * width, coords[0] + .5 * width, num = (width/(.9 * self.step))):
			for y in numpy.linspace(coords[1] - .5 * height, coords[1] + .5 * height, num = (height/(.9 * self.step))):
				self[x][y] = True


class GridList(list):
	def __init__(self, origin, dimensions, step):
		self.data = [None for i in range(0, int(2*dimensions[1]/step) + 1)]
		self.dimensions = dimensions
		self.origin = origin
		self.step = step

	def __getitem__(self, key):
		if abs(int(key)) <= self.dimensions[1]:
			index = int(key/self.step) + self.origin[1]
			return self.data[index]
		else:
			raise IndexError

	def __setitem__(self, key, item):
		if abs(int(key)) <= self.dimensions[1]:
			index = int(key/self.step) + self.origin[1]
			self.data[index] = item
		else:
			raise IndexError

	def __contains__(self, item):
		return abs(int(item[1])) <= self.dimensions[1]

	def __iter__(self):
		for item in self.data:
			yield item