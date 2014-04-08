import math

class OccupancyGrid(object):
	def __init__(self, m, n, step):
		"""
			Initializes a new grid with dimensions [-m, m], [-n, n]. Each index represents a cell with 
			size = step. Each index contains a boolean indicating occupancy.
		"""

		self.dimensions = (m, n)
		self.origin = (int(m/step + 1), int(n/step + 1))
		if step > 1:
			raise TypeError("Step size cannot be greater than one.")
		self.step = step
		self.map = [GridList(n, self.origin, self.dimensions, self.step) for i in range(0, int(2*m/self.step) + 2)]

	def __getitem__(self, key):
		if abs(int(key)) <= self.dimensions[0]:
			index = int(key/self.step) + self.origin[0]
			return self.map[index]
		else:
			raise IndexError

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
			next_coords = (coords[0] +  math.cos(angle)*self.step, coords[1] +  math.sin(angle)*self.step)
			return self.rayTrace(next_coords, angle)

	def rayTraceTo(self, coords1, coords2):
		"""
			Given two sets of coordinates, finds the angle between the coordinates and checks the path
			Returns true if the path is clear, false if blocked.
		"""
		angle = math.atan2((coords2[1] - coords1[1]), (coords2[0] - coords1[0]))
		return self.map2grid(self.rayTrace(coords1, angle)) == self.map2grid(coords2)


class GridList(list):
	def __init__(self, n, origin, dimensions, step):
		self.data = [False for i in range(0, int(2*n/step) + 2)]
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
			self.data[index] = bool(item)
		else:
			raise IndexError

	def __contains__(self, item):
		return abs(int(item[1])) <= self.dimensions[1]
