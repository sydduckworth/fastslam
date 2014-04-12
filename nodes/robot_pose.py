

#data structure used to store robot pose
class Robot_Pose():

	def __init__(self, x = 0, y = 0, theta = 0):
		self.loc = (x, y)
		self.theta = theta