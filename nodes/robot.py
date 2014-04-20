from abc import ABCMeta, abstractmethod

class Robot:
	__metaclass__ = ABCMeta

	@abstractmethod
	def sensorModel(self, **kwargs):
		raise NotImplementedError

	@abstractmethod
	def motionModel(self, **kwargs):
		raise NotImplementedError



class Turtlebot(Robot):
	def sensorModel(self, **kwargs):
		pass

	def motionModel(self, **kwargs):
		pass


class Jaguar(Robot):
	def sensorModel(self, **kwargs):
		pass

	def motionModel(self, **kwargs):
		pass

class GenericBot(Robot):
	def __init__(self, sensor_model, motion_model):
		self.sensor_model = sensor_model
		self.motion_model = motion_model

	def sensorModel(self, **kwargs):
		return self.sensor_model.update(**kwargs)

	def motionModel(self, **kwargs):
		return self.motion_model.update(**kwargs)

#Test Code
"""
def smodel(a, b, c):
	print(a, b, c);

def mmodel(**kwargs):
	pass

tbot = GenericBot(smodel, mmodel)
tbot.sensorModel(a="A", b="B", c="C")
"""