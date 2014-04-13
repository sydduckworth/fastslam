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
		self.sensor_model.update(**kwargs)

	def motionModel(self, **kwargs):
		self.motion_model.update(**kwargs)

#Test Code
"""
def smodel(**kwargs):
	print(kwargs.keys())

def mmodel(**kwargs):
	print(kwargs.values())

tbot = GenericBot(smodel, mmodel)
d = {"A":1, "B":2, "C":3}
tbot.sensorModel(**d)
tbot.motionModel(**d)
"""