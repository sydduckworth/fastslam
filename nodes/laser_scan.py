

'''
Data structure used to store laser scan readings
	- Probably replaced just by passing the LaserScan Message object around
'''

class Laser_Scan():

	def __init__(self, scan_message = None):

		self.angle_min = scan_message.angle_min if scan_message != None else 0
		self.angle_max = scan_message.angle_max if scan_message != None else 0
		self.angle_increment = scan_message.angle_increment if scan_message != None else 0
		self.time_increment = scan_message.time_increment if scan_message != None else 0
		self.scan_time = scan_message.scan_time if scan_message != None else 0
		self.range_min = scan_message.range_min if scan_message != None else 0
		self.range_max = scan_message.range_max if scan_message != None else 0
		self.ranges = scan_message.ranges if scan_message != None else []
		self.intensities = scan_message.intensities if scan_message != None else []
