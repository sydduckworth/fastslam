import math

'''
This module contains various useful utilities.

'''



'''
getCND

This function gets the cumulative normal distribution using erf and erfc functions.
 - The cumulative normal distribution function describes the probability that random variable X will be found at a 
 	value less than or equal to x.
'''
def getCND(x, mean, stddev):
	z = float(x - mean) / float(stddev)
	if (z >= 0.0):
		return 0.5 + 0.5 * math.erf(z / math.sqrt(2.0))
	else:
		return 0.5 * math.erfc(-z / math.sqrt(2.0))

'''
This function gets the value of the normal distribution at x.
'''
def getND(x, mean, stddev):
	denom = math.sqrt(2.0 * math.pi) * stddev * math.exp(math.pow((x - mean), 2) / (2.0 * math.pow(stddev, 2)))
	return 1.0/denom

#Wrap an angle to be between -180 < < 180 (in radians though)
def wrap_angle(angle):
    while angle >= math.pi:
        angle = angle - 2*math.pi
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

def euclidean_distance(a,b):
    d = math.hypot(b[0]-a[0],b[1]-a[1])
    return d