import math

'''
This module contains various useful probability utilities.

'''



'''
getCND

This function gets the cumulative normal distribution using erf and erfc functions.
'''
def getCND(x, mean, stddev):
	z = float(x - mean) / float(stddev)
	if (z >= 0.0):
		return 0.5 + 0.5 * math.erf(z / math.sqrt(2.0))
	else:
		return 0.5 * erfc(-z / math.sqrt(2.0))

'''
This function gets the value of the normal distribution at x.
'''
def getND(x, mean, stddev):
	denom = math.sqrt(2.0 * math.pi) * stddev * math.exp(math.pow((x - mean), 2) / (2.0 * math.pow(stddev, 2)))
	return 1.0/denom