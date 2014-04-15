from occupancygrid import *
import math

if __name__ == "__main__":
	x = OccupancyGrid(10,10,.1)
	x[0][0] = True
	print(x.rayTrace((-9,-9), math.radians(45)))