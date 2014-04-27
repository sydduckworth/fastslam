from occupancygrid import *
from drawing_tools import *
from robot_pose import *
import math
import timeit

if __name__ == "__main__":
	x = OccupancyGrid((10,10),.2)
	r = 5
	init_angle = 0
	end_angle = 2 * math.pi
	step = .5 * math.atan2(x.step, r)
	while init_angle <= end_angle:
		point_x = r * math.cos(init_angle) + 0
		point_y = r * math.sin(init_angle) + 0
		#Clear all points up to the sensed point, then set the sensed point to occupied
		#x.fillTo((0, 0), (point_x, point_y))
		x[point_x][point_y] = True
		init_angle += step

	gridToNpyFile(x, RobotPose(), "./maps", "gridtest")
	npyToMapIMG("./maps/gridtest.npy", (10, 10), .2, 5)
