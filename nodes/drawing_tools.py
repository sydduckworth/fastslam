'''
Dependences: Python Image Library, Numpy
'''
import os
import numpy as np
from PIL import Image
import occupancygrid

'''
ensures a directories existance, if it doesn't exist, it will create it
'''
def ensure_dir(directory): 
	d = os.path.dirname(directory)
	if (not os.path.exists(directory)):
		os.makedirs(directory)

'''
This function converts an occupancy grid object to a numpy array and saves that
array out to a .npy file in the dest_dir directory with the filename provided. 
* Note: fname should not contain a file extension.
* Note: Format of dest_dir and fname is critical to successful file saving
'''

def gridToNpyFile(occupancy_grid, robot_pose, dest_dir = "./default", fname = "default"):
	#ensure the existance of given directory to store files
	ensure_dir(dest_dir)
	dimensions = occupancy_grid.dimensions 
	step = occupancy_grid.step
	array = [] #easiest to append grid cells to regular python array then convert this array to numpy array
	robo_thresh = .2
	robot_indicator = -1 #this is used to distinguish robot location in grid
	occupancy_grid[robot_pose.x][robot_pose.y] = -1
	for x in occupancy_grid:
		array.append([])
		for y in x:
			array[-1].append(y)

	np_grid = np.asarray(array)
	np.save(str(dest_dir) + "/" + str(fname) + ".npy", np_grid)

'''
fpath is the full path and file name of the .npy file to be converted into a .jpg (ex: './default/default.npy')
grid_dim is in format of OccupancyGrid dimensions (m, n) where grid is [-m, m] by [-n, n]
pixel_width_per_cell describes the number of pixels used to describe the width and height of a grid cell
'''
#TODO: assert .npy file type/error checking
def npyToMapIMG(fpath, grid_dim, step, pixel_width_per_cell):
	np_grid = np.load(fpath) #load grid from npy file
	im_height = int((2 * grid_dim[1] / step ) * pixel_width_per_cell) 	#calculate image height
	im_width = int((2 * grid_dim[0] / step ) * pixel_width_per_cell) 	#calculate image width
	image = Image.new("RGB", (im_width, im_height), "blue") 	#create new image
	image_pix = image.load()                                    #load image pixels for manipulation
	grid_x = 0      #keeps track of current grid x coord
	x_pixel_cnt = 0 #counts number of pixels in x direction we've drawn 
	for x in xrange(0, im_width):
		grid_y = 0 #keeps track of current grid y coord
		y_pixel_cnt = 0 #counts number of pixels in y directino we've drawn
		for y in xrange(im_height - 1, 0, -1):
			if (np_grid[grid_x, grid_y] == None):
				#if unknown, color grey
				image_pix[x, y] = (100, 100, 100)
			elif (np_grid[grid_x, grid_y] == False):
				#if not occupied, color white
				image_pix[x, y] = (255, 255, 255)
			elif (np_grid[grid_x, grid_y] == True):
				#if occupied, color black
				image_pix[x, y] = (0, 0, 0)
			elif (np_grid[grid_x, grid_y] == -1):
				#if robot, color red
				image_pix[x, y] = (255, 0, 0)

			y_pixel_cnt += 1
			#if the y pixel count equals desired pixel width per cell, reset it and move on to next grid cell
			if (y_pixel_cnt == pixel_width_per_cell):
				y_pixel_cnt = 0
				grid_y += 1

		x_pixel_cnt += 1
		#if the x pixel count equals desired pixel width per cell, reset it and move on to next grid line
		if (x_pixel_cnt == pixel_width_per_cell):
			x_pixel_cnt = 0
			grid_x += 1

	image.save(str(fpath).replace(".npy", "") + ".jpg") #save file out as jpg


'''
test code to make sure above functions work
'''
if __name__ == "__main__":
	test_grid = occupancygrid.OccupancyGrid((15, 10), .5)
	test_grid[0][0] = False
	test_grid[15][5] = True
	gridToNpyFile(test_grid)
	npyToMapIMG("./default/default.npy", (15, 10), .5, 5)