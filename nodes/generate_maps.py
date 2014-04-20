import drawing_tools

number_maps = 5

if __name__ == "__main__":
	for i in range(0, number_maps):
		drawing_tools.npyToMapIMG("./maps/map" + str(i) + ".npy", (15,15), .1, 5)