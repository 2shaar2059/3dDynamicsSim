#https://stackoverflow.com/questions/48911643/set-uvc-equivilent-for-a-3d-quiver-plot-in-matplotlib


import numpy as np
from math import sin, cos
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import matplotlib.pyplot as plt
import numpy as np


timestamps = []
rotations = []
translations  = []

inputFile = "3dAnimationData.txt"

linesOfDataPerTimestep = 8 #number of lines recorded at each timestep in the log

with open(inputFile, 'r') as f:
	lines = f.readlines()
	print(len(lines))
	for i in range(0, len(lines), linesOfDataPerTimestep):
		timestamps.append(float(lines[i].strip()))
		roation_matrix_row_1 = lines[i+1].split()
		roation_matrix_row_2 = lines[i+2].split()
		roation_matrix_row_3 = lines[i+3].split()

		translation_vector_row_1 = lines[i+4].strip()
		translation_vector_row_2 = lines[i+5].strip()
		translation_vector_row_3 = lines[i+6].strip()

		rotations.append(np.asarray([
										roation_matrix_row_1,
										roation_matrix_row_2,
										roation_matrix_row_3
										]).astype(np.float))

		translations.append(np.asarray([
										translation_vector_row_1,
										translation_vector_row_2,
										translation_vector_row_3
										]).astype(np.float))
dt = timestamps[1]-timestamps[0]


fig = plt.figure()
ax = fig.gca(projection='3d')

def show_grid(minX, minY, minZ, maxX, maxY, maxZ):
	# Setting the axes properties
	ax.set_xlim3d([minX, maxX])
	ax.set_xlabel('X')

	ax.set_ylim3d([minY, maxY])
	ax.set_ylabel('Y')

	ax.set_zlim3d([minZ, maxZ])
	ax.set_zlabel('Z')

	ax.set_title('3D Test')


show_grid(-20,-20,-20,20,20,20)

x_arrow_fixed = ax.quiver(0,0,0,10,0,0, color="red")
y_arrow_fixed = ax.quiver(0,0,0,0,10,0, color="green")
z_arrow_fixed = ax.quiver(0,0,0,0,0,10, color="blue")

pos3d = translations[0]
x_start = pos3d[0]
y_start = pos3d[1]
z_start = pos3d[2]

ext=3

x_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start+ext,y_start,z_start, color="red")
y_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start+ext,z_start, color="green")
z_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start,z_start+ext, color="blue")

moving_frame = [[x_arrow_moving, y_arrow_moving, z_arrow_moving]]

totalTimestamps = len(timestamps)
fps = 30 #frames per second in the animation

timestamps_per_frame = int(1/(fps*dt))
totalFrames = int(totalTimestamps/timestamps_per_frame)
print(totalTimestamps, timestamps_per_frame, totalFrames)
def update_plot(num, moving_frame):
	idx = num*timestamps_per_frame
	pos3d = translations[idx]

	moving_frame[0].set_segments([[pos3d,pos3d+rotations[idx][:,0]]])
	moving_frame[1].set_segments([[pos3d,pos3d+rotations[idx][:,1]]])
	moving_frame[2].set_segments([[pos3d,pos3d+rotations[idx][:,2]]])
	plt.suptitle("{:.2f} seconds".format(idx*dt))
	return moving_frame

ani = animation.FuncAnimation(fig, update_plot, frames=totalFrames, fargs=(moving_frame), interval=750/fps, blit=False)
plt.show()

