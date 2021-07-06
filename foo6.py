#https://stackoverflow.com/questions/48911643/set-uvc-equivilent-for-a-3d-quiver-plot-in-matplotlib


import numpy as np
from math import sin, cos
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import matplotlib.pyplot as plt
import numpy as np


pi = 3.141592

def get_pos(t):
	return 10*np.asarray([
						[cos(t)],
						[sin(t)],
						[0.1*t]
					])

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

pos3d = get_pos(0)
x_start = pos3d[0][0]
y_start = pos3d[1][0]
z_start = pos3d[2][0]


ext=3

x_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start+ext,y_start,z_start, color="red")
y_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start+ext,z_start, color="green")
z_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start,z_start+ext, color="blue")

moving_frame = [[x_arrow_moving, y_arrow_moving, z_arrow_moving]]

def update_plot(num, moving_frame):
	time = num*dt
	pos3d = get_pos(time)
	x_start = pos3d[0][0]
	y_start = pos3d[1][0]
	z_start = pos3d[2][0]

	moving_frame[0].set_segments([[[x_start,y_start,z_start],[x_start+ext,y_start,z_start]]])
	moving_frame[1].set_segments([[[x_start,y_start,z_start],[x_start,y_start+ext,z_start]]])
	moving_frame[2].set_segments([[[x_start,y_start,z_start],[x_start,y_start,z_start+ext]]])

	return moving_frame

maxTime = 10
dt = 0.01
ani=animation.FuncAnimation(fig, update_plot, frames=int(maxTime/dt), fargs=(moving_frame), interval=1, blit=False)
plt.show()

