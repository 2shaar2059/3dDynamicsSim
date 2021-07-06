#https://stackoverflow.com/questions/48911643/set-uvc-equivilent-for-a-3d-quiver-plot-in-matplotlib


import numpy as np
from math import sin, cos
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

pi = 3.141592

def get_pos(t):
	return np.asarray([
						[cos(t)],
						[sin(t)],
						[t]
					])


def show_grid(minX, minY, minZ, maxX, maxY, maxZ):
	# Setting the axes properties
	ax.set_xlim3d([minX, maxX])
	ax.set_xlabel('X')

	ax.set_ylim3d([minY, maxY])
	ax.set_ylabel('Y')

	ax.set_zlim3d([minZ, maxZ])
	ax.set_zlabel('Z')

	ax.set_title('3D Test')

maxTime = 10
dt = 0.1

"""

currTime = 0
while currTime <= maxTime:
	pos3d = get_pos(currTime)
	print(pos3d[0][0], pos3d[1][0], pos3d[2][0], pos3d[0][0]+1, pos3d[1][0]+1, pos3d[2][0]+1)
	ax.quiver(pos3d[0][0], pos3d[1][0], pos3d[2][0], pos3d[0][0]+1, pos3d[1][0]+1, pos3d[2][0]+1, color="red")


	show_grid(-20,-20,-20,20,20,20)
	plt.draw()
	plt.pause(0.001)
	plt.cla()

	currTime+=dt
"""
show_grid(-20,-20,-20,20,20,20)

x_arrow_fixed = ax.quiver(0,0,0,10,0,0, color="red")
y_arrow_fixed = ax.quiver(0,0,0,0,10,0, color="green")
z_arrow_fixed = ax.quiver(0,0,0,0,0,10, color="blue")

pos3d = get_pos(0)
x_start = pos3d[0][0]
y_start = pos3d[1][0]
z_start = pos3d[2][0]

x_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start+1,y_start,z_start, color="red")
y_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start+1,z_start, color="green")
z_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start,z_start+1, color="blue")

moving_frame = [[x_arrow_moving, y_arrow_moving, z_arrow_moving]]

def update_plot(num, moving_frame):
	time = num*dt
	pos3d = get_pos(time)
	x_start = pos3d[0][0]
	y_start = pos3d[1][0]
	z_start = pos3d[2][0]

	#x_arrow_fixed.set_segments([time, time, time, 2*time, 2*time, 2*time])
	"""
	segs = np.array((0,0,0,5*sin(time), 5*cos(time), 5)).reshape(6,-1)
	new_segs = [[[x,y,z],[u,v,w]] for x,y,z,u,v,w in zip(*segs.tolist())]
	print(new_segs, "\n")
	x_arrow_fixed.set_segments(new_segs)
	"""
	moving_frame[0].set_segments([[[x_start,y_start,z_start],[x_start+1,y_start,z_start]]])
	moving_frame[1].set_segments([[[x_start,y_start,z_start],[x_start,y_start+1,z_start]]])
	moving_frame[2].set_segments([[[x_start,y_start,z_start],[x_start,y_start,z_start+1]]])


	#x_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start+1,y_start,z_start, color="red")
	#y_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start+1,z_start, color="green")
	#z_arrow_moving = ax.quiver(x_start,y_start,z_start,x_start,y_start,z_start+1, color="blue")
	return moving_frame

ani=animation.FuncAnimation(fig, update_plot, frames=int(maxTime/dt), fargs=(moving_frame), interval=10, blit=False)
plt.show()