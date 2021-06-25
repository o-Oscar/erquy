
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
import time
import time

import erquy_py
from visualizer import Visualizer

urdf_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", "ball", "ball.urdf")
meshes_path = join(dirname(dirname(str(abspath(__file__)))), "data", "ball")

viz = Visualizer(urdf_path, meshes_path)

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)

# config
world.setGravity(np.asarray([0, 0, -10]))
# world.setGravity(np.asarray([0, 0, 0]))
world.setTimeStep(0.01)

q = np.asarray([0, 0, 2, 0, 0, 0, 1])
u = np.asarray([0, 0, 0, 0, 0, 0])
world.setState (q, u)
viz.update(q)

input ()
for i in range(10000):

	print("Step (python)")
	print(q, u)
	for i in range(10):
		world.integrate()

		q, u = world.getState()
		viz.update(q)
		if True:
			time.sleep(0.03)
		else:
			input()


