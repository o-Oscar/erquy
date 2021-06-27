
import numpy as np
np.set_printoptions(precision=4, suppress=True)
import sys
import os
from os.path import dirname, join, abspath
import time
import time

def sleep (dt):
	start = time.time()
	while start + dt < time.time():
		pass 

import erquy_py
from visualizer import Visualizer

if True:
	urdf_name = "elbow"
	q = np.asarray([0, 0, 3, 0, 0, 0, 1, -np.pi/2])
	a = np.pi/2
	q = np.asarray([0, 0, 3, 0, np.sin(a/2), 0, np.cos(a/2), -np.pi/2])
	u = np.asarray([0, 0, 0, 0, 0, 0, 0])
elif True:
	urdf_name = "ball"
	q = np.asarray([0, 0, 2, 0, 0, 0, 1])
	u = np.asarray([0, 0, 0, 0, 0, 0])

urdf_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name, urdf_name + ".urdf")
meshes_path = join(dirname(dirname(str(abspath(__file__)))), "data", urdf_name)

viz = Visualizer(urdf_path, meshes_path)

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)

# config
world.setGravity(np.asarray([0, 0, -10]))
# world.setGravity(np.asarray([0, 0, 0]))
world.setTimeStep(0.001)


if not world.nq() == q.shape[0] or not world.nv() == u.shape[0]:
	print("q expected : ", world.nq()) 
	print("q receivec : ", q.shape[0]) 
	print("u expected : ", world.nv()) 
	print("u receivec : ", u.shape[0])
	exit()

world.setState (q, u)
viz.update(q)

input ()
m = 0
for i in range(100):
	start = time.time()
	for k in range(30):
		world.integrate()
	print(time.time()-start)
	q, u = world.getState()
	print(q, u)

	viz.update(q)
	
	if True:
		sleep(0.03)
	else:
		input()
	
	


