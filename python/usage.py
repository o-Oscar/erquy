
import numpy as np
np.set_printoptions(precision=4, suppress=True)
import sys
import os
from os.path import dirname, join, abspath
import time
import time

def sleep (dt):
	start = time.time()
	while start + dt > time.time():
		pass 

import erquy_py
from visualizer import Visualizer

if True:
	urdf_name = "elbow"
	q = np.asarray([0, 0, .5, 0, 0, 0, 1, -np.pi/2])
	a = np.pi/4
	q = np.asarray([0, 0, 1, np.sin(a/2), 0, 0, np.cos(a/2), -np.pi/3])
	u = np.asarray([0, 0, 3, 0, 0, 0, -2])
	# q = [-0.82232715, -4.81442289 , 0.04004642 , 0.48802241 ,-0.49703782 ,-0.52562734, 0.48836814 ,-2.30298627]
	# u = [ 0.01005541, -0.00816949 ,-0.00397218 , 0.00030107,  0.01002735 , 0.00010885 ,-0.02149628]
	# q = np.asarray(q)
	# u = np.asarray(u) * 0
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
world.setTimeStep(0.01)


if not world.nq() == q.shape[0] or not world.nv() == u.shape[0]:
	print("q expected : ", world.nq()) 
	print("q receivec : ", q.shape[0]) 
	print("u expected : ", world.nv()) 
	print("u receivec : ", u.shape[0])
	exit()

world.setState (q, u)
viz.update(q)

world.integrate()

input ()
m = 0
for i in range(10000):
	start = time.time()
	for k in range(1):
		world.integrate()
	dt = time.time()-start
	print(dt, "->", (1*0.01)/dt, "fois le temps réel")
	q, u = world.getState()
	# print(q, u)
	all_jac = list(world.getJacobians())

	if len(all_jac) > 42:
		m += 1
		# [print(jac) for jac in all_jac]
		# jac = np.asarray(all_jac[0])
		# f = np.asarray([0, 0, 0.310829])
		# M = np.asarray(world.getM())
		# print("coucou")
		# print(jac.T @ f)
		# print(np.linalg.inv(M))
		# print(np.linalg.inv(M) @ jac.T @ f)
		if m > 0:
			exit()

	viz.update(q)
	
	if True:
		sleep(0.01)
	else:
		input()
	
	
	


