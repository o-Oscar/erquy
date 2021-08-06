
import numpy as np
np.set_printoptions(precision=4, suppress=True)
import sys
import os
from os.path import dirname, join, abspath
import time
import time
import matplotlib.pyplot as plt

import erquy_py

vizualize = True

urdf_name = "collision"
q = np.asarray([0.37645, 0, 2, 0, 0, 0, 1])
u = np.asarray([0, 0, 0, 0, 0, 0])
kp = u*0
kd = u*0
q_target = q
# q_target = np.asarray([0, 0, 1, 0, 0, 0, 1] + [0, 0, 0] * 4)
u_target = u * 0

urdf_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name, urdf_name + ".urdf")
meshes_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name)

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)
if vizualize:
	from visualizer import Visualizer
	viz = Visualizer(urdf_path, meshes_path)
	viz.update(q)
	input()

# config
world.setGravity(np.asarray([0, 0, -10]))
# world.setGravity(np.asarray([0, 0, 0]))
world.setTimeStep(0.01)

world.setState (q, u)
world.setPdGains (kp, kd)
world.setPdTarget (q, u)

all_z = []

for i in range(1000):
	for i in range(10):
		world.integrate()
	input()
	q, u = world.getState()
	n_contact, contact_joint_id, full_lamb = world.getContactInfos()
	if n_contact > 0:
		print(n_contact, contact_joint_id, full_lamb)
	# print(q, u)
	all_z.append(q[2])
	viz.update(q)


plt.plot(all_z)
plt.show()
