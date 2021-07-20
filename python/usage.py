"""

export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH

py-spy record -o profile.svg --native python3 python/usage.py speed
file:///home/oscar/workspace/erquy/build/profile.svg
"""


import numpy as np
np.set_printoptions(precision=4, suppress=True)
import sys
import os
from os.path import dirname, join, abspath
import time
import time
import matplotlib.pyplot as plt

def sleep (dt):
	start = time.time()
	while start + dt > time.time():
		pass 

SPEED_TEST = len(sys.argv) > 1 and sys.argv[1] == "speed"
VISU_TEST = len(sys.argv) > 1 and sys.argv[1] == "visualize"

import erquy_py
if not SPEED_TEST:
	from visualizer import Visualizer

if True:
	urdf_name = "idefX"
	q = np.asarray([0, 0, 1, 0, 0, 0, 1] + [0, np.pi/4, -np.pi/2] * 4)
	u = np.asarray([0, 0, 0, 0, 0, 0] + [0, 0, 0] * 4)
	kp = np.asarray([0] * 6 + [72] * 12)
	kd = np.asarray([0] * 6 + [7.2] * 12)
	q_target = q
	# q_target = np.asarray([0, 0, 1, 0, 0, 0, 1] + [0, 0, 0] * 4)
	u_target = u * 0
elif True:
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
meshes_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name)


if not SPEED_TEST:
	viz = Visualizer(urdf_path, meshes_path)

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)


if not world.nq() == q.shape[0] or not world.nv() == u.shape[0]:
	print("q expected : ", world.nq()) 
	print("q receivec : ", q.shape[0]) 
	print("u expected : ", world.nv()) 
	print("u receivec : ", u.shape[0])
	exit()

# config
world.setGravity(np.asarray([0, 0, -10]))
# world.setGravity(np.asarray([0, 0, 0]))
world.setTimeStep(0.01)

world.setState (q, u)
world.setPdGains (kp, kd)
world.setPdTarget (q_target, u_target)

if not SPEED_TEST:
	viz.update(q)
	input ()

all_forces = [[] for i in range(4)]

m = 0
all_start = time.time()
for i in range(1000):
	start = time.time()
	for k in range(1):
		world.integrate()
	dt = time.time()-start
	print(dt, "->", (1*0.01)/dt, "fois le temps réel")

	if VISU_TEST:
		n_contact, contact_joint_id, full_lamb = world.getContactInfos()
		for contact_pair in contact_joint_id:
			for i, id in enumerate([4, 7, 10, 13]):
				if id in contact_pair:
					all_forces[i].append([np.sqrt(np.sum(np.square(full_lamb[3*i:3*i+2]))), full_lamb[3*i+2]])
		

	if not SPEED_TEST:
		q, u = world.getState()
		# print(q, u)

	if not SPEED_TEST and not VISU_TEST:
		viz.update(q)
		
		if True:
			sleep(0.01)
		else:
			input()

all_delta_time = time.time() - all_start
print(all_delta_time, "->", (1000*0.01)/all_delta_time, "fois le temps réel")

if VISU_TEST:
	fig, axs = plt.subplots(2, 2)
	axs[0][0].plot(np.stack(all_forces[0]))
	axs[1][0].plot(np.stack(all_forces[1]))
	axs[0][1].plot(np.stack(all_forces[2]))
	axs[1][1].plot(np.stack(all_forces[3]))
	plt.show()
	


