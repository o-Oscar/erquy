
import numpy as np
np.set_printoptions(precision=4, suppress=True)
import sys
import os
from os.path import dirname, join, abspath
import time
import time
import matplotlib.pyplot as plt

import erquy_py

urdf_name = "idefX"
q = np.asarray([0, 0, 2, 0, 0, 0, 1] + [0, np.pi/4, -np.pi/2] * 4)
u = np.asarray([0, 0, 0, 0, 0, 0] + [0, 0, 0] * 4)
kp = np.asarray([0] * 6 + [72] * 12)
kd = np.asarray([0] * 6 + [7.2] * 12)
q_target = q
# q_target = np.asarray([0, 0, 1, 0, 0, 0, 1] + [0, 0, 0] * 4)
u_target = u * 0

urdf_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name, urdf_name + ".urdf")
meshes_path = join(dirname(dirname(dirname(str(abspath(__file__))))), "data", urdf_name)

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)
world.setState(q, u)

print(world.getState())
print(list(world.getFrameNames()))
print(world.getFrameIdxByName("FL_foot"))
idx = world.getFrameIdxByName("FL_foot")

print(world.getFramePosition(idx))
print(world.getFrameOrientation(idx))
print(world.getFrameVelocity(idx))
print(world.getFrameAngularVelocity(idx))

print(world.getContactInfos())