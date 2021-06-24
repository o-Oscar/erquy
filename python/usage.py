
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
import time

import erquy_py

urdf_path = join(dirname(dirname(str(abspath(__file__)))), "data", "ball", "ball.urdf")
meshes_path = join(dirname(dirname(str(abspath(__file__)))), "data", "ball")

world = erquy_py.World()
world.loadUrdf (urdf_path, meshes_path)

q = np.asarray([0, 0, 1.5, 0, 0, 0, 1])
u = np.asarray([0, 0, 1, 0, 0, 0])
world.setState (q, u)

world.integrate()

q, u = world.getState()

"""
viz = erquy_py.Visualizer(urdf_path, meshes_path)
viz.update(q)
"""
