
import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
import time

from pinocchio.visualize import GepettoVisualizer
from pinocchio.visualize import MeshcatVisualizer

pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "data",)
mesh_dir = pinocchio_model_dir
model_path = join(pinocchio_model_dir, "ball", "ball.urdf")
urdf_filename = "ball.urdf"
urdf_model_path = model_path
help(pin.JointModelFreeFlyer)
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)
# viz = GepettoVisualizer (model, collision_model, visual_model)
viz = MeshcatVisualizer (model, collision_model, visual_model)
try:
	viz.initViewer(open=False)
except ImportError as err:
	print(err)
	exit(0)

viz.loadViewerModel()
print(pin.neutral(model))
viz.display(np.asarray([0, 0, 1.5, 0, 0, 0, 1]))

for i in range(100):
	time.sleep(1)

"""
try:
	viz.loadViewerModel("pinocchio")
except AttributeError as err:
	print(err)
	exit(0)

"""