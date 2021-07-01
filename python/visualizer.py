from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin
import meshcat.transformations as tf

class Visualizer:
	def __init__ (self, urdf_path, meshes_path):
		self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(urdf_path, meshes_path)	
		self.viz = MeshcatVisualizer (self.model, self.collision_model, self.visual_model)
		try:
			self.viz.initViewer(open=False)
		except ImportError as err:
			print(err)
			exit(0)
		self.viz.loadViewerModel()
		
	def update (self, q):
		self.viz.display(q)