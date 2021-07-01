import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path
import numpy as np

"""
python3 ../data/idefX/create_urdf.py
"""

def add_colors (robot):
	all_colors = [	("black", "0.0 0.0 0.0 1.0"),
					("blue", "0.0 0.0 0.8 1.0"),
					("green", "0.0 0.8 0.0 1.0"),
					("grey", "0.2 0.2 0.2 1.0"),
					("silver", "0.913725490196 0.913725490196 0.847058823529 1.0"),
					("orange", "1.0 0.423529411765 0.0392156862745 1.0"),
					("brown", "0.870588235294 0.811764705882 0.764705882353 1.0"),
					("red", "0.8 0.0 0.0 1.0"),
					("white", "1.0 1.0 1.0 1.0")]
	for name, rgba in all_colors:
		material = ET.SubElement(robot, 'material')
		material.set("name", name)
		
		color = ET.SubElement(material, 'color')
		color.set("rgba", rgba)

def add_origin (root, pos, rot):
	origin = ET.SubElement(root, 'origin')
	origin.set("xyz", pos)
	origin.set("rpy", rot)

def add_inertia (root, mass, inertia):

	mass_xml = ET.SubElement(root, 'mass')
	mass_xml.set("value", mass)
	
	inertia_xml = ET.SubElement(root, 'inertia')
	for name, value in inertia.items(): # zip([ixx, ixy, ixz, iyy, iyz, izz], inertia):
		inertia_xml.set(name, str(value))

def add_contact_material (collision, material_name):
	
	material = ET.SubElement(collision, 'material')
	material.set("name", "")
	contact = ET.SubElement(material, 'contact')
	contact.set("name", material_name)

def add_box (root, size):
	geometry = ET.SubElement(root, 'geometry')
	box = ET.SubElement(geometry, 'box')
	box.set("size", size)

def add_box_link (robot, name, pos, rot, size, mass, inertia, color, contact_material):
	
	link = ET.SubElement(robot, 'link')
	link.set("name", name)
	
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, pos, rot)
	add_box(visual, size)
	material = ET.SubElement(visual, 'material')
	material.set("name", color)
	
	collision = ET.SubElement(link, 'collision')
	add_origin(collision, pos, rot)
	add_box(collision, size)
	add_contact_material (collision, contact_material)
	
	inertial = ET.SubElement(link, 'inertial')
	add_origin(inertial, pos, rot)
	add_inertia(inertial, mass, inertia)

def add_box_link_wrapper (robot, name, pos, rot, size, mass, contact_material="default_material", color=None):
	#volume = size[0]*size[1]*size[2]
	#density = mass/volume
	
	inertia_str = {	"ixx": mass / 12 * (size[1]**2 + size[2]**2),
					"ixy":0,
					"ixz":0,
					"iyy": mass / 12 * (size[0]**2 + size[2]**2),
					"iyz":0,
					"izz": mass / 12 * (size[0]**2 + size[1]**2)}
	
	
	pos_str = " ".join((str(x) for x in pos))
	rot_str = " ".join((str(x) for x in rot))
	size_str = " ".join((str(x) for x in size))
	mass_str = str(mass)
	color_str = "grey" if color is None else color
	
	add_box_link (robot, name, pos_str, rot_str, size_str, mass_str, inertia_str, color_str, contact_material)

def add_cylinder (section, l, r):
	geometry = ET.SubElement(section, 'geometry')
	cylinder = ET.SubElement(geometry, 'cylinder')
	cylinder.set("length", str(l))
	cylinder.set("radius", str(r))
	
def add_cylinder_link (robot, name, pos, rot, l, r, mass, inertia, color, contact_material):
	
	link = ET.SubElement(robot, 'link')
	link.set("name", name)
	
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, pos, rot)
	add_cylinder(visual, l, r)
	material = ET.SubElement(visual, 'material')
	material.set("name", color)
	
	collision = ET.SubElement(link, 'collision')
	add_origin(collision, pos, rot)
	add_cylinder(collision, l, r)
	add_contact_material (collision, contact_material)
	
	inertial = ET.SubElement(link, 'inertial')
	add_origin(inertial, pos, rot)
	add_inertia(inertial, mass, inertia)

def add_cylinder_link_wrapper (robot, name, pos, rot, l, r, mass, contact_material="default_material", color=None):
	#volume = size[0]*size[1]*size[2]
	#density = mass/volume
	
	inertia_str = {	"ixx":1/12*mass*(3*r*r+l*l),
					"ixy":0,
					"ixz":0,
					"iyy":1/12*mass*(3*r*r+l*l),
					"iyz":0,
					"izz":1/2*mass*r*r}
	
	l_str = str(l)
	r_str = str(r)
	pos_str = " ".join((str(x) for x in pos))
	rot_str = " ".join((str(x) for x in rot))
	mass_str = str(mass)
	color_str = "grey" if color is None else color
	
	add_cylinder_link (robot, name, pos_str, rot_str, l_str, r_str, mass_str, inertia_str, color_str, contact_material)

	
def add_joint (robot, name, parent_name, child_name, type, pos, rot, axis, max_torque):
	
	joint = ET.SubElement(robot, 'joint')
	joint.set("name", name)
	joint.set("type", type)
	
	
	origin = ET.SubElement(joint, 'origin')
	origin.set("rpy",rot)
	origin.set("xyz", pos)
	
	parent = ET.SubElement(joint, 'parent')
	parent.set("link", parent_name)

	child = ET.SubElement(joint, 'child')
	child.set("link", child_name)

	axis_xml = ET.SubElement(joint, 'axis')
	axis_xml.set("xyz", axis)

	dynamics = ET.SubElement(joint, 'dynamics')
	dynamics.set("damping", "0")
	dynamics.set("friction", "0")
	
	
	limit = ET.SubElement(joint, 'limit')
	limit.set("effort", max_torque)
	limit.set("velocity", str(100000))

def add_simple_joint (robot, name, parent_name, child_name, type, pos, rot):
	
	joint = ET.SubElement(robot, 'joint')
	joint.set("name", name)
	joint.set("type", type)
	
	origin = ET.SubElement(joint, 'origin')
	origin.set("rpy",rot)
	origin.set("xyz", pos)
	
	parent = ET.SubElement(joint, 'parent')
	parent.set("link", parent_name)

	child = ET.SubElement(joint, 'child')
	child.set("link", child_name)


def add_joint_wrapper (robot, parent_name, child_name, pos, rot, axis=None, max_torque=None, type="revolute"):
	if type == "revolute":
		assert axis is not None
	name = child_name + "_joint"
	pos_str = " ".join((str(x) for x in pos))
	rot_str = " ".join((str(x) for x in rot))
	max_torque_strs = str(max_torque) if max_torque is not None else "10000"
	if not type == "revolute":
		add_simple_joint (robot, name, parent_name, child_name, type, pos_str, rot_str)
	else:
		axis_str = " ".join((str(x) for x in axis))
		add_joint (robot, name, parent_name, child_name, type, pos_str, rot_str, axis_str, max_torque_strs)

def add_ground (robot) :
	name = "ground"
	link = ET.SubElement(robot, 'link')
	link.set("name", name)
	
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, "0 0 -1", "0 0 0")
	add_box(visual, "100 100 2")
	material = ET.SubElement(visual, 'material')
	material.set("name", "white")
	
	collision = ET.SubElement(link, 'collision')
	add_origin(collision, "0 0 -1", "0 0 0")
	add_box(collision, "100 100 2")
	
	return name

objects = {
	"trunk": {
		"mass" : "7.5",
		"inertia" : [0.05, 0, 0, 0.05, 0, 0.05],
		"use_mesh" : True,
	},
	"motor": {
		"mass" : "0.7",
		"inertia" : [0.00184, 0, 0, 0.00184, 0, 0.0035],
		"use_mesh" : True,
	},
	"arm": {
		"mass" : "0.2",
		"inertia" : [0.0002, 0, 0, 0.0002, 0, 0.00008],
		"use_mesh" : True,
	},
	"forearm": {
		"mass" : "0.1",
		"inertia" : [0.0001, 0, 0, 0.0001, 0, 0.00004],
		"use_mesh" : True,
	},
	"foot": {
		"mass" : "0.01",
		"inertia" : [0.000006, 0, 0, 0.000006, 0, 0.000006],
		"use_mesh" : True,
		"use_collision" : True,
	},
}

def add_debug_visual (link, pos, rot) :
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, pos, rot)

	geometry = ET.SubElement(visual, 'geometry')
	sphere = ET.SubElement(geometry, 'sphere')
	sphere.set("radius", "0.03")

	# add_cylinder(visual, l, r) TODO : Add mesh based on object
	material = ET.SubElement(visual, 'material')
	material.set("name", "blue")

def add_mesh_visual (link, pos, rot, mesh_name, color) :
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, pos, rot)

	geometry = ET.SubElement(visual, 'geometry')
	mesh = ET.SubElement(geometry, 'mesh')
	mesh.set("filename", "../data/idefX/meshes/" + mesh_name+".dae")

	# add_cylinder(visual, l, r) TODO : Add mesh based on object
	material = ET.SubElement(visual, 'material')
	material.set("name", color)

def add_link_wrapper (robot, name, pos, rot, object, color="brown"):
	pos_str = " ".join((str(x) for x in pos))
	rot_str = " ".join((str(x) for x in rot))
	
	link = ET.SubElement(robot, 'link')
	link.set("name", name)
	
	
	if "use_mesh" in objects[object] and objects[object]["use_mesh"]:
		add_mesh_visual (link, pos_str, rot_str, object, color)
	elif True:
		add_debug_visual (link, pos_str, rot_str)
	"""
	visual = ET.SubElement(link, 'visual')
	add_origin(visual, pos, rot)
	# add_cylinder(visual, l, r) TODO : Add mesh based on object
	material = ET.SubElement(visual, 'material')
	material.set("name", "blue")
	"""

	if "use_collision" in objects[object] and objects[object]["use_collision"]:
		collision = ET.SubElement(link, 'collision')
		add_origin(collision, pos_str, rot_str)
		geometry = ET.SubElement(collision, 'geometry')
		sphere = ET.SubElement(geometry, 'sphere')
		sphere.set("radius", "0.02")

		
	
	inertia_str = {	key: value for key, value in zip(["ixx","ixy","ixz","iyy","iyz", "izz"], objects[object]["inertia"]) }
	inertial = ET.SubElement(link, 'inertial')
	add_origin(inertial, pos_str, rot_str)
	add_inertia(inertial, objects[object]["mass"], inertia_str)

def create_idefX ():
	robot = ET.Element('robot')
	robot.set("name", "idefX")
	add_colors(robot)
	
	motor_m = 0.7
	trunk_m = 16.7 - 12*motor_m - 4*0.2
	trunk_inertial = {"mass": trunk_m, "inertia": [trunk_m*0.02/3*x for x in (1, 0, 0, 1, 0, 1)]}
	trunk_i = [trunk_m*0.02/3*x for x in (1, 0, 0, 1, 0, 1)]
	
	ground = add_ground (robot)
	

	trunk_name = "trunk"
	add_link_wrapper(robot, trunk_name, (0, 0, 0), (0, 0, 0), object="trunk")
	add_joint_wrapper (robot, ground, trunk_name, (0, 0, 0), (0, 0, 0), type="floating")

	for strid, facx, facy in [("FL", 1, 1), ("FR", 1, -1), ("BL", -1, 1), ("BR", -1, -1)]:
		max_troque = 11 # N.m
		
		first_motor_name = strid+"_first_motor"
		add_joint_wrapper (robot, trunk_name, first_motor_name, (facx*(0.34-0.035)/2, facy*0.075, 0.), (0, 0, 0), type="fixed")
		add_link_wrapper(robot, first_motor_name, (0, 0, 0), (0, -np.pi/2, np.pi*(facx+1)/2), object="motor")
		
		
		clavicle_name = strid+"_clavicle"
		add_joint_wrapper (robot, trunk_name, clavicle_name, (facx*0.45/2, facy*0.075, -0.0), (0, 0, 0), (1, 0, 0))
		add_link_wrapper(robot, clavicle_name, (0, facy*(0.05-0.035/2), 0), (0, -np.pi/2, np.pi/2 + np.pi*(facy+1)/2), object="motor")
		
		
		arm_name = strid+"_arm"
		add_joint_wrapper (robot, clavicle_name, arm_name, (0, facy*(0.05+0.04/2), 0), (0, 0, 0), axis=(0, 1, 0), type="revolute")
		l1 = 0.196
		add_link_wrapper(robot, arm_name, (0, 0, -l1/2), (0, 0, 0), object="arm")
		
		
		third_motor_name = strid+"_third_motor"
		add_joint_wrapper (robot, arm_name, third_motor_name, (0., facy*(0.035+0.04)/2, 0.), (0, 0, 0), type="fixed")
		add_link_wrapper(robot, third_motor_name, (0, 0, 0), (0, -np.pi/2, np.pi/2 + np.pi*(facy-1)/2), object="motor")
		
		
		forearm_name = strid+"_forearm"
		add_joint_wrapper (robot, arm_name, forearm_name, (0, facy*0., -l1), (0, 0, 0), axis=(0, 1, 0), type="revolute")
		l2 = 0.200
		add_link_wrapper(robot, forearm_name, (0, 0, -l2/2), (0, 0, 0), object="forearm")


		foot_name = strid+"_foot"
		add_joint_wrapper (robot, forearm_name, foot_name, (0, 0, -l2), (0, 0, 0), (0, 1, 0), type="fixed")
		add_link_wrapper(robot, foot_name, (0, 0, 0), (0, 0, 0), object="foot")
	
	return robot



	

if __name__ == "__main__":
	robot = create_idefX()

	with open(str(Path(__file__).parent) + "/idefX.urdf", "w") as f:
		xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="\t")
		f.write(xmlstr)