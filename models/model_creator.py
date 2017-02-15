#!/usr/bin/env python2

# -*- coding: utf-8 -*-
"""
A script that creates the tethered_turtlebot sdf for gazebo.
The model configuration can be changed editing the file settings.config
@author: Matheus Laranjeira
"""

class Color:
	red = 0.0
	green = 0.0
	blue = 0.0
	alpha = 1.0

	def __init__(self, red, green, blue):
		self.red = red
		self.green = green
		self.blue = blue

class Pose:
	x = 0.0
	y = 0.0
	z = 0.0
	phi = 0.0
	theta = 0.0
	psi = 0.0

	def __init__(self, x, y, z, phi, theta, psi):
		self.x = x
		self.y = y
		self.z = z
		self.phi = phi
		self.theta = theta
		self.psi = psi

class Inertial:
	pose = Pose
	mass = 0.0
	ixx = 0.0
	ixy = 0.0
	ixz = 0.0
	iyy = 0.0
	iyz = 0.0
	izz = 0.0

	def __init__(self, pose, mass, ixx, ixy, ixz, iyy, iyz, izz):
		self.pose = pose
		self.mass = mass
		self.ixx = ixx
		self.ixy = ixy
		self.ixz = ixz
		self.iyy = iyy
		self.iyz = iyz
		self.izz = izz
		
class Sphere:
	pose = Pose
	radius = 0.0


	def __init__(self, pose, radius):
		self.pose = pose
		self.radius = radius

class Rope_link:
	name = ""
	pose = Pose
	radius = 0.0
	length = 0.0
	min_depth = 0.0
	mu = 0.0
	mu2 = 0.0
	inertial = Inertial
	color = Color
	sphere = Sphere

	def __init__(self, name, pose, radius, length, min_depth, mu, mu2, inertial, color):
		self.name = name
		self.pose = pose
		self.radius = radius
		self.length = length
		self.min_depth = min_depth
		self.mu = mu
		self.mu2 = mu2
		self.inertial = inertial
		self.color = color
		
class Rope_joint:
	name = ""
	parent = "parent"
	child = "child"
	pose = Pose
	lowlim = -1.57
	upplim = 1.57
	damping = 0.1
	cfm_damping = 1

	def __init__(self, name, parent, child, pose, lowlim, upplim, damping):
		self.name = name
		self.pose = pose
		self.lowlim = lowlim
		self.upplim = upplim
		self.damping = damping

import os	
import lxml.etree as ltr
from io import StringIO, BytesIO
import decimal
from gazebo_sdf import *
from math import log10

if __name__ == "__main__":
	
	# Read setting from file
	script_dir = os.path.dirname(__file__) # absolute dir the script is in
	filename = "settings.config"
	file_path = os.path.join(script_dir, filename)
	file = open(file_path,'r')
	for line in file:
		if '\n' == line[-1]:
			line = line[:-1]
		exec(line.split(' = ',2)[0] + ' = ' + line.split(' = ',2)[1])
	file.close


	ROOT = ltr.Element("sdf", version="1.4")
	MODEL = ltr.SubElement(ROOT, "model", name = "tethered_turtlebots")

	"""
	# Add turtlebot1 model
	file_path = os.path.join(script_dir, "turtlebot1.sdf")
	with open(file_path, 'r') as myfile:
		data = myfile.read()
	TURTLE1 = ltr.XML("<root>data</root>")
	MODEL.text = str(ltr.tostring(TURTLE1))
	print ltr.tostring(TURTLE1)
	"""
		
	# Create link object
	linkname = "link_0"
	frame_i = Pose(0.1350, 0.0000, 0.4100, 0.0000, 0.0000, 0.0000)
	frame_o = Pose(0.0000, 0.0000, 0.0000, 0.0000, 1.5708, 0.0000)
	frame_s = Pose(-0.5*length, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000)
	inertial = Inertial(frame_o,mass,ixx,ixy,ixz,iyy,iyz,izz)
	color = Color(1, 0.25, 0)
	rope_link = Rope_link(linkname, frame_i, radius, length, min_depth, mu, mu2, inertial, color)
	rope_link.sphere.pose = frame_s
	# Create joint object
	jointname = "joint_0"
	jointpose = Pose(-0.5*length, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000)
	lowlim = -1.57
	upplim = 1.57
	parent  = "parent"
	child = "child"
	rope_joint = Rope_joint(jointname, parent, child, jointpose,lowlim,upplim,damping)
	for i in range(nelem):
		# Define link name and joint name, parent and child link
		if(i == 0):
			# link and joint index = i
			rope_link.name = rope_link.name.replace(rope_link.name[-1],str(i))
			rope_joint.name = rope_joint.name.replace(rope_joint.name[-1],str(i))
			# if first joint, parent link is robot1 attachment point (base_foot1print)
			rope_joint.parent = "base_foot1print"
		elif(i > 0 and log10(i) <= 1.0):
			# joint parent link is rope previous link
			rope_joint.parent = rope_link.name
			# update rope link with current index
			rope_link.name = rope_link.name.replace(rope_link.name[-1],str(i))
			# update joint with current index
			rope_joint.name = rope_joint.name.replace(rope_joint.name[-1],str(i))
		elif(log10(i) > 1.0 and  log10(i) < 2.0):
			rope_joint.parent = rope_link.name
			rope_link.name = rope_link.name.replace(rope_link.name[-2:],str(i))
			rope_joint.name = rope_joint.name.replace(rope_joint.name[-2:],str(i))
		# joint child is always the current link
		rope_joint.child = rope_link.name
		# Add link to model
		create_rope_link(MODEL,rope_link)
		# Update pose to add following link in next loop
		rope_link.pose.x = rope_link.pose.x + rope_link.length
		# Add joint to model
		create_univ_joint(MODEL,rope_joint)
	# Add final joint attaching the rope to the leader
	rope_joint.name = "joint_"+str(nelem)
	rope_joint.pose = Pose(0.5*length, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000)
	rope_joint.parent = "base_foot2print"
	rope_joint.child = rope_link.name
	create_univ_joint(MODEL,rope_joint)


	# WRITE THE MODEL
	file_out = os.path.join(script_dir, output)
	f = open(file_out, 'w')
	f.write('<?xml version="1.0" ?>\n')
	f.write(ltr.tostring(ROOT, pretty_print=True))
	f.close()

