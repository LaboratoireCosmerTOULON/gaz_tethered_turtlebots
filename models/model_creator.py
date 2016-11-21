#!/usr/bin/env python2

# -*- coding: utf-8 -*-
"""
A script that creates the tethered_turtlebot sdf for gazebo.
The model configuration can be changed editing the file settings.config
@author: Matheus Laranjeira
"""
import lxml.etree as ltr
import decimal
from gazebo_sdf import *

class Color:
	red = 0.0
	green = 0.0
	blue = 0.0

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

file = open('settings.config','r')  
for line in file:
    if '\n' == line[-1]:
        line = line[:-1]
    exec(line.split(' = ',2)[0] + ' = ' + line.split(' = ',2)[1])
file.close


ROOT = ltr.Element("sdf", version="1.4")
MODEL = ltr.SubElement(ROOT, "model", name = "tethered_turtlebots")

# Create rope_link
frame_i = Pose(0.1350, 0.0000, 0.4100, 0.0000, 0.0000, 0.0000)
frame_o = Pose(0.0000, 0.0000, 0.0000, 0.0000, 1.5708, 0.0000)
inertial = Inertial(frame_o,mass,ixx,ixy,ixz,iyy,iyz,izz)
color = Color(1, 0.25, 0)
rope_link = Rope_link("link_00", frame_i, radius, length, min_depth, mu, mu2, inertial, color)
create_rope_link(MODEL,rope_link)

# WRITE THE MODEL
f = open(output, 'w')
f.write('<?xml version="1.0" ?>\n')
f.write(ltr.tostring(ROOT, pretty_print=True))
f.close()

