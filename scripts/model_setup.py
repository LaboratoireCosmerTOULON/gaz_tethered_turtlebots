#!/usr/bin/python

# -*- coding: utf-8 -*-
"""
A script that configures the tethered turtlebots base model
variables settings:
 - Turtles positions
@author: Matheus Laranjeira
"""

from __future__ import division
import os
import sys
#import lxml.etree as ltr
from io import StringIO, BytesIO
import decimal
from gazebo_sdf import *
from math import log10, sqrt, tan, atan2, sin, asin, cos, sinh, acosh, pi
from model_creator import *

def changeRopePose(lines,wPo1,wPo2,s,rlink,rlen,hmax,n):
	# WARNING: assuming only change in X and Y position
	
	# tether direction
	Dx = wPo1[0] - wPo2[0]
	Dy = wPo1[1] - wPo2[1]
	theta = atan2(Dy,Dx)
	tt = tan(theta)
	#print tt, theta*180/3.1416
	
	# Some cat properties
	dL = 0.01 # tether element length in meters
	h = s[0]*hmax # sag
	C = (2*h)/(rlen**2 - h**2); # Catenary constant
	D = (1/C)*acosh(C*h + 1); # Span between attachment points
	
	# link pose (first link_0)
	wPl = wPo2
	dx = dL/sqrt(1 + tt**2 + (sinh(C*wPl[1]/sin(theta) - C*D)/cos(theta))**2)
	dy = tt*dx
	Y = wPl[1] + 0.5*dy
	dz = sinh(C*Y/sin(theta) - C*D)/(cos(theta))*dx
	print "dx dy dz ",dx, dy, dz
	print "Y dz/dL ", Y, dz/dL
	wPl[4] = asin(-dz/dL)
	wPl[5] = theta
	
	# line in model to be changed (l) and line step(dl)
	l = 2022
	dl = 82

	for i in range(0,n):
		#print wPl
		lines[l] = "\t\t"+"<pose>"+str(wPl[0]) +" "+str(wPl[1])+" "+str(wPl[2])+" "+ str(wPl[3]) +" "+str(wPl[4]) +" "+str(wPl[5])+"</pose>"+"\n"
		
		# next tether element pose
		dx = dL/sqrt(1 + tt**2 + (sinh(C*wPl[1]/sin(theta) - C*D)/cos(theta))**2)
		dy = tt*dx
		Y = wPl[1] + 0.5*dy
		dz = 1/(cos(theta))*sinh(C*Y/sin(theta) - C*D) * dx
		wPl[0] = wPl[0] + dx
		wPl[1] = wPl[1] + dy
		wPl[2] = wPl[2] + dz
		wPl[3] = 0
		wPl[4] = asin(-dz/dL)#-atan2(dz,dx)
		
		l = l + dl
	return lines
	
def changeTurtlePose(lines,wPr,n):
	# WARNING: assuming only change in X and Y position
	
	# left wheel Pose in robot frame
	rPlw = np.zeros(6)
	rPlw[1] = 0.115; rPlw[2] = 0.0352; rPlw[3] = -1.5708; 
	# right wheel Pose in robot frame
	rPrw = np.zeros(6)
	rPrw[1] = -0.115; rPrw[2] = 0.0352; rPrw[3] = -1.5708; 
	
	# Lines to be modified (if turtle1 or turtle2)
	if(n==1):
		bl = 35 # base footprint line
		lwl = 861 # left wheel line
		rwl = 933 # right wheel line
	else:
		bl = 1028 # base footprint line
		lwl = 1854 # left wheel line
		rwl = 1926 # right wheel line
		
	# base_footprint
	lines[bl] = "\t\t"+"<pose>"+str(wPr[0]) +" "+str(wPr[1])+" "+str(wPr[2])+" "+ str(wPr[3]) +" "+str(wPr[4]) +" "+str(wPr[5])+"</pose>"+"\n"
	# left wheel
	wPlw = wPr + rPlw
	lines[lwl] = "\t\t"+"<pose>"+str(wPlw[0]) +" "+str(wPlw[1])+" "+str(wPlw[2])+" "+ str(wPlw[3]) +" "+str(wPlw[4]) +" "+str(wPlw[5])+"</pose>"+"\n"
	# right wheel
	wPrw = wPr + rPrw
	lines[rwl] = "\t\t"+"<pose>"+str(wPrw[0]) +" "+str(wPrw[1])+" "+str(wPrw[2])+" "+ str(wPrw[3]) +" "+str(wPrw[4]) +" "+str(wPrw[5])+"</pose>"+"\n"
	
	return lines

def configFrames(wPr2,s,rlink,rlen,hmax,nelem):
	# calculate model frames main frame poses
	# WARNING: supposing only X-Y displacement, no rotation
	
	# rope attach. pose at turtle2
	r2Po2 = np.zeros(6)
	r2Po2[0] = rlink.pose.x; r2Po2[2] = rlink.pose.z
	
	a = s[0] # h/hmax
	b = s[1] # sin(theta)
	h = a*hmax # rope sag

	# Catenary constant
	C = (2*h)/(rlen**2 - h**2);
	# Span between attachment points
	D = (1/C)*acosh(C*h + 1);
	dx = 2*D*sqrt(1 - b**2)
	dy = 2*D*b
	
	# Sigma_1 pose expressed in Sigma_2
	o2Po1 = np.zeros(6)
	o2Po1[0] = dx; o2Po1[1] = dy;
	
	# rope attach. pose at turtle1
	r1Po1 = np.zeros(6)
	r1Po1[0] = -0.15; r1Po1[2] = rlink.pose.z;
	o1Pr1 = -r1Po1
	
	# turtle1 pose in world frame
	wPr1 = wPr2 + r2Po2 + o2Po1 + o1Pr1
	
	return r2Po2, o2Po1, r1Po1, wPr1
	
def configModel(script_dir,basefile,outfile,wPr2,s,rlink,rlen,hmax,nelem):
	# in and out files
	inputfile_path = os.path.join(script_dir, basefile)
	outputfile_path = os.path.join(script_dir, outfile)
	
	# Read turtlebot model from file
	fin = open(inputfile_path, 'r')
	fout = open(outputfile_path, 'w')
	lines = fin.readlines()
	
	# modify turtlebot position
	r2Po2, o2Po1, r1Po1, wPr1 = configFrames(wPr2,s,rlink,rlen,hmax,nelem)
	changeTurtlePose(lines,wPr1,1)
	changeTurtlePose(lines,wPr2,2)
	
	# modify rope position
	wPo1 = wPr1 + r1Po1
	wPo2 = wPr2 + r2Po2
	changeRopePose(lines,wPo1,wPo2,s,rlink,rlen,hmax,nelem)
	
	# write new model in file
	for line in lines:
		fout.write(line)
		
	fin.close
	fout.close
	return

def main():
	
	# Read tether settings from file
	# absolute dir the script is in
	script_dir = os.path.dirname(__file__)
	# read file
	filename = "settings.config"
	file_path = os.path.join(script_dir, filename)
	f = open(file_path,'r')
	for line in f:
		if ('\n' == line[-1]):
			line = line[:-1]
		exec(line.split(' = ',2)[0] + ' = ' + line.split(' = ',2)[1])
	f.close
	
	# Load tether params
	# catch tether params passed by command line
	rlen = 0.5*length*nelem # tether half length
	s = np.zeros(2) # 'a', 'b'
	s[0] = sys.argv[1]; s[1] = sys.argv[2]
	
	# Turtlebots positions
	# turtle2 pose = world origin
	wPr2 = np.zeros(6)
	
	# create rope link with the loaded settings
	r2Po2 = Pose(0.1300, 0.0000, 0.4000, 0.0000, 0.0000, 0.0000) # Sigma_2 frame pose
	mo = Pose(0.0000, 0.0000, 0.0000, 0.0000, 1.5708, 0.0000) # rope elem center of mass
	inertial = Inertial(mo,mass,ixx,ixy,ixz,iyy,iyz,izz)
	ropelink = Rope_link("link_0", r2Po2, radius, length, min_depth, mu, mu2, inertial, color)
	
	# Read tethered turtlebots BASE MODEL
	baseModelFile = "../models/tethered_turtlebots_BASEMODEL.sdf"
	outFile = "../models/tethered_turtlebots.sdf"
	file_path = os.path.join(script_dir, baseModelFile)
	
	# config model
	configModel(script_dir,baseModelFile,outFile,wPr2,s,ropelink,rlen,hmax,nelem)
	return

if __name__ == "__main__":
	main()
