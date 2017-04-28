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
from math import log10, acosh, sqrt, pow
from model_creator import *

def calcT1Position(wPr2,s,rlen,hmax):
	# WARNING: supposing only X-Y displacement, no rotation
	
	# rope attach. pose at turtle2
	r2Po2 = np.zeros(6)
	r2Po2[0] = 0.135; r2Po2[2] = 0.41
	
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
	
	# rope attach. pose at turtle21
	r1Po1 = np.zeros(6)
	r1Po1[0] = -0.155; r1Po1[2] = 0.41;
	o1Pr1 = -r1Po1
	
	# turtle1 pose in world frame
	wPr1 = wPr2 + r2Po2 + o2Po1 + o1Pr1
	
	return wPr1
	
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
		bl = 5 # base footprint line
		lwl = 831 # left wheel line
		rwl = 903 # right wheel line
	else:
		bl = 993 # base footprint line
		lwl = 1819 # left wheel line
		rwl = 1891 # right wheel line
		
	# base_footprint
	lines[bl] = "\t\t"+"<pose>"+str(wPr[0]) +" "+str(wPr[1])+" "+str(wPr[2])+" "+ str(wPr[3]) +" "+str(wPr[4]) +" "+str(wPr[5])+"</pose>"+"\n"
	# left wheel
	wPlw = wPr + rPlw
	lines[lwl] = "\t\t"+"<pose>"+str(wPlw[0]) +" "+str(wPlw[1])+" "+str(wPlw[2])+" "+ str(wPlw[3]) +" "+str(wPlw[4]) +" "+str(wPlw[5])+"</pose>"+"\n"
	# right wheel
	wPrw = wPr + rPrw
	lines[rwl] = "\t\t"+"<pose>"+str(wPrw[0]) +" "+str(wPrw[1])+" "+str(wPrw[2])+" "+ str(wPrw[3]) +" "+str(wPrw[4]) +" "+str(wPrw[5])+"</pose>"+"\n"
	
	return lines
	
def changeRopePose(lines,t2p,rlink,n):
	# WARNING: assuming only change in X and Y position
	rlp = np.zeros(6) # rope link pose
	rlp[0] = rlink.pose.x; rlp[1] = rlink.pose.y; rlp[2] = rlink.pose.z;
	rlp[3] = rlink.pose.phi; rlp[4] = rlink.pose.theta; rlp[5] = rlink.pose.psi;
	# rope link0 pose
	rlp[0] = t2p[0] + rlp[0]
	rlp[1] = t2p[1] + rlp[1]
	
	#float h = a*hmax_; # rope sag
	# Catenary constant
	#float C = 2*h/(pow(rlen,2) - pow(h,2));
	# Span between attachment points
	#float D = (1/C)*acosh(C*h + 1);

	l = 1982
	dl = 82
	for i in range(0,n-1):
		lines[l] = "\t\t"+"<pose>"+str(rlp[0]) +" "+str(rlp[1])+" "+str(rlp[2])+" "+ str(rlp[3]) +" "+str(rlp[4]) +" "+str(rlp[5])+"</pose>"+"\n"
		rlp[0] = rlp[0] + rlink.length
		l = l + dl
		
	return lines
	
def configModel(script_dir,basefile,outfile,wPr1,wPr2,rlink,nelem):
	# in and out files
	inputfile_path = os.path.join(script_dir, basefile)
	outputfile_path = os.path.join(script_dir, outfile)
	
	# Read turtlebot model from file
	fin = open(inputfile_path, 'r')
	fout = open(outputfile_path, 'w')
	lines = fin.readlines()
	
	# modify turtlebot position
	changeTurtlePose(lines,wPr1,1)
	changeTurtlePose(lines,wPr2,2)
	changeRopePose(lines,wPr2,rlink,nelem)
	
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
	wPr1 = calcT1Position(wPr2,s,rlen,hmax)
	
	# create rope link with the loaded settings
	r2Po2 = Pose(0.1350, 0.0000, 0.4100, 0.0000, 0.0000, 0.0000) # Sigma_2 frame pose
	mo = Pose(0.0000, 0.0000, 0.0000, 0.0000, 1.5708, 0.0000) # rope elem center of mass
	inertial = Inertial(mo,mass,ixx,ixy,ixz,iyy,iyz,izz)
	ropelink = Rope_link("link_0", r2Po2, radius, length, min_depth, mu, mu2, inertial, color)
	
	# Read tethered turtlebots BASE MODEL
	baseModelFile = "../models/tethered_turtlebots_BASEMODEL.sdf"
	outFile = "../models/tethered_turtlebots.sdf"
	file_path = os.path.join(script_dir, baseModelFile)
	
	# config models
	configModel(script_dir,baseModelFile,outFile,wPr1,wPr2,ropelink,nelem)
	return

if __name__ == "__main__":
	main()
