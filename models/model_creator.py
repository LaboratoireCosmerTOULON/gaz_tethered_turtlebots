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

file = open('settings.config','r')  
for line in file:
    if '\n' == line[-1]:
        line = line[:-1]
    exec(line.split(' = ',2)[0] + ' = ' + line.split(' = ',2)[1])
file.close


ROOT = ltr.Element("sdf", version="1.4")
MODEL = ltr.SubElement(ROOT, "model", name = "model_creator")

create_rope_link(MODEL,"link_00",0,0,0,0,0,0,seg_r,seg_l)

# WRITE THE MODEL
f = open(output, 'w')
f.write(ltr.tostring(ROOT, pretty_print=True))
f.close()

