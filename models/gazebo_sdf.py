import math
import lxml.etree as ltr
import numpy as np
np.set_printoptions(precision=3)

def create_box(root,x,y,z):
    GEOMETRY = ltr.SubElement(root,"geometry")
    BOX = ltr.SubElement(GEOMETRY,"box")
    SIZE = ltr.SubElement(BOX,"size")
    SIZE.text = str(x) +" "+str(y)+" "+str(z)
    return root
    
def create_cylinder(root,radius,length):
    GEOMETRY = ltr.SubElement(root,"geometry")
    CYLINDER = ltr.SubElement(GEOMETRY,"cylinder")
    RADIUS = ltr.SubElement(CYLINDER,"radius")
    LENGTH = ltr.SubElement(CYLINDER,"length")
    RADIUS.text = str(radius)
    LENGTH.text = str(length)
    return root

def create_rope_link(root,link_name,x,y,z,phi,theta,psi,radius,length):
    LINK = ltr.SubElement(root, "link", name=link_name)
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(x) +" "+str(y)+" "+str(z)+" "+ str(math.pi/2) +" "+ str(0) +" "+str(0)
    COLLISION = ltr.SubElement(LINK, "collision", name=link_name + "_collision")
    create_cylinder(COLLISION,radius,length)
    VISUAL = ltr.SubElement(LINK, "visual",name=link_name+"_visual")
    create_cylinder(VISUAL,radius,length)
    return root

def create_drum(root,drum_name,x,y,z,radius,width):
    LINK = ltr.SubElement(root, "link", name=drum_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "1"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(x) +" "+str(y)+" "+str(z)+" "+ str(math.pi/2) +" "+ str(0) +" "+str(0)
    COLLISION = ltr.SubElement(LINK, "collision",name=drum_name+"_collision")
    create_cylinder(COLLISION,radius,width)
    VISUAL = ltr.SubElement(LINK, "visual",name=drum_name+"_visual")
    create_cylinder(VISUAL,radius,width)
    return root
    
def create_base(root,base_name,x,y,z,dx,dy,dz):
    LINK = ltr.SubElement(root, "link", name=base_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "0"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(x) +" "+str(y)+" "+str(z)+" "+ str(0)+" "+str(0)+" "+str(0)
    COLLISION = ltr.SubElement(LINK, "collision",name=base_name+"_collision")
    create_box(COLLISION,dx,dy,dz)
    VISUAL = ltr.SubElement(LINK, "visual",name=base_name+"_visual")
    create_box(VISUAL,dx,dy,dz)
    return root
    
def create_rev_joint(root,joint_name,parent,child,pose,xyz):
    JOINT = ltr.SubElement(root, "joint", type="revolute", name=joint_name)
    POSE = ltr.SubElement(JOINT, "pose")
    POSE.text = str(pose)
    PARENT = ltr.SubElement(JOINT,"parent")
    PARENT.text = str(parent)
    CHILD = ltr.SubElement(JOINT,"child")
    CHILD.text = str(child)
    AXIS = ltr.SubElement(JOINT,"axis")
    XYZ = ltr.SubElement(AXIS,"xyz")
    XYZ.text = str(xyz)
    return root
    
def add_pad(root,pad_name,coords,dx,dy,dz):
    LINK = ltr.SubElement(root, "link", name=pad_name)
    SELF_COLLIDE = ltr.SubElement(LINK, "self_collide")
    SELF_COLLIDE.text = "1"
    POSE = ltr.SubElement(LINK, "pose")
    POSE.text = str(coords[0]) +" "+str(coords[1])+" "+str(coords[2])+" "+ str(coords[3])+" "+str(coords[4])+" "+str(coords[5])
    COLLISION = ltr.SubElement(LINK, "collision",name=pad_name+"_collision")
    create_box(COLLISION,dx,dy,dz)
    VISUAL = ltr.SubElement(LINK, "visual",name=pad_name+"_visual")
    create_box(VISUAL,dx,dy,dz)
    return root
    
