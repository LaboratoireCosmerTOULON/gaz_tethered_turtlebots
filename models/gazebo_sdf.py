import math
import lxml.etree as ltr
import numpy as np
np.set_printoptions(precision=3)

def create_cylinder(root,radius,length):
    GEOMETRY = ltr.SubElement(root,"geometry")
    CYLINDER = ltr.SubElement(GEOMETRY,"cylinder")
    RADIUS = ltr.SubElement(CYLINDER,"radius")
    LENGTH = ltr.SubElement(CYLINDER,"length")
    RADIUS.text = str(radius)
    LENGTH.text = str(length)
    return root

def create_inertial(root,rope_link):
    POSE = ltr.SubElement(root, "pose")
    pose = rope_link.inertial.pose
    POSE.text 	= str(pose.x) +" "+str(pose.y)+" "+str(pose.z)+" "+ str(pose.phi) +" "+ str(pose.theta) +" "+str(pose.psi)
    MASS	= ltr.SubElement(root,"mass")
    MASS.text = str(rope_link.inertial.mass)
    INERTIA	= ltr.SubElement(root,"inertia")
    IXX	= ltr.SubElement(INERTIA,"ixx")
    IXX.text = str(rope_link.inertial.ixx)
    IXY	= ltr.SubElement(INERTIA,"ixy")
    IXY.text = str(rope_link.inertial.ixy)
    IXZ	= ltr.SubElement(INERTIA,"ixz")
    IXZ.text = str(rope_link.inertial.ixz)
    IYY	= ltr.SubElement(INERTIA,"iyy")
    IYY.text = str(rope_link.inertial.iyy)
    IYZ	= ltr.SubElement(INERTIA,"iyz")
    IYZ.text = str(rope_link.inertial.iyz)
    IZZ	= ltr.SubElement(INERTIA,"izz")
    IZZ.text = str(rope_link.inertial.izz)
    return root
    
def create_surface(root,rope_link):
    SURFACE = ltr.SubElement(root,"surface")
    CONTACT = ltr.SubElement(SURFACE,"contact")
    ODE = ltr.SubElement(CONTACT,"ode")
    MIN_DEPTH = ltr.SubElement(ODE,"min_depth")
    MIN_DEPTH.text = str(rope_link.min_depth)
    FRICTION = ltr.SubElement(SURFACE,"contact")
    ODEf = ltr.SubElement(CONTACT,"ode")
    MU = ltr.SubElement(ODEf,"mu")
    MU.text = str(rope_link.mu)
    MU2 = ltr.SubElement(ODEf,"mu2")
    MU2.text = str(rope_link.mu2)
    return root

def create_rope_link(root,rope_link):
    LINK		= ltr.SubElement(root, "link", name=rope_link.name)
    POSE 		= ltr.SubElement(LINK, "pose")
    POSE.text 	= str(rope_link.pose.x) +" "+str(rope_link.pose.y)+" "+str(rope_link.pose.z)+" "+ str(rope_link.pose.phi) +" "+ str(rope_link.pose.theta) +" "+str(rope_link.pose.psi)
    INERTIAL	= ltr.SubElement(LINK, "inertial")
    create_inertial(INERTIAL,rope_link)
    COLLISION 	= ltr.SubElement(LINK, "collision", name=rope_link.name + "_collision")
    create_cylinder(COLLISION,rope_link.radius,rope_link.length)
    create_surface(COLLISION, rope_link)
    VISUAL 		= ltr.SubElement(LINK, "visual",name=rope_link.name+"_visual")
    create_cylinder(VISUAL,rope_link.radius,rope_link.length)
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
    
    
