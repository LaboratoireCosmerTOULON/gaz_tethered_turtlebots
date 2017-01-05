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
    
    
def create_sphere(root,radius):
    GEOMETRY = ltr.SubElement(root,"geometry")
    SPHERE = ltr.SubElement(GEOMETRY,"sphere")
    RADIUS = ltr.SubElement(SPHERE,"radius")
    RADIUS.text = str(radius)
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
    FRICTION = ltr.SubElement(SURFACE,"friction")
    ODEf = ltr.SubElement(FRICTION,"ode")
    MU = ltr.SubElement(ODEf,"mu")
    MU.text = str(rope_link.mu)
    MU2 = ltr.SubElement(ODEf,"mu2")
    MU2.text = str(rope_link.mu2)
    return root

def create_material(root,rope_link):
    AMBIENT	= ltr.SubElement(root,"ambient")
    AMBIENT.text = str(rope_link.color.red)+" "+ str(rope_link.color.green)+" "+ str(rope_link.color.blue) + " " + str(rope_link.color.alpha)
    DIFFUSE	= ltr.SubElement(root,"diffuse")
    DIFFUSE.text = str(rope_link.color.red)+" "+ str(rope_link.color.green)+" "+ str(rope_link.color.blue) + " " + str(rope_link.color.alpha)
    SPECULAR = ltr.SubElement(root,"specular")
    SPECULAR.text = str(0.1)+" "+ str(0.1)+" "+ str(0.1) + " " + str(1)
    EMISSIVE	= ltr.SubElement(root,"emissive")
    EMISSIVE.text = str(0)+" "+ str(0)+" "+ str(0) + " " + str(0)
    return root
    
def create_collision(root,rope_link):
    POSE = ltr.SubElement(root, "pose")
    pose = rope_link.inertial.pose
    POSE.text 	= str(pose.x) +" "+str(pose.y)+" "+str(pose.z)+" "+ str(pose.phi) +" "+ str(pose.theta) +" "+str(pose.psi)
    create_cylinder(root,rope_link.radius,rope_link.length)
    create_surface(root, rope_link)
    return root

def create_visual(root,rope_link):
    POSE = ltr.SubElement(root, "pose")
    pose = rope_link.inertial.pose
    POSE.text 	= str(pose.x) +" "+str(pose.y)+" "+str(pose.z)+" "+ str(pose.phi) +" "+ str(pose.theta) +" "+str(pose.psi)
    create_cylinder(root,rope_link.radius,rope_link.length)
    MATERIAL = ltr.SubElement(root, "material")
    create_material(MATERIAL,rope_link)
    return root
    
def create_visual_sphere(root,rope_link):
    POSE = ltr.SubElement(root, "pose")
    pose = rope_link.sphere.pose
    POSE.text 	= str(pose.x) +" "+str(pose.y)+" "+str(pose.z)+" "+ str(pose.phi) +" "+ str(pose.theta) +" "+str(pose.psi)
    create_sphere(root,rope_link.radius)
    MATERIAL = ltr.SubElement(root, "material")
    create_material(MATERIAL,rope_link)
    return root

def create_rope_link(root,rope_link):
    LINK		= ltr.SubElement(root, "link", name=rope_link.name)
    POSE 		= ltr.SubElement(LINK, "pose")
    POSE.text 	= str(rope_link.pose.x) +" "+str(rope_link.pose.y)+" "+str(rope_link.pose.z)+" "+ str(rope_link.pose.phi) +" "+ str(rope_link.pose.theta) +" "+str(rope_link.pose.psi)
    INERTIAL	= ltr.SubElement(LINK, "inertial")
    create_inertial(INERTIAL,rope_link)
    COLLISION 	= ltr.SubElement(LINK, "collision", name=rope_link.name + "_collision")
    create_collision(COLLISION,rope_link)
    VISUAL 		= ltr.SubElement(LINK, "visual",name=rope_link.name+"_visual")
    create_visual(VISUAL,rope_link)
    VISUAL_SPH	= ltr.SubElement(LINK, "visual",name=rope_link.name+"_visual_sphere")
    create_visual_sphere(VISUAL_SPH,rope_link)
    return root

def create_axis(root,rope_joint):
    AXIS = ltr.SubElement(root,"axis")
    XYZ = ltr.SubElement(AXIS,"xyz")
    XYZ.text = "0 1 0"
    LIMIT = ltr.SubElement(AXIS,"limit")
    LOWER = ltr.SubElement(LIMIT,"lower")
    LOWER.text = str(rope_joint.lowlim)
    UPPER = ltr.SubElement(LIMIT,"upper")
    UPPER.text = str(rope_joint.upplim)
    DYNAM = ltr.SubElement(AXIS,"dynamics")
    DAMPI = ltr.SubElement(DYNAM,"damping")
    DAMPI.text = str(rope_joint.damping)
    
def create_axis2(root,rope_joint):
    AXIS2 = ltr.SubElement(root,"axis2")
    XYZ2 = ltr.SubElement(AXIS2,"xyz")
    XYZ2.text = "0 0 1"
    LIMIT = ltr.SubElement(AXIS2,"limit")
    LOWER = ltr.SubElement(LIMIT,"lower")
    LOWER.text = str(rope_joint.lowlim)
    UPPER = ltr.SubElement(LIMIT,"upper")
    UPPER.text = str(rope_joint.upplim)
    DYNAM = ltr.SubElement(AXIS2,"dynamics")
    DAMPI = ltr.SubElement(DYNAM,"damping")
    DAMPI.text = str(rope_joint.damping)
    
def create_joint_physics(root):
    PHYSICS = ltr.SubElement(root,"physics")
    ODE = ltr.SubElement(PHYSICS,"ode")
    CFM = ltr.SubElement(ODE,"cfm_damping")
    CFM.text = str(1)
    return root

def create_univ_joint(root,rope_joint):
    JOINT = ltr.SubElement(root, "joint", type="universal", name=rope_joint.name)
    POSE = ltr.SubElement(JOINT, "pose")
    pose = rope_joint.pose
    POSE.text 	= str(pose.x) +" "+str(pose.y)+" "+str(pose.z)+" "+ str(pose.phi) +" "+ str(pose.theta) +" "+str(pose.psi)
    PARENT = ltr.SubElement(JOINT,"parent")
    PARENT.text = str(rope_joint.parent)
    CHILD = ltr.SubElement(JOINT,"child")
    CHILD.text = str(rope_joint.child)
    create_axis(JOINT,rope_joint)
    create_axis2(JOINT,rope_joint)
    create_joint_physics(JOINT)
    return root
    
    
