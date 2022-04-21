import random
import numpy as np
import torch

from controller import Controller

from odio_urdf.odio_urdf import *

# HELPER FUNCTIONS ====================================
def gen_random_axis():
    vec = np.random.rand(3) - 0.5
    return vec / np.linalg.norm(vec)

def gen_random_dims(max_dim=1, min_dim=0.2):
    rand_vec = np.array([random.uniform(min_dim, max_dim) for _ in range(3)])
    return rand_vec / np.linalg.norm(rand_vec)

def gen_random_surface_pos(sizes):
    
    bounds = sizes / 2.0
    x = random.uniform(-bounds[0], bounds[0])
    y = random.uniform(-bounds[1], bounds[1])
    z = random.uniform(-bounds[2], bounds[2])
    rand_pos = [x, y, z]
   
    # if face == 'left': rand_pos[0] = -bounds[0] # negative x 
    # if face == 'right': rand_pos[0] = bounds[0] # positive x
    # if face == 'back': rand_pos[1] = -bounds[1] # negative y
    # if face == 'front': rand_pos[1] = bounds[1] # positive y
    # if face == 'bottom': rand_pos[2] = -bounds[2] # negative z 
    # if face == 'top': rand_pos[2] = bounds[2] # positive z

    locked_idx = random.randint(0, 2)
    rand_pos[locked_idx] = random.choice([-bounds[locked_idx], bounds[locked_idx]])
    
    return np.array(rand_pos)

def gen_random_limits(max_rot=3.0):
    lower = random.uniform(-3.14, 2.0)
    offset = random.uniform(0.5, 2.0)

    return lower, np.minimum(lower + offset, 3.14)

def get_opposing_face(face):
    if face == 'left': return 'right'
    if face == 'right': return 'left'
    if face == 'back': return 'front'
    if face == 'front': return 'back'
    if face == 'bottom': return 'top'
    if face == 'top': return 'bottom'

def np_to_xml(arr):
    return " ".join([str(x) for x in arr])

def xml_to_np(arr):
    return np.array([float(x) for x in arr.split(' ')])

def create_link(name, position, size):
    
    pos_str = np_to_xml(position)
    sz_str = np_to_xml(size)

    mass = np.prod(size) * 2.0
    c = mass / 12.0

    ixx = c * (size[0] ** 2 + size[2] ** 2)
    iyy = c * (size[1] ** 2 + size[2] ** 2)
    izz = c * (size[1] ** 2 + size[0] ** 2)

    return Link(
        Inertial(
            Origin(xyz=pos_str), 
            Mass(value=mass), 
            Inertia(ixx=ixx, iyy=iyy, izz=izz)
        ),
        Visual(
            Origin(xyz=pos_str),
            Geometry(Box(size=sz_str)),
            Material(Color(rgba="0 1.0 1.0 1.0"), name='cyan')
        ),
        Collision(
            Origin(xyz=pos_str),
            Geometry(Box(size=sz_str)),
        ),
        name=name,
    )

def create_joint(name, parent, child, joint_type, pos, axis, lower, upper):

    pos_str = np_to_xml(pos)
    axis_str = np_to_xml(axis)

    return Joint(
        Parent(link=parent),
        Child(link=child),
        Origin(xyz=pos_str),
        Axis(xyz=axis_str),
        Limit(effort="0", lower=str(lower), upper=str(upper), velocity="0.0"),
        name=name,
        type=joint_type
    )

def gen_random_creature(min_joints=2, max_joints=4, hidden_size=16):

    creat = Creature()

    num_joints = random.randint(min_joints, max_joints)

    # first create the root link
    root_sizes = gen_random_dims()
    root_link = create_link("0", np.array([0, 0, 2.0]), root_sizes)

    creat.links.append(root_link)
    creat.link_terminal.append(True)

    for i in range(num_joints):

        # choose a random parent
        parent = random.choice(creat.links)
        parent_id = parent.name
        creat.link_terminal[int(parent_id)] = False
        parent_sizes = xml_to_np(parent[2][1][0].size)

        new_id = len(creat.links)

        child_sizes = gen_random_dims()
        
        new_joint_pos = gen_random_surface_pos(parent_sizes) 
        
        if parent_id == "0":
            new_joint_pos += np.array([0, 0, 2.0])
        
        new_link = create_link(
            str(new_id), 
            np.zeros(3),
            child_sizes
        )

        joint_lo, joint_hi = gen_random_limits()
        new_joint = create_joint(
            f"{parent_id}_{new_id}",
            parent_id,
            str(new_id),
            "revolute",
            new_joint_pos,
            gen_random_axis(),
            joint_lo,
            joint_hi
        )

        creat.links.append(new_link)
        creat.link_terminal.append(True)
        creat.joints.append(new_joint)
    
    num_joints = len(creat.joints)

    new_controller = Controller(
        (num_joints * 2) + (num_joints + 1),
        hidden_size,
        num_joints
    )
    
    creat.controller = new_controller

    return creat

class Creature():
    def __init__(self):
        
        self.links = []
        self.link_terminal = []
        self.joints = []
        self.controller = None

    def add_link(self, link):
        self.links.append(link)
        
    def add_joint(self, joint):
        self.joints.append(joint)
    
    def num_links(self):
        return len(self.links)

    def num_joints(self):
        return len(self.joints)

    def step(self, input_vec):
        return self.controller.forward(input_vec)

    def mutate(self):
        self.controller.mutate()
        
        # terminal link size mutation ===========
        while True:
            rand_idx = random.randint(0, self.num_links() - 1)
            if self.link_terminal[rand_idx]:
                break

        rand_link = self.links[rand_idx]

        curr_size = xml_to_np(rand_link[1][1][0].size)
        curr_name = rand_link.name
        curr_pos = xml_to_np(rand_link[0][0].xyz)

        rand_val = np.random.rand(3) - 0.5
        new_size = (curr_size + rand_val).clip(0.2, 1.0)
        
        self.links[rand_idx] = create_link(curr_name, curr_pos, new_size)
        # =======================================


        # joint axis mutation ===================
        rand_idx = random.randint(0, self.num_joints() - 1)
        rand_joint = self.joints[rand_idx]
        curr_axis = xml_to_np(rand_joint[3].xyz)

        rand_val = np.random.rand(3) - 0.5
        new_axis = (curr_axis + rand_val)
        new_axis /= np.linalg.norm(new_axis)
        rand_joint[3].xyz = np_to_xml(new_axis)
        # =====================================
        

    def get_urdf(self):
        return Robot(
            *self.links,
            *self.joints
        )
