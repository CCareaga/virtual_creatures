import random
import numpy as np

from creature import gen_random_creature
from odio_urdf.odio_urdf import *

# def gen_random_axis():
#     vec = np.random.rand(3) 
#     return vec / np.linalg.norm(vec)
# 
# def gen_random_dims(max_dim=1, min_dim=0.1):
#     return np.array([random.uniform(min_dim, max_dim) for _ in range(3)])
# 
# def gen_random_surface_pos(sizes):
#     bounds = sizes / 2.0
#     x = random.uniform(-bounds[0], bounds[0])
#     y = random.uniform(-bounds[1], bounds[1])
#     z = random.uniform(-bounds[2], bounds[2])
#     rand_pos = [x, y, z]
# 
#     locked_idx = random.randint(0, 2)
#     rand_pos[locked_idx] = random.choice([-bounds[locked_idx], bounds[locked_idx]])
#     
#     return np.array(rand_pos)
# 
# def gen_random_limits(max_rot=3.0):
#     lower = random.uniform(-3.14, 2.0)
#     offset = random.uniform(0.5, 2.0)
# 
#     return lower, np.minimum(lower + offset, 3.14)
# 
# def np_to_xml(arr):
#     return " ".join([str(x) for x in arr])
# 
# def create_link(name, position, size):
#     
#     pos_str = np_to_xml(position)
#     sz_str = np_to_xml(size)
# 
#     mass = np.prod(size)
#     c = mass / 12
# 
#     ixx = c * (position[0] ** 2 + position[2] ** 2)
#     iyy = c * (position[1] ** 2 + position[2] ** 2)
#     izz = c * (position[1] ** 2 + position[0] ** 2)
# 
#     return Link(
#         Inertial(
#             Origin(xyz=pos_str), 
#             Mass(value=mass), 
#             Inertia(ixx=ixx, iyy=iyy, izz=izz)
#         ),
#         Visual(
#             Origin(xyz=pos_str),
#             Geometry(Box(size=sz_str)),
#             Material(Color(rgba="0 1.0 1.0 1.0"), name='cyan')
#         ),
#         Collision(
#             Origin(xyz=pos_str),
#             Geometry(Box(size=sz_str)),
#         ),
#         name=name,
#     )
# 
# def create_joint(name, parent, child, joint_type, pos, axis, lower, upper):
# 
#     pos_str = np_to_xml(pos)
#     axis_str = np_to_xml(axis)
# 
#     return Joint(
#         Parent(link=parent),
#         Child(link=child),
#         Origin(xyz=pos_str),
#         Axis(xyz=axis_str),
#         Limit(effort="0", lower=str(lower), upper=str(upper), velocity="0.0"),
#         name=name,
#         type=joint_type
#     )
# 
# 
# root_sizes = gen_random_dims()
# root_pos = np.array([0, 0, 1])
# 
# joint1_pos = root_pos + gen_random_surface_pos(root_sizes)
# joint1_lo, joint1_hi = gen_random_limits()
# 
# link2_sizes = gen_random_dims()
# joint2_pos = gen_random_surface_pos(link2_sizes)
# joint2_lo, joint2_hi = gen_random_limits()
# 
# link1 = create_link(
#     "0", 
#     np.array([0, 0, 1]),
#     root_sizes
# )
# 
# joint1 = create_joint(
#     "0_1",
#     "0",
#     "1",
#     "revolute",
#     joint1_pos,
#     gen_random_axis(),
#     joint1_lo,
#     joint1_hi
# )
# 
# link2 = create_link(
#     "1", 
#     np.array([0, 0, 0]),
#     link2_sizes
# )
# 
# joint2 = create_joint(
#     "1_2",
#     "1",
#     "2",
#     "revolute",
#     joint2_pos,
#     gen_random_axis(),
#     joint2_lo,
#     joint2_hi
# )
# 
# link3 = create_link(
#     "2", 
#     np.array([0, 0, 0]),
#     gen_random_dims()
# )
# 
# 
# my_robot = Robot(
#     link1,
#     joint1,
#     link2,
#     joint2,
#     link3,
#     name="creature"
# )
# 
# print(my_robot)

test = gen_random_creature()
print(test.get_urdf())

with open('test.urdf', 'w') as f:
    f.write(str(test.get_urdf()))
