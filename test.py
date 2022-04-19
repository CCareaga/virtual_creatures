import os
from time import sleep 
import tempfile
import random
import pickle
import argparse

import pybullet as p
import pybullet_data

import numpy as np
import torch

def load_creature(c):

    tf = tempfile.NamedTemporaryFile(delete=False)
    with open(tf.name, 'w') as f:
        f.write(str(c.get_urdf()))

    c_id = p.loadURDF(tf.name)
        
    os.remove(tf.name)

    c.bullet_id = c_id

    for j in range(c.num_joints()):
        p.enableJointForceTorqueSensor(c_id, j, 1)
        p.setJointMotorControl2(c_id, j, controlMode=p.VELOCITY_CONTROL, force=0)

def get_input_vector(creature, plain_id):
    c_id = creature.bullet_id
    num_joints = creature.num_joints()
    joint_inds = list(range(num_joints))

    joint_vec = []
    for js in p.getJointStates(c_id, joint_inds):
        joint_vec += js[:2]
    
    joint_vec = torch.Tensor(joint_vec)
    
    # compute links in contact with the ground
    contact_vec = torch.zeros((num_joints + 1))
    for pt in p.getContactPoints(c_id, plane_id):
        contact_vec[pt[3] + 1] = 1.0


    return torch.cat((joint_vec, contact_vec), 0)

def apply_torques(creature, torques):
    c_id = creature.bullet_id
    joint_inds = list(range(creature.num_joints()))

    for j in joint_inds:
        p.setJointMotorControl2(c_id, j, controlMode=p.TORQUE_CONTROL, force=torques[j])



parser = argparse.ArgumentParser(description='test a generated creature')
parser.add_argument('--checkpoint', type=str, help='stored creature pickle file')
args = parser.parse_args()


# bullet_client = p.connect(p.DIRECT)
bullet_client = p.connect(p.GUI)

p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
    
with open(args.checkpoint, 'rb') as f:
    creature = pickle.load(f)

load_creature(creature)

dt = 1/240

while True:
    p.stepSimulation()

    input_vec = get_input_vector(creature, plane_id) 
    torques = creature.step(input_vec)
    apply_torques(creature, torques)

    sleep(dt)
