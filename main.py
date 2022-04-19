import os
import copy
from time import sleep 
import tempfile
import random
import pickle

import pybullet as p
import pybullet_data

import numpy as np
import torch

from controller import Controller
from creature import gen_random_creature

POP_SIZE = 25
TOP_K = 5
MUTATE = 10
NUM_GENERATIONS = 100
NUM_STEPS = 1000

# ==================================================================
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

# ==================================================================

population = []

for i in range(POP_SIZE):
    creature = gen_random_creature()
    population.append(creature)
    
dt = 1 / 240


for gen in range(NUM_GENERATIONS):
    
    scores = []

    for i, creature in enumerate(population):
        
        bullet_client = p.connect(p.DIRECT)
        # bullet_client = p.connect(p.GUI)
        
        p.setGravity(0,0,-9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        
        load_creature(creature)

        for _ in range(NUM_STEPS):
            p.stepSimulation()
            # sleep(dt)
            
            input_vec = get_input_vector(creature, plane_id) 
            torques = creature.step(input_vec)
            apply_torques(creature, torques)

        end_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id)
        p.disconnect()

        distance = sum([x**2 for x in end_pos])
        scores.append(distance)
        
    sorted_pop = [i[0] for i in sorted(zip(population, scores), key=lambda x:x[1], reverse=True)]
    best_k = sorted_pop[:TOP_K]
    new_pop = best_k[:]

    for i in range(MUTATE):
        parent = random.choice(best_k)

        child = copy.deepcopy(parent)
        child.controller.mutate()
        new_pop.append(child)

    for i in range(POP_SIZE - (TOP_K + MUTATE)):
        creature = gen_random_creature()
        population.append(creature)
        
    population = new_pop
    print(sorted(scores)[-1])
    
    best = sorted_pop[0]

    with open(f'models/best_{gen}.pkl', 'wb') as f:
        pickle.dump(best, f, pickle.HIGHEST_PROTOCOL)
    

