import os
import tempfile
from time import sleep, time

import pybullet as p
import pybullet_data

import numpy as np
import torch

from creature import xml_to_np

# ==================================================================
def load_creature(client, c, plane_id):

    tf = tempfile.NamedTemporaryFile(delete=False)
    with open(tf.name, 'w') as f:
        f.write(str(c.get_urdf()))

    c_id = p.loadURDF(tf.name, flags=p.URDF_MAINTAIN_LINK_ORDER)
        
    os.remove(tf.name)

    c.bullet_id = c_id
    
    for j in range(c.num_joints()):
        p.enableJointForceTorqueSensor(c_id, j, 1, physicsClientId=client)
        p.setJointMotorControl2(c_id, j, controlMode=p.VELOCITY_CONTROL, force=0, physicsClientId=client)
    
    prev_pos = np.zeros(3)
    counter = 0

    while True:

        p.stepSimulation(physicsClientId=client)
    
        max_speed = 0
        for j in range(c.num_joints()):
            p.setJointMotorControl2(c_id, j, controlMode=p.TORQUE_CONTROL, force=0, physicsClientId=client)

            link_velo = p.getLinkState(c_id, j, computeLinkVelocity=True, physicsClientId=client)[6]
            link_velo = np.array(link_velo)
            link_speed = np.linalg.norm(link_velo)

            max_speed = max(link_speed, max_speed)

        if max_speed < 0.001:
            break

        if counter > 1000:
            # print("failed to find stable creature state")
            return -1

        counter += 1

    for j in range(c.num_joints()):
        p.setJointMotorControl2(c_id, j, controlMode=p.VELOCITY_CONTROL, force=0, physicsClientId=client)

    return 1


def get_input_vector(client, creature, plane_id):
    c_id = creature.bullet_id
    num_joints = creature.num_joints()
    joint_inds = list(range(num_joints))

    joint_vec = []
    for js in p.getJointStates(c_id, joint_inds, physicsClientId=client):
        joint_vec += js[:2]
    
    joint_vec = torch.Tensor(joint_vec)
    
    # compute links in contact with the ground
    contact_vec = torch.zeros((num_joints + 1))
    for pt in p.getContactPoints(c_id, plane_id, physicsClientId=client):
        contact_vec[pt[3] + 1] = 1.0

    return torch.cat((joint_vec, contact_vec), 0)

def apply_torques(client, creature, torques):
    c_id = creature.bullet_id
    joint_inds = list(range(creature.num_joints()))

    for j in joint_inds:
        p.setJointMotorControl2(c_id, j, controlMode=p.TORQUE_CONTROL, force=torques[j], physicsClientId=client)

        curr_vel = p.getJointState(c_id, j, physicsClientId=client)[1]
        max_vel = 30.0
        if curr_vel > max_vel:
            p.setJointMotorControl2(c_id, j, controlMode=p.VELOCITY_CONTROL, force=10, physicsClientId=client)
        if curr_vel < -max_vel:
            p.setJointMotorControl2(c_id, j, controlMode=p.VELOCITY_CONTROL, force=10, physicsClientId=client)

def get_positions_vector(client, creature, goal_pos):



def train_simulation(creature, num_steps=10000):
    client = p.connect(p.DIRECT)
    
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    status = load_creature(client, creature, plane_id)
    if status == -1:
        p.disconnect(physicsClientId=client)
        return 0.0

    fitness = 0.0
    start_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
    prev_pos = start_pos

    goal_pos = np.array([1., 1., 0.])
    goal_vel = np.array([0., 0., 0.])

    for i in range(1, num_steps):
        p.stepSimulation(physicsClientId=client)

        input_vec = get_input_vector(client, creature, plane_id) 
        # posisiton_vec = get_positions_vector(client, 



        torques = creature.step(input_vec)
        apply_torques(client, creature, torques)
        
        base_velo, _ = p.getBaseVelocity(creature.bullet_id, physicsClientId=client)
        link_velo = np.array(base_velo)[:2]
        speed = np.linalg.norm(link_velo)

        curr_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
        curr_pos = np.array(curr_pos)
        
        delta = np.abs(prev_pos - curr_pos).sum()
        if delta < 0.001:
            break
        else:
            # print(delta)
            prev_pos = curr_pos
    
        fitness += delta

    
    end_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id)
    end_pos = np.array(end_pos)[:2]
    start_pos = np.array(start_pos)[:2]
    p.disconnect(physicsClientId=client)

    fitness = np.abs(start_pos - end_pos).sum()

    return fitness
    
def test_simulation(creature):
    client = p.connect(p.GUI)
    
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    load_creature(client, creature, plane_id)

    distance = 0.0
    prev_pos = np.array([0, 0, 0])

    dt = 1.0 / 240.

    goal_pos = np.array([1., 1., 0.])
    goal_vel = np.array([0., 0., 0.])

    while True:
        p.stepSimulation(physicsClientId=client)
        
        start = time()

        p.addUserDebugLine(
            goal_pos, 
            goal_pos + np.array([0, 0, 0.5]),
            lineWidth=3,
            lifeTime=0.5,
            lineColorRGB=np.array([1, 0 ,0]),
            physicsClientId=client
        )
        
        goal_change = (np.random.rand(3) - 0.5) / 1000
        goal_change[-1] = 0.0

        goal_vel = (goal_vel + goal_change).clip(-0.01, 0.01)
        goal_pos += goal_vel

        input_vec = get_input_vector(client, creature, plane_id) 
        torques = creature.step(input_vec)
        apply_torques(client, creature, torques)
        
        duration = time() - start
        sleep(max(0, dt - duration))
        
        link_velo = p.getLinkState(creature.bullet_id, 0, computeLinkVelocity=True, physicsClientId=client)[6]
        link_velo = np.array(link_velo)
        delta = np.linalg.norm(link_velo)

        # curr_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
        # curr_pos = np.array(curr_pos)

        # print(np.power(curr_pos, 2).sum())
        
        # delta = np.abs(prev_pos - curr_pos).sum()

        # if delta.item() == 0:
        #     break
        # else:
        #     prev_pos = curr_pos
        # 
        # distance += delta
        # print(delta)
        
    
    p.disconnect(physicsClientId=client)

# ==================================================================

