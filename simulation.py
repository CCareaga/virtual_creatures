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

def positional_encode(x, dim=4):
    encoding = np.zeros(dim)
    
    for i in range(0, dim, 2):
        encoding[i] = np.sin(x / (10000 ** ((2 * i) / dim)))
        encoding[i + 1] = np.cos(x / (10000 ** ((2 * i) / dim)))
         
    return encoding 

def get_positions_vector(client, creature, goal_pos):
    c_id = creature.bullet_id 

    curr_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)

    curr_x = positional_encode(curr_pos[0])
    curr_y = positional_encode(curr_pos[1])

    goal_x = positional_encode(goal_pos[0])
    goal_y = positional_encode(goal_pos[1])

    positional_vec = np.concatenate((curr_x, curr_y, goal_x, goal_y))

    return torch.from_numpy(positional_vec).float()

def train_following(creature, num_steps=10000):
    fitness = 0

    for i in range(10):
        fitness += train_following_single(creature)

    return fitness / 10

def train_following_single(creature, num_steps=10000):
    client = p.connect(p.DIRECT)
    
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    status = load_creature(client, creature, plane_id)
    if status == -1:
        p.disconnect(physicsClientId=client)
        return -float('inf')

    fitness = 0.0
    start_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
    start_pos = np.array(start_pos)[:2]
    prev_pos = start_pos

    goal_pos = np.random.rand(2) - 0.5
    goal_pos /= (np.linalg.norm(goal_pos) / 5)
    goal_pos += start_pos

    goal_dist = np.power(start_pos - goal_pos, 2).sum()

    for i in range(1, num_steps):
        p.stepSimulation(physicsClientId=client)

        # goal_change = (np.random.rand(3) - 0.5) / 1000
        # goal_change[-1] = 0.0

        # goal_vel = (goal_vel + goal_change).clip(-0.01, 0.01)
        # goal_pos += goal_vel

        input_vec = get_input_vector(client, creature, plane_id) 

        # if following is True:
        position_vec = get_positions_vector(client, creature, goal_pos)
        input_vec = torch.cat((input_vec, position_vec))

        torques = creature.step(input_vec)
        apply_torques(client, creature, torques)
        
        base_velo, _ = p.getBaseVelocity(creature.bullet_id, physicsClientId=client)
        link_velo = np.array(base_velo)[:2]
        speed = np.linalg.norm(link_velo)

        curr_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
        curr_pos = np.array(curr_pos)[:2]
        
        delta = np.abs(prev_pos - curr_pos).sum()
        if delta < 0.001:
            break
        else:
            # print(delta)
            prev_pos = curr_pos
        
        goal_dist = np.power(curr_pos - goal_pos, 2).sum()
        fitness += -goal_dist

    end_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id)
    end_pos = np.array(end_pos)[:2]

    p.disconnect(physicsClientId=client)

    # fitness = np.abs(start_pos - end_pos).sum()
    fitness = -np.power(goal_pos - end_pos, 2).sum()
    
    return fitness


def train_walking(creature, num_steps=10000):
    client = p.connect(p.DIRECT)
    
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    status = load_creature(client, creature, plane_id)
    if status == -1:
        p.disconnect(physicsClientId=client)
        return 0.0

    start_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id, physicsClientId=client)
    prev_pos = start_pos

    for i in range(1, num_steps):
        p.stepSimulation(physicsClientId=client)

        input_vec = get_input_vector(client, creature, plane_id) 
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
            prev_pos = curr_pos
    
    end_pos, _ = p.getBasePositionAndOrientation(creature.bullet_id)
    end_pos = np.array(end_pos)[:2]
    start_pos = np.array(start_pos)[:2]

    p.disconnect(physicsClientId=client)

    fitness = np.abs(start_pos - end_pos).sum()

    return fitness
    
def test_following(creature, record=False, out_file=None):
    client = p.connect(p.GUI)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    load_creature(client, creature, plane_id)

    if record:
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, out_file)

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

        # if following is True:
        position_vec = get_positions_vector(client, creature, goal_pos)
        input_vec = torch.cat((input_vec, position_vec))

        torques = creature.step(input_vec)
        apply_torques(client, creature, torques)
        
        duration = time() - start
        sleep(max(0, dt - duration))
        
        link_velo = p.getLinkState(creature.bullet_id, 0, computeLinkVelocity=True, physicsClientId=client)[6]
        link_velo = np.array(link_velo)
        delta = np.linalg.norm(link_velo)

    p.disconnect(physicsClientId=client)

def test_walking(creature, record=False, out_file=None):
    client = p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    
    p.setGravity(0, 0, -9.8, physicsClientId=client)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    load_creature(client, creature, plane_id)

    if record:
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, out_file)

    distance = 0.0
    prev_pos = np.array([0, 0, 0])

    dt = 1.0 / 240.

    goal_pos = np.array([1., 1., 0.])
    goal_vel = np.array([0., 0., 0.])

    while True:
        p.stepSimulation(physicsClientId=client)
        
        start = time()

        input_vec = get_input_vector(client, creature, plane_id) 
        torques = creature.step(input_vec)
        apply_torques(client, creature, torques)
        
        duration = time() - start
        sleep(max(0, dt - duration))
        
        link_velo = p.getLinkState(creature.bullet_id, 0, computeLinkVelocity=True, physicsClientId=client)[6]
        link_velo = np.array(link_velo)
        delta = np.linalg.norm(link_velo)

    p.disconnect(physicsClientId=client)
# ==================================================================

