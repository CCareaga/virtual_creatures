from time import sleep 
import numpy as np
import pybullet as p
import pybullet_data

# bullet_client = p.connect(p.DIRECT)
bullet_client = p.connect(p.GUI)

p.setGravity(0,0,-9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")


# p.loadURDF("creature.urdf")
creature = p.loadURDF("test.urdf", flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
print(creature)

for j in range(p.getNumJoints(creature)):
  p.setJointMotorControl2(creature, j, controlMode=p.VELOCITY_CONTROL, force=0)

step = 1 / 60
t = 0

while True:
    t += step
    p.stepSimulation()
    sleep(step)
    
    f = np.sin(t)

    for j in range(p.getNumJoints(creature)):
        p.setJointMotorControl2(creature, j, controlMode=p.TORQUE_CONTROL, force=f)


p.disconnect()

