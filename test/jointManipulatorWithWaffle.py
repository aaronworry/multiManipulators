import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
waffle = p.loadURDF("../model/turtlebot3/waffle.urdf", [0, 0, 0], useFixedBase = 0)
manipulator = p.loadURDF("../model/open_manipulator/manipulator.urdf", [-0.092, 0, 0.091], useFixedBase = 0)
joint = p.createConstraint(waffle, -1, manipulator, -1, p.JOINT_FIXED, None, [-0.092, 0., 0.091], [0., 0., 0.])
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

#moving robot
p.resetBaseVelocity(waffle, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.])
print(p.getNumConstraints())


for _ in range(300):
    p.stepSimulation()
    time.sleep(1. / 10.)
