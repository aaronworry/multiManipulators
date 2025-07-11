import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
robot = p.loadURDF("../model/open_manipulator/manipulator.urdf", [0, 0, 0], useFixedBase = 1)
#s = p.getNumJoints(robot)
#print(s, p.getJointInfo(robot, 3))
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

Orientation = p.getQuaternionFromEuler([0., math.pi/2, 0.])
goal_pos = [0.15, 0., 0.4]
targetPositionsJoints = p.calculateInverseKinematics(robot, 3, goal_pos, targetOrientation = Orientation)
p.setJointMotorControlArray(robot, range(4), p.POSITION_CONTROL, targetPositions=targetPositionsJoints)

for _ in range(300):
    p.stepSimulation()
    time.sleep(1. / 30.)

