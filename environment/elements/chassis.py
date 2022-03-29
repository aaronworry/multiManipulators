import pybullet as p
import numpy as np
from baseChassis import BaseChassis
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
pdir = os.path.dirname(parentdir)


from controller.chassisController import MecanumController, DifferentialController

def get_euler_from_quaternion(quaternion):
    return list(p.getEulerFromQuaternion(quaternion))

class Mecanum(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(MecanumController)

        self.lateral_velocity = 0.
        self.linear_velocity = 0.
        self.reached = False

    def _createBody(self):
        path = os.path.join(os.path.dirname(__file__), "../../assets/dingo/urdf/dingo-o.urdf")
        return p.loadURDF(path, self.position, useFixedBase=0)

    def step(self):
        self.update()
        self.reached = self.move_to(self.target_position, self.target_orientation)

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.lateral_velocity]
        return self.state

    def move_to(self, position, orientation):
        vx, vy, w, flag = self.controller.control_position(self.pose, position, orientation)
        p.resetBaseVelocity(self.id, linearVelocity=[vx, vy, 0.], angularVelocity=[0., 0., w])
        # ?
        current_orientation = get_euler_from_quaternion(self.pose[1])[2]
        self.linear_velocity = vx * np.cos(current_orientation) + vy * np.sin(current_orientation)
        self.lateral_velocity = vy * np.cos(current_orientation) - vx * np.sin(current_orientation)
        if flag:
            return True
        else:
            return False



class Differential(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(DifferentialController)

        self.linear_velocity = 0.
        self.angular_velocity = 0.

    def _createBody(self):
        pass

    def step(self):
        self.update()
        self.reached = self.move_to(self.target_position, self.target_orientation)

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.angular_velocity]
        return self.state

    def move_to(self, position, orientation):
        return False
