import os
import numpy as np
import math
import pybullet as p


def restrict_heading_range(h):
    return (h + math.pi) % (2 * math.pi) - math.pi

def orientation_to_heading(o):
    # Note: Only works for z-axis rotations
    return 2 * math.acos(math.copysign(1, o[2]) * o[3])



class BoxOnConveyor():
    def __init__(self, env, position, thing_type):
        self.env = env
        self.type = thing_type

        self.position = position
        self.speed = None     # speed of things, when thing on conveyor, it equal to the speed of conveyor.
        self.state = None
        self.finished = False   # describe whether the grab task is finished


        self.id = self._createBody()

    def _createBody(self):
        if self.type == "cube":
            cube_path = os.path.join(os.path.dirname(__file__),"../model/conveyor/blue_cube.urdf")
            return p.loadURDF(cube_path, self.position, useFixedBase = 0)
        elif self.type == 'cylinder':
            cylinder_path = os.path.join(os.path.dirname(__file__),"../model/conveyor/yellow_cylinder.urdf")
            return p.loadURDF(cylinder_path, self.position, useFixedBase = 0)


    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        self.position = position
        return self.position


    def reset(self, position_x, position_y):  #0.3
        # put the task thing on conveyor
        position = [position_x, position_y, 0.2]
        if self.type == "cube":
            position = [position_x, position_y, 0.2]
        elif self.type == "cylinder":
            position = [position_x, position_y, 0.2]
        Orientation = [0., 0., 0., 1.]
        self.finished = False
        p.resetBasePositionAndOrientation(self.id, position, Orientation, physicsClientId=self.env.client)


    def _move(self):
        if self.speed:
            p.resetBaseVelocity(self.id, linearVelocity=[0., self.speed, 0.], angularVelocity=[0., 0., 0.], physicsClientId=self.env.client)


