import os
import numpy as np
import math
import pybullet as p



class Thing():
    def __init__(self, env, position, thing_type, color):
        self.env = env
        self.type = thing_type
        self.position = position
        self.id = self._createBody()
        self.color = color

    def _createBody(self):
        if self.type == "cube":
            cube_path = os.path.join(os.path.dirname(__file__),"../../assets/conveyor/blue_cube.urdf")
            return p.loadURDF(cube_path, self.position, useFixedBase = 0)
        elif self.type == 'cylinder':
            cylinder_path = os.path.join(os.path.dirname(__file__),"../../assets/conveyor/yellow_cylinder.urdf")
            return p.loadURDF(cylinder_path, self.position, useFixedBase = 0)


    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        self.position = position
        return self.position

    def set_color(self):
        pass

    def reset(self, pose):  #0.3
        position = pose[0]
        orientation = pose[1]
        p.resetBasePositionAndOrientation(self.id, position, orientation, physicsClientId=self.env.client)


