import os, sys
import numpy as np
import math
import pybullet as p
import time
import pybullet_data
from .end_effectors import Suction, Robotiq85, Robotiq2F85
from .UR5 import UR5_new
from .chassis import Mecanum, Differential
from transforms3d import euler
import collections
from math import pi

class Manipulator():
    def __init__(self, env, position, ee_type = 'Suction', chassis_type = 'Mecanum'):
        self.env = env

        self.position = position
        self.ur_base_pose = None

        self.ee_type = ee_type
        self.chassis_type = chassis_type

        self.ee = None
        self.chassis = None
        self.ur5 = None

        self.base_pose = 1      # base of arm

        self.reward = 0
        self._reward = 0

        self.state = None

        self.ee_tip = 10  #link id of end_effector

        self.action = None    # idle, goToGrab, grab, goToplace, place, idle
        self.last_action = None      # the action state that robot has just finished
        self.thing = None

        self.pick_pose = None   # the pose before it grab the task
        self.place_pose = None

        self.target_position = None
        self.move_flag = True

        self._createBody()

    def _createBody(self):
        if self.chassis_type:
            self.chassis = Mecanum(self.env, self.position)
            self.ur5 = UR5_new(self.env, self.ur_base_pose, 0, "type1")
            self.chassis_ur5_constraint = p.createConstraint(self.chassis.id, -1, self.ur5.id, -1, p.JOINT_FIXED, None,
                                                             self.chassis.position, [0., 0., 0.])
        else:
            self.ur5 = UR5_new(self.env, self.ur_base_pose, 1, "type1")

    def set_chassis(self, chassis_type):
        self.chassis_type = chassis_type
        if self.chassis:
            p.removeConstraint(self.chassis_ur5_constraint)
            self.chassis.remove()
        self.chassis = Mecanum(self.env, self.position)
        self.chassis_ur5_constraint = p.createConstraint(self.chassis.id, -1, self.ur5.id, -1, p.JOINT_FIXED, None,
                                                         self.chassis.position, [0., 0., 0.])

    def set_ee(self, ee_type):
        self.ee_type = ee_type
        if self.ee:
            self.ee.remove()
        self.ee = Robotiq2F85(self.env, self.ur5, self.ur5.color)

        # ee is in class ur5


    def reset(self):
        if self.chassis:
            self.chassis.reset()

        self.ur5.reset()
        self.ee.release()

        self.reward = 0  # Cumulative returned rewards.
        self.action = 'idle'   # move, idle, grab
        self.working = False

    def move_collect(self, thing):
        if self.action == 'idle' and thing:
            self.chassis.set_target(thing.get_position(), self.chassis.orientation)
            self.ur5.step()
            self.action = 'move'
            self.last_action = 'idle'
        elif self.action == 'move':
            self.chassis.step()
            self.position = self.chassis.position
            self.ur5.step()
            if self.chassis.reached:
                self.action = 'grab'
                self.last_action = 'move'
        elif self.action == 'grab':
            self.ur5.place_position = np.array([self.chassis.position[0] - 0.2 * np.cos(self.chassis.orientation), self.chassis.position[0] - 0.2 * np.sin(self.chassis.orientation), 0.6])
            self.ur5.pick_and_place_FSM(thing)
            if self.ur5.grab_finished:
                self.env.available_thing_ids_set.add(thing)
                self.env.removed_thing_ids_set.add(thing)
                self.action = 'idle'
                self.last_action = 'move'


    def state(self):
        return self.state

    def reward(self):
        info = {}
        if self._reward > 0:
            self.reward += self._reward
            self._reward = 0
            info['success'] = 1
            return self.reward, info, 1
        else:
            info['success'] = 0
            return self.reward, info, 0

    def get_ee_pose(self):
        return p.getLinkState(self.ee.id_body, 0, physicsClientId=self.env.client)[0:2]
