import os
import numpy as np
import pybullet as p
import pybullet_data
import torch

from .elements.UR5 import UR5_new
from .elements.box import Box

class Env():
    '''
        robot_config = {'type1': robot1_num, 'type2': robot2_num}  robot1_num + robot2_num = 1
        box_config = {'cube': cube_num, 'cylinder': cylinder_num}  cylinder_num = 0
        num_thing:   max number of each type things                    num_thing = cube_num
        '''
    def __init__(self, display=True, hz=240, robot_config=None, box_config=None, env_name='UR_Conveyor'):
        # one static manipulator
        # a pile of box with different color
        # using these boxes to erect shape as request
        pass