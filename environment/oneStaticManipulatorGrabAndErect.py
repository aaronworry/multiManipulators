import os
import numpy as np
import pybullet as p
import pybullet_data
import torch

from elements.UR5 import UR5_new
from elements.box import Box

COLORS = {'red': [0.4, 0, 0], 'green': [0, 0.4, 0], 'blue': [0, 0, 0.4], 'black': [0, 0, 0], 'pink': [0.4, 0, 0.4],
          'yellow': [0.4, 0.4, 0], 'cyan': [0, 0.4, 0.4]}
Colors = [[0.4, 0, 0], [0, 0.4, 0], [0, 0, 0.4], [0, 0, 0], [0.4, 0, 0.4], [0.4, 0.4, 0], [0, 0.4, 0.4]]

class Env():
    '''
        robot_config = {'type1': robot1_num, 'type2': robot2_num}  robot1_num + robot2_num = 1
        box_config = {'cube': cube_num, 'cylinder': cylinder_num}  cylinder_num = 0
        num_thing:   max number of each type things                    num_thing = cube_num
        '''
    def __init__(self, display=True, hz=240, robot_config=None, box_config=None, env_name='One_Static_Manipulator'):
        # static manipulator
        # a pile of box with different color
        # using these boxes to erect shape as request
        self.robot_config = robot_config
        self.box_config = box_config
        self.env_name = env_name
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        if self.robot_config is None:
            self.robot_config = [{'type1': 1}, {'type2': 0}]

        if self.box_config is None:
            self.box_config = [{'cube': 2}, {'cylinder': 0}]

        self.robot_ids = []  # the model id
        self.robots = []  # the robot object
        self.robot_groups = [[] for _ in range(len(self.robot_config))]  # [[all object of robot1], [all object of robot2]]

        self.box_ids = []
        self.boxes = []
        self.box_groups = [[] for _ in range(len(self.box_config))]

        self.available_box_ids_set = None  # boxes took part in the task
        self.removed_box_ids_set = None  # boxes not in task

        self.client = None

        self.display = display
        if self.display:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.resetSimulation(physicsClientId=self.client)
        # p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setTimeStep(1. / hz, physicsClientId=self.client)
        p.setGravity(0., 0., -10, physicsClientId=self.client)  # -9.8
        self._create_env()
        p.setRealTimeSimulation(0, physicsClientId=self.client)

    def _create_env(self):
        planeID = p.loadURDF("plane.urdf")

        for idx, box_group in enumerate(self.box_config):
            box_type, count = next(iter(box_group.items()))
            for n in range(count):
                if box_type == 'cube':
                    box = Box(self, [0.6, 0, 0.6*n+0.3], 'green')  # load boxes
                    self.boxes.append(box)
                    self.box_groups[idx].append(box)
                    self.box_ids.append(box.id)

        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                robot = UR5_new(self, [0, 0, 0], robot_type)  # set the pose of ur
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)
                self.robot_ids.append(robot.id)

        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task

    def reset(self, cube_num=2):
        # init
        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task

        for i in range(cube_num):
            self.available_box_ids_set.add(self.box_groups[0][i])
            if self.box_groups[0][i] in self.removed_box_ids_set:
                self.removed_box_ids_set.remove(self.box_groups[0][i])

        for robot in self.robots:
            robot.reset()

        # 重置物体

        obs = None
        return obs

    def _computeObs(self):
        pass

    def close(self):
        p.disconnect(physicsClientId=self.client)

    def step(self, action = None):
        # all None or (at least 1 not None)
        reward, info= 0, {}

        # 机械臂运动

        p.stepSimulation(physicsClientId=self.client)
        done = 1

        obs = self._computeObs()
        return obs, reward, done

class Task():
    def __init__(self, object, pick_pose, place_pose):
        self.object = object
        self.pick_pose = pick_pose
        self.place_pose = place_pose

    def set_object(self, object):
        self.object = object

    def set_pick_pose(self, pick_pose):
        self.pick_pose = pick_pose

    def set_place_pose(self, place_pose):
        self.place_pose = place_pose


class TaskQuene():
    # a small task describe a manipulator pick an object and place it, containing (object_id, pick_pose, place_pose)
    # TaskQuene contain a sequence of a task
    # manipulator can decompose a Task into many small tasks, then process it
    """
    a queue of task
    """
    def __init__(self):
        pass



if __name__ == '__main__':
    env = Env(robot_config=[{'type1': 1}, {'type2': 0}])

