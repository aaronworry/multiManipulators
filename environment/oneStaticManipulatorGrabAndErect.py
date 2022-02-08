import os
import numpy as np
import pybullet as p
import pybullet_data
import time
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
    def __init__(self, display=True, hz=240, radius = 0.6, robot_config=None, box_config=None, env_name='Static_Manipulator'):
        # static manipulator
        # a pile of box with different color
        # using these boxes to erect shape as request
        self.robot_config = robot_config
        self.box_config = box_config
        self.env_name = env_name
        self.radius = radius
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
        planeID = p.loadURDF("plane.urdf", [0, 0, -0.01])

        for idx, box_group in enumerate(self.box_config):
            box_type, count = next(iter(box_group.items()))
            for n in range(count):
                if box_type == 'cube':
                    box = Box(self, [0, 0, 0.3*n+0.3], color='pink')  # load boxes
                    self.boxes.append(box)
                    self.box_groups[idx].append(box)
                    self.box_ids.append(box.id)

        num = self.robot_config[0]['type1'] + self.robot_config[1]['type2']
        poses = self._cal_circular_poses(self.radius, num)

        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                if robot_type == 'type1':
                    robot = UR5_new(self, poses[kk], robot_type)  # set the pose of ur
                else:
                    robot = UR5_new(self, poses[kk + self.robot_config[0]['type1']], robot_type)
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)
                self.robot_ids.append(robot.id)

        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task


    def _cal_circular_poses(self, radius, num):
        return [[[radius * np.cos(i * np.pi * 2 / num), radius * np.sin(i * np.pi * 2 / num), 0], p.getQuaternionFromEuler([0, 0, i * np.pi * 2 / num])] for i in range(num)]

    def reset(self, cube_num=2):
        # init
        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task

        for i in range(cube_num):
            self.available_box_ids_set.add(self.box_groups[0][i])
            if self.box_groups[0][i] in self.removed_box_ids_set:
                self.removed_box_ids_set.remove(self.box_groups[0][i])

        self.robot_reset()

        obs = self.stack_objects()

        return obs

    def robot_reset(self):
        # 将多个机械臂围成一个圈，圆心为远点，半径为 0.6
        pass

    def stack_objects(self):
        # 重置物体,物体如何摆放

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


def sync(i, start_time, timestep):
    """Syncs the stepped simulation with the wall-clock.
    Function `sync` calls time.sleep() to pause a for-loop
    running faster than the expected timestep.
    Parameters
    ----------
    i : int
        Current simulation iteration.
    start_time : timestamp
        Timestamp of the simulation start.
    timestep : float
        Desired, wall-clock step of the simulation's rendering.
    """
    if timestep > .04 or i%(int(1/(24*timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i*timestep):
            time.sleep(timestep*i - elapsed)

if __name__ == '__main__':


    env = Env(robot_config=[{'type1': 1}, {'type2': 0}])
    episode = 1
    for k in range(episode):
        observation = env.reset(cube_num=2)
        start = time.time()
        step = 0
        done = False
        while step < 100 or (not done):
            action = 1
            obs, reward, done = env.step(action)
            sync(step, start, env.TIMESTEP)
            step += 1
    env.close()

