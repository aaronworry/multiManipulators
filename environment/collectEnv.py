
import math
import numpy as np
import pybullet as p
import pybullet_data
import time
import os, sys


from .elements.manipulator import Manipulator
from .elements.baseThing import Thing

PLACE_STEP = 0.0003
PLACE_DELTA_THRESHOLD = 0.005

class Env():
    '''
        robot_config = {'type1': robot1_num, 'type2': robot2_num}
        thing_config = {'cube': cube_num, 'cylinder': cylinder_num}
        num_thing:   max number of each tye things
    '''
    def __init__(self, display=True, hz=240, robot_config=None, thing_config=None, env_name='collect'):
        self.planeID = None
        self.robot_config = robot_config
        self.thing_config = thing_config
        self.env_name = env_name
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        np.random.seed(5)

        self.conveyor_speed = 0

        if self.robot_config is None:
            self.robot_config = [{'type1': 1}, {'type2': 1}]
        self.num_robots = sum(sum(g.values()) for g in self.robot_config)
        self.robot_group_types = [next(iter(g.keys())) for g in self.robot_config]

        if self.thing_config is None:
            self.thing_config = [{'cube': 2}, {'cylinder': 2}]
        self.num_things = sum(sum(g.values()) for g in self.thing_config)
        self.thing_group_types = [next(iter(g.keys())) for g in self.thing_config]

        self.robots = []      # the robot object
        self.robot_groups = []  # [[all object of robot1], [all object of robot2]]

        self.things = []
        self.thing_groups = []

        self.available_thing_ids_set = None     # things took part in the task
        self.removed_thing_ids_set = None       # things not in task

        self.client = None

        self.display = display
        if self.display:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.resetSimulation(physicsClientId=self.client)
        #p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setTimeStep(1. / hz, physicsClientId=self.client)
        p.setGravity(0., 0., -9.8, physicsClientId=self.client)

        #if self.display:
        #    target = self.p.getDebugVisualizerCamera()[11]
        #    self.p.resetDebugVisualizerCamera(
        #        cameraDistance=1.1,
        #        cameraYaw=90,
        #        cameraPitch=-25,
        #        cameraTargetPosition=target)

        self._create_env()
        p.setRealTimeSimulation(0, physicsClientId=self.client)

        



    # @property
    # def is_static(self, robot):
    #    v = [np.linalg.norm(p.getJointState(robot, i)[1]) for i in range(p.getNumJoints(robot))]
    #
    #    v = p.getJointStates(robot, range(p.getNumJoints(robot)))
    #
    #    return all(np.array(v) < 5e-3)

    def _create_env(self):

        self.planeID = p.loadURDF("plane.urdf")

        self.robots = []
        self.robot_groups = [[] for _ in range(len(self.robot_config))]
        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            i = 0
            for kk in range(count):
                if robot_type == "type1":
                    Y = -0.5 - kk
                    X = 0
                    # if i > 0:
                    #     Y = -0.3
                    #     X = 1.8
                elif robot_type == "type2":
                    Y = 0.5 + kk
                    X = 0
                    # if i > 0:
                    #     Y = 0.7
                    #     X = 2.7
                robot = Manipulator(self, [X, Y, 0.], 0., robot_type)  # set the pose of ur
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)
                i += 1
                

        self.things = []
        self.thing_groups = [[] for _ in range(len(self.thing_config))]
        for thing_group_index, t in enumerate(self.thing_config):
            thing_type, count = next(iter(t.items()))
            # add 5 cube and 5 cylinder in the env, people can't see it. To decrease the time that reset() will speed.
            i = 0
            for n in range(2):
                if thing_type == 'cylinder':
                    Y = 1
                elif thing_type == 'cube':
                    Y = -1
                thing = Thing(self, [1+(n%5), Y + (n//5) * Y, 0.], thing_type)  # load things
                # a = (np.random.rand() - 0.5) * 4 + 3
                # b = (np.random.rand() - 0.5) * 2.6
                # thing = Thing(self, [a, b, 0.], thing_type)
                self.things.append(thing)
                self.thing_groups[thing_group_index].append(thing)

        self.available_thing_ids_set = set()    # a set that include the task thing
        self.removed_thing_ids_set = set(self.things)  # not in task

    def add_object(self, position, thing_type):
        thing = Thing(self, position, thing_type)
        self.things.append(thing)
        self.thing_groups[thing_type].append(thing)
        return thing

    def seed(self, seed=None):
        self._random = np.random.RandomState(seed)
        return seed

    def reset(self, cube_num=2, cylinder_num=2):
        # init
        self.available_thing_ids_set = set()
        self.removed_box_ids_set = set(self.things)

        for i in range(cube_num):
            self.available_thing_ids_set.add(self.thing_groups[0][i])
            self.removed_thing_ids_set.remove(self.thing_groups[0][i])
        for j in range(cylinder_num):
            self.available_thing_ids_set.add(self.thing_groups[1][j])
            self.removed_thing_ids_set.remove(self.thing_groups[1][j])

        for robot in self.robots:
            robot.reset()

        time_step = 0
        while time_step < 480:
            p.stepSimulation(physicsClientId=self.client)
            time_step += 1

        self.set_all_thing()

        

        #self.p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        #p.stepSimulation()
        #obs, _, _, _ = self.step()
        
        #p.setRealTimeSimulation(0) 
        obs = self._computeObs()
        return obs
    
    def _computeObs(self):
        # computer the obs, algorithm can use it to assign the task
        ROBOT = []
        for robot in self.robots:
            position = robot.position
            if robot.ur5.type == 'type1':
                ROBOT.append([1, position, robot.action, robot.reward])
            elif robot.ur5.type == 'type2':
                ROBOT.append([2, position, robot.action, robot.reward])
        THING = []
        for thing in self.available_thing_ids_set:
            position = thing.get_position()
            if thing.type == 'cube':
                THING.append([1, position])
            elif thing.type == 'cylinder':
                THING.append([2, position])
        return [ROBOT, THING]


    def set_all_thing(self):
        i = 0
        j = 0
        for thing in self.available_thing_ids_set:
            thing.reset([thing.init_position,[0., 0., 0., 1.]])
            """
            if thing.type == 'cube':
                thing.reset([[1.5+i, -1, 0.5],[0., 0., 0., 1.]])
                i += 1
            elif thing.type == 'cylinder':
                thing.reset([[1.5+j, 1, 0.5],[0., 0., 0., 1.]])
                j += 1
            """

    @property
    def info(self):
        info = {}
        ################################################
        return info

    def close(self):
        p.disconnect(physicsClientId=self.client)


    def step(self, action=None):
        if action is not None:
            for i in range(len(self.robots)):
                self.robots[i].move_collect(action[i])
        p.stepSimulation(physicsClientId=self.client)

        reward, info = 0, {}
        #for robot in self.robots:
        #    _sum_reward, _info, _step_reward = robot.reward()
        done = False
        
        if len(self.available_thing_ids_set) == 0:
            done = True

        # Add ground truth robot state into info.
        info.update(self.info)

        obs = self._computeObs()

        return obs, reward, done, info


