# 多个移动机械臂收集物品
import math
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
from UR5 import UR5
from thingOnConveyor import ThingOnConveyor
import gym
import time

PLACE_STEP = 0.0003
PLACE_DELTA_THRESHOLD = 0.005

class Env(gym.Env):
    '''
        robot_config = {'type1': robot1_num, 'type2': robot2_num}
        thing_config = {'cube': cube_num, 'cylinder': cylinder_num}
        num_thing:   max number of each tye things
    '''
    def __init__(self, display=True, hz=240, robot_config=None, thing_config=None, env_name='UR_Conveyor'):
        self.robot_config = robot_config
        self.thing_config = thing_config
        self.env_name = env_name
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        self.conveyor_speed = 0

        if self.robot_config is None:
            self.robot_config = [{'type1': 2}, {'type2': 2}]
        self.num_robots = sum(sum(g.values()) for g in self.robot_config)
        self.robot_group_types = [next(iter(g.keys())) for g in self.robot_config]

        if self.thing_config is None:
            self.thing_config = [{'cube': 2}, {'cylinder': 2}]
        self.num_things = sum(sum(g.values()) for g in self.thing_config)
        self.thing_group_types = [next(iter(g.keys())) for g in self.thing_config]

        self.robot_ids = []   # the model id
        self.robots = []      # the robot object
        self.robot_groups = []  # [[all object of robot1], [all object of robot2]]

        self.thing_ids = []
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

        planeID = p.loadURDF("plane.urdf")

        conveyor_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.6, 24, 0.2), physicsClientId=self.client)
        conveyor_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.6, 24, 0.2),
                                                       rgbaColor=[0.4, 0.4, 0.4, 1.0], physicsClientId=self.client)
        self.conveyor_id = p.createMultiBody(0, conveyor_collision_shape_id, conveyor_visual_shape_id, (0, 0, 0), physicsClientId=self.client)

        platform_collision_id_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), physicsClientId=self.client)
        platform_visual_id_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), rgbaColor=[1., 1., 1., 1.0], physicsClientId=self.client)
        self.platform1_id = p.createMultiBody(0, platform_collision_id_1, platform_visual_id_1, (-1., 0, 0), physicsClientId=self.client)

        platform_collision_id_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), physicsClientId=self.client)
        platform_visual_id_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), rgbaColor=[1., 1., 1., 1.0], physicsClientId=self.client)
        self.platform2_id = p.createMultiBody(0, platform_collision_id_2, platform_visual_id_2, (1., 0, 0), physicsClientId=self.client)

        self.robot_ids = []
        self.robots = []
        self.robot_groups = [[] for _ in range(len(self.robot_config))]
        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                if robot_type == "type1":
                    X = -1
                    x = -1.5
                elif robot_type == "type2":
                    X = 1
                    x = 1.5
                robot = UR5(self, [X, 0.6 - kk * 1.2, 0.2], robot_type)  # set the pose of ur
                # 添加盒子 box
                #box = BOX(self, [x, 0.6 - kk * 1.2, 0.])
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)
                self.robot_ids.append(robot.id)
                

        self.thing_ids = []
        self.things = []
        self.thing_groups = [[] for _ in range(len(self.thing_config))]
        for thing_group_index, t in enumerate(self.thing_config):
            thing_type, count = next(iter(t.items()))
            # add 5 cube and 5 cylinder in the env, people can't see it. To decrease the time that reset() will speed.
            for n in range(5):
                if thing_type == 'cylinder':
                    Y = 1
                elif thing_type == 'cube':
                    Y = -1
                thing = ThingOnConveyor(self, [4, Y, 0.3 * n], thing_type)  # load things
                self.things.append(thing)
                self.thing_groups[thing_group_index].append(thing)
                self.thing_ids.append(thing.id)

        self.available_thing_ids_set = set()    # a set that include the task thing
        self.removed_thing_ids_set = set(self.things)  # not in task

    def add_object(self, position, thing_type):
        thing = ThingOnConveyor(self, position, thing_type)
        self.things.append(thing)
        self.thing_ids.append(thing.id)
        self.thing_groups[thing_type].append(thing)
        return thing

    def seed(self, seed=None):
        self._random = np.random.RandomState(seed)
        return seed

    def reset(self, speed, cube_num=1, cylinder_num=1, margin=1.):
        # init
        self.conveyor_speed = speed
        for thing in self.things:
            thing.speed = self.conveyor_speed
        for i in range(cube_num):
            self.available_thing_ids_set.add(self.thing_groups[0][i])
            self.removed_thing_ids_set.remove(self.thing_groups[0][i])
        for j in range(cylinder_num):
            self.available_thing_ids_set.add(self.thing_groups[1][j])
            self.removed_thing_ids_set.remove(self.thing_groups[1][j])

        for robot in self.robots:
            robot.reset()

        self.put_things_on_conveyor(self.available_thing_ids_set, margin)

        

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
            position_y = robot.position[1]
            if robot.type == 'type1':
                ROBOT.append([1, position_y, robot.action, robot.reward])
            elif robot.type == 'type2':
                ROBOT.append([2, position_y, robot.action, robot.reward])
        THING = []
        for thing in self.available_thing_ids_set:
            position = thing.get_position()
            if thing.type == 'cube':
                THING.append([1, position[0], position[1]])
            elif thing.type == 'cylinder':
                THING.append([2, position[0], position[1]])
        return [ROBOT, THING, self.conveyor_speed]


    def put_things_on_conveyor(self, thing_sets, margin):
        # margin:  the distence between two things
        i = 0
        for thing in thing_sets:
            if thing.type == 'cube':
                thing.reset(-0.3, 1.5 + i * margin)
            elif thing.type == 'cylinder':
                thing.reset(0.2, 1.5 + i * margin)
            i += 1
            #thing._move_on_conveyor()

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
                # control robot grab things
                self.robots[i].step_discrete(action[i])
        for thing in self.available_thing_ids_set:
            thing._move()
        # simulation step
        p.stepSimulation(physicsClientId=self.client)

        reward, info = 0, {}
        #for robot in self.robots:
        #    _sum_reward, _info, _step_reward = robot.reward()
        done = False
        num_idle = 0
        for robot in self.robots:
            if robot.action == 'idle':
                num_idle += 1
        
        if len(self.available_thing_ids_set) <= 0 and num_idle == len(self.robots):
            done = True

        # Add ground truth robot state into info.
        info.update(self.info)

        obs = self._computeObs()

        return obs, reward, done, info


