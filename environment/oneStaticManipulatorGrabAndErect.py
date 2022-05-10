import os
import numpy as np
import pybullet as p
import pybullet_data
import time
import torch

from .elements.UR5 import UR5_new
from .elements.manipulator import Manipulator
from .elements.baseThing import Thing

COLORS = {'red': [0.4, 0, 0], 'green': [0, 0.4, 0], 'blue': [0, 0, 0.4], 'black': [0, 0, 0], 'pink': [0.4, 0, 0.4],
          'yellow': [0.4, 0.4, 0], 'cyan': [0, 0.4, 0.4]}
Colors = [[0.4, 0, 0], [0, 0.4, 0], [0, 0, 0.4], [0, 0, 0], [0.4, 0, 0.4], [0.4, 0.4, 0], [0, 0.4, 0.4]]

class Env():
    '''
        robot_config = {'type1': robot1_num, 'type2': robot2_num}  robot1_num + robot2_num = 1
        box_config = {'cube': cube_num, 'cylinder': cylinder_num}  cylinder_num = 0
        num_thing:   max number of each type things                    num_thing = cube_num
        '''
    def __init__(self, display=True, hz=240, radius = 0.6, robot_config=None, thing_config=None, env_name='Static_Manipulator'):
        # static manipulator
        # a pile of box with different color
        # using these boxes to erect shape as request
        self.robot_config = robot_config
        self.thing_config = thing_config
        self.env_name = env_name
        self.radius = radius
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        if self.robot_config is None:
            self.robot_config = [{'type1': 0}, {'type2': 1}]

        if self.thing_config is None:
            self.thing_config = [{'cube': 2}, {'cylinder': 2}]

        self.robot_ids = []  # the model id
        self.robots = []  # the robot object
        self.robot_groups = [[] for _ in range(len(self.robot_config))]  # [[all object of robot1], [all object of robot2]]

        self.thing_ids = []
        self.things = []
        self.thing_groups = [[] for _ in range(len(self.thing_config))]

        self.available_thing_ids_set = None  # boxes took part in the task
        self.removed_thing_ids_set = None  # boxes not in task

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
        poseListCube = [[0.3, -0.4, 0.0], [0.3, 0.4, 0.0]]
        poseListCylinder = [[0.1, -0.4, 0.0], [0.1, 0.4, 0.0]]
        for idx, thing_group in enumerate(self.thing_config):
            thing_type, count = next(iter(thing_group.items()))
            for n in range(count):
                if thing_type == 'cube':
                    a, b, c = poseListCube[n]
                else:
                    a, b, c = poseListCylinder[n]
                thing = Thing(self, [a, b, c], thing_type)  # load boxes
                self.things.append(thing)
                self.thing_groups[idx].append(thing)

        num = self.robot_config[0]['type1'] + self.robot_config[1]['type2']
        poses = self._cal_circular_poses(self.radius, num)

        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                if robot_type == 'type1':
                    robot = UR5_new(self, poses[kk], 1, robot_type)  # set the pose of ur
                else:
                    # robot = UR5_new(self, poses[kk + self.robot_config[0]['type1']], 1, robot_type)
                    ur_base_pose = ((-0.6, 0.0, 0.0), p.getQuaternionFromEuler((0., 0., 0.)))
                    robot = UR5_new(self, ur_base_pose, 1, robot_type)
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)

        self.available_thing_ids_set = set()  # a set that include the task box
        self.removed_thing_ids_set = set(self.things)  # not in task


    def _cal_circular_poses(self, radius, num):
        return [[[radius * np.cos(i * np.pi * 2 / num), radius * np.sin(i * np.pi * 2 / num), 0], p.getQuaternionFromEuler([0, 0, i * np.pi * 2 / num])] for i in range(num)]

    def reset(self, cube_num=2, cylinder_num=2):
        # init
        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.things)  # not in task

        for i in range(cube_num):
            self.available_thing_ids_set.add(self.thing_groups[0][i])
            self.removed_thing_ids_set.remove(self.thing_groups[0][i])
        for j in range(cylinder_num):
            self.available_thing_ids_set.add(self.thing_groups[1][j])
            self.removed_thing_ids_set.remove(self.thing_groups[1][j])

        self.robot_reset()

        obs = self.stack_objects()


        return obs

    def robot_reset(self):
        for robot in self.robots:
            robot.place_position2 = np.array([0.4, 0.0, 0.6])
            robot.reset()


    def stack_objects(self):
        for thing in self.available_thing_ids_set:
            thing.reset([thing.init_position,[0., 0., 0., 1.]])

        obs = None
        return obs

    def disable_UR5(self):
        """机械臂停止"""
        for robot in self.robots:
            robot.disable()    #将机械臂移出工作空间

    def enable_UR5(self):
        """机械臂启动"""
        for robot in self.robots:
            robot.enable()  #将机械臂加入工作空间

    def get_relative_pose(self, poseA, poseB):
        # cal A refer B
        posA = np.array(poseA[0])
        oriA = np.quaternion(*poseA[1])

        posB = np.array(poseB[0])
        oriB = np.quaternion(*poseB[1])

        pos = np.linalg.norm(posA - posB)
        orn = (oriA * oriB.inverse()).angle()
        # Get smallest positive angle
        orn = orn % (np.pi * 2)
        if orn > np.pi:
            orn = 2 * np.pi - orn
        return pos, orn

    def _computeObs(self):
        return None

    def _getState(self):
        pass

    def _getReward(self):
        pass



    def close(self):
        p.disconnect(physicsClientId=self.client)

    def step(self):
        # all None or (at least 1 not None)
        reward, info= 0, {}

        # 机械臂运动
        for robot in self.robots:
            if robot.ready:
                robot.pick_and_place_FSM()


        p.stepSimulation(physicsClientId=self.client)
        done = False

        if len(self.available_thing_ids_set) == 0:
            done = True

        obs = self._computeObs()
        return obs, reward, done, info

"""
    
    ur5.set_pose()
    ur5.set_arm_joints()
    ur5.step()
    
    ur5.get_pose()[0] , ur5.get_pose()[1]
    
    ur5.check_collision() 
    ur5.global_to_ur5_frame()
    
    ur5.get_end_effector_pose()
    ur5.get_arm_joint_values()
    ur5.get_link_global_positions()
    
    ur5.closest_points_to_others
    
    if self.action_type == 'delta':
        ur5.control_arm_joints_delta(action)
    elif self.action_type == 'target-norm':
        ur5.control_arm_joints_norm(action)
"""


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

def cal_action_quene(observation):

    # 一个deque
    return 1

if __name__ == '__main__':


    env = Env(robot_config=[{'type1': 1}, {'type2': 0}])
    episode = 1
    for k in range(episode):
        observation = env.reset(cube_num=2)
        # 使用一个deque表示一系列任务，机械臂按步骤完成一个deque的子任务，直至这个deque为空
        actionList = cal_action_quene(observation=None)
        start = time.time()
        step = 0
        done = False
        while step < 100 or (not done):
            action = actionList[0]
            obs, reward, done, finished = env.step(action)
            if finished:
                actionList.pop()
                finished = False
            sync(step, start, env.TIMESTEP)
            step += 1
    env.close()

