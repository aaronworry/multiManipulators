import pybullet as p
import time
import os, sys
import numpy as np
import pdb

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)

from environment.collectEnv import Env



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
    if timestep > .04 or i % (int(1 / (24 * timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i * timestep):
            time.sleep(timestep * i - elapsed)

def cal_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def get_action(robots, ROBOT, thingset, THING):
    # ROBOT = [[1, position_y, robot.action, robot.reward] ... ]
    # THING = [[2, position[0], position[1]] ... ]
    # robots : all robot in env
    # thingset:  avaliable_things
    ##### assign task for every robot
    result = [None for i in range(len(robots))]
    for i in range(len(robots)):
        things = list(thingset)
        distance = np.inf
        for j in range(len(things)):
            if ROBOT[i][0] == THING[j][0]:
                r_pos = np.array(ROBOT[i][1])
                t_pos = np.array(THING[j][1])
                dis = cal_distance(r_pos, t_pos)
                if dis < distance:
                    distance = dis
                    result[i] = things[j]
    return result

def init(robots, things):
    R1 = set()
    R2 = set()
    T1 = set()
    T2 = set()
    for robot in robots:
        if robot.type == 'type1':
            R1.add(robot)
        else:
            R2.add(robot)
    for thing in things:
        if thing.type == 'cube':
            T1.add(thing)
        else:
            T2.add(thing)
    return R1, R2, T1, T2

def allocate(robots, things):
    # robots: a list, robot.thing == None
    # things: a list, things not allocated
    m = len(robots)
    n = len(things)
    result = set()
    if m == 0 or n == 0:
        return result
    Map = np.zeros((m, n))
    for j in range(n):
        for i in range(m):
            Map[i][j] = cal_distance(robots[i].chassis.get_position(), things[j].get_position())
    while np.min(Map) < 1000.:
        a, b = np.unravel_index(np.argmin(Map), Map.shape)
        robots[a].thing = things[b]
        result.add(things[b])
        for j in range(n):
            for i in range(m):
                if j == b or i == a:
                    Map[i][j] = np.inf
    return result  # a set, things has been assigned

def taskAs(T1, T2, R1, R2):
    task1 = list(T1)
    task2 = list(T2)
    robot1 = []
    robot2 = []
    for item in R1:
        if item.thing == None:
            robot1.append(item)
    for item in R2:
        if item.thing == None:
            robot2.append(item)
    set1 = allocate(robot1, task1)
    set2 = allocate(robot2, task2)
    t1 = T1 - set1
    t2 = T2 - set2
    return t1, t2

env = Env(robot_config=[{'type1': 4}, {'type2': 4}], thing_config=[{'cube': 10}, {'cylinder': 10}])
episode = 3
for k in range(episode):
    obs = env.reset(cube_num=10, cylinder_num=10)
    R1, R2, T1, T2 = init(env.robots, env.available_thing_ids_set)
    start = time.time()

    step = 0
    done = False
    R = 0
    while not done:
        # time.sleep(100)
        # action = get_action(env.robots, obs[0], env.available_thing_ids_set, obs[1])
        T1, T2 = taskAs(T1, T2, R1, R2)
        obs, reward, done, info = env.step()
        R += reward
        sync(step, start, env.TIMESTEP)
        step += 1
env.close()


