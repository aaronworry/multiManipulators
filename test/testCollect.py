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
    return np.linalg.norm(pos1 - pos2)

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


env = Env(robot_config=[{'type1': 1}, {'type2': 1}], thing_config=[{'cube': 10}, {'cylinder': 10}])
episode = 3
for k in range(episode):
    obs = env.reset(cube_num=10, cylinder_num=10)

    start = time.time()

    step = 0
    done = False
    R = 0
    while not done:
        # time.sleep(100)
        action = get_action(env.robots, obs[0], env.available_thing_ids_set, obs[1])
        obs, reward, done, info = env.step(action)
        R += reward
        sync(step, start, env.TIMESTEP)
        step += 1
        # if step >= 5:
        #    time.sleep(100)
env.close()


