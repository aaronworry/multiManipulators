import pybullet as p
from grabEnv import Env
import time

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


def get_action(robots, ROBOT, thingset, THING):
    # ROBOT = [[1, position_y, robot.action, robot.reward] ... ]
    # THING = [[2, position[0], position[1]] ... ]
    # robots : all robot in env
    # thingset:  avaliable_things
    ##### assign task for every robot
    result = [None for i in range(len(robots))]
    for i in range(len(robots)):
        dis = 500.
        things = list(thingset)
        for j in range(len(things)):
            if ROBOT[i][0] == THING[j][0] and (THING[j][2] - ROBOT[i][1]) >= -0.1 and (THING[j][2] - ROBOT[i][1]) <= 0.4:
                if (THING[j][2] - ROBOT[i][1]) <= dis:
                    dis = (THING[j][2] - ROBOT[i][1])
                    result[i] = things[j]
    return result



env = Env(robot_config=[{'type1': 0}, {'type2': 1}], thing_config=[{'cube':0}, {'cylinder':2}])
episode = 10
for k in range(episode):
    obs = env.reset(speed = -0.2, cube_num = 0, cylinder_num = 2, margin=1.)    # margin: the distance between thing and next thing
    start = time.time()
    step = 0
    done = False
    reward = 0
    while step < 100 or (not done):
        action = get_action(env.robots, obs[0], env.available_thing_ids_set, obs[1])
        obs, reward, done, info = env.step(action)
        sync(step, start, env.TIMESTEP)
        step += 1
env.close()
    

