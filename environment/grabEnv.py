import os
import numpy as np
import pybullet as p
import pybullet_data
import torch

from .elements.UR5 import UR5
from .elements.boxOnConveyor import BoxOnConveyor
PLACE_STEP = 0.0003
PLACE_DELTA_THRESHOLD = 0.005


class Env:
    '''
    robot_config = {'type1': robot1_num, 'type2': robot2_num}
    box_config = {'cube': cube_num, 'cylinder': cylinder_num}
    num_box:   max number of each tye boxes
    '''
    def __init__(self, display=True, hz=240, robot_config=None, box_config=None, env_name='UR_Conveyor'):
        self.robot_config = robot_config
        self.box_config = box_config
        self.env_name = env_name
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        self.conveyor_speed = -0.2
        self.conveyor_ep = -3.2
        self.box_height = [0.05, 0.1]
        self.last_box = None
        self.margin = 0.

        if self.robot_config is None:
            self.robot_config = [{'type1': 2}, {'type2': 2}]

        if self.box_config is None:
            self.box_config = [{'cube': 2}, {'cylinder': 2}]

        self.robot_ids = []  # the model id
        self.robots = []  # the robot object
        self.robot_groups = [[] for _ in range(len(self.robot_config))]  # [[all object of robot1], [all object of robot2]]

        self.box_ids = []
        self.boxes = []
        self.box_groups = [[] for _ in range(len(self.box_config))]

        self.available_box_ids_set = None  # boxes took part in the task
        self.removed_box_ids_set = None  # boxes not in task

        self.put_leaked_box_on_conveyor = []
        self.put_grasped_box_on_conveyor = []

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

        len_conveyor = 32
        conveyor_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.6, len_conveyor, 0.2),
                                                             physicsClientId=self.client)
        conveyor_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.6, len_conveyor, 0.2),
                                                       rgbaColor=[0.4, 0.4, 0.4, 1.0], physicsClientId=self.client)
        self.conveyor_id = p.createMultiBody(0, conveyor_collision_shape_id, conveyor_visual_shape_id, (0, 0, 0),
                                             physicsClientId=self.client)

        platform_collision_id_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, len_conveyor, 0.2),
                                                         physicsClientId=self.client)
        platform_visual_id_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, len_conveyor, 0.2), rgbaColor=[1., 1., 1., 1.0],
                                                   physicsClientId=self.client)
        self.platform1_id = p.createMultiBody(0, platform_collision_id_1, platform_visual_id_1, (-1., 0, 0),
                                              physicsClientId=self.client)

        platform_collision_id_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, len_conveyor, 0.2),
                                                         physicsClientId=self.client)
        platform_visual_id_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, len_conveyor, 0.2), rgbaColor=[1., 1., 1., 1.0],
                                                   physicsClientId=self.client)
        self.platform2_id = p.createMultiBody(0, platform_collision_id_2, platform_visual_id_2, (1., 0, 0),
                                              physicsClientId=self.client)


        
        for idx, box_group in enumerate(self.box_config):
            box_type, count = next(iter(box_group.items()))
            for n in range(count):
                if box_type == 'cylinder':
                    y_pos = 1
                elif box_type == 'cube':
                    y_pos = -1
                box = BoxOnConveyor(self, n, [4, y_pos, self.box_height[idx] * n], box_type)  # load boxes
                self.boxes.append(box)
                self.box_groups[idx].append(box)
                self.box_ids.append(box.id)

        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                if robot_type == "type1":
                    X = -1
                    x = -1.5
                elif robot_type == "type2":
                    X = 1
                    x = 1.5
                basket_path = os.path.join(os.path.dirname(__file__), "../model/conveyor/basket.urdf")
                basket = p.loadURDF(basket_path, [X + x / 2, 0.6 - kk * 1.2 - x / 5, 0.], useFixedBase=1)

        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task

    def add_object(self, position, box_type):
        box = BoxOnConveyor(self, position, box_type)
        self.boxes.append(box)
        self.box_ids.append(box.id)
        self.box_groups[box_type].append(box)
        return box

    def seed(self, seed=None):
        self._random = np.random.RandomState(seed)
        return seed

    def reset(self, speed, cube_num=1, cylinder_num=1, margin=0.5):
        # init
        self.available_box_ids_set = set()  # a set that include the task box
        self.removed_box_ids_set = set(self.boxes)  # not in task

        self.conveyor_speed = speed
        self.margin = margin
        for box in self.boxes:
            box.speed = self.conveyor_speed
        for i in range(cube_num):
            self.available_box_ids_set.add(self.box_groups[0][i])
            if self.box_groups[0][i] in self.removed_box_ids_set:
                self.removed_box_ids_set.remove(self.box_groups[0][i])
        for j in range(cylinder_num):
            self.available_box_ids_set.add(self.box_groups[1][j])
            if self.box_groups[1][j] in self.removed_box_ids_set:
                self.removed_box_ids_set.remove(self.box_groups[1][j])

        self.clearRobotAndEffector()

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
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)
                self.robot_ids.append(robot.id)
        
        
        for robot in self.robots:
            print("start reset robot..")
            robot.reset()
            print("finish reset robot..")

        print("start put_boxes_on_conveyor")
        self.put_boxes_on_conveyor(self.available_box_ids_set, self.margin)
        print("finish put_boxes_on_conveyor")
        obs = self._computeObs()
        return obs

    def clearRobotAndEffector(self):
        if len(self.robots) > 0:
            for robot in self.robots:
                robot.remove()
                del robot
            self.robot_ids = []  # the model id
            self.robots = []  # the robot object
            self.robot_groups = [[] for _ in range(len(self.robot_config))] 
            

    def _computeObs(self):
        # computer the obs, algorithm can use it to assign the task
        ROBOT = []
        for robot in self.robots:
            position_y = robot.position[1]
            if robot.type == 'type1':
                ROBOT.append([1, position_y, robot.action, robot.reward])
            elif robot.type == 'type2':
                ROBOT.append([2, position_y, robot.action, robot.reward])
        BOX = []
        for box in self.available_box_ids_set:
            position = box.get_position()
            box_id = box.box_id
            if box.type == 'cube':
                BOX.append([1, box_id, position[0], position[1]])
            elif box.type == 'cylinder':
                BOX.append([2, box_id, position[0], position[1]])
        return [ROBOT, BOX, self.conveyor_speed]

    def extract_inputs(self, obs, horizon_box):
        """
        extract needed inputs from observation
        :param obs: the state of boxes
        :return: the coordinates of boxes in the defined horizon
                 the state of robots and conveyor belt
        """
        box_state, box_ids, rbt_cb_state = [], [], []
        sorted_boxes = sorted(obs[1], key=lambda kv: kv[3], reverse=False)  # TODO
        sorted_boxes = [box for box in sorted_boxes if box[3] > self.conveyor_ep]  # -3.2 is the y axis of 4th robot
        for box in sorted_boxes[:horizon_box]:
            box_ids.append(box[1])
            box_state.append(box[2:])  # box_type, box_id, x, y
        for j in range(len(obs[0])):
            rb_info, rb_j = [], obs[0][j]
            rb_action = 1 if rb_j[2] == "idle" else 0
            # rb_info.append(rb_j[0], rb_j[1], rb_action)  # robot type, y axis, idle or not
            rb_info.append(rb_action)  # idle or not
            rbt_cb_state.extend(rb_info)
        rbt_cb_state.extend([obs[2]])
        # [20, 2] -> [[x0, y0], [x2, y2], ..., [x19,y19]], [1, 5] -> [s0, s1, ..., s3, belt_speed]
        # box_state = torch.unsqueeze(torch.from_numpy(np.transpose(np.array(box_state, dtype=np.float32))), 0)
        # rbt_cb_state = torch.unsqueeze(torch.from_numpy(np.transpose(np.array(rbt_cb_state, dtype=np.float32))), 0)
        return box_ids, box_state, rbt_cb_state

    def put_boxes_on_conveyor(self, box_sets, margin):
        # margin: the distance between box
        for i, box in enumerate(box_sets):
            if box.type == 'cube':
                box.reset(-0.4, 1.5 + i * margin)  # x, y positions
            elif box.type == 'cylinder':
                box.reset(0.2, 1.5 + i * margin)
            if self.last_box is None or box.get_position()[1] >= self.last_box.get_position()[1]:
                self.last_box = box
            # box._move_on_conveyor()

    def put_processed_boxes_on_conveyor(self):
        boxes_back = self.put_leaked_box_on_conveyor + self.put_grasped_box_on_conveyor
        if len(boxes_back) >= 1:
            for box in boxes_back:
                if box.type == 'cube':
                    box.reset(-0.4, self.last_box.get_position()[1] + self.margin)  # x, y positions
                elif box.type == 'cylinder':
                    box.reset(0.2, self.last_box.get_position()[1] + self.margin)
                self.last_box = box
            self.put_leaked_box_on_conveyor = []
            self.put_grasped_box_on_conveyor = []

    @property
    def info(self):
        info = {}
        return info

    def close(self):
        p.disconnect(physicsClientId=self.client)

    def step(self, action=None):
        # all None or (at least 1 not None)
        reward, info, leaked_boxes = 0, {}, []
        if action.count(None) != len(self.robots):
            for i, robot in enumerate(self.robots):
                # FSM, control robot grab boxes
                self.robots[i].step_discrete(action[i])
                _reward, _info, _success = robot.get_reward()
                reward += _reward
        else:
            reward = -40

        for box in self.available_box_ids_set:
            box.move()   # boxes move forward
            box.step()   # put the box back to conveyor belt if it reaches the end
        leaked_boxes = self.put_leaked_box_on_conveyor
        self.put_processed_boxes_on_conveyor()
        # simulation step
        p.stepSimulation(physicsClientId=self.client)

        done = False
        num_idle = 0
        for robot in self.robots:
            if robot.action == 'idle':
                num_idle += 1

        # if len(self.available_box_ids_set) <= 0 and num_idle == len(self.robots):
        if num_idle == len(self.robots):
            done = True

        # Add ground truth robot state into info.
        # info.update(self.info)
        obs = self._computeObs()
        return obs, reward, done, leaked_boxes
