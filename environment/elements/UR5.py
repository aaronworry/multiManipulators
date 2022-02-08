import os, sys
import numpy as np
import math
import pybullet as p
import time
import pybullet_data
from .end_effectors import Suction, Robotiq85, Robotiq2F85
from transforms3d import euler
import collections
from math import pi
from .pybullet_utils import (
    get_self_link_pairs,
    violates_limits,
    get_difference_fn,
    get_distance_fn,
    get_sample_fn,
    get_extend_fn,
    control_joints,
    set_joint_positions,
    get_joint_positions,
    get_link_pose,
    inverse_kinematics,
    forward_kinematics
)

def distance(position1, position2):
    return np.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2 + (position1[2] - position2[2]) ** 2)

class UR5():
    def __init__(self, env, position, rtype):
        self.env = env
        self.type = rtype

        self.position = position

        self.working = False # whether it is working

        # speed of the UR's joint when processing task
        self.ur_speed = 0.05
        self.reward = 0
        self._reward = 0

        self.state = None
        self.ik_flag = False

        self.ee_tip = 10  #link id of end_effector

        self.action = None    # idle, goToGrab, grab, goToplace, place, idle   
        self.last_action = None      # the action state that robot has just finished
        self.thing = None

        self.pick_pose = None   # the pose before it grab the task
        self.place_pose = None

        self.target_joint = None    # save the target_joint when moveTo a pose
        self.flag = 0   # record how many step when a _move() finished

        self.id = self._createBody()

        # joints can be controlled
        self.n_joints = p.getNumJoints(self.id, physicsClientId=self.env.client)
        self.joint_info = [p.getJointInfo(self.id, i, physicsClientId=self.env.client) for i in range(self.n_joints)]
        self.controled_joints = [j[0] for j in self.joint_info if j[2] == p.JOINT_REVOLUTE]

        self.ee_orientation = None
        # set the ee according to the task, there is only one ee realized
        if rtype == 'type1':
            self.ee = Suction(self.env, self.id, 9, self.env.things)
        elif rtype == 'type2':
            self.ee = Robotiq85(self.env, self.id, 9, self.env.things)
        self.ee_orientation = self.ee.orientation

        if self.type == 'type1':
            self.place_pose = ((self.position[0] - 0.3, self.position[1], self.position[2] + 0.5), self.ee_orientation) # pose when release the ee
        elif self.type == 'type2':
            self.place_pose = ((self.position[0] + 0.3, self.position[1], self.position[2] + 0.5), self.ee_orientation) # pose when release the ee


        # init pose of ee
        self.end_init_pose = ((self.position[0]/2., self.position[1], 0.55), self.ee_orientation)
        # ur move to init state
        while distance(self.end_init_pose[0], self.ee.get_position()) >= 0.2:
            timeout = self._move_to(self.end_init_pose[0], self.end_init_pose[1])




    def _createBody(self):
        ur5_path = os.path.join(os.path.dirname(__file__),"../../assets/ur5_defi/ur5.urdf")
        return p.loadURDF(ur5_path, self.position, useFixedBase = 1)

    def reset(self):
        while distance(self.end_init_pose[0], self.ee.get_position()) >= 0.2:
            timeout = self._move_to(self.end_init_pose[0], self.end_init_pose[1])

        self.ee.release()
        self.reward = 0  # Cumulative returned rewards.
        self.action = 'idle'
        self.working = False

    def step_discrete(self, thing):
        #### FSM
        if thing == None and self.action == 'idle' and self.last_action == 'place':
        #    # move to init pose
            if type == 'type1':
                self._move(self.end_init_pose[0], self.end_init_pose[1], self.flag, self.ur_speed)
            elif type == 'type2':
                self.ee.still()

        if self.action == 'idle' and not self.pick_pose and not self.working and thing:
            self.thing = thing
            self.pick_pose = ((thing.get_position()[0], self.position[1], thing.get_position()[2]), self.ee_orientation)
            self.working = True
            self.action = 'goToGrab'
            self.last_action = 'idle'
        elif self.action == 'goToGrab':

            #move to the pose that ur can grab something
            self._move(self.pick_pose[0], self.pick_pose[1], self.flag, self.ur_speed)
            if self.ik_flag == False:
                self.action = 'grab'
                self.last_action = 'goToGrab'
        elif self.action == 'grab':
            if self.type == 'type1':
                if self.ee.detect_collision_with_thing(self.thing):
                    p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.], physicsClientId=self.env.client)
                    self.ee.grab_thing(self.thing)
                if self.ee.check_grasp():
                    self.action = 'backToPlace'
                    self.last_action = 'grab'
            elif self.type == 'type2':
                self.ee.still()
                if (self.thing.get_position()[1] - self.ee.get_position()[1]) <= 0.2:
                    p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.],
                                        physicsClientId=self.env.client)
                    self.ee.grab_thing(self.thing)
                    self.action = 'closeGripper'
                    self.last_action = 'grab'
        elif self.action == 'closeGripper':
            self.ee.close_gripper()
            if self.ee.open_close_flag == False:
                self.action = 'backToPlace'
                self.last_action = 'closeGripper'
        elif self.action == 'backToPlace':
            # print(self.action)
            # go to the place pose
            self._move(self.place_pose[0], self.place_pose[1], self.flag, self.ur_speed)
            if self.ik_flag == False:
                self.action = 'place'
                self.last_action = 'backToPlace'
                if self.type == 'type2':
                    self.ee.still()
                    self.action = 'openGripper'
                    self.last_action = 'backToPlace'
        elif self.action == 'openGripper':
            self.ee.open_gripper()
            if self.ee.open_close_flag == False:
                self.action = 'place'
                self.last_action = 'openGripper'

        elif self.action == 'place':
            # release the ee
            if self.type == 'type1':
                self._reward = self.ee.throw_thing()
            elif self.type == 'type2':
                self.ee.still()
                self._reward = self.ee.throw_thing()
            p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.],
                                physicsClientId=self.env.client)
            self.action = 'idle'
            self.last_action = 'place'
            self.pick_pose = None
            self.working = False
            self.thing = None

    def state(self):
        return self.state

    def reward(self):
        info = {}
        if self._reward > 0:
            self.reward += self._reward
            self._reward = 0
            info['success'] = 1
            return self.reward, info, 1
        else:
            info['success'] = 0
            return self.reward, info, 0

    def get_ee_pose(self):
        return p.getLinkState(self.ee.id_body, 0, physicsClientId=self.env.client)[0:2]

    def solve_IK(self, goal_pos, Orientation):
        targetPositionsJoints = p.calculateInverseKinematics(self.id, 7, goal_pos, targetOrientation=Orientation,
                                                             lowerLimits=[-np.pi, -2.3562, -np.pi, -17, -17, -17],
                                                             upperLimits=[np.pi, -0.523, np.pi, 17, 17, 17],
                                                             jointRanges=[2 * np.pi, 1.8332, 2 * np.pi, 34, 34, 34],  # * 6,
                                                             restPoses=np.float32(np.array(
                                                                 [-1, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi).tolist(),
                                                             maxNumIterations=100,
                                                             residualThreshold=1e-5, physicsClientId=self.env.client)
        joints = np.float32(targetPositionsJoints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
        return joints

    def move_joint(self, target_joint, speed=0.01, timeout=5):
        # block, only used in reset
        t0 = time.time()
        while (time.time() - t0) < timeout:
            current_joint = [p.getJointState(self.id, i, physicsClientId=self.env.client)[0] for i in self.controled_joints]
            current_joint = np.array(current_joint)
            different = target_joint - current_joint
            if all(np.abs(different) < 0.03):
                return False

            # Move with constant velocity
            norm = np.linalg.norm(different)
            v = different / norm if norm > 0 else 0
            step_joint = current_joint + v * speed
            gains = np.ones(len(self.controled_joints))
            p.setJointMotorControlArray(
                bodyIndex=self.id,
                jointIndices=self.controled_joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=step_joint,
                positionGains=gains, physicsClientId=self.env.client)
            p.stepSimulation(physicsClientId=self.env.client)
        print(f'Warning: movej exceeded {timeout} second timeout. Skipping.')
        return True

    def move_joint_discrete(self, target_joint, flag, speed=0.01):
        # flag:   record how many step it has passed
        # compute one step control
        current_joint = [p.getJointState(self.id, i, physicsClientId=self.env.client)[0] for i in self.controled_joints]
        current_joint = np.array(current_joint)
        different = target_joint - current_joint
        if all(np.abs(different) < 0.2):
            self.target_joint = None
            self.ik_flag = False
            flag = 0

        # Move with constant velocity
        norm = np.linalg.norm(different)
        v = different / norm if norm > 0 else 0
        step_joint = current_joint + v * speed
        gains = np.ones(len(self.controled_joints))
        p.setJointMotorControlArray(
            bodyIndex=self.id,
            jointIndices=self.controled_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=step_joint,
            positionGains=gains, physicsClientId=self.env.client)
        flag += 1



    def _move_to(self, position, Orientation, speed=0.01):
        target_joint = self.solve_IK(position, Orientation)
        return self.move_joint(target_joint, speed)

    def _move(self, position, Orientation, flag, speed=0.01):
        # _move next step
        if self.ik_flag==False:
            self.target_joint = self.solve_IK(position, Orientation)
            self.ik_flag = True
        self.move_joint_discrete(self.target_joint, flag, speed)


class UR5_new():
    joint_epsilon = 0.01
    joints_count = 6
    next_available_color = 0
    workspace_radius = 0.85
    colors = [
        [0.4, 0, 0],
        [0, 0, 0.4],
        [0, 0.4, 0.4],
        [0.4, 0, 0.4],
        [0.4, 0.4, 0],
        [0, 0, 0],
    ]
    LINK_COUNT = 10

    GROUPS = {
        'arm': ["shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"],
        'gripper': None
    }

    GROUP_INDEX = {
        'arm': [1, 2, 3, 4, 5, 6],
        'gripper': None
    }

    INDEX_NAME_MAP = {
        0: 'world_joint',
        1: 'shoulder_pan_joint',
        2: 'shoulder_lift_joint',
        3: 'elbow_joint',
        4: 'wrist_1_joint',
        5: 'wrist_2_joint',
        6: 'wrist_3_joint',
        7: 'ee_fixed_joint',
        8: 'wrist_3_link-tool0_fixed_joint',
        9: 'base_link-base_fixed_joint'
    }

    LOWER_LIMITS = [-2 * pi, -2 * pi, -pi, -2 * pi, -2 * pi, -2 * pi]
    UPPER_LIMITS = [2 * pi, 2 * pi, pi, 2 * pi, 2 * pi, 2 * pi]
    MAX_VELOCITY = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2]
    MAX_FORCE = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

    HOME = [0, 0, 0, 0, 0, 0]
    UP = [0, -1.5707, 0, -1.5707, 0, 0]
    RESET = [0, -1, 1, 0.5, 1, 0]
    EEF_LINK_INDEX = 7

    def __init__(self, env, pose, rtype, velocity=1.0, enabled=True, acceleration=2.0, training=True):

        self.velocity = velocity
        self.acceleration = acceleration
        self.pose = pose
        self.env = env
        self.type = rtype
        self.color = self.colors[self.next_available_color]
        self.next_available_color = (self.next_available_color + 1) % len(self.colors)
        self.target_joint_values = None
        self.enabled = enabled
        self.subtarget_joint_actions = False

        self.pick_pose = None
        self.place_pose = None
        '''
        if training:
            self.id = p.loadURDF('../../assets/ur5/ur5_training.urdf',
                                      self.pose[0],
                                      self.pose[1],
                                      flags=p.URDF_USE_SELF_COLLISION)
            self.ee = None
            p.changeVisualShape(
                self.id,
                self.EEF_LINK_INDEX,
                textureUniqueId=-1,
                rgbaColor=(
                    self.color[0],
                    self.color[1],
                    self.color[2], 0.5))
        else:
        '''
        ur5_path = os.path.join(os.path.dirname(__file__), "../../assets/ur5/ur5.urdf")
        self.id = p.loadURDF(ur5_path, self.pose[0], self.pose[1], flags=p.URDF_USE_SELF_COLLISION)
        self.ee = Robotiq2F85(self.env, self, self.color)

        robot_joint_info = [p.getJointInfo(self.id, i, physicsClientId=self.env.client) for i in range(p.getNumJoints(self.id))]
        self._robot_joint_indices = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_lower_limits = [x[8] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_upper_limits = [x[9] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self.home_config = [-np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]

        # for motion planning
        """
        self.arm_difference_fn = get_difference_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_distance_fn = get_distance_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_sample_fn = get_sample_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.arm_extend_fn = get_extend_fn(
            self.body_id, self.GROUP_INDEX['arm'])
        self.link_pairs = get_self_link_pairs(
            self.body_id,
            self.GROUP_INDEX['arm'])
        self.closest_points_to_others = []
        self.closest_points_to_self = []
        self.max_distance_from_others = 0.5
        """

    def update_closest_points(self):
        pass

    def check_collision(self, collision_distance):
        # collision with other

        # self collision
        pass

    def pick_and_place_FSM(self):
        pass

    def compute_next_subtarget_joints(self):
        current_joints = self.get_arm_joint_values()
        if type(self.target_joint_values) != np.ndarray:
            self.target_joint_values = np.array(self.target_joint_values)
        subtarget_joint_values = self.target_joint_values - current_joints
        dt = 1. / self.env.hz
        dj = dt * self.velocity
        max_relative_joint = max(abs(subtarget_joint_values))
        if max_relative_joint < dj: # if it can reach in 1 step
            return subtarget_joint_values
        subtarget_joint_values = dj * subtarget_joint_values / max_relative_joint
        return subtarget_joint_values + current_joints

    def disable(self):
        self.enabled = False
        self.set_pose(self.init_pose, [0., 0., 0., 1.])
        self.reset()
        self.step()

    def enable(self):
        self.enabled = True

    def step(self):
        if self.ee is not None:
            self.ee.still()
        if self.subtarget_joint_actions:
            control_joints(self.id, self.GROUP_INDEX['arm'], self.compute_next_subtarget_joints(), velocity=self.velocity, acceleration=self.acceleration)

    def get_pose(self):
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)

    def set_pose(self, pose):
        self.pose = pose
        p.resetBasePositionAndOrientation(self.id, self.pose[0], self.pose[1], physicsClientId=self.env.client)
        if self.ee is not None:
            self.ee.update_ee_pose()

    def global_to_ur5_frame(self, position, rotation=None):
        # relative pos and rot in ur5_base frame
        self_pos, self_rot = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        invert_self_pos, invert_self_rot = p.invertTransform(
            self_pos, self_rot)
        ur5_frame_pos, ur5_frame_rot = p.multiplyTransforms(
            invert_self_pos, invert_self_rot,
            position, invert_self_rot if rotation is None else rotation
        )
        return ur5_frame_pos, ur5_frame_rot

    def get_link_global_positions(self):
        linkstates = [p.getLinkState(self.id, link_id, computeForwardKinematics=True, physicsClientId=self.env.client) for link_id in range(UR5.LINK_COUNT)]
        link_world_positions = [world_pos for world_pos, world_rot, _, _, _, _, in linkstates]
        return link_world_positions

    def get_arm_joint_values(self):
        return np.array(get_joint_positions(self.id, self.GROUP_INDEX['arm']))

    def reset(self):
        self.set_arm_joints(self.home_config)

    def get_end_effector_pose(self, link=None):
        link = link if link is not None else self.EEF_LINK_INDEX
        return get_link_pose(self.id, link)

    def violates_limits(self):
        return violates_limits(self.id, self.GROUP_INDEX['arm'], self.get_arm_joint_values())

    def set_target_end_eff_pos(self, pos):
        self.set_arm_joints(self.inverse_kinematics(position=pos))

    def inverse_kinematics(self, position, orientation=None):
        return inverse_kinematics(self.id, self.EEF_LINK_INDEX, position, orientation)

    def forward_kinematics(self, joint_values):
        return forward_kinematics(self.id, self.GROUP_INDEX['arm'], joint_values, self.EEF_LINK_INDEX)

    def control_arm_joints(self, joint_values, velocity=None):
        velocity = self.velocity if velocity is None else velocity
        self.target_joint_values = joint_values
        if not self.subtarget_joint_actions:
            control_joints(self.id, self.GROUP_INDEX['arm'], self.target_joint_values, velocity=velocity, acceleration=self.acceleration)

    def control_arm_joints_delta(self, delta_joint_values, velocity=None):
        self.control_arm_joints(self.get_arm_joint_values() + delta_joint_values, velocity=velocity)

    def control_arm_joints_norm(self, normalized_joint_values, velocity=None):
        self.control_arm_joints(self.unnormalize_joint_values(normalized_joint_values), velocity=velocity)

    # NOTE: normalization is between -1 and 1
    @staticmethod
    def normalize_joint_values(joint_values):
        return (joint_values - np.array(UR5.LOWER_LIMITS)) / (np.array(UR5.UPPER_LIMITS) - np.array(UR5.LOWER_LIMITS)) * 2 - 1

    @staticmethod
    def unnormalize_joint_values(normalized_joint_values):
        return (0.5 * normalized_joint_values + 0.5) * (np.array(UR5.UPPER_LIMITS) - np.array(UR5.LOWER_LIMITS)) + np.array(UR5.LOWER_LIMITS)

    def at_target_joints(self):
        actual_joint_state = self.get_arm_joint_values()
        return all([np.abs(actual_joint_state[joint_id] - self.target_joint_values[joint_id]) < self.joint_epsilon for joint_id in range(len(actual_joint_state))])

    def set_arm_joints(self, joint_values):
        set_joint_positions(self.id, self.GROUP_INDEX['arm'], joint_values)
        self.control_arm_joints(joint_values=joint_values)
        if self.ee is not None:
            self.ee.update_ee_pose()
