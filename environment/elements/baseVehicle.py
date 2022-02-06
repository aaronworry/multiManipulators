import numpy as np
import math
import pybullet as p
import pybullet_data
from abc import ABC, abstractmethod
from collectVehicle import CollectVehicle
from grabVehicle import GrabVehicle
from pushVehicle import PushVehicle
import gym

from vehicleController import VehicleController

def restrict_heading_range(h):
    return (h + math.pi) % (2 * math.pi) - math.pi

def orientation_to_heading(o):
    # Note: Only works for z-axis rotations
    return 2 * math.acos(math.copysign(1, o[2]) * o[3])

def heading_to_orientation(h):
    return p.getQuaternionFromEuler((0, 0, h))

#一个对象，和pybullet中的一个车辆进行映射
class Vehicle(ABC):

    @abstractmethod
    def __init__(self, env, index):
        self.env = env
        self.index = index
        self._last_step_simulation_count = -1
        self.position = None
        self.orientation = None
        self.state = None

        self.target_position = None
        self.action = None
        self.awaiting_action = False

        self.waypoint_pos = None
        self.waypoint_ori = None

        self.controller = VehicleController()

        self.collision = False   #set([self.id])
        self.collision_obstacle = False
        self.collision_vehicle = False

        self.id = self._createBody()
        self.cid = self.env.p.createConstraint(self.id, -1, -1, -1, p.JOINT_FIXED, None, (0, 0, 0), (0, 0, 0))

    def _createBody(self):
        #collision_shape_id = self.env.p.createCollisionShapeArray(shapeTypes=shape_types, radii=radii, halfExtents=half_extents, lengths=lengths, collisionFramePositions=frame_positions)
        #visual_shape_id = self.env.p.createVisualShapeArray(shapeTypes=shape_types, radii=radii, halfExtents=half_extents, lengths=lengths, rgbaColors=rgba_colors, visualFramePositions=frame_positions)
        return self.env.p.createMultiBody(0.5, collision_shape_id, visual_shape_id)

    def step(self):
        self.controller.step()

    def getState(self):
        return self.state

    def getReward(self):
        return reward

    def reset(self):
        self.action = None
        self.target_position = None
        self.waypoint_pos = None
        self.waypoint_ori = None
        self.controller.reset()

    def get_position_and_orientation(self):
        if self._last_step_simulation_count < self.env.step_simulation_count:
            position, Orientation = self.env.p.getBasePositionAndOrientation(self.id)
            self.position = (position[0], position[1], 0)  # Use immutable tuples to represent positions
            self.orientation = orientation_to_heading(Orientation)
            self._last_step_simulation_count = self.env.step_simulation_count
        return self.position, self.orientation

    def reset_pose(self, position_x, position_y, orientation):
        position = (position_x, position_y, 0)
        Orientation = heading_to_orientation(orientation)
        self.env.p.resetBasePositionAndOrientation(self.id, position, Orientation)
        self.env.p.changeConstraint(self.cid, jointChildPivot=position, jointChildFrameOrientation=Orientation, maxForce=Robot.CONSTRAINT_MAX_FORCE)
        self._last_step_simulation_count = -1

    def check_for_collisions(self):
        for body_a_id in self.collision:
            for contact_point in self.env.p.getContactPoints(body_a_id):
                body_b_id = contact_point[2]
                if body_b_id in self.collision:
                    continue
                if body_b_id in self.env.obstacle_collision_body_b_ids_set:
                    self.collision_obstacle = True
                if body_b_id in self.env.robot_collision_body_b_ids_set:
                    self.collision_vehicle = True
                if self.collision_obstacle or self.collision_vehicle:
                    break

    @staticmethod
    def get_robot_cls(robot_type):
        if robot_type == 'pushing_robot':
            return PushVehicle
        if robot_type == 'grab_robot':
            return GrabVehicle
        if robot_type == 'collect_robot':
            return CollectVehicle
        raise Exception(robot_type)

    @staticmethod
    def get_robot(robot_type, *args):
        return Vehicle.get_robot_cls(robot_type)(*args)
