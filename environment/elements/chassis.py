import pybullet as p
from baseChassis import BaseChassis
from chassisController import MecanumController, DifferentialController


class Mecanum(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(MecanumController)

        self.lateral_velocity = 0.
        self.linear_velocity = 0.

    def _createBody(self):
        pass

    def step(self):
        pass

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.lateral_velocity]
        return self.state


class Differential(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(DifferentialController)

        self.linear_velocity = 0.
        self.angular_velocity = 0.

    def _createBody(self):
        pass

    def step(self):
        pass

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.angular_velocity]
        return self.state
