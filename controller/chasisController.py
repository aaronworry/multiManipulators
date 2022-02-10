class ChasisController():
    def __init__(self, robot):
        self.robot = robot
        self.state = None
        self.prev_position = None
        self.prev_orientation = None
        self.sim_steps = 0
        self.consecutive_turning_sim_steps = None  # Used to detect if robot is stuck and oscillating
        self.manipulation_sim_step_target = 0
        self.manipulation_sim_steps = 0

    def reset(self):
        self.state = None
        self.prev_position = None
        self.prev_orientation = None
        self.sim_steps = 0
        self.consecutive_turning_sim_steps = None

    def action(self):
        self.state = 'moving'  # 'work', 'unload'

    def step(self):
        if self.state == 'moving':
            pass
        elif self.state == 'work':
            pass
        elif self.state == 'unload':
            pass

