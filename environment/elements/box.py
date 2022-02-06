from baseThing import Thing

class Box(Thing):
    def __init__(self, env, position, color):
        super().__init__(env, position, 'cube', color)
        self.velocity = 0.

    def _move(self):
        pass