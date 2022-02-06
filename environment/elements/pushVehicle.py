from baseVehicle import Vehicle

class PushVehicle(Vehicle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.thing_dist_closer = 0    #?

    def process_thing_position(self, thing_id, initial_cube_positions):
        if thing_id not in initial_cube_positions:
            return
        thing_position = self.env.get_thing_position(thing_id)
        # dist_closer = self.mapper.distance_to_receptacle(initial_cube_positions[cube_id]) - self.mapper.distance_to_receptacle(cube_position)
        # self.cube_dist_closer += dist_closer