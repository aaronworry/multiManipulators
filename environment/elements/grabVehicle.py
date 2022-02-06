from baseVehicle import Vehicle
import math
import pybullet as p


class GrabVehicle(Vehicle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.potential_thing_id = None
        self.end_effector_id = self._create_end_effector()
        self.end_effector_cid = None
        self.attach_end_effector()
        self.end_effect_location = 0.05
        self.end_effect_dis_thred = 0.001

        self.grab_state = 'ready'    #ready, doing, finish, ready
        self.thing_id = None  #
        self.lift_cid = None  #


    def reset_pose(self, *args):
        super.reset_pose(*args)
        if self.end_effector_cid is not None:
            self.detach_end_effector()
            self.attach_end_effector() 

    def ray_test_thing(self):
        #ray find whether collect thing
        #using ray to detect it
        position = self.waypoint_pos[-1]
        orientation = self.waypoint_ori[-1]
        ray_from = (
            position[0] + math.cos(orientation) * self.end_effect_location,
            position[1] + math.sin(orientation) * self.end_effect_location,
            0.05
        )
        ray_to = (
            position[0] + math.cos(orientation) * self.end_effect_location + math.cos(orientation) * self.end_effect_dis_thred,
            position[1] + math.sin(orientation) * self.end_effect_location + math.sin(orientation) * self.end_effect_dis_thred,
            0.05
        )
        body_id = self.env.p.rayTestBatch([ray_from], [ray_to])[0][0]
        if body_id in self.env.available_thing_ids_set:
            return body_id
        return None

    def _create_end_effector(self):
        thing_width = 0.007
        half_extents = (thing_width / 2, 0.009, thing_width / 2)
        collision_shape_id = self.env.p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        visual_shape_id = self.env.p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=(0, 0, 0, 0))
        # Make invisible by setting alpha to 0

        return self.env.p.createMultiBody(0.001, collision_shape_id, visual_shape_id)


    def detach_end_effector(self):
        self.env.p.removeConstraint(self.end_effector_cid)
        self.end_effector_cid = None
        self.env.p.resetBasePositionAndOrientation(self.end_effector_id, (0, 0, -1000), (0, 0, 0, 1))
        self.collision.remove(self.end_effector_id)
        self.env.robot_collision_body_b_ids_set.remove(self.end_effector_id)

    def attach_end_effector(self):
        if self.end_effector_cid is not None:
            return

        #设置执行器的位置

        #添加约束

        #机器人id_set，环境id_set更新

        pass

    def grab_thing(self, thing_id):
        self.thing_id = thing_id
        self.grab_state = 'doing'

        #关系更新
        self.env.available_thing_ids_set.remove(self.thing_id)
        self.collision.add(self.thing_id)
        self.env.robot_collision_body_b_ids_set.add(self.thing_id)

        #move thing through manipulator
        self._grab_thing_and_reset_thing_pose()

        #添加约束
        parent_frame_position = (-0.2, 0, 0.5)
        self.lift_cid = self.env.p.createConstraint(self.id, -1, self.thing_id, -1, p.JOINT_FIXED, None, parent_frame_position, (0, 0, 0))

    def unload_thing(self, thing_id):
        #取消约束
        self.env.p.removeConstraint(self.lift_cid)
        self.lift_cid = None
        #卸载动作
        self._unload_thing_and_reset_thing_pose()
        #关系更新
        self.env.remove_cube(self.thing_id)
        self.lift_state = 'ready'
        self.collision.remove(self.thing_id)
        self.env.robot_collision_body_b_ids_set.remove(self.thing_id)
        self.thing_id = None


    def _grab_thing_and_reset_thing_pose(self):

        #self.env.p.resetBasePositionAndOrientation(self.thing_id, cube_position, heading_to_orientation(current_heading))
        pass

    def _unload_thing_and_reset_thing_pose(self):
        pass