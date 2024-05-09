
from robowaiter.scene.scene import Scene
from robowaiter.behavior_lib._base.Behavior import Bahavior
class SceneOT(Scene):

    def __init__(self, robot):
        super().__init__(robot)
        self.signal_event_list = [
            (3, self.add_walker, (3, 0, 700)),
            (1, self.control_walker, (0, False, 100, 60, 520, 0)),
            (1, self.customer_say, (0, "It's quite toasty inside, could you please lower the air conditioning temperature?")),  # 给我来杯酸奶和冰红茶，我坐在对面的桌子那儿。
            # (5, self.control_walker, (0, False, 100, -250, 480, 0)),
        ]


    def _reset(self):
        # self.add_walkers([[0, 880], [250, 1200]])
        self.gen_obj_tmp()





        pass


    def _run(self):
        pass


if __name__ == '__main__':
    import os
    from robowaiter.robot.robot import Robot

    robot = Robot()

    # create task
    task = SceneOT(robot)
    task.reset()
    task.run()
