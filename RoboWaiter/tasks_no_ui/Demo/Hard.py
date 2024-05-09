

from robowaiter.scene.scene import Scene
from robowaiter.behavior_lib._base.Behavior import Bahavior\

class SceneOT(Scene):

    def __init__(self, robot):
        super().__init__(robot)
        self.signal_event_list = [
            # (3, self.customer_say, ("System", "It's quite toasty inside, could you please lower the air conditioning temperature?")),
            (3, self.add_walker, (27, 0, 700)),
            (1, self.control_walker, (0, False, 100, 60, 520, 0)),
            (1, self.customer_say, (0, "Could you please turn on the coffee machine at the bar, and make sure the floor is clean or the tube light is off? I enjoy my coffee in a cozy environment.")),  # 给我来杯酸奶和冰红茶，我坐在对面的桌子那儿。
            # (5, self.control_walker, (0, False, 100, -250, 480, 0)),
        ]


    def _reset(self):
        # self.add_walkers([[0, 880], [250, 1200]])
        # self.gen_obj_tmp()
        self.gen_obj()

        start_robowaiter = self.default_state["condition_set"]

        all_obj_place = Bahavior.all_object | Bahavior.tables_for_placement | Bahavior.tables_for_guiding
        start_robowaiter |= {f'Not RobotNear({place})' for place in all_obj_place if place != 'Bar'}
        start_robowaiter |= {f'Not Holding({obj})' for obj in Bahavior.all_object}
        start_robowaiter |= {f'Exists({obj})' for obj in Bahavior.all_object if
                             obj != 'Coffee' and obj != 'Water' and obj != 'Dessert'}
        # 'Softdrink' 在Table1
        start_robowaiter |= {f'Not On(Softdrink,{place})' for place in Bahavior.all_place if place != "Table1"}
        start_robowaiter |= {f'Not On(VacuumCup,{place})' for place in Bahavior.all_place if place != "Table2"}


        start_robowaiter |= {f'On({obj},Bar)' for obj in Bahavior.all_object if
                             obj != 'Coffee' and obj != 'Water' and obj != 'Dessert' \
                             and obj != 'Softdrink' and obj != 'VacuumCup'}
        for place in Bahavior.all_place:
            if place != "Bar":
                start_robowaiter |= {f'Not On({obj},{place})' for obj in Bahavior.all_object}
            # start_robowaiter |= {f'On({obj},{place})' for obj in Bahavior.all_object if
            #                      obj != 'Coffee' and obj != 'Water' and obj != 'Dessert'}


        make_obj = {"Coffee", 'Water', 'Dessert'}
        for place in Bahavior.all_place:
            start_robowaiter |= {f'Not On({obj},{place})' for obj in make_obj}

        self.state["condition_set"] = start_robowaiter
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
