"""

Interactive scenario, input

"""

from robowaiter.scene.scene import Scene

class SubScene(Scene):

    def __init__(self, robot):
        super().__init__(robot)
        # Insert events occurring in the scenario here.

    def _reset(self):
        pass


    def _step(self):
        if len(self.sub_task_seq.children) == 0:
            question = input("Please enter a command：")
            if question[-1] == ")":
                print(f"Set Goals:{question}")
                self.new_set_goal(question)
            else:
                self.customer_say("System",question)



if __name__ == '__main__':
    from robowaiter.robot.robot import Robot

    robot = Robot()

    # create task
    task = SubScene(robot)
    task.reset()
    task.run()
