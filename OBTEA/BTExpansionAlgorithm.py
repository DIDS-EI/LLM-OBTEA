import random
import numpy as np
import copy
import time

from OptimalBTExpansionAlgorithm import ControlBT,Leaf,generate_random_state,Action,state_transition,conflict
import re



class BTExpAlgorithm:
    def __init__(self,verbose=False):
        self.bt = None
        self.nodes = []
        self.traversed = []
        self.conditions = []
        self.conditions_index = []
        self.verbose = verbose
        # print (self.conditions_list[0])

    def clear(self):
        self.bt = None
        self.nodes = []
        self.traversed = []
        self.conditions = []
        self.conditions_index = []
        self.traversed_state_num=0
        self.fot_times =0
        self.expand_conds = 0
        self.tree_size = 0

    def run_algorithm_selTree(self, start, goal, actions):
        self.traversed_state_num=0

        bt = ControlBT(type='cond')
        g_node = Leaf(type='cond', content=goal,mincost=0)
        bt.add_child([g_node])
        self.expand_conds +=1
        self.conditions.append(goal)
        self.nodes.append(g_node)  # condition node list
        self.traversed_state_num+=1

        val, obj = bt.tick(start)
        canrun = False
        if val == 'success' or val == 'running':
            canrun = True

        while not canrun:

            self.fot_times += 1

            # print("canrun:",canrun)
            index = -1
            for i in range(0, len(self.nodes)):
                if self.nodes[i].content in self.traversed:
                    continue
                else:
                    c_node = self.nodes[i]
                    index = i
                    break
            if index == -1:
                print('Failure')
                return False

            subtree = ControlBT(type='?')
            subtree.add_child([copy.deepcopy(c_node)])
            c = c_node.content

            if self.verbose:
                print("Select the expansion condition node：",c)

            if c != goal:
                if c <= start:
                    return bt

            act_num = 0

            for i in range(0, len(actions)):



                if not c & ((actions[i].pre | actions[i].add) - actions[i].del_set) <= set():

                    if (c - actions[i].del_set) == c:

                        c_attr = (actions[i].pre | c) - actions[i].add
                        valid = True

                        if conflict(c_attr):
                            continue

                        for j in self.traversed:
                            if j <= c_attr:
                                valid = False
                                break

                        if valid:

                            sequence_structure = ControlBT(type='>')
                            c_attr_node = Leaf(type='cond', content=c_attr, mincost=0)
                            a_node = Leaf(type='act', content=actions[i], mincost=0)
                            sequence_structure.add_child([c_attr_node, a_node])
                            subtree.add_child([sequence_structure])
                            self.nodes.append(c_attr_node)
                            self.expand_conds += 1
                            act_num+=1
                            # self.traversed_state_num+=1
                            if self.verbose:
                                print("Expansion completed: a_node=%s, corresponding new condition c_attr=%s" \
                                      % (actions[i].name, c_attr))

            self.traversed_state_num += act_num

            parent_of_c = c_node.parent
            parent_of_c.children[0] = subtree

            self.traversed.append(c)

            val, obj = bt.tick(start)
            canrun = False
            if val == 'success' or val == 'running':
                canrun = True
            self.tree_size = self.bfs_cal_tree_size_subtree(bt)
        return bt


    def run_algorithm_test(self, start, goal, actions):
        self.bt = self.run_algorithm_selTree(start, goal, actions)
        return True



    def run_algorithm(self, start, goal, actions):

        self.bt = ControlBT(type='cond')
        subtree = ControlBT(type='?')
        if len(goal) > 1:
            for g in goal:
                bt_sel_tree = self.run_algorithm_selTree(start, g, actions)

                subtree.add_child([bt_sel_tree.children[0]])
            self.bt.add_child([subtree])
        else:
            self.bt = self.run_algorithm_selTree(start, goal[0], actions)
        return True


    def print_solution(self):
        print("========= Baseline ==========")
        nodes_ls = []
        nodes_ls.append(self.bt)
        while len(nodes_ls) != 0:
            parnode = nodes_ls[0]
            print("Parrent:", parnode.type)
            for child in parnode.children:
                if isinstance(child, Leaf):
                    print("---- Leaf:", child.content)
                elif isinstance(child, ControlBT):
                    print("---- ControlBT:", child.type)
                    nodes_ls.append(child)
            print()
            nodes_ls.pop(0)
        print("========= Baseline ==========\n")


    def dfs_ptml_many_act(self, parnode, is_root=False):
        for child in parnode.children:
            if isinstance(child, Leaf):
                if child.type == 'cond':

                    if is_root and len(child.content) > 1:
                        self.ptml_string += "sequence{\n"
                        self.ptml_string += "cond "
                        c_set_str = '\n cond '.join(map(str, child.content)) + "\n"
                        self.ptml_string += c_set_str
                        self.ptml_string += '}\n'
                    else:
                        self.ptml_string += "cond "
                        c_set_str = '\n cond '.join(map(str, child.content)) + "\n"
                        self.ptml_string += c_set_str

                elif child.type == 'act':

                    child.content.name = re.sub(r'\d+', '', child.content.name)

                    if '(' not in child.content.name:
                        self.ptml_string += 'act ' + child.content.name + "()\n"
                    else:
                        self.ptml_string += 'act ' + child.content.name + "\n"
            elif isinstance(child, ControlBT):
                if child.type == '?':
                    self.ptml_string += "selector{\n"
                    self.dfs_ptml_many_act(parnode=child)
                elif child.type == '>':
                    self.ptml_string += "sequence{\n"
                    self.dfs_ptml_many_act(parnode=child)
                self.ptml_string += '}\n'

    def get_ptml_many_act(self):
        self.ptml_string = "selector{\n"
        self.dfs_ptml_many_act(self.bt.children[0],is_root=True)
        self.ptml_string += '}\n'
        return self.ptml_string


    def bfs_cal_tree_size(self):
        from collections import deque
        queue = deque([self.bt.children[0]])

        if isinstance(self.bt.children[0], Leaf):
            return 0

        count = 0
        while queue:
            current_node = queue.popleft()
            count += 1
            for child in current_node.children:
                if isinstance(child, Leaf):
                    count += 1
                else:
                    queue.append(child)
        return count

    def bfs_cal_tree_size_subtree(self,bt):
        from collections import deque
        queue = deque([bt.children[0]])

        if isinstance(bt.children[0], Leaf):
            return 0

        count = 0
        while queue:
            current_node = queue.popleft()
            count += 1
            for child in current_node.children:
                if isinstance(child, Leaf):
                    count += 1
                else:
                    queue.append(child)
        return count


if __name__ == '__main__':

    bt_algo_opt = False


    actions = []
    a = Action(name='movebtob')
    a.pre = {1, 2}
    a.add = {3}
    a.del_set = {1, 4}
    actions.append(a)
    a = Action(name='moveatob')
    a.pre = {1}
    a.add = {5, 2}
    a.del_set = {1, 6}
    actions.append(a)
    a = Action(name='moveatoa')
    a.pre = {7}
    a.add = {8, 2}
    a.del_set = {7, 6}
    actions.append(a)

    start = {1, 7, 4, 6}
    goal = {3}
    algo = BTExpAlgorithm()
    algo.clear()
    algo.run_algorithm(start, goal, list(actions))
    state = start
    steps = 0
    val, obj = algo.bt.tick(state)
    while val != 'success' and val != 'failure':
        state = state_transition(state, obj)
        print(obj.name)
        val, obj = algo.bt.tick(state)
        if (val == 'failure'):
            print("bt fails at step", steps)
        steps += 1
    if not goal <= state:
        print("wrong solution", steps)
    else:
        print("right solution", steps)
    print(algo.bt.count_size() - 1)
    algo.clear()

