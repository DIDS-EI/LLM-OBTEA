import copy
import random
from BehaviorTree import Leaf,ControlBT
import re
import heapq
import time
import numpy as np
class CondActPair:
    def __init__(self, cond_leaf,act_leaf):
        self.cond_leaf = cond_leaf
        self.act_leaf = act_leaf
    def __lt__(self, other):
        # Define priority comparison: compare based on the value of cost.
        return self.act_leaf.mincost < other.act_leaf.mincost

# Define the action class, actions include preconditions, add effects, and delete effects.
class Action:
    def __init__(self,name='anonymous action',pre=set(),add=set(),del_set=set(),cost=1,vaild_num=0,vild_args=set()):
        self.pre=copy.deepcopy(pre)
        self.add=copy.deepcopy(add)
        self.del_set=copy.deepcopy(del_set)
        self.name=name
        self.cost=cost
        self.vaild_num=vaild_num
        self.vild_args = vild_args

    def __str__(self):
        return self.name
    #
    # Randomly generate an action from a state.
    def generate_from_state(self,state,num):
        for i in range(0,num):
            if i in state:
                if random.random() >0.5:
                    self.pre.add(i)
                    if random.random() >0.5:
                        self.del_set.add(i)
                    continue
            if random.random() > 0.5:
                self.add.add(i)
                continue
            if random.random() >0.5:
                self.del_set.add(i)

    # def generate_from_state_local(self,literals_num_set):
    #     # pre_num = random.randint(0, min(pre_max, len(state)))
    #     # self.pre = set(np.random.choice(list(state), pre_num, replace=False))
    #     #
    #     # add_set = literals_num_set - self.pre
    #     # add_num = random.randint(0, len(add_set))
    #     # self.add = set(np.random.choice(list(add_set), add_num, replace=False))
    #     #
    #     # del_set = literals_num_set - self.add
    #     # del_num = random.randint(0, len(del_set))
    #     # self.del_set = set(np.random.choice(list(del_set), del_num, replace=False))
    #
    #     pre_num = random.randint(0, len(state))
    #     self.pre = set(random.sample(state, pre_num))
    #
    #     add_set = literals_num_set - self.pre
    #     add_num = random.randint(0, len(add_set))
    #     self.add = set(random.sample(add_set, add_num))
    #
    #     del_set = literals_num_set - self.add
    #     del_num = random.randint(0, len(del_set))
    #     self.del_set = set(random.sample(del_set, del_num))


    def generate_from_state_local(self,state,literals_num_set,all_obj_set=set(),obj_num=0,obj=None):
        # pre_num = random.randint(0, min(pre_max, len(state)))
        # self.pre = set(np.random.choice(list(state), pre_num, replace=False))
        #
        # add_set = literals_num_set - self.pre
        # add_num = random.randint(0, len(add_set))
        # self.add = set(np.random.choice(list(add_set), add_num, replace=False))
        #
        # del_set = literals_num_set - self.add
        # del_num = random.randint(0, len(del_set))
        # self.del_set = set(np.random.choice(list(del_set), del_num, replace=False))

        pre_num = random.randint(0, len(state))
        self.pre = set(random.sample(state, pre_num))

        add_set = literals_num_set - self.pre
        add_num = random.randint(0, len(add_set))
        self.add = set(random.sample(add_set, add_num))

        del_set = literals_num_set - self.add
        del_num = random.randint(0, len(del_set))
        self.del_set = set(random.sample(del_set, del_num))

        if all_obj_set!=set():
            self.vaild_num = random.randint(1, obj_num-1)
            self.vild_args = (set(random.sample(all_obj_set, self.vaild_num)))
            if obj!=None:
                self.vild_args.add(obj)
                self.vaild_num = len(self.vild_args)

    def update(self,name,pre,del_set,add):
        self.name = name
        self.pre = pre
        self.del_set = del_set
        self.add = add
        return self


    def print_action(self):
        print (self.pre)
        print(self.add)
        print(self.del_set)



# Generate a random state
def generate_random_state(num):
    result = set()
    for i in range(0,num):
        if random.random()>0.5:
            result.add(i)
    return result

# Generate a successor state from a state and an action
def state_transition(state,action):
    if not action.pre <= state:
        print ('error: action not applicable')
        return state
    new_state=(state | action.add) - action.del_set
    return new_state



def conflict(c):
    have_at = False
    for str in c:
        if 'Not' not in str and 'RobotNear' in str:
            if have_at:
                return True
            have_at = True

    Holding = False
    HoldingNothing = False
    for str in c:
        if 'Not ' not in str and 'Holding(Nothing)' in str:
            HoldingNothing = True
        if 'Not' not in str and 'Holding(Nothing)' not in str and 'Holding' in str:
            if Holding:
                return True
            Holding = True
        if HoldingNothing and Holding:
            return True
    return False


import heapq


# The complete planning algorithm proposed in this paper
class OptBTExpAlgorithm:
    def __init__(self,verbose=False,lit_act_dic=None):
        self.bt = None
        self.nodes=[]
        self.traversed=[]
        self.mounted=[]
        self.conditions=[]
        self.conditions_index=[]
        self.verbose=verbose
        self.goal=None
        self.bt_merge = True
        self.lit_act_dic = lit_act_dic

    def clear(self):
        self.bt = None
        self.nodes = []
        self.traversed = [] # Store conditions.
        self.expanded = []
        self.conditions = []
        self.conditions_index = []
        self.traversed_state_num=0
        self.fot_times = 0
        self.expand_conds=0
        self.tree_size=0
        self.bt_without_merge = None
        self.subtree_count=1

    # Run the planning algorithm, calculating the behavior tree self.bt from the initial state, goal state, and available actions
    def run_algorithm_selTree(self, start, goal, actions,merge_time=99999999):


        self.traversed_state_num=0

        self.goal = goal
        if self.verbose:
            print("\nAlgorithm begins！")
        bt = ControlBT(type='cond')
        # The initial behavior tree contains only the goal conditions
        gc_node = Leaf(type='cond', content=goal, mincost=0) # For consistency, they appear in pairs.
        ga_node = Leaf(type='act', content=None, mincost=0)
        subtree = ControlBT(type='?')
        subtree.add_child([gc_node])  # The subtree first retains the expanded node
        self.expand_conds+=1
        bt.add_child([subtree])
        cond_anc_pair = CondActPair(cond_leaf=gc_node,act_leaf=ga_node)

        heapq.heappush(self.nodes, cond_anc_pair)
        self.expanded.append(goal)
        self.traversed_state_num += 1

        self.traversed = [goal] # the set of expanded conditions
        min_cost = float('inf')

        if goal <= start:
            self.bt_without_merge = bt
            return bt, 0

        while len(self.nodes)!=0:

            self.fot_times+=1

            #  Find the condition for the shortest cost path
            # ======================== Next Goal ============================ #
            min_cost = float ('inf')
            pair_node = heapq.heappop(self.nodes)

            if self.verbose:
                print("Select an expansion condition node：",pair_node.cond_leaf.content)
            # Update self.nodes and self.traversed
            c = pair_node.cond_leaf.content
            # Mount the action node and extend BT. T = Eapand(T,c,A(c))



            if c!=goal:
                if c!=set():
                    sequence_structure = ControlBT(type='>')
                    sequence_structure.add_child(
                        [pair_node.cond_leaf, pair_node.act_leaf])
                    subtree.add_child([copy.deepcopy(sequence_structure)])  # The subtree is constantly changing, with its parent being self.bt.
                    # self.expanded.append(copy.deepcopy(pair_node))
                    # self.expanded.append(pair_node.cond_leaf.content)
                    #
                    # if c <= start:
                    #     if self.bt_merge:
                    #         # bt = self.merge_adjacent_conditions_stack(bt)
                    #         bt = self.merge_adjacent_conditions_stack_time(bt,merge_time=merge_time)
                    #     return bt, min_cost
                else:
                    subtree.add_child([copy.deepcopy(pair_node.act_leaf)])
                self.expand_conds += 1
                self.expanded.append(c)
                if c <= start:
                    self.tree_size = self.bfs_cal_tree_size_subtree(bt)
                    self.bt_without_merge = bt
                    if self.bt_merge:
                        # bt = self.merge_adjacent_conditions_stack(bt)
                        bt = self.merge_adjacent_conditions_stack_time(bt,merge_time=merge_time)
                    return bt, min_cost



                if self.verbose:
                    print("Expansion completed: a_node=%s, corresponding new condition c_attr=%s, mincost=%d" \
                          % (cond_anc_pair.act_leaf.content.name, cond_anc_pair.cond_leaf.content,
                             cond_anc_pair.cond_leaf.mincost))

            if self.verbose:
                print("Iterate through all actions to find those that meet the conditions")
            current_mincost = pair_node.cond_leaf.mincost
            # ======================== End Next Goal ============================ #

            # ====================== Action Trasvers ============================ #
            traversed_current = []

            # act_tmp_set = set()
            # for lit in c:
            #     act_tmp_set |= self.lit_act_dic[lit]

            # for i in act_tmp_set:
            for i in range(0, len(actions)):

                # if c=={'RobotNear(Chips)', 'Holding(Nothing)'} and actions[i].name=='Clean(Chairs)0':
                #     xx=1
                if not c & ((actions[i].pre | actions[i].add) - actions[i].del_set) <= set()  :
                    if (c - actions[i].del_set) == c:
                        if self.verbose:
                            print("———— Conditions met, expansion possible")
                        c_attr = (actions[i].pre | c) - actions[i].add

                        if conflict(c_attr):
                            if self.verbose:
                                print("———— Conflict: action %s, condition %s"% (actions[i].name,c_attr))
                            continue

                        # Pruning operation: the current conditions are supersets of previously expanded conditions
                        valid = True

                        for j in self.expanded:
                            if j <= c_attr:
                                valid = False
                                break


                        if valid:
                            # c_attr_string = "".join(sorted(list(c_attr)))
                            c_attr_node = Leaf(type='cond', content=c_attr, mincost=current_mincost + actions[i].cost)
                            a_attr_node = Leaf(type='act', content=actions[i], mincost=current_mincost + actions[i].cost)
                            cond_anc_pair = CondActPair(cond_leaf=c_attr_node, act_leaf=a_attr_node)
                            # heapq.heappush(self.nodes, copy.deepcopy(cond_anc_pair))
                            heapq.heappush(self.nodes, cond_anc_pair)


                            self.traversed_state_num+=1
                            traversed_current.append(c_attr)
                            # Put all action nodes that meet the conditions into a list
                            if self.verbose:
                                print("———— -- %s Put into the list if it meets the conditions, corresponding to c as %s" % (actions[i].name,c_attr))

            # print(len(traversed_current))
            self.traversed.extend(traversed_current)
            # ====================== End Action Trasvers ============================ #
        self.tree_size = self.bfs_cal_tree_size_subtree(bt)
        self.bt_without_merge = bt
        if self.bt_merge:
            # bt = self.merge_adjacent_conditions_stack(bt)
            bt = self.merge_adjacent_conditions_stack_time(bt,merge_time=merge_time)
        if self.verbose:
            print("Algorithm concludes！\n")
        return bt,min_cost


    def run_algorithm(self, start, goal, actions,merge_time=3):
        self.bt = ControlBT(type='cond')
        subtree = ControlBT(type='?')

        subtree_with_costs_ls=[]

        self.subtree_count = len(goal)

        if len(goal) > 1:
            for g in goal:
                bt_sel_tree,mincost = self.run_algorithm_selTree(start, g, actions)
                subtree_with_costs_ls.append((bt_sel_tree,mincost))
            # Sort and add once again.
            sorted_trees = sorted(subtree_with_costs_ls, key=lambda x: x[1])
            for tree,cost in sorted_trees:
                subtree.add_child([tree.children[0]])
            self.bt.add_child([subtree])
        else:
            self.bt,mincost = self.run_algorithm_selTree(start, goal[0], actions,merge_time=merge_time)
        return True


    def merge_subtree(self,merge_time):

        self.bt_aftermerge = ControlBT(type='cond')
        subtree = ControlBT(type='?')

        if self.subtree_count > 1:
            for i in range(self.subtree_count):
                bt_sel_tree = self.bt.children[0].children[i]

                bt_sel_tree_m = ControlBT(type='cond')
                bt_sel_tree_m.add_child([bt_sel_tree])

                bt_sel_tree_m = self.merge_adjacent_conditions_stack_time(copy.deepcopy(bt_sel_tree_m),merge_time=merge_time)
                subtree.add_child([bt_sel_tree_m.children[0]])
            self.bt_aftermerge.add_child([subtree])
        else:
            self.bt_aftermerge = self.merge_adjacent_conditions_stack_time(copy.deepcopy(self.bt),merge_time=merge_time)
        return self.bt_aftermerge

    def run_algorithm_test(self, start, goal, actions):
        self.bt,mincost = self.run_algorithm_selTree(start, goal, actions)
        return True


    def merge_adjacent_conditions_stack_time(self,bt_sel,merge_time=9999999):

        merge_time = min(merge_time,500)


        bt = ControlBT(type='cond')
        sbtree = ControlBT(type='?')
        bt.add_child([sbtree])

        parnode = bt_sel.children[0]
        stack=[]
        time_stack=[]
        for child in parnode.children:
            if isinstance(child, ControlBT) and child.type == '>':
                if stack==[]:
                    stack.append(child)
                    time_stack.append(0)
                    continue
                last_child = stack[-1]
                last_time = time_stack[-1]



                if last_time<merge_time and isinstance(last_child, ControlBT) and last_child.type == '>':
                    set1 = last_child.children[0].content
                    set2 = child.children[0].content
                    inter = set1 & set2

                    # print("merge time:", last_time,set1,set2)

                    if inter!=set():
                        c1 = set1-set2
                        c2 = set2-set1
                        inter_node = Leaf(type='cond', content=inter)
                        c1_node = Leaf(type='cond', content=c1)
                        c2_node = Leaf(type='cond', content=c2)
                        a1_node = last_child.children[1]
                        a2_node = child.children[1]


                        if (c1==set() and isinstance(last_child.children[1], Leaf) and isinstance(child.children[1], Leaf) \
                               and isinstance(last_child.children[1].content, Action) and isinstance(child.children[1].content, Action)):
                            continue

                        if len(last_child.children)==3 and \
                            isinstance(last_child.children[2], Leaf) and isinstance(child.children[1], Leaf) \
                                and isinstance(last_child.children[2].content, Action) and isinstance( child.children[1].content, Action) \
                                and last_child.children[2].content.name == child.children[1].content.name \
                                and c1==set() and c2!=set():
                                    last_child.children[1].add_child([c2_node])
                                    continue
                        elif len(last_child.children)==3:
                            stack.append(child)
                            time_stack.append(0)
                            continue

                        if isinstance(last_child.children[1], Leaf) and isinstance(child.children[1], Leaf) \
                            and isinstance(last_child.children[1].content, Action) and isinstance(child.children[1].content, Action) \
                                and last_child.children[1].content.name == child.children[1].content.name:

                            if c2==set():
                                tmp_tree = ControlBT(type='>')
                                tmp_tree.add_child(
                                    [inter_node, a1_node])
                            else:
                                _sel = ControlBT(type='?')
                                _sel.add_child([c1_node, c2_node])
                                tmp_tree = ControlBT(type='>')
                                tmp_tree.add_child(
                                    [inter_node, _sel,a1_node])
                        else:
                            if c1 == set():
                                seq1 = last_child.children[1]
                            else:
                                seq1 = ControlBT(type='>')
                                seq1.add_child([c1_node, a1_node])

                            if c2 == set():
                                seq2 = child.children[1]
                            else:
                                seq2 = ControlBT(type='>')
                                seq2.add_child([c2_node, a2_node])
                            sel = ControlBT(type='?')
                            sel.add_child([seq1, seq2])
                            tmp_tree = ControlBT(type='>')
                            tmp_tree.add_child(
                                [inter_node,sel])

                        stack.pop()
                        time_stack.pop()
                        stack.append(tmp_tree)
                        time_stack.append(last_time+1)

                    else:
                        stack.append(child)
                        time_stack.append(0)
                else:
                    stack.append(child)
                    time_stack.append(0)
            else:
                stack.append(child)
                time_stack.append(0)

        for tree in stack:
            sbtree.add_child([tree])
        bt_sel = bt
        return bt_sel


    def print_solution(self,without_merge=False):
        print("========= BT ==========")
        nodes_ls = []
        if without_merge==True:
            nodes_ls.append(self.bt_without_merge)
        else:
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
        print("========= BT ==========\n")


    def get_all_state_leafs(self):
        state_leafs=[]

        nodes_ls = []
        nodes_ls.append(self.bt)
        while len(nodes_ls) != 0:
            parnode = nodes_ls[0]
            for child in parnode.children:
                if isinstance(child, Leaf):
                    if child.type == "cond":
                        state_leafs.append(child.content)
                elif isinstance(child, ControlBT):
                    nodes_ls.append(child)
            nodes_ls.pop(0)

        return state_leafs

    def dfs_ptml(self,parnode,is_root=False):
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
                    if '(' not in child.content.name:
                        self.ptml_string += 'act ' + child.content.name + "()\n"
                    else:
                        self.ptml_string += 'act ' + child.content.name + "\n"
            elif isinstance(child, ControlBT):
                if child.type == '?':
                    self.ptml_string += "selector{\n"
                    self.dfs_ptml(parnode=child)
                elif child.type == '>':
                    self.ptml_string += "sequence{\n"
                    self.dfs_ptml( parnode=child)
                self.ptml_string += '}\n'


    def get_ptml(self):
        self.ptml_string = "selector{\n"
        self.dfs_ptml(self.bt.children[0],is_root=True)
        self.ptml_string += '}\n'
        return self.ptml_string

    def dfs_ptml_many_act(self, parnode, is_root=False):
        for child in parnode.children:
            if isinstance(child, Leaf):
                if child.type == 'cond':

                    # if is_root and len(child.content) > 1:
                    if is_root and len(child.content) > 1:
                        self.ptml_string += "sequence{\n"
                        self.ptml_string += "cond "
                        c_set_str = '\n cond '.join(map(str, child.content)) + "\n"
                        self.ptml_string += c_set_str
                        self.ptml_string += '}\n'
                    # elif is_root:
                    else:
                        self.ptml_string += "cond "
                        c_set_str = '\n cond '.join(map(str, child.content)) + "\n"
                        self.ptml_string += c_set_str



                elif child.type == 'act':

                    child.content.name = re.sub(r'\)\d+', ')', child.content.name)
                    if '(' not in child.content.name:
                        self.ptml_string += 'act ' + child.content.name + "()\n"
                    else:
                        self.ptml_string += 'act ' + child.content.name + "\n"
            elif isinstance(child, ControlBT):
                if child.type == '?':
                    self.ptml_string += "selector{\n"
                    if len(child.children)>2:
                        self.dfs_ptml_many_act(parnode=child, is_root=True)
                    else:
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

    def save_ptml_file(self,file_name):
        self.ptml_string = "selector{\n"
        self.dfs_ptml(self.bt.children[0])
        self.ptml_string += '}\n'
        with open(f'./{file_name}.ptml', 'w') as file:
            file.write(self.ptml_string)
        return self.ptml_string


    def bfs_cal_tree_size(self):
        from collections import deque
        queue = deque([self.bt.children[0]])

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