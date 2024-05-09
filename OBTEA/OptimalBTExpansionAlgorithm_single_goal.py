import copy
import random
from BehaviorTree import Leaf,ControlBT
import numpy as np


class CondActPair:
    def __init__(self, cond_leaf,act_leaf):
        self.cond_leaf = cond_leaf
        self.act_leaf = act_leaf


class Action:
    def __init__(self,name='anonymous action',pre=set(),add=set(),del_set=set(),cost=1):
        self.pre=copy.deepcopy(pre)
        self.add=copy.deepcopy(add)
        self.del_set=copy.deepcopy(del_set)
        self.name=name
        self.cost=cost

    def __str__(self):
        return self.name

    def generate_from_state_local(self,state,literals_num_set):


        pre_num = random.randint(0, len(state))
        self.pre = set(random.sample(state, pre_num))

        add_set = literals_num_set - self.pre
        add_num = random.randint(0, len(add_set))
        self.add = set(random.sample(add_set, add_num))

        del_set = literals_num_set - self.add
        del_num = random.randint(0, len(del_set))
        self.del_set = set(random.sample(del_set, del_num))


def generate_random_state(num):
    result = set()
    for i in range(0,num):
        if random.random()>0.5:
            result.add(i)
    return result

def state_transition(state,action):
    if not action.pre <= state:
        print ('error: action not applicable')
        return state
    new_state=(state | action.add) - action.del_set
    return new_state


def conflict(c):
    have_at = False
    for str in c:
        if 'At' in str:
            if not have_at:
                have_at = True
            else:
                return True
    return False


class OptBTExpAlgorithm:
    def __init__(self,verbose=False):
        self.bt = None
        self.nodes=[]
        self.traversed=[]
        self.mounted=[]
        self.conditions=[]
        self.conditions_index=[]
        self.verbose=verbose
        self.goal=None

    def clear(self):
        self.bt = None
        self.nodes = []
        self.traversed = []
        self.expanded = []
        self.conditions = []
        self.conditions_index = []


    # def run_algorithm(self,goal,actions,scene):
    def run_algorithm(self, start, goal, actions):
        # self.scene = scene

        self.goal = goal

        if self.verbose:
            print("\nStart！")


        self.bt = ControlBT(type='cond')

        gc_node = Leaf(type='cond', content=goal, mincost=0)
        ga_node = Leaf(type='act', content=None, mincost=0)
        subtree = ControlBT(type='?')
        subtree.add_child([copy.deepcopy(gc_node)])
        self.bt.add_child([subtree])

        # self.conditions.append(goal)
        cond_anc_pair = CondActPair(cond_leaf=gc_node,act_leaf=ga_node)
        self.nodes.append(copy.deepcopy(cond_anc_pair)) # the set of explored but unexpanded conditions
        self.traversed = [goal] # the set of expanded conditions

        while len(self.nodes)!=0:

            #  Find the condition for the shortest cost path
            pair_node = None
            min_cost = float ('inf')
            index= -1
            for i,cond_anc_pair in enumerate(self.nodes):


                if cond_anc_pair.cond_leaf.mincost < min_cost:
                    min_cost = cond_anc_pair.cond_leaf.mincost
                    pair_node = copy.deepcopy(cond_anc_pair)
                    index = i

            if self.verbose:
                print("Select an expansion condition node：",pair_node.cond_leaf.content)
            # Update self.nodes and self.traversed
            self.nodes.pop(index) #  the set of explored but unexpanded conditions. self.nodes.remove(pair_node)
            c = pair_node.cond_leaf.content

            # Mount the action node and extend BT. T = Eapand(T,c,A(c))
            if c!=goal:
                if c!=set():

                    sequence_structure = ControlBT(type='>')
                    sequence_structure.add_child(
                        [copy.deepcopy(pair_node.cond_leaf), copy.deepcopy(pair_node.act_leaf)])
                    subtree.add_child([copy.deepcopy(sequence_structure)])
                    self.expanded.append(copy.deepcopy(pair_node))

                    if c <= start:


                        return True
                else:
                    subtree.add_child([copy.deepcopy(pair_node.act_leaf)])


                if self.verbose:
                    print("Expansion completed: a_node=%s, corresponding new condition c_attr=%s, mincost=%d" \
                          % (cond_anc_pair.act_leaf.content.name, cond_anc_pair.cond_leaf.content,
                             cond_anc_pair.cond_leaf.mincost))

            if self.verbose:
                print("Iterate through all actions to find those that meet the conditions")
            current_mincost = pair_node.cond_leaf.mincost

            for i in range(0, len(actions)):

                if not c & ((actions[i].pre | actions[i].add) - actions[i].del_set) <= set()  :
                    if (c - actions[i].del_set) == c:
                        if self.verbose:
                            print("———— Conditions met, expansion possible")
                        c_attr = (actions[i].pre | c) - actions[i].add


                        valid = True
                        for j in self.traversed:
                            if j <= c_attr:
                                valid = False
                                if self.verbose:
                                    print("———— --Conflict")
                                break

                        if valid:
                            c_attr_node = Leaf(type='cond', content=c_attr, mincost=current_mincost + actions[i].cost)
                            a_attr_node = Leaf(type='act', content=actions[i], mincost=current_mincost + actions[i].cost)
                            cond_anc_pair = CondActPair(cond_leaf=c_attr_node, act_leaf=a_attr_node)
                            self.nodes.append(copy.deepcopy(cond_anc_pair))  # condition node list
                            self.traversed.append(c_attr)
                            if self.verbose:
                                print("———— -- %s Put into the list if it meets the conditions, corresponding to c as %s" % (actions[i].name,c_attr))

        # self.merge_adjacent_conditions_stack()
        # self.merge_adjacent_conditions_stack_old()
        # self.merge_adjacent_conditions()
        if self.verbose:
            print("End！\n")
        return True

    def merge_adjacent_conditions_stack(self):
        bt = ControlBT(type='cond')
        sbtree = ControlBT(type='?')
        bt.add_child([sbtree])

        parnode = copy.deepcopy(self.bt.children[0])
        stack=[]
        for child in parnode.children:
            if isinstance(child, ControlBT) and child.type == '>':
                if stack==[]:
                    stack.append(child)
                    continue
                last_child = stack[-1]
                if isinstance(last_child, ControlBT) and last_child.type == '>':
                    set1 = last_child.children[0].content
                    set2 = child.children[0].content
                    inter = set1 & set2
                    if inter!=set():
                        c1 = set1-set2
                        c2 = set2-set1
                        inter_node = Leaf(type='cond', content=inter)
                        c1_node = Leaf(type='cond', content=c1)
                        c2_node = Leaf(type='cond', content=c2)
                        a1_node = copy.deepcopy(last_child.children[1])
                        a2_node = copy.deepcopy(child.children[1])


                        if (c1==set() and isinstance(last_child.children[1], Leaf) and isinstance(child.children[1], Leaf) \
                               and isinstance(last_child.children[1].content, Action) and isinstance(child.children[1].content, Action)):
                            continue

                        if len(last_child.children)==3 and \
                            isinstance(last_child.children[2], Leaf) and isinstance(child.children[1], Leaf) \
                                and isinstance(last_child.children[2].content, Action) and isinstance( child.children[1].content, Action) \
                                and last_child.children[2].content.name == child.children[1].content.name \
                                and c1==set() and c2!=set():
                                    last_child.children[1].add_child([copy.deepcopy(c2_node)])
                                    continue
                        elif len(last_child.children)==3:
                            stack.append(child)
                            continue


                        if isinstance(last_child.children[1], Leaf) and isinstance(child.children[1], Leaf) \
                            and isinstance(last_child.children[1].content, Action) and isinstance(child.children[1].content, Action) \
                                and last_child.children[1].content.name == child.children[1].content.name:

                            if c2==set():
                                tmp_tree = ControlBT(type='>')
                                tmp_tree.add_child(
                                    [copy.deepcopy(inter_node), copy.deepcopy(a1_node)])
                            else:
                                _sel = ControlBT(type='?')
                                _sel.add_child([copy.deepcopy(c1_node), copy.deepcopy(c2_node)])
                                tmp_tree = ControlBT(type='>')
                                tmp_tree.add_child(
                                    [copy.deepcopy(inter_node), copy.deepcopy(_sel),copy.deepcopy(a1_node)])
                        else:
                            if c1 == set():
                                seq1 = copy.deepcopy(last_child.children[1])
                            else:
                                seq1 = ControlBT(type='>')
                                seq1.add_child([copy.deepcopy(c1_node), copy.deepcopy(a1_node)])

                            if c2 == set():
                                seq2 = copy.deepcopy(child.children[1])
                            else:
                                seq2 = ControlBT(type='>')
                                seq2.add_child([copy.deepcopy(c2_node), copy.deepcopy(a2_node)])
                            sel = ControlBT(type='?')
                            sel.add_child([copy.deepcopy(seq1), copy.deepcopy(seq2)])
                            tmp_tree = ControlBT(type='>')
                            tmp_tree.add_child(
                                [copy.deepcopy(inter_node), copy.deepcopy(sel)])

                        stack.pop()
                        stack.append(tmp_tree)

                    else:
                        stack.append(child)
                else:
                    stack.append(child)
            else:
                stack.append(child)

        for tree in stack:
            sbtree.add_child([tree])
        self.bt = copy.deepcopy(bt)

    def merge_adjacent_conditions(self):
        bt = ControlBT(type='cond')
        sbtree = ControlBT(type='?')

        bt.add_child([sbtree])

        parnode = copy.deepcopy(self.bt.children[0])
        skip_next = False
        for i in range(len(parnode.children) - 1):
            current_child = parnode.children[i]
            next_child = parnode.children[i + 1]

            if isinstance(current_child, ControlBT) and isinstance(next_child, ControlBT) and current_child.type == '>' and next_child.type == '>':

                if not skip_next:
                    set1 = current_child.children[0].content
                    set2 = next_child.children[0].content
                    if set1>=set2:
                        inter = set1 & set2
                        dif = set1 - set2


                        tmp_sub_seq = ControlBT(type='>')
                        c2 = Leaf(type='cond', content=dif)
                        a1 = Leaf(type='act', content=current_child.children[1].content)
                        tmp_sub_seq.add_child(
                            [copy.deepcopy(c2), copy.deepcopy(a1)])

                        tmp_sub_tree_sel = ControlBT(type='?')
                        a2 = Leaf(type='act', content=next_child.children[1].content)
                        tmp_sub_tree_sel.add_child(
                            [copy.deepcopy(tmp_sub_seq), copy.deepcopy(a2)])

                        tmp_tree = ControlBT(type='>')
                        c1 = Leaf(type='cond', content=inter)
                        tmp_tree.add_child(
                            [copy.deepcopy(c1), copy.deepcopy(tmp_sub_tree_sel)])

                        sbtree.add_child([tmp_tree])
                        skip_next = True

                elif skip_next:
                    sbtree.add_child([current_child])
            else:
                sbtree.add_child([current_child])

            sbtree.add_child([next_child])

        self.bt = copy.deepcopy(bt)


    def print_solution(self):
        print("========= BT ==========")
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


    def save_ptml_file(self,file_name):
        self.ptml_string = "selector{\n"
        self.dfs_ptml(self.bt.children[0])
        self.ptml_string += '}\n'
        with open(f'./{file_name}.ptml', 'w') as file:
            file.write(self.ptml_string)
        return self.ptml_string
