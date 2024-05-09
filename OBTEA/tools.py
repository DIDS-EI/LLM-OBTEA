import copy

from tabulate import tabulate
import numpy as np
import random

from OptimalBTExpansionAlgorithm import generate_random_state,state_transition
from OptimalBTExpansionAlgorithm import Action,OptBTExpAlgorithm
from BTExpansionAlgorithm import BTExpAlgorithm


import time
np.random.seed(1)
random.seed(1)
def print_action_data_table(goal,start,actions):
    data = []
    for a in actions:
        data.append([a.name ,a.pre ,a.add ,a.del_set ,a.cost])
    data.append(["Goal" ,goal ," " ,"Start" ,start])
    print(tabulate(data, headers=["Name", "Pre", "Add" ,"Del" ,"Cost"], tablefmt="fancy_grid"))  # grid plain simple github fancy_grid


def BTTest_old(bt_algo_opt=True,seed=1,literals_num=10,depth=10,iters=10,total_count=1000):

    random.seed(seed)
    literals_num=literals_num
    depth = depth
    iters= iters
    total_tree_size = []
    total_action_num = []
    total_state_num = []
    total_steps_num=[]
    total_cost=[]
    total_tick=[]
    #fail_count=0
    #danger_count=0
    success_count =0
    failure_count = 0
    planning_time_total = 0.0

    error = False

    for count in range (total_count):

        action_num = 1
        states = []
        actions = []
        start = generate_random_state(literals_num)
        state = copy.deepcopy(start)
        states.append(state)
        #print (state)


        for i in range (0,depth):
            a = Action()
            a.generate_from_state(state,literals_num)
            a.cost = random.randint(1, 100)
            if not a in actions:
                a.name = "a"+str(action_num)
                action_num+=1
                actions.append(a)
            state = state_transition(state,a)
            if state in states:
                pass
            else:
                states.append(state)
                #print(state)

        goal = states[-1]
        state = copy.deepcopy(start)
        for i in range (0,iters):
            a = Action()
            a.generate_from_state(state,literals_num)
            if not a in actions:
                a.name = "a"+str(action_num)
                action_num+=1
                actions.append(a)
            state = state_transition(state,a)
            if state in states:
                pass
            else:
                states.append(state)
            state = random.sample(states,1)[0]


        if bt_algo_opt:
            algo = OptBTExpAlgorithm(verbose=False)
        else:
            algo = BTExpAlgorithm(verbose=False)
        algo.clear()


        start_time = time.time()
        if algo.run_algorithm(start, goal, actions):
            total_tree_size.append( algo.bt.count_size()-1)
        else:
            print ("error")
        end_time = time.time()
        planning_time_total += (end_time-start_time)

        state=start
        steps=0
        current_cost = 0
        current_tick_time=0
        val, obj, cost, tick_time = algo.bt.cost_tick(state,0,0)

        current_tick_time+=tick_time
        current_cost += cost
        while val !='success' and val !='failure':
            state = state_transition(state,obj)
            val, obj,cost, tick_time = algo.bt.cost_tick(state,0,0)
            current_cost += cost
            current_tick_time += tick_time
            if(val == 'failure'):
                print("bt fails at step",steps)
                error = True
                break
            steps+=1
            if(steps>=500):
                break
        if not goal <= state:
            failure_count+=1
            error = True
        else:
            success_count+=1
            total_steps_num.append(steps)
        if error:
            print_action_data_table(goal, start, list(actions))
            algo.print_solution()
            break


        algo.clear()
        total_action_num.append(len(actions))
        total_state_num.append(len(states))
        total_cost.append(current_cost)
        total_tick.append(current_tick_time)

    print("success:",success_count,"failure:",failure_count)
    print("Total Tree Size: mean=",np.mean(total_tree_size), "std=",np.std(total_tree_size, ddof=1))
    print("Total Steps Num: mean=",np.mean(total_steps_num),"std=",np.std(total_steps_num,ddof=1))
    print("Average Number of States:",np.mean(total_state_num))
    print("Average Number of Actions",np.mean(total_action_num))
    print("Planning Time Total:",planning_time_total,planning_time_total/1000.0)
    print("Average Number of Ticks", np.mean(total_tick),"std=",np.std(total_tick,ddof=1))
    print("Average Cost of Execution:", np.mean(total_cost),"std=",np.std(total_cost,ddof=1))
    if bt_algo_opt:
        print("============= End OptBT Test ==============")
    else:
        print("============= End XiaoCai BT Test ==============")




def BTTest(bt_algo_opt=True,seed=1,literals_num=10,depth=10,iters=10,total_count=1):


    random.seed(seed)
    literals_num=literals_num
    depth = depth
    iters= iters
    total_tree_size = []
    total_action_num = []
    total_state_num = []
    total_steps_num=[]
    total_cost=[]
    total_tick=[]
    #fail_count=0
    #danger_count=0
    success_count =0
    failure_count = 0
    planning_time_total = 0.0

    error = False

    for count in range (total_count):

        action_num = 1

        states = []
        actions = []
        start = generate_random_state(literals_num)
        state = copy.deepcopy(start)
        states.append(state)
        for i in range (0,depth):
            a = Action()
            a.generate_from_state(state,literals_num)
            a.cost = random.randint(1, 100)
            if not a in actions:
                a.name = "a"+str(action_num)
                action_num+=1
                actions.append(a)
            state = state_transition(state,a)
            if state in states:
                pass
            else:
                states.append(state)

        goal = states[-1]
        state = copy.deepcopy(start)
        for i in range (0,iters):
            a = Action()
            a.generate_from_state(state,literals_num)
            if not a in actions:
                a.name = "a"+str(action_num)
                action_num+=1
                actions.append(a)
            state = state_transition(state,a)
            if state in states:
                pass
            else:
                states.append(state)
            state = random.sample(states,1)[0]


        if bt_algo_opt:
            algo = OptBTExpAlgorithm(verbose=False)
        else:
            algo = BTExpAlgorithm(verbose=False)
        algo.clear()

        start_time = time.time()
        if count ==  0 :
            print_action_data_table(goal, start, list(actions))
        if algo.run_algorithm_test(start, goal, actions):
            total_tree_size.append( algo.bt.count_size()-1)
        else:
            print ("error")
        end_time = time.time()
        planning_time_total += (end_time-start_time)


        state=start
        steps=0
        current_cost = 0
        current_tick_time=0
        val, obj, cost, tick_time = algo.bt.cost_tick(state,0,0)

        current_tick_time+=tick_time
        current_cost += cost
        while val !='success' and val !='failure':
            print(state, obj)
            state = state_transition(state,obj)
            val, obj,cost, tick_time = algo.bt.cost_tick(state,0,0)

            current_cost += cost
            current_tick_time += tick_time
            if(val == 'failure'):
                print("bt fails at step",steps)
                error = True
                break
            steps+=1
            if(steps>=500):
                break
        if not goal <= state:
            failure_count+=1
            error = True
        else:
            success_count+=1
            total_steps_num.append(steps)
        if error:
            print_action_data_table(goal, start, list(actions))
            algo.print_solution()
            break

        print("step:",steps)
        algo.clear()
        total_action_num.append(len(actions))
        total_state_num.append(len(states))
        total_cost.append(current_cost)
        total_tick.append(current_tick_time)

    print("success:",success_count,"failure:",failure_count)
    print("Total Tree Size: mean=",np.mean(total_tree_size), "std=",np.std(total_tree_size, ddof=1))
    print("Total Steps Num: mean=",np.mean(total_steps_num),"std=",np.std(total_steps_num,ddof=1))
    print("Average Number of States:",np.mean(total_state_num))
    print("Average Number of Actions",np.mean(total_action_num))
    print("Planning Time Total:",planning_time_total,planning_time_total/total_count)
    print("Average Number of Ticks", np.mean(total_tick),"std=",np.std(total_tick,ddof=1))
    print("Average Cost of Execution:", np.mean(total_cost),"std=",np.std(total_cost,ddof=1))
    if bt_algo_opt:
        print("============= End OptBT Test ==============")
    else:
        print("============= End XiaoCai BT Test ==============")



def get_act_start_goal(seed=1,literals_num=10,depth=10,iters=10,total_count=1000):
        act_list=[]
        start_list=[]
        goal_list=[]

        for count in range(total_count):
            action_num=1
            states = []
            actions = []
            start = generate_random_state(literals_num)
            state = copy.deepcopy(start)
            states.append(state)
            for k in range(int(iters/5)):
                state = copy.deepcopy(start)
                for i in range(0, depth):
                    a = Action()
                    a.generate_from_state(state, literals_num)
                    a.cost = random.randint(1, 100)
                    if not a in actions:
                        a.name = "a" + str(action_num)
                        action_num += 1
                        actions.append(a)
                    state = state_transition(state, a)
                    if state in states:
                        pass
                    else:
                        states.append(state)

            goal = states[-1]
            state = copy.deepcopy(start)
            for i in range(0, int(iters/5)):
                a = Action()
                a.generate_from_state(state, literals_num)
                if not a in actions:
                    a.name = "a" + str(action_num)
                    action_num += 1
                    actions.append(a)
                state = state_transition(state, a)
                if state in states:
                    pass
                else:
                    states.append(state)
                state = random.sample(states, 1)[0]

            act_list.append(actions)
            start_list.append(start)
            goal_list.append(goal)
        return act_list, start_list, goal_list


def cal_tree_cond_tick(start,goal,bt):


    state = start
    error=False
    current_cost = 0
    current_cond_tick_time = 0

    val, obj, cost, tick_time, cond_times = bt.cost_tick_cond(state, 0, 0, 0)

    current_cond_tick_time += cond_times
    current_cost += cost
    while val != 'success' and val != 'failure':
        state = state_transition(state, obj)
        val, obj, cost, tick_time, cond_times = bt.cost_tick_cond(state, 0, 0, 0)
        current_cost += cost
        current_cond_tick_time += cond_times
        if (val == 'failure'):
            error = True
            break
    if not goal <= state:
        error = True
    if error:
        print("Merge Error")
    return error,current_cost,current_cond_tick_time


def BTTest_act_start_goal(bt_algo_opt,act_list,start_list,goal_list,literals_num=None):

    if bt_algo_opt:
        print("============= OptBT Test ==============")
    else:
        print("============= XiaoCai BT Test ==============")


    total_tree_size = []
    total_action_num = []
    total_state_num = []
    total_steps_num=[]
    total_cost=[]
    total_tick=[]
    success_count =0
    failure_count = 0
    planning_time_total = 0.0
    planning_time_ls=[]

    total_fot_times=[]
    total_expand_conds=[]

    total_cond_tick = []
    total_cond_tick_without_merge=[]
    total_cost_without_merge=[]

    error = False


    for count, (actions, start, goal) in enumerate(zip(act_list, start_list, goal_list)):


        if count % 50 == 0:
            print(count)

        states=[]

        state = copy.deepcopy(start)
        states.append(state)



        if bt_algo_opt:

            algo = OptBTExpAlgorithm(verbose=False)
        else:
            algo = BTExpAlgorithm(verbose=False)
        algo.clear()


        start_time = time.time()
        algo_right = algo.run_algorithm_test(start, goal, actions)
        end_time = time.time()
        planning_time_total += (end_time - start_time)
        planning_time_ls.append(end_time - start_time)
        planning_time_total += (end_time - start_time)

        if algo_right:
            total_tree_size.append(algo.tree_size)
            total_state_num.append(algo.traversed_state_num)
            total_fot_times.append(algo.fot_times)
            total_expand_conds.append(algo.expand_conds)
        else:
            print("error")



        state=start
        steps=0
        current_cost = 0
        current_tick_time=0
        current_cond_tick_time = 0
        val, obj, cost, tick_time, cond_times = algo.bt.cost_tick_cond(state, 0, 0, 0)

        current_tick_time+=tick_time
        current_cond_tick_time += cond_times
        current_cost += cost
        while val !='success' and val !='failure':
            state = state_transition(state,obj)

            val, obj, cost, tick_time, cond_times = algo.bt.cost_tick_cond(state, 0, 0, 0)
            current_cost += cost
            current_tick_time += tick_time
            current_cond_tick_time += cond_times
            if(val == 'failure'):
                print("bt fails at step",steps)
                error = True
                break
            steps+=1
            if(steps>=500):
                break
        if not goal <= state:
            failure_count+=1
            error = True
        else:
            success_count+=1
            total_steps_num.append(steps)
        if error:
            print_action_data_table(goal, start, list(actions))
            algo.print_solution()
            break

        if bt_algo_opt:

            merge_error,merge_cost,merge_cond_tick_times = cal_tree_cond_tick(start,goal,algo.bt_without_merge)
            total_cond_tick_without_merge.append(merge_cond_tick_times)
            total_cost_without_merge.append(merge_cost)


        algo.clear()
        total_cost.append(current_cost)
        total_tick.append(current_tick_time)
        total_cond_tick.append(current_cond_tick_time)

    print("success:", success_count, "failure:", failure_count)
    print("Total Steps Num: mean=", np.mean(total_steps_num), "std=", np.std(total_steps_num, ddof=1))

    print("*** Average total_for_times:", round(np.mean(total_fot_times),2), round(np.std(total_fot_times),2))

    print("Planning Time Total:", planning_time_total)
    print("*** Expanded Conds: mean=", round(np.mean(total_expand_conds),2), "std=", round(np.std(total_expand_conds, ddof=1),2))
    print("*** Tree Size: mean=", round(np.mean(total_tree_size),2), "std=", round(np.std(total_tree_size, ddof=1),2))
    print("*** Planning Time mean=:",  round(np.mean(planning_time_ls),4), "std=", round(np.std(planning_time_ls),4))
    print("*** Ticks:", round(np.mean(total_tick),3), "std=", round(np.std(total_tick, ddof=1),3))
    print("*** Cond Ticks:", round(np.mean(total_cond_tick), 3), "std=", round(np.std(total_cond_tick, ddof=1), 3))
    print("*** Average Cost of Execution:", round(np.mean(total_cost),3), "std=", round(np.std(total_cost, ddof=1),3))
    if bt_algo_opt:
        print("---------------------------------------")
        print("*** Withour Merge avg Cost:", round(np.mean(total_cost_without_merge), 3), "std=", round(np.std(total_cost_without_merge, ddof=1), 3))
        print("*** Withour Merge Cond Ticks :", round(np.mean(total_cond_tick_without_merge), 3), "std=", round(np.std(total_cond_tick_without_merge, ddof=1), 3))
    expand_state_num = [round(np.mean(total_expand_conds), 3), round(np.std(total_expand_conds), 3)]
    tree_size = [round(np.mean(total_tree_size), 3), round(np.std(total_tree_size, ddof=1), 3)]
    ticks = [round(np.mean(total_tick), 3), round(np.std(total_tick, ddof=1), 3)]
    cond_ticks = [round(np.mean(total_cond_tick), 3), round(np.std(total_cond_tick, ddof=1), 3)]
    cost = [round(np.mean(total_cost), 3), round(np.std(total_cost, ddof=1), 3)]
    step = [round(np.mean(total_steps_num), 3), round(np.std(total_steps_num, ddof=1), 3)]
    state_num = [round(np.mean(total_state_num),3), round(np.std(total_state_num),3)]
    for_num = [round(np.mean(total_fot_times),2), round(np.std(total_fot_times),2)]
    plan_time = [round(np.mean(planning_time_ls), 5), round(np.std(planning_time_ls), 5), round(planning_time_total, 5)]


    if bt_algo_opt:
        wm_cond_ticks = [round(np.mean(total_cond_tick_without_merge), 3),round(np.std(total_cond_tick_without_merge), 3)]
    else:
        wm_cond_ticks=[0,0]



    tmp_ls=[]
    tmp_ls.extend(tree_size)
    tmp_ls.extend(ticks)
    tmp_ls.extend(wm_cond_ticks)
    tmp_ls.extend(cond_ticks)
    tmp_ls.extend(cost)
    tmp_ls.extend(step)

    tmp_ls.extend(expand_state_num)
    tmp_ls.extend(for_num)
    tmp_ls.extend(plan_time)
    return tmp_ls


    if bt_algo_opt:
        print("============= End OptBT Test ==============")
    else:
        print("============= End XiaoCai BT Test ==============")

