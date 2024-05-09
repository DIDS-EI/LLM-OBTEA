#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# enconding = utf8
import sys
import time
import grpc

# import camera
from robowaiter.proto import camera

sys.path.append('./')
sys.path.append('../')

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid1 import make_axes_locatable

import GrabSim_pb2_grpc
import GrabSim_pb2

channel = grpc.insecure_channel('localhost:30001', options=[
    ('grpc.max_send_message_length', 1024 * 1024 * 1024),
    ('grpc.max_receive_message_length', 1024 * 1024 * 1024)
])

sim_client = GrabSim_pb2_grpc.GrabSimStub(channel)



def Init():
    sim_client.Init(GrabSim_pb2.NUL())




def AcquireAvailableMaps():
    AvailableMaps = sim_client.AcquireAvailableMaps(GrabSim_pb2.NUL())
    print(AvailableMaps)





def SetWorld(map_id=0, scene_num=1):
    print('------------------SetWorld----------------------')
    world = sim_client.SetWorld(GrabSim_pb2.BatchMap(count=scene_num, mapID=map_id))




def Observe(scene_id=0):
    print('------------------show_env_info----------------------')
    scene = sim_client.Observe(GrabSim_pb2.SceneID(value=scene_id))
    print(
        f"location:{[scene.location]}, rotation:{scene.rotation}\n",
        f"joints number:{len(scene.joints)}, fingers number:{len(scene.fingers)}\n",
        f"objects number: {len(scene.objects)}, walkers number: {len(scene.walkers)}\n"
        f"timestep:{scene.timestep}, timestamp:{scene.timestamp}\n"
        f"collision:{scene.collision}, info:{scene.info}")





def Reset(scene_id=0):
    print('------------------Reset----------------------')
    scene = sim_client.Reset(GrabSim_pb2.ResetParams(scene=scene_id))
    print(scene)



def navigation_move(cur_objs, scene_id=0, map_id=0):
    print('------------------navigation_move----------------------')
    scene = sim_client.Observe(GrabSim_pb2.SceneID(value=scene_id))
    walk_value = [scene.location.X, scene.location.Y, scene.rotation.Yaw]
    print("position:", walk_value)
    objs_name_set = set()

    if map_id == 11:  # coffee
        v_list = [[247,520], [247, 700], [270, 1100], [55, 940], [30, 900], [30, 520], [160, -165], [247, 0],[247, 520]]
    else:
        v_list = [[0.0, 0.0]]

    for walk_v in v_list:
        walk_v = walk_v + [scene.rotation.Yaw - 90, 200, 10]
        print("walk_v", walk_v)
        action = GrabSim_pb2.Action(scene=scene_id, action=GrabSim_pb2.Action.ActionType.WalkTo, values=walk_v)
        scene = sim_client.Do(action)
        cur_objs, objs_name_set = camera.get_semantic_map(GrabSim_pb2.CameraName.Head_Segment, cur_objs, objs_name_set)
        print(scene.info)
    return cur_objs


if __name__ == '__main__':
    map_id = 11
    scene_num = 1
    cur_objs = []


    Init()
    AcquireAvailableMaps()
    SetWorld(map_id, scene_num)
    time.sleep(5.0)

    for i in range(scene_num):
        Observe(i)
        Reset(i)

        cur_objs = navigation_move(cur_objs, i, map_id)
        print(cur_objs)
