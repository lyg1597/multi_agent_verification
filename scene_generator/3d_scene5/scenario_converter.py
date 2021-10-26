# from demo import problem
import json
import numpy as np
import importlib 
import sys
from rrt_3d import *
import polytope as pc
from plot_polytope3d import *
import copy

def convert_to_json_3D(fn = './scene.json',obs = [], init_list = [], goal = [], edge_list = {}, trans_res = 0.01):
    f = open(fn,'w+')
    data = {
        "reachability_engine":"",
        "grid_resolution": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        "time_step": 0.1,
        "unsafeSet":[],
        "symmetry_level":"",
        "refine_threshold":0,
        "agents":[],
    }

    agent_template = {
        # "variables":[],
        "initialSet":[],
        # "timeHorizons":[],
        "mode_list":[],
        "edge_list":[],
        # "guards":[],
        # "directory":"examples/models/NN_car",
        "initialModeID": -1
    }

    unsafe_set_list = []
    for obstecle in obs:
        unsafe_set = ['Box']
        obstecle_A = np.array(obstecle[0])
        obstecle_b = np.array(obstecle[1])
        lower = []
        upper = []
        for i in range(obstecle_A.shape[0]//2):
            tmp1 = round(float(obstecle_A[i*2,i]*obstecle_b[i*2,0]),2)
            tmp2 = round(float(obstecle_A[i*2+1,i]*obstecle_b[i*2+1,0]),2)
            if tmp1<tmp2:
                lower.append(tmp1)
                upper.append(tmp2)
            else:
                lower.append(tmp2)
                upper.append(tmp1)
        unsafe_set = unsafe_set + [[lower,upper]]
        unsafe_set_list.append(unsafe_set)
    data['unsafeSet'] = unsafe_set_list

    for j in range(len(init_list)):
        agent = copy.deepcopy(agent_template)
        initial_set = []
        init = init_list[j]
        if init != None and init != []:
            initial_set = initial_set + ['Box']
            init_A = np.array(init[0])
            init_b = np.array(init[1])
            
            lower = []
            upper = []

            for i in range(init_A.shape[0]//2):
                tmp1 = round(float(init_A[i*2,i]*init_b[i*2,0]),2)
                tmp2 = round(float(init_A[i*2+1,i]*init_b[i*2+1,0]),2)
                if tmp1<tmp2:
                    lower.append(tmp1)
                    upper.append(tmp2)
                else:
                    lower.append(tmp2)
                    upper.append(tmp1)
            for i in range(3):
                lower.append(-0.5)
                upper.append(0.5)
            initial_set = initial_set + [[lower,upper]]
        agent['initialSet'] = initial_set

        # goal_set_list = []
        # for g in goal:
        #     goal_set = ['Box']
        #     g_A = np.array(g[0])
        #     g_b = np.array(g[1])
        #     lower = []
        #     upper = []
        #     for i in range(g_A.shape[0]//2):
        #         tmp1 = round(float(g_A[i*2,i]*g_b[i*2,0]),2)
        #         tmp2 = round(float(g_A[i*2+1,i]*g_b[i*2+1,0]),2)
        #         if tmp1<tmp2:
        #             lower.append(tmp1)
        #             upper.append(tmp2)
        #         else:
        #             lower.append(tmp2)
        #             upper.append(tmp1)
        #     goal_set = goal_set + [[lower,upper]]
        #     goal_set_list.append(goal_set)
        # data['goalSet'] = goal_set_list

        start_poly = pc.Polytope(init[0],init[1])
        start_pt = list(start_poly.chebXc)
        start_pt = [round(val,2) for val in start_pt]
        init_mode = ["follow_waypoint",[0,0,0]+start_pt]
        mode_list = [init_mode]
        time_horizon = [10]
        edge_dict = {tuple(start_pt):0}
        edges = edge_list[j]
        for key in edges:
            edge = edges[key]
            start_pt = [round(val,2) for val in edge.start]
            end_pt = [round(val,2) for val in edge.end]
            mode = ["follow_waypoint",start_pt+end_pt]
            mode_list.append(mode)
            time_horizon.append(10)
            edge_dict[tuple(edge.end)] = len(mode_list)-1
        agent['mode_list'] = mode_list
        # data['agents'][0]['timeHorizons'] = time_horizon

        guard_list = []
        transition_list = []
        init_mode_id = 1
        for key in edges:
            edge = edges[key]
            if tuple(edge.start) in edge_dict:
                src_idx = edge_dict[tuple(edge.start)]
                dest_idx = edge_dict[tuple(edge.end)] 
                transition_list.append(tuple([src_idx,dest_idx]))
                guard_lower = [edge.start[0]-trans_res,edge.start[1]-trans_res,edge.start[2]-trans_res]
                guard_lower = [round(val,2) for val in guard_lower]
                guard_lower = guard_lower + [-100,-100,-100]
                guard_upper = [edge.start[0]+trans_res,edge.start[1]+trans_res,edge.start[2]+trans_res]
                guard_upper = [round(val,2) for val in guard_upper]
                guard_upper = guard_upper + [100,100,100]
                guard = ["Box",[guard_lower,guard_upper]]
                guard_list.append(guard)
                if src_idx == 0:
                    init_mode_id = dest_idx
        # data['agents'][0]['guards'] = guard_list
        agent['edge_list'] = transition_list
        agent['initialModeID'] = init_mode_id
        data["agents"].append(copy.deepcopy(agent))

    json.dump(data,f)

def run_planner(obs, init_list, goal_list, search_area, color = 'b'):
    path_list = []
    edge_list = []

    for i in range(len(init_list)):
        init = init_list[i]
        A_start = init[0]
        b_start = init[1]
        # A_end = goal[0]
        # b_end = goal[1]
        goal = [goal_list[i]]
        # print(A_start, b_start)
        rrt = RRT3D(start=[A_start, b_start],
                    goal_list=goal,
                    rand_area=search_area,
                    obstacle_list=obs,
                    # expand_theta= [0,np.pi/3,np.pi/2,2*np.pi/3,np.pi,4*np.pi/3, 3*np.pi/2, 5*np.pi/3],
                    expand_dis=[3], max_iter = 30000, color = color)
        path,edges = rrt.planning(animation = True)
        
        if len(path)!=0:
            path_list.append(path)
            edge_list.append(edges)

    return path_list, edge_list 

def bloat_scene(obs, init, goal, factor):
    # init_b = init[1]
    # init_b = init_b - factor
    # init = (init[0],init_b)

    tmp_obs = []
    for A,b in obs:
        b = b + factor
        tmp_obs.append((A,b))

    # tmp_goal = []
    # for A,b in goal:
    #     b = b - factor
    #     tmp_goal.append((A,b))
    tmp_goal = goal


    return tmp_obs, init, tmp_goal

if __name__ == "__main__":
    # input_fn = sys.argv[1]
    # output_fn = sys.argv[2]
    input_fn = 'testplot'
    output_fn = 'scene_3d_complex.json'

    search_area = [0, 125, 0, 96, 0, 18]
    # fp, path, desc = importlib.find_module(input_fn) 
    scene = importlib.import_module(input_fn) 
    obs, init, goal = scene.problem()

    obs_bloated, init_bloated, goal_bloated = bloat_scene(obs,init,goal,1.5)
    path_list = []
    edge_list = {}
    color = ['b','g','r']
    # for i in range(1):
    path_list,edge_list = run_planner(obs_bloated, init_bloated, goal_bloated, search_area, color[0])
        # path_list = path_list + paths
        # edge_list.update(edges)        
    
    # fig = plt.figure()
    # axes = fig.add_subplot(111, projection='3d')

    # for A,b in obs:
    #     plot_polytope_3d(A, b, ax = axes, color = 'red')

    # for A,b in goal:
    #     plot_polytope_3d(A, b, ax = axes, color = 'green')

    # plot_polytope_3d(init[0], init[1], ax = axes, color = 'blue')
    # # plt.grid()

    fig = plt.figure()
    for (A,b) in obs:
        vtc = ppm.compute_polytope_vertices(A[0:4,0:2],b[0:4,:])
        ppm.plot_polygon(vtc, color = 'r')
        
    for (A,b) in goal:
        vtc = ppm.compute_polytope_vertices(A[0:4,0:2],b[0:4,:])
        ppm.plot_polygon(vtc, color = 'g')

    for (A,b) in init:
        vtc = ppm.compute_polytope_vertices(A[0:4,0:2],b[0:4,:])
        ppm.plot_polygon(vtc, color = 'b')
    
    # # for path in path_list:
    # #     path = np.array(path)
    # #     plt.plot(path[:,0],path[:,1])
    for edges in edge_list:
        for key in edges:
            edge = edges[key]
            plt.plot([edge.start[0], edge.end[0]],[edge.start[1], edge.end[1]], color = edge.color)
    # # for i in range(len(path)):
    # #     plt.plot(path[i][0],path[i][1])
    # axes.set_xlim(0, 144)
    # axes.set_ylim(0, 144)
    # axes.set_zlim(0, 36)
    plt.show()

    convert_to_json_3D(fn = output_fn, obs = obs, init_list = init, goal = goal, edge_list = edge_list, trans_res = 0.5)
