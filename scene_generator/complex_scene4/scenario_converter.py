# from demo import problem
import json
import numpy as np
import importlib 
import sys
from rrt import *
import polytope as pc

def convert_to_json(fn = './scene.json',obs = [], init = [], edges = {}, trans_res = 0.01):
    f = open(fn,'w+')
    data = {
        "reachability_engine":"",
        "grid_resolution": [0.5, 0.5, 0.5],
        "time_step": 0.1,
        "unsafeSet":[],
        "symmetry_level":"",
        "agents":[{
            "variables":[],
            "initialSet":[],
            "timeHorizons":[],
            "mode_list":[],
            "edge_list":[],
            "guards":[],
            "directory":"examples/models/NN_car",
            "initialModeID": -1
        }],
    }
    initial_set = []
    if init != None and init != []:
        initial_set = initial_set + ['Box']
        init_A = np.array(init[0])
        init_b = np.array(init[1])
        
        lower = []
        upper = []

        for i in range(init_A.shape[0]//2):
            tmp1 = float(init_A[i*2,i]*init_b[i*2,0])
            tmp2 = float(init_A[i*2+1,i]*init_b[i*2+1,0])
            if tmp1<tmp2:
                lower.append(tmp1)
                upper.append(tmp2)
            else:
                lower.append(tmp2)
                upper.append(tmp1)
        initial_set = initial_set + [[lower,upper]]
    data['agents'][0]['initialSet'] = initial_set

    unsafe_set_list = []
    for obstecle in obs:
        unsafe_set = ['Box']
        obstecle_A = np.array(obstecle[0])
        obstecle_b = np.array(obstecle[1])
        lower = []
        upper = []
        for i in range(obstecle_A.shape[0]//2):
            tmp1 = float(obstecle_A[i*2,i]*obstecle_b[i*2,0])
            tmp2 = float(obstecle_A[i*2+1,i]*obstecle_b[i*2+1,0])
            if tmp1<tmp2:
                lower.append(tmp1)
                upper.append(tmp2)
            else:
                lower.append(tmp2)
                upper.append(tmp1)
        unsafe_set = unsafe_set + [[lower,upper]]
        unsafe_set_list.append(unsafe_set)
    data['unsafeSet'] = unsafe_set_list

    start_poly = pc.Polytope(init[0],init[1])
    start_pt = list(start_poly.chebXc)
    time_horizon = [10]
    init_mode = ["follow_waypoint",[0,0]+start_pt]
    mode_list = [init_mode]
    edge_dict = {tuple(start_pt):0}
    for key in edges:
        edge = edges[key]
        mode = ["follow_waypoint",edge.start+edge.end]
        mode_list.append(mode)
        edge_dict[tuple(edge.end)] = len(mode_list)-1
        time_horizon.append(10)
    data['agents'][0]['mode_list'] = mode_list
    data['agents'][0]['timeHorizons'] = time_horizon

    guard_list = []
    transition_list = []
    for key in edges:
        edge = edges[key]
        if tuple(edge.start) in edge_dict:
            src_idx = edge_dict[tuple(edge.start)]
            dest_idx = edge_dict[tuple(edge.end)] 
            transition_list.append(tuple([src_idx,dest_idx]))
            guard = ["Box",[[edge.start[0]-trans_res,edge.start[1]-trans_res,-6.28],[edge.start[0]+trans_res,edge.start[1]+trans_res,6.28]]]
            guard_list.append(guard)
    data['agents'][0]['guards'] = guard_list
    data['agents'][0]['edge_list'] = transition_list

    json.dump(data,f)

def run_planner(obs, init, goal, search_area, color = 'b'):
    A_start = init[0]
    b_start = init[1]
    # A_end = goal[0]
    # b_end = goal[1]

    rrt = RRT(start=[A_start, b_start],
                goal_list=goal,
                rand_area=search_area,
                obstacle_list=obs,
                expand_dis=[3], max_iter = 30000, color = color)
    path,edges = rrt.planning(animation = True)
    if len(path)!=0:
        return path, edges
    else:
        return [],{}

def bloat_scene(obs, init, goal, factor):
    tmp_init = init
    # init_b = init[1]
    # init_b = init_b - factor
    # init = (init[0],init_b)

    tmp_obs = []
    for A,b in obs:
        b = b + factor
        tmp_obs.append((A,b))

    tmp_goal = goal
    # tmp_goal = []
    # for A,b in goal:
    #     b = b - factor
    #     tmp_goal.append((A,b))


    return tmp_obs, init, tmp_goal

if __name__ == "__main__":
    # input_fn = sys.argv[1]
    # output_fn = sys.argv[2]
    input_fn = 'complex3'
    output_fn = 'scene_complex.json'

    search_area = [1,239,1,119]
    # fp, path, desc = importlib.find_module(input_fn) 
    scene = importlib.import_module(input_fn) 
    obs, init, goal = scene.problem()

    obs_bloated, init_bloated, goal_bloated = bloat_scene(obs,init,goal,1)
    path_list = []
    edge_list = {}
    color = ['b','g','r']
    for i in range(1):
        paths,edges = run_planner(obs_bloated, init_bloated, goal_bloated, search_area,color[i])
        path_list = path_list + paths
        edge_list.update(edges)        
    
    plt.figure()

    for A,b in obs:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')

    for g in goal:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(g[0],g[1]), color = 'green')

    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(init[0],init[1]), color = 'blue')
    plt.xlim(0, 240)
    plt.ylim(0, 120)
    plt.grid()

    # for path in path_list:
    #     path = np.array(path)
    #     plt.plot(path[:,0],path[:,1])
    for key in edge_list:
        edge = edge_list[key]
        plt.plot([edge.start[0],edge.end[0]],[edge.start[1],edge.end[1]],edge.color)
    # for i in range(len(path)):
    #     plt.plot(path[i][0],path[i][1])
    plt.show()

    convert_to_json(fn = output_fn, obs = obs, init = init, edges = edge_list,trans_res=0.25)