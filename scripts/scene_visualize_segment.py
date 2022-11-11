import json
import numpy as np
import matplotlib.pyplot as plt
import polytope as pc
import pypoman as ppm
import os 

def get_next_mode_idx(mode_parameters, mode_list):
    idx = -1
    for i, mode in enumerate(mode_list):
        if mode[1][:2] == mode_parameters[2:]:
            return i
    return -1

def scene_visualize(fn, title, x_lim, y_lim):
    f = open(fn,'r')
    config = json.load(f)

    unsafe_set = []
    if "unsafeSet" in config:
        unsafe_set = config["unsafeSet"]
    
    goal_set = []
    if "goalSet" in config:
        goal_set = config["goalSet"]

    # mode_list = config["agents"][0]["mode_list"]

    # initial_set = config["agents"][0]["initialSet"]

    # edge_list = config["agents"][0]["edge_list"]

    # time_list = config["agents"][0]["timeHorizons"]

    for i in range(len(unsafe_set)):
        if unsafe_set[i][0] == "Box":
            box = unsafe_set[i][1]
            poly = pc.box2poly(np.array(box).T)
        elif unsafe_set[i][0] == "Matrix":
            mats = unsafe_set[i][1]
            poly = pc.Polytope(np.array(mats[0]),np.array(mats[1]))
        elif unsafe_set[i][0] == "Vertices":
            vertices =  np.array(unsafe_set[i][1])
            if vertices.size == 0:
                continue
            poly = pc.qhull(np.array(vertices))  
        poly = pc.projection(poly,[1,2])
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#d9d9d9', alpha = 1)

    # for i in range(len(goal_set)):
    #     box = goal_set[i][1]
    #     poly = pc.box2poly(np.array(box).T)
    #     poly = pc.projection(poly,[1,2])
    #     ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#8dd3c7')

    agent_list = config['agents']
    for i, agent in enumerate(agent_list):
        print(f"plotting plan for agent {i}")
        mode_list = agent['mode_list']
        init_mode_id = agent['initialModeID']
        init_box = agent['initialSet'][1]
        poly = pc.box2poly(np.array(init_box).T)
        poly = pc.projection(poly,[1,2])
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = 'blue')
        idx = init_mode_id 
        mode_parameters = []
        # while idx>=0:
        #     # print(idx)
        #     mode_parameters = mode_list[idx][1]
        #     plt.plot([mode_parameters[0],mode_parameters[2]],[mode_parameters[1],mode_parameters[3]],'k')
        #     idx = get_next_mode_idx(mode_parameters, mode_list)

        # for mode in mode_list[1:]:
        mode = mode_list[50]
        mode_parameters = mode[1]
        plt.plot([mode_parameters[0],mode_parameters[2]],[mode_parameters[1],mode_parameters[3]],'k')
            # idx = get_next_mode_idx(mode_parameters, mode_list)
    

        # goal_state = mode_parameters[2:]
        # goal_box = [[goal_state[0]-3, goal_state[1]-3], [goal_state[0]+3, goal_state[1]+3]]            
        # poly = pc.box2poly(np.array(goal_box).T)
        # poly = pc.projection(poly,[1,2])
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = 'green')
        #     
        # for i in range(len(mode_list)):
        #     mode = mode_list[i][1]
        #     if len(mode) == 4:
        #         # plt.plot([mode[0],mode[2]],[mode[1],mode[3]],'k.')
        #         plt.plot([mode[0],mode[2]],[mode[1],mode[3]],'k')
        #     elif len(mode) == 6:
        #         # plt.plot([mode[0],mode[3]],[mode[1],mode[4]],'k.')
        #         plt.plot([mode[0],mode[3]],[mode[1],mode[4]],'k')
        
        # box = initial_set[1]
        # poly = pc.box2poly(np.array(box).T)
        # poly = pc.projection(poly,[1,2])
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#80b1d3', alpha = 1, linewidth=1)
                
        # print("Number of modes:", len(mode_list))
        # print("Number of edges:", len(edge_list))
        # print("Number of obsatles:", len(unsafe_set))
        # print("Total time horizon:", sum(time_list))

    # A_rect = np.array([[-1,0],
    #                    [1,0],
    #                    [0,-1],
    #                    [0,1]])

    # b_goal1 = np.array([[-240-15],  [240+21], [-3], [9]])
    # b_goal2 = np.array([[-240-39],  [240+45], [-3], [9]])
    # b_goal3 = np.array([[-240-87],  [240+93], [-3], [9]])
    # b_goal4 = np.array([[-240-3],   [240+9], [-15], [21]])
    # b_goal5 = np.array([[-240-63],  [240+69], [-27], [33]])
    # b_goal6 = np.array([[-240-39],  [240+45], [-39], [45]])
    # b_goal7 = np.array([[-240-27],  [240+33], [-63], [69]])
    # b_goal8 = np.array([[-240-63],  [240+69], [-63], [69]])
    # b_goal9 = np.array([[-240-3],   [240+9], [-87], [93]])
    # b_goal10 = np.array([[-240-87],  [240+93], [-87], [93]])
    # b_goal11 = np.array([[-240-111], [240+117], [-3], [9]])
    # b_goal12 = np.array([[-240-111], [240+117], [-39], [45]])
    # b_goal13 = np.array([[-240-111], [240+117], [-75], [81]])
    # b_goal14 = np.array([[-240-3],   [240+9], [-111], [117]])
    # b_goal15 = np.array([[-240-39],  [240+45], [-111], [117]])
    # b_goal16 = np.array([[-240-75],  [240+81], [-111], [117]])
    # b_goal17 = np.array([[-240-111], [240+117], [-111], [117]])

    # b1_goal1 = np.array([[-360-15],  [360+21], [-3], [9]])
    # b1_goal2 = np.array([[-360-39],  [360+45], [-3], [9]])
    # b1_goal3 = np.array([[-360-87],  [360+93], [-3], [9]])
    # b1_goal4 = np.array([[-360-3],   [360+9], [-15], [21]])
    # b1_goal5 = np.array([[-360-63],  [360+69], [-27], [33]])
    # b1_goal6 = np.array([[-360-39],  [360+45], [-39], [45]])
    # b1_goal7 = np.array([[-360-27],  [360+33], [-63], [69]])
    # b1_goal8 = np.array([[-360-63],  [360+69], [-63], [69]])
    # b1_goal9 = np.array([[-360-3],   [360+9], [-87], [93]])
    # b1_goal10 = np.array([[-360-87],  [360+93], [-87], [93]])
    # b1_goal11 = np.array([[-360-111], [360+117], [-3], [9]])
    # b1_goal12 = np.array([[-360-111], [360+117], [-39], [45]])
    # b1_goal13 = np.array([[-360-111], [360+117], [-75], [81]])
    # b1_goal14 = np.array([[-360-3],   [360+9], [-111], [117]])
    # b1_goal15 = np.array([[-360-39],  [360+45], [-111], [117]])
    # b1_goal16 = np.array([[-360-75],  [360+81], [-111], [117]])
    # b1_goal17 = np.array([[-360-111], [360+117], [-111], [117]])

    # goals = [(A_rect, b_goal1),
    #         (A_rect, b_goal2),
    #         (A_rect, b_goal3),
    #         (A_rect, b_goal4),
    #         (A_rect, b_goal5),
    #         (A_rect, b_goal6),
    #         (A_rect, b_goal7),
    #         (A_rect, b_goal8),
    #         (A_rect, b_goal9),
    #         (A_rect, b_goal10),
    #         (A_rect, b_goal11),
    #         (A_rect, b_goal12),
    #         (A_rect, b_goal13),
    #         (A_rect, b_goal14),
    #         (A_rect, b_goal15),
    #         (A_rect, b_goal16),
    #         (A_rect, b_goal17),
    #         (A_rect, b1_goal1),
    #         (A_rect, b1_goal2),
    #         (A_rect, b1_goal3),
    #         (A_rect, b1_goal4),
    #         (A_rect, b1_goal5),
    #         (A_rect, b1_goal6),
    #         (A_rect, b1_goal7),
    #         (A_rect, b1_goal8),
    #         (A_rect, b1_goal9),
    #         (A_rect, b1_goal10),
    #         (A_rect, b1_goal11),
    #         (A_rect, b1_goal12),
    #         (A_rect, b1_goal13),
    #         (A_rect, b1_goal14),
    #         (A_rect, b1_goal15),
    #         (A_rect, b1_goal16),
    #         (A_rect, b1_goal17)]

    # for g in goals:
    #     ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(g[0],g[1]), color = 'green')


    plt.xlim(x_lim[0],x_lim[1])
    plt.ylim(y_lim[0],y_lim[1])
    # plt.title("title")
    plt.show()

if __name__ == "__main__":
    fn = "scenario_hscc/map2-2d-17.json"
    fn = os.path.abspath(fn)
    title = "scene_complex2"
    x_lim = [-6,486]
    y_lim = [-6,126]
    scene_visualize(fn, title, x_lim, y_lim)
