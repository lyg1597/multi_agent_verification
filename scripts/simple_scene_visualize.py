import numpy as np
import matplotlib.pyplot as plt
import polytope as pc
import pypoman as ppm
import os 
import copy 

def simple_scene_visualize(unsafe_set, agent_plan_list):
    for i in range(len(unsafe_set)):
        box = unsafe_set[i]
        poly = pc.box2poly(np.array(box).T)
        poly = pc.projection(poly,[1,2])
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#d9d9d9', alpha = 1)
 
    for agent_plan in agent_plan_list:
        init_box = [[agent_plan[0][0]-1, agent_plan[0][1]-1],[agent_plan[0][0]+1, agent_plan[0][1]+1]]
        goal_box = [[agent_plan[-1][2]-0.5, agent_plan[-1][3]-0.5],[agent_plan[-1][2]+0.5, agent_plan[-1][3]+0.5]]
        init_poly = pc.box2poly(np.array(init_box).T)
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(init_poly.A,init_poly.b), color = 'blue')
        goal_poly = pc.box2poly(np.array(goal_box).T)
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(goal_poly.A,goal_poly.b), color = 'green')

        for plan in agent_plan:
            plt.plot([plan[0], plan[2]], [plan[1], plan[3]], 'k')
    plt.xlim(15,100)
    plt.ylim(3,22)
    plt.show()

if __name__ == "__main__":
    wp_list = []
    unsafeset_list = []

    num_agents = 5
        
    raw_wp_list = [
        [20.0, 5.0, 20.0, 10.0],
        [20.0, 10.0, 20.0, 15.0],
        [20.0, 15.0, 25.0, 15.0],
        [25.0, 15.0, 30.0, 15.0],
        [30.0, 15.0, 35.0, 15.0],
        [35.0, 15.0, 35.0, 20.0],
    ]
    raw_unsafeset_list = [
        ["Box", [[23,5,-100],[27,12,100]]],
        ["Box", [[35.8,18,-100],[43,20,100]]],
    ]

    # raw_wp_list = [
    #     [0, 0, -5, 0],
    #     [-5, 0, -10, 0],
    #     [-10, 0, -10, 5],
    #     [-10, 5, -15, 5],
    #     [-15, 5, -15, 0],
    #     [-15, 0, -15, -5]
    # ]
    # raw_unsafeset_list = [
    #     [[-10, -5, -100],[0,-3,100]],
    # ]
    
    for i in range(num_agents):
        wp1 = []
        for j in range(len(raw_wp_list)):
            raw_wp = copy.deepcopy(raw_wp_list[j])
            raw_wp[0] += i*12
            raw_wp[2] += i*12
            wp1.append(raw_wp)
        for j in range(len(raw_unsafeset_list)):
            raw_unsafeset = copy.deepcopy(raw_unsafeset_list[j][1])
            raw_unsafeset[0][0] += i*12
            raw_unsafeset[1][0] += i*12
            unsafeset_list.append(raw_unsafeset)
        wp_list.append(wp1)

    simple_scene_visualize(unsafeset_list, wp_list)