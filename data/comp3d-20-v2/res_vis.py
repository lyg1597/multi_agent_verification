import pyvista as pv
import polytope as pc 
import numpy as np 
import pickle 
import json 
from plot_polytope3d_vista import *

def res_vis():
    with open('agent_plan','rb') as f:
        agent_plan = pickle.load(f)

    with open('agent_state','rb') as f:
        agent_state = pickle.load(f)

    with open('agent_tube', 'rb') as f:
        agent_tube, agent_tube_cached, agent_tube_plan, agent_tube_time, agent_tube_result = pickle.load(f)

    with open('comp3d-20-v1.json', 'r') as f:
        scenario_config = json.load(f)

    step = 20

    p = pv.Plotter()

    # Plot obstacles 
    obstacle_list = scenario_config['unsafeSet']
    for obstacle in obstacle_list:
        obstacle_type = obstacle[0]
        if obstacle_type == "Matrix":
            obstacle_A = np.array(obstacle[1][0])
            obstacle_b = np.array(obstacle[1][1])
            plot_polytope_3d(obstacle_A[:-6,:3], obstacle_b[:-6,:], p, color = "#d9d9d9", trans=1)
    A_rect = np.array([[-1,0,0],
                        [1,0,0],
                        [0,-1,0],
                        [0,1,0],
                        [0,0,-1],
                        [0,0,1]])
    b = np.array([[50], [300], [50], [300], [2], [0]])
    plot_polytope_3d(A = A_rect, b = b, ax = p, color = '#d9d9d9', trans=1, edge=True)
    
    for agent_config in scenario_config['agents']:
        # mode_list = agent_config['mode_list']
        # for i in range(1,len(mode_list)):
        #     mode = mode_list[i]
        #     plot_line_3d(mode[1][:3], mode[1][3:], ax = p, line_width = 10, color = color[j])
        
        initialSet = agent_config['initialSet']
        poly = pc.box2poly(np.array(initialSet[1])[:,:3].T)
        plot_polytope_3d(A = poly.A, b = poly.b, ax = p, color = 'b')

        goalSet = agent_config['goalSet']
        poly = pc.box2poly(np.array(goalSet[1])[:,:3].T)
        plot_polytope_3d(A = poly.A, b = poly.b, ax = p, color = 'g')


    for key in agent_tube:
        print(f'plotting tube {key}')
        tube_list  = agent_tube[key]
        for i,tube in enumerate(tube_list):
            res = agent_tube_result[key][i]
            if res == 1:
                color = "#ffed6f"
            else:
                color = "#fb8072"
            for i in range(0, len(tube), step):
                box = tube[i]
                poly = pc.box2poly(np.array(box)[:,:3].T)
                A = poly.A 
                b = poly.b
                plot_polytope_3d(A, b, p, color = color, trans=1)
        
    p.show()

if __name__ == "__main__":
    res_vis()