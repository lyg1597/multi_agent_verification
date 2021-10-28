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
        agent_tube, agent_tube_cached, agent_tube_plan = pickle.load(f)

    with open('comp3d-20.json', 'r') as f:
        scenario_config = json.load(f)

    step = 10

    p = pv.Plotter()

    # Plot obstacles 
    obstacle_list = scenario_config['unsafeSet']
    for obstacle in obstacle_list:
        obstacle_type = obstacle[0]
        if obstacle_type == "Matrix":
            obstacle_A = np.array(obstacle[1][0])
            obstacle_b = np.array(obstacle[1][1])
            plot_polytope_3d(obstacle_A[:-6,:3], obstacle_b[:-6,:], p, color = "#d9d9d9", trans=1)

    for key in agent_tube:
        print(f'plotting tube {key}')
        tube_list  = agent_tube[key]
        for tube in tube_list:
            for i in range(0, len(tube), step):
                box = tube[i]
                poly = pc.box2poly(np.array(box)[:,:3].T)
                A = poly.A 
                b = poly.b
                plot_polytope_3d(A, b, p, color = "#ffed6f", trans=1)
        
    p.show()

if __name__ == "__main__":
    res_vis()