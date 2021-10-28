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

    with open('comp3d-20-v2.json', 'r') as f:
        scenario_config = json.load(f)

    step = 5

    time_aligned_segments = {}
    time_aligned_plan = {}
    time_val_list = []
    time_aligned_result = {}
    for key in agent_tube_time:
        for i in range(len(agent_tube_time[key])):
            time_val = agent_tube_time[key][i]
            if round(time_val/10)*10 in time_aligned_segments:
                tube = agent_tube[key][i]
                plan = agent_tube_plan[key][i]
                # result = a
                if key in time_aligned_segments[round(time_val/10)*10]:
                    time_aligned_segments[round(time_val/10)*10][key].append(tube)
                else:
                    time_aligned_segments[round(time_val/10)*10][key]= [tube]
                    
                if key in time_aligned_plan[round(time_val/10)*10]:
                    time_aligned_plan[round(time_val/10)*10][key].append(plan)
                else:
                    time_aligned_plan[round(time_val/10)*10][key] = [plan]
            else:
                tube = agent_tube[key][i]
                plan = agent_tube_plan[key][i]
                time_aligned_segments[round(time_val/10)*10] = {}
                time_aligned_segments[round(time_val/10)*10][key] = [tube]
                time_aligned_plan[round(time_val/10)*10] = {}
                time_aligned_plan[round(time_val/10)*10][key] = [plan]
            if round(time_val/10)*10 not in time_val_list:
                time_val_list.append(round(time_val/10)*10)
    time_val_list.sort()
    p = pv.Plotter()


    color_list = ["#fb8072", "#fdb462", "#ffed6f", "#ffffb3"]
    base_val = 200
    for k in range(4):
        time_val_idx = base_val + k*10
        # time_val_idx = 130
        # Plot specific agent tubes
        for j in range(time_val_idx, time_val_idx+10):
            agent = [9, 10, 12, 13, 15, 16]
            time_val = time_val_list[j]
            for agent_idx in agent:
                if agent_idx in time_aligned_segments[time_val]:
                    agent_tube_list = time_aligned_segments[time_val][agent_idx]
                    plan_list = time_aligned_plan[time_val][agent_idx]
                    for i, tube in enumerate(agent_tube_list):
                        plan = plan_list[i]
                        for i in range(0, len(tube), step):
                            box = tube[i]
                            box[0][0] -= 0.5
                            box[0][1] -= 0.5
                            box[0][2] -= 0.5
                            box[1][0] += 0.5
                            box[1][1] += 0.5
                            box[1][2] += 0.5
                            if abs(plan[2] - plan[5]) > 1:
                                pass
                            poly = pc.box2poly(np.array(box)[:,:3].T)
                            A = poly.A 
                            b = poly.b
                            plot_polytope_3d(A, b, p, color = color_list[k], trans=1, edge=True)

    # time_val_idx = 140
    # # Plot specific agent tubes
    # for j in range(time_val_idx, time_val_idx+10):
    #     agent = [9, 10, 12, 13, 15, 16]
    #     time_val = time_val_list[j]
    #     for agent_idx in agent:
    #         if agent_idx in time_aligned_segments[time_val]:
    #             agent_tube_list = time_aligned_segments[time_val][agent_idx]
    #             for tube in agent_tube_list:
    #                 for i in range(0, len(tube), step):
    #                     box = tube[i]
    #                     poly = pc.box2poly(np.array(box)[:,:3].T)
    #                     A = poly.A 
    #                     b = poly.b
    #                     plot_polytope_3d(A, b, p, color = "#fb8072", trans=1)

    # time_val_idx = 150
    # # Plot specific agent tubes
    # for j in range(time_val_idx, time_val_idx+10):
    #     agent = [9, 10, 12, 13, 15, 16]
    #     time_val = time_val_list[j]
    #     for agent_idx in agent:
    #         if agent_idx in time_aligned_segments[time_val]:
    #             agent_tube_list = time_aligned_segments[time_val][agent_idx]
    #             for tube in agent_tube_list:
    #                 for i in range(0, len(tube), step):
    #                     box = tube[i]
    #                     poly = pc.box2poly(np.array(box)[:,:3].T)
    #                     A = poly.A 
    #                     b = poly.b
    #                     plot_polytope_3d(A, b, p, color = "#fdb462", trans=1)

    # time_val_idx = 160
    # # Plot specific agent tubes
    # for j in range(time_val_idx, time_val_idx+10):
    #     agent = [9, 10, 12, 13, 15, 16]
    #     time_val = time_val_list[j]
    #     for agent_idx in agent:
    #         if agent_idx in time_aligned_segments[time_val]:
    #             agent_tube_list = time_aligned_segments[time_val][agent_idx]
    #             for tube in agent_tube_list:
    #                 for i in range(0, len(tube), step):
    #                     box = tube[i]
    #                     poly = pc.box2poly(np.array(box)[:,:3].T)
    #                     A = poly.A 
    #                     b = poly.b
    #                     plot_polytope_3d(A, b, p, color = "#ffed6f", trans=1)

    # time_val_idx = 170
    # # Plot specific agent tubes
    # for j in range(time_val_idx, time_val_idx+10):
    #     agent = [9, 10, 12, 13, 15, 16]
    #     time_val = time_val_list[j]
    #     for agent_idx in agent:
    #         if agent_idx in time_aligned_segments[time_val]:
    #             agent_tube_list = time_aligned_segments[time_val][agent_idx]
    #             for tube in agent_tube_list:
    #                 for i in range(0, len(tube), step):
    #                     box = tube[i]
    #                     poly = pc.box2poly(np.array(box)[:,:3].T)
    #                     A = poly.A 
    #                     b = poly.b
    #                     plot_polytope_3d(A, b, p, color = "#ffffb3", trans=1)


    # Plot obstacles 
    obstacle_list = scenario_config['unsafeSet']
    for obstacle in obstacle_list:
        obstacle_type = obstacle[0]
        if obstacle_type == "Matrix":
            obstacle_A = np.array(obstacle[1][0])
            obstacle_b = np.array(obstacle[1][1])
            # plot_polytope_3d(obstacle_A[:-6,:3], obstacle_b[:-6,:], p, color = "#d9d9d9", trans=1)
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

    # tmp = [10, 15]
    # for key in tmp:
    #     print(f'plotting tube {key}')
    #     tube_list  = agent_tube[key]
    #     for tube in tube_list:
    #         for i in range(0, len(tube), step):
    #             box = tube[i]
    #             poly = pc.box2poly(np.array(box)[:,:3].T)
    #             A = poly.A 
    #             b = poly.b
    #             plot_polytope_3d(A, b, p, color = "#ffed6f", trans=1)
        
    p.show()

if __name__ == "__main__":
    res_vis()