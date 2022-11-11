import json 
from typing import List 
import numpy as np 
import pypoman as ppm 
import matplotlib.pyplot as plt 
import polytope as pc 
from plot_polytope3d_vista import *

class Waypoint:

    def __init__(self, mode: str, mode_parameters: List[float], time_bound: float, id: int):
        self.mode: str = mode
        self.mode_parameters: List[float] = mode_parameters
        self.time_bound: float = time_bound
        self.id = id
        # self.unsafeset_list = unsafeset_list

    def is_equal(self, other_waypoint: List[float]):
        return tuple(self.mode_parameters) == tuple(other_waypoint.mode_parameters)
        # self.delta: np.array = (self.original_guard[1, :] - self.original_guard[0, :]) / 2
    # TODO add helper function to check if point is inside guard

def get_edge(edge_list, src_id):
    for edge in edge_list:
        if edge[0] == src_id:
            return edge[0], edge[1]
    return None, None 

def get_waypoints(init_mode_id, edge_list, mode_list):
    wp = []
    id = init_mode_id
    while id is not None:
        mode = mode_list[id]
        wp.append(Waypoint('follow_waypoint',mode[1],3.0,0))
        src, dest = get_edge(edge_list, id)
        id = dest    
    return wp

def count_intersection(fn):
    f = open(fn,'r')
    agent_data = json.load(f)
    wp_list = []
    shortest_wp = float('inf')
    longest_wp = -float('inf')
    total_wp = 0

    for i in range(len(agent_data['agents'])):
        agent = agent_data['agents'][i]
        init_mode_id = agent['initialModeID']
        edge_list = agent['edge_list']
        mode_list = agent['mode_list']
        wp = get_waypoints(init_mode_id, edge_list, mode_list)
        wp_list.append(wp)
        if len(wp) < shortest_wp:
            shortest_wp = len(wp)
        if len(wp) > longest_wp:
            longest_wp = len(wp)
        total_wp += len(wp)

    num_intersection = 0
    # ax = pv.Plotter(off_screen = False)
    for i in range(longest_wp):
        print(i)
        unsafe_set = []

        if "unsafeSet" in agent_data:
            unsafe_set = agent_data["unsafeSet"]
            for unsafe in unsafe_set:
                unsafe_type = unsafe[0]
                if unsafe_type == 'Matrix':
                    A = np.array(unsafe[1][0])
                    b = np.array(unsafe[1][1])
                    # plot_polytope_3d(A = A[:-6,:3], b = b[:-6,:], ax = ax, color = '#bdbdbd', trans=1, edge=True)


        plotted_waypoints = []
        for j in range(len(wp_list)):
            if len(wp_list[j])<=i:
                continue
            wp1 = wp_list[j][i].mode_parameters
            for k in range(j+1, len(wp_list)):
                if len(wp_list[k])<=i: 
                    continue
                wp2 = wp_list[k][i].mode_parameters
                val = np.linalg.norm(np.array(wp1) - np.array(wp2))
                if val < 10:
                    num_intersection += 1
                    if k not in plotted_waypoints:
                        # plot_line_3d(wp2[:3], wp2[3:], ax = ax, line_width = 5, color = 'r')
                        # plt.plot([wp2[0], wp2[2]], [wp2[1], wp2[3]], 'r')
                        plotted_waypoints.append(k)
                    if j not in plotted_waypoints:
                        # plot_line_3d(wp1[:3], wp1[3:], ax = ax, line_width = 5, color = 'r')
                        # plt.plot([wp1[0], wp1[2]], [wp1[1], wp1[3]], 'r')
                        plotted_waypoints.append(j)
                # if doIntersect(p1, q1, p2, q2):
                #     num_intersection += 1
        for j in range(len(wp_list)):
            if len(wp_list[j])>i and j not in plotted_waypoints:
                wp1 = wp_list[j][i].mode_parameters
                # plot_line_3d(wp1[:3], wp1[3:], ax = ax, line_width = 5, color = 'b')
                plotted_waypoints.append(j)
        # ax.show(screenshot=f'./data/seg_fig_{i}.png')
        # ax.clear()
    # ax.show()

    print(num_intersection, longest_wp, shortest_wp, total_wp)

if __name__ == "__main__":
    fn = './scenario_hscc/map4-3d-20.json'
    count_intersection(fn)
    