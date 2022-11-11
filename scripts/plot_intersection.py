import json 
from typing import List 
import numpy as np 
import pypoman as ppm 
import matplotlib.pyplot as plt 
import polytope as pc 

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
 
# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r):
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False
 
def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
     
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.
     
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if (val > 0):
         
        # Clockwise orientation
        return 1
    elif (val < 0):
         
        # Counterclockwise orientation
        return 2
    else:
         
        # Collinear orientation
        return 0
 
# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1,q1,p2,q2):
     
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
 
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
 
    # Special Cases
 
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        return True
 
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True
 
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True
 
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True
 
    # If none of the cases
    return False

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
    for i in range(longest_wp):
        print(i)
        unsafe_set = []

        if "unsafeSet" in agent_data:
            unsafe_set = agent_data["unsafeSet"]

        for j in range(len(unsafe_set)):
            if unsafe_set[j][0] == "Box":
                box = unsafe_set[j][1]
                poly = pc.box2poly(np.array(box).T)
            elif unsafe_set[j][0] == "Matrix":
                mats = unsafe_set[j][1]
                poly = pc.Polytope(np.array(mats[0]),np.array(mats[1]))
            elif unsafe_set[j][0] == "Vertices":
                vertices =  np.array(unsafe_set[j][1])
                if vertices.size == 0:
                    continue
                poly = pc.qhull(np.array(vertices))  
            poly = pc.projection(poly,[1,2])
            ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#d9d9d9', alpha = 1)

        plotted_waypoints = []
        for j in range(len(wp_list)):
            if len(wp_list[j])<=i:
                continue
            wp1 = wp_list[j][i].mode_parameters
            p1 = Point(wp1[0], wp1[1])
            q1 = Point(wp1[2],wp1[3])
            for k in range(j+1, len(wp_list)):
                if len(wp_list[k])<=i: 
                    continue
                wp2 = wp_list[k][i].mode_parameters
                p2 = Point(wp2[0], wp2[1])
                q2 = Point(wp2[2], wp2[3])
                val = np.linalg.norm(np.array(wp1) - np.array(wp2))
                if val < 10:
                    num_intersection += 1
                    if k not in plotted_waypoints:
                        plt.plot([wp2[0], wp2[2]], [wp2[1], wp2[3]], 'r')
                        plotted_waypoints.append(k)
                    if j not in plotted_waypoints:
                        plt.plot([wp1[0], wp1[2]], [wp1[1], wp1[3]], 'r')
                        plotted_waypoints.append(j)
                # if doIntersect(p1, q1, p2, q2):
                #     num_intersection += 1
        for j in range(len(wp_list)):
            if len(wp_list[j])>i and j not in plotted_waypoints:
                wp1 = wp_list[j][i].mode_parameters
                plt.plot([wp1[0], wp1[2]], [wp1[1], wp1[3]], 'b')
                plotted_waypoints.append(j)
        plt.xlim([0,480])
        plt.xlabel('x')
        plt.ylim([0,120])
        plt.ylabel('y')
        plt.savefig(f'./data/seg_fig_{i}.png',dpi=200)
        plt.clf()

    print(num_intersection, longest_wp, shortest_wp, total_wp)

if __name__ == "__main__":
    fn = './scenario_hscc/map3-2d-34.json'
    count_intersection(fn)
    