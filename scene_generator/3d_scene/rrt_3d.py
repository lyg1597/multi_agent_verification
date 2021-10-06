# SAT-plan vs. RRT
# Written by: Kristina Miller

"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""
import csv

import math
import random

import matplotlib.pyplot as plt

show_animation = False

import polytope as pc
import pypoman as ppm
import numpy as np
import time

def check_in_polytope(pt, plygn):
    A = plygn[0]
    b = plygn[1]

    z = np.matmul(A,pt)
    is_in = True
    for i in range(len(z)):
        if z[i] > b[i]:
            is_in = False
            break
    return is_in

class RRT3D:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None
            self.idx = 0

    class Edge:
        """
        Edge of result graph
        """
        def __init__(self, node, parent, color = 'b'):
            self.start = [parent.x,parent.y,parent.z]
            self.end = [node.x,node.y,node.z]
            self.color = color
            
    def __init__(self, start, goal_list, obstacle_list, rand_area, color = 'b',
                 expand_dis=[3.0], expand_theta = [0,np.pi/3,2*np.pi/3,np.pi,4*np.pi/3, 5*np.pi/3], 
                 expand_omega = [0,np.pi/3,2*np.pi/3,np.pi,4*np.pi/3, 5*np.pi/3],
                 path_resolution=0.01, goal_sample_rate=0, max_iter= 5000):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        """
        New Parameters

        start set: [A_start, b_start] -> Use Chebyshev center as starting point
        goal set: [A_goal, b_goal] -> Use Chebyshev center as goal point
        """
        start_poly = pc.Polytope(start[0],start[1])
        end_poly_list = []
        for i in range(len(goal_list)):
            end_poly_list.append(pc.Polytope(goal_list[i][0], goal_list[i][1]))
        self.start_vtc = ppm.compute_polytope_vertices(start[0], start[1])
        self.end_vtc_list = []
        for i in range(len(goal_list)):
            self.end_vtc_list.append(ppm.compute_polytope_vertices(goal_list[i][0], goal_list[i][1]))
        self.goal_list = goal_list
        start_pt = start_poly.chebXc
        end_pt_list = []
        for i in range(len(goal_list)):
            end_pt_list.append(end_poly_list[i].chebXc)

        self.start = self.Node(start_pt[0], start_pt[1], start_pt[2])
        self.end = []
        for i in range(len(end_pt_list)):
            self.end.append(self.Node(end_pt_list[i][0], end_pt_list[i][1], end_pt_list[i][2]))
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.expand_theta = expand_theta
        self.expand_omega = expand_omega
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.reach_node_list = []
        self.reach_node_idx = []
        self.color = color

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        i = 0
        while i < self.max_iter or len(self.reach_node_idx) < len(self.goal_list):
            i += 1
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis, self.expand_theta, self.expand_omega)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            safe, idx = self.in_goal(self.node_list[-1].x, self.node_list[-1].y, self.node_list[-1].z, self.goal_list)
            if safe:
                self.reach_node_list.append(self.node_list[-1])
                self.node_list.pop()
                if idx not in self.reach_node_idx:
                    self.reach_node_idx.append(idx)
                # pass 
                # return self.generate_final_course(len(self.node_list) - 1) ,i

            # if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= max(self.expand_dis):
            #     # print("Goal!!")
            #     # print('iter:'+str(i))
            #     return self.generate_final_course(len(self.node_list) - 1) ,i


            if animation and i % 5:
                self.draw_graph(rnd_node)

        path_list = self.generate_final_course()
        edges = self.generate_edges()
        return path_list, edges  # cannot find path

    def steer(self, from_node, to_node, extend_length=[float("inf")], extend_theta = [float("inf")], extend_omega = [float("inf")]):

        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, theta, omega = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        # if extend_length > d:
        #     extend_length = d
        absDiffFunc = lambda list_value : abs(list_value - d)
        extend_length = min(extend_length,key = absDiffFunc)

        absDiffFunc = lambda list_value: abs(list_value - theta)
        theta = min(extend_theta, key = absDiffFunc)

        absDiffFunc = lambda list_value: abs(list_value - theta)
        theta = min(extend_theta, key = absDiffFunc)

        absDiffFunc = lambda list_value: abs(list_value - omega)
        omega = min(extend_omega, key = absDiffFunc)

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta) * math.cos(omega)
            new_node.y += self.path_resolution * math.sin(theta) * math.cos(omega)
            new_node.z += self.path_resolution * math.sin(omega)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.z = to_node.z
            new_node.path_x[-1] = to_node.x
            new_node.path_y[-1] = to_node.y
            new_node.path_z[-1] = to_node.z

        new_node.parent = from_node

        return new_node

    def generate_final_course(self):
        path_list = []
        for node in self.reach_node_list:
            path = []
            while node.parent is not None:
                path.append([node.x, node.y, node.z])
                node = node.parent
            path.append([node.x, node.y, node.z])
            path_list.append(path)
        return path_list

    def generate_edges(self):
        edge_dict = {}
        for node in self.reach_node_list:
            while node.parent is not None:
                if (node.x,node.y,node.z) in edge_dict:
                    break
                parent_node = node.parent
                edge = self.Edge(node,parent_node, self.color)
                edge_dict[(node.parent.x,node.parent.y,node.parent.z,node.x,node.y,node.z)] = edge
                node = parent_node
        return edge_dict

    # def calc_dist_to_goal(self, x, y):
    #     dx = x - self.end.x
    #     dy = y - self.end.y
    #     return math.sqrt(dx ** 2 + dy ** 2)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            i = random.randint(0,len(self.end)-1)
            rnd = self.Node(self.end[i].x, self.end[i].y, self.end[i].z)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (A,b) in self.obstacle_list:
            vtc = ppm.compute_polytope_vertices(A,b)
            ppm.plot_polygon(vtc, color = 'r')
            '''
            plt.plot(ox, oy, "ok", ms=30 * size)
            '''

        ppm.plot_polygon(self.start_vtc, color = 'b')
        # ppm.plot_polygon(self.end_vtc, color = 'g')
        # plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def in_goal(x, y, z, goal_list):
        # TODO: Change this function to handle a list of goals.
        safe = False
        for i in range(len(goal_list)):
            if check_in_polytope(np.array([x,y,z]),(goal_list[i][0],goal_list[i][1])):
                safe = True
                break
        return safe,i

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        # dlist = []
        # for node in node_list:
            # if node not in self.reach_node_list:
                # dlist.append()
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 + (node.z - rnd_node.z)**2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):
        safe = True
        for (A,b) in obstacleList:
            x_list = [x for x in node.path_x]
            y_list = [y for y in node.path_y]
            z_list = [z for z in node.path_z]
            pts = [np.array([x_list[i], y_list[i], z_list[i]]) for i in range(len(x_list))]

            for pt in pts:
                if safe:
                    safe = not check_in_polytope(pt, (A, b))
                else:
                    break
        return safe
        '''
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            print(pts)

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe
        '''

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        dxy = math.sqrt(dx**2 + dy**2)
        d = math.sqrt(dx ** 2 + dy ** 2 + dz**2)
        theta = math.atan2(dy, dx)%(2*np.pi)
        omega = math.atan2(dz, dxy)%(2*np.pi)
        # if omega > np.pi:
        #     omega -= np.pi*2
        return d, theta, omega


# def main(A_start, b_start,
#          A_end, b_end,
#          obstacleList, search_area):

#     rrt = RRT(start=[A_start, b_start],
#               goal_list=goal,
#               rand_area=search_area,
#               obstacle_list=obstacleList)
#     path, iters = rrt.planning(animation=show_animation)

#     if path is None:
#         found_path = None
#     else:
#         x_r, y_r = ([x for (x, y) in path], [y for (x, y) in path])
#         x_r.reverse()
#         y_r.reverse()
#         found_path = (x_r, y_r)

#     return found_path, found_path != None, iters

# def run_rrt(scenario, search_area, envname):
#     O, Theta, Goal = scenario()

#     stats = [['RRT', envname],['path?', 'time', 'iters', 'segments']]
#     for i in range(100):
#         t_start = time.time()
#         xref, path_bool, j = main(A_start = Theta[0], b_start = Theta[1], A_end = Goal[0], b_end = Goal[1], obstacleList=O, search_area = search_area)
#         t_end = time.time()
#         t = t_end - t_start
#         if path_bool:
#             segs = len(xref[0]) - 1
#         else:
#             segs = 'N/A'
#         stats.append([path_bool, t, j+1, segs])

#     with open('results/RRTvSAT-Plan/RRT_'+envname+'.csv', 'w', newline='') as file:
#         writer = csv.writer(file)
#         writer.writerows(stats)
#     print('Saved RRT results to results/RRTvSAT-Plan/RRT_'+envname+'.csv')

#     return stats
