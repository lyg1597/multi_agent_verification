# Demo
# Written by: Kristina Miller
import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt
from plot_polytope3d import *

def problem():
    A_rect = np.array([[-1,0,0],
                       [1,0,0],
                       [0,-1,0],
                       [0,1,0],
                       [0,0,-1],
                       [0,0,1]])

    # b1_1 = np.array([[0], [4.5], [0], [4.5], [-7.5], [10.5]])
    b1_2 = np.array([[-7.5], [10.5], [0], [4.5], [0], [4.5]])
    b1_3 = np.array([[-13.5], [18], [0], [4.5], [0], [4.5]])
    b1_4 = np.array([[0], [4.5], [-7.5], [10.5], [0], [4.5]])
    b1_5 = np.array([[-7.5], [10.5], [-7.5], [10.5], [0], [4.5]])
    # b2_6 = np.array([[-13.5], [18], [-7.5], [10.5], [-7.5], [10.5]])
    b1_7 = np.array([[0], [4.5], [-13.5], [18], [0], [4.5]])
    b1_8 = np.array([[-7.5], [10.5], [-13.5], [18], [0], [4.5]])
    b1_9 = np.array([[-13.5], [18], [-13.5], [18], [0], [4.5]])
    
    b2_1 = np.array([[0], [4.5], [0], [4.5], [-7.5], [10.5]])
    b2_2 = np.array([[-7.5], [10.5], [0], [4.5], [-7.5], [10.5]])
    b2_3 = np.array([[-13.5], [18], [0], [4.5], [-7.5], [10.5]])
    b2_4 = np.array([[0], [4.5], [-7.5], [10.5], [-7.5], [10.5]])
    b2_5 = np.array([[-7.5], [10.5], [-7.5], [10.5], [-7.5], [10.5]])
    b2_6 = np.array([[-13.5], [18], [-7.5], [10.5], [-7.5], [10.5]])
    b2_7 = np.array([[0], [4.5], [-13.5], [18], [-7.5], [10.5]])
    # b2_8 = np.array([[-7.5], [10.5], [-13.5], [18], [-7.5], [10.5]])
    b2_9 = np.array([[-13.5], [18], [-13.5], [18], [-7.5], [10.5]])
    
    b3_1 = np.array([[0], [4.5], [0], [4.5], [-13.5], [18.5]])
    # b3_2 = np.array([[-7.5], [10.5], [0], [4.5], [-13.5], [18.5]])
    b3_3 = np.array([[-13.5], [18], [0], [4.5], [-13.5], [18.5]])
    b3_4 = np.array([[0], [4.5], [-7.5], [10.5], [-13.5], [18.5]])
    b3_5 = np.array([[-7.5], [10.5], [-7.5], [10.5], [-13.5], [18.5]])
    b3_6 = np.array([[-13.5], [18], [-7.5], [10.5], [-13.5], [18.5]])
    b3_7 = np.array([[0], [4.5], [-13.5], [18], [-13.5], [18.5]])
    b3_8 = np.array([[-7.5], [10.5], [-13.5], [18], [-13.5], [18.5]])
    # b3_9 = np.array([[-13.5], [18], [-13.5], [18], [-13.5], [18.5]])
    
    obstacles = [(A_rect, b1_2),
                 (A_rect, b1_3),
                 (A_rect, b1_4),
                 (A_rect, b1_5),
                 (A_rect, b1_7),
                 (A_rect, b1_8),
                 (A_rect, b1_9),
                 (A_rect, b2_1),
                 (A_rect, b2_2),
                 (A_rect, b2_3),
                 (A_rect, b2_4),
                 (A_rect, b2_5),
                 (A_rect, b2_6),
                 (A_rect, b2_7),
                 (A_rect, b2_9),
                 (A_rect, b3_1),
                 (A_rect, b3_3),
                 (A_rect, b3_4),
                 (A_rect, b3_5),
                 (A_rect, b3_6),
                 (A_rect, b3_7),
                 (A_rect, b3_8)]

    #TODO: Create initial set
    b0 = np.array([[-1.5], [4.5], [-1.5], [4.5], [-1.5], [4.5]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-13.5], [16.5], [-7.5], [10.5], [-1.5], [4.5]])
    bg2 = np.array([[-7.5], [10.5], [-13.5], [16.5], [-7.5], [10.5]])
    bg3 = np.array([[-7.5], [10.5], [-1.5], [4.5], [-13.5], [16.5]])
    bg4 = np.array([[-13.5], [16.5], [-13.5], [16.5], [-13.5], [16.5]])
    goal = [(A_rect, bg1),
            (A_rect, bg2),
            (A_rect, bg3),
            (A_rect, bg4)]
    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    fig = plt.figure()
    axes = fig.add_subplot(111, projection='3d')
    
    for A,b in obs:
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')
        plot_polytope_3d(A, b, ax = axes, color = 'red')

    for A,b in goal:
        plot_polytope_3d(A, b, ax = axes, color = 'green')

    
    plot_polytope_3d(Theta[0], Theta[1], ax = axes, color = 'blue')

    plot_line_3d([0,0,0],[0,0,18],ax = axes)
    # plt.xlim(0, 24)
    # plt.ylim(0, 24)
    plt.grid()
    plt.show()
