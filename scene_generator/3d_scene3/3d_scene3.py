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

    b1 = np.array([[0], [9], [0], [9], [0], [36]])
    b2 = np.array([[-15], [21], [0], [9],[0], [24]])
    b3 = np.array([[-27], [33], [0], [9], [0], [36]])
    b4 = np.array([[-39], [48], [0], [9], [0], [12]])
       
    b5 = np.array([[-15], [21], [-15], [21], [0], [36]])
    b6 = np.array([[-27], [33], [-15], [21], [0], [36]])
    b7 = np.array([[-39], [48], [-15], [21], [0], [36]])
    
    b8 = np.array([[0], [9], [-27], [33], [0], [36]])
    b9 = np.array([[-15], [21], [-27], [33], [0], [36]])
    b10 = np.array([[-27], [33], [-27], [33], [0], [36]])
    b11 = np.array([[-39], [48], [-27], [33], [0], [36]])
    
    b12 = np.array([[0], [9], [-39], [48], [0], [12]])
    b13 = np.array([[-15], [21], [-39], [48], [0], [36]])
    b14 = np.array([[-27], [33], [-39], [48], [0], [36]])
    b15 = np.array([[-39], [48], [-39], [48], [0], [24]])
    
    b16 = np.array([[0],[9],[0],[33],[-24],[36]])
    b17 = np.array([[-27],[33],[0],[21],[-12],[36]])
    b18 = np.array([[-27],[48],[-15],[21],[-24],[36]])
    b19 = np.array([[0],[21],[-27],[33],[0],[24]])
    b20 = np.array([[-15],[21],[-27],[48],[-12],[36]])
    b21 = np.array([[-27],[33],[-27],[48],[-24],[36]])
    b22 = np.array([[-27],[33],[-27],[48],[0],[12]])
    
    # TODO: Create more obstacles

    obstacles = [(A_rect, b1),
                 (A_rect, b2),
                 (A_rect, b3),
                 (A_rect, b4),
                 (A_rect, b5),
                 (A_rect, b6),
                 (A_rect, b7),
                 (A_rect, b8),
                 (A_rect, b9),
                 (A_rect, b10),
                 (A_rect, b11),
                 (A_rect, b12),
                 (A_rect, b13),
                 (A_rect, b14),
                 (A_rect, b15),
                 (A_rect, b16),
                 (A_rect, b17),
                 (A_rect, b18),
                 (A_rect, b19),
                 (A_rect, b20),
                 (A_rect, b21),
                 (A_rect, b22)]

    #TODO: Create initial set
    b0 = np.array([[-5], [7], [-41], [42], [-17], [19]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-3], [9], [-15], [21], [-3], [9]])
    bg2 = np.array([[-15], [21], [-3], [9], [-27], [33]])
    bg3 = np.array([[-39], [45], [-3], [9], [-15], [21]])
    bg4 = np.array([[-39], [45], [-39], [45], [-27], [33]])
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
