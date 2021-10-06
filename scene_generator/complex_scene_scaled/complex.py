# Demo
# Written by: Kristina Miller
import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt

def problem():
    A_rect = np.array([[-1,0],
                       [1,0],
                       [0,-1],
                       [0,1]])

    b1 = np.array([[0], [9], [0], [9]])
    b2 = np.array([[-27], [33], [-3], [9]])
    b3 = np.array([[-15], [21], [-15], [21]])
    b4 = np.array([[-27], [33], [-15], [21]])
    b5 = np.array([[-39], [48], [-15], [25]])
    b6 = np.array([[0], [9], [-27], [33]])
    b7 = np.array([[-15], [21], [-27], [33]])
    b8 = np.array([[-27], [33], [-27], [33]])
    b9 = np.array([[-39], [48], [-27], [33]])
    b10 = np.array([[-15], [21], [-39], [48]])
    b11 = np.array([[-27], [33], [-39], [48]])
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
                 (A_rect, b11)]

    #TODO: Create initial set
    b0 = np.array([[-5], [7], [-41], [43]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-3], [9], [-15], [21]])
    bg2 = np.array([[-15], [21], [-3], [9]])
    bg3 = np.array([[-39], [45], [-3], [9]])
    bg4 = np.array([[-39], [45], [-39], [45]])
    goal = [(A_rect, bg1),
            (A_rect, bg2),
            (A_rect, bg3),
            (A_rect, bg4)]
    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    for A,b in obs:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')

    for A,b in goal:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'green')

    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(Theta[0],Theta[1]), color = 'blue')
    plt.xlim(0, 48)
    plt.ylim(0, 48)
    plt.grid()
    plt.show()
