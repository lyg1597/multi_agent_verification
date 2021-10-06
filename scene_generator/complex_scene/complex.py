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

    b1 = np.array([[0], [4.5], [0], [4.5]])
    b2 = np.array([[-13.5], [16.5], [-1.5], [4.5]])
    b3 = np.array([[-7.5], [10.5], [-7.5], [10.5]])
    b4 = np.array([[-13.5], [16.5], [-7.5], [10.5]])
    b5 = np.array([[-19.5], [24], [-7.5], [10.5]])
    b6 = np.array([[0], [4.5], [-13.5], [16.5]])
    b7 = np.array([[-7.5], [10.5], [-13.5], [16.5]])
    b8 = np.array([[-13.5], [16.5], [-13.5], [16.5]])
    b9 = np.array([[-19.5], [24], [-13.5], [16.5]])
    b10 = np.array([[-7.5], [10.5], [-19.5], [24]])
    b11 = np.array([[-13.5], [16.5], [-19.5], [24]])
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
    b0 = np.array([[-1.5], [4.5], [-19.5], [22.5]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-1.5], [4.5], [-7.5], [10.5]])
    bg2 = np.array([[-7.5], [10.5], [-1.5], [4.5]])
    bg3 = np.array([[-19.5], [22.5], [-1.5], [4.5]])
    bg4 = np.array([[-19.5], [22.5], [-19.5], [22.5]])
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
    plt.xlim(0, 24)
    plt.ylim(0, 24)
    plt.grid()
    plt.show()
