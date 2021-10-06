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

    b1 = np.array([[-12], [102], [0], [6]])
    b2 = np.array([[0], [6], [-12], [90]])
    b3 = np.array([[0], [90], [-84], [90]])
    b4 = np.array([[-96], [102], [0], [78]])
    b5 = np.array([[-12], [48], [-12], [18]])
    b6 = np.array([[-54], [90], [-12], [18]])
    b7 = np.array([[-12], [18], [-12], [78]])
    b8 = np.array([[-84], [90], [-12], [78]])
    b9 = np.array([[-12], [48], [-72], [78]])
    b10 = np.array([[-54], [90], [-72], [78]])
    b11 = np.array([[-24], [78], [-24], [30]])
    b12 = np.array([[-24], [78], [-60], [66]])
    b13 = np.array([[-24], [30], [-24], [42]])
    b14 = np.array([[-72], [78], [-24], [42]])
    b15 = np.array([[-24], [30], [-48], [66]])
    b16 = np.array([[-72], [78], [-48], [66]])
    b17 = np.array([[-36], [48], [-36], [54]])
    b18 = np.array([[-54], [66], [-36], [54]])    
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
                 (A_rect, b18)]

    #TODO: Create initial set
    b0 = np.array([[-5], [6], [-5], [6]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-48], [54], [-12], [18]])
    bg2 = np.array([[-48], [54], [-42], [48]])
    bg3 = np.array([[-72], [78], [-42], [48]])
    bg4 = np.array([[-96], [102], [-84], [90]])
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
    plt.xlim(0, 102)
    plt.ylim(0, 90)
    plt.grid()
    plt.show()
