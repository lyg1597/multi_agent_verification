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
    b2 = np.array([[-27], [33], [0], [9]])
    b3 = np.array([[-51], [57], [0], [9]])
    b4 = np.array([[-63], [69], [0], [9]])
    b5 = np.array([[-75], [81], [0], [9]])
    
    # b3 = np.array([[-0], [9], [-15], [21]])
    b6 = np.array([[-15], [21], [-15], [21]])
    b7 = np.array([[-27], [33], [-15], [21]])
    b8 = np.array([[-39], [45], [-15], [21]])
    b9 = np.array([[-51], [57], [-15], [21]])
    b10 = np.array([[-63], [69], [-15], [21]])
    b11 = np.array([[-75], [81], [-15], [21]])
    b12 = np.array([[-87], [96], [-15], [21]])
    
    b13 = np.array([[-0], [9], [-27], [33]])
    b14 = np.array([[-15], [21], [-27], [33]])
    b15 = np.array([[- 27], [33], [-27], [33]])
    b16 = np.array([[-39], [45], [-27], [33]])
    b17 = np.array([[-51], [57], [-27], [33]])
    b18 = np.array([[-75], [81], [-27], [33]])
    b19 = np.array([[-87], [96], [-27], [33]])
    
    b20 = np.array([[-15], [21], [-39], [45]])
    b21 = np.array([[-27], [33], [-39], [45]])
    b22 = np.array([[-51], [57], [-39], [45]])
    b23 = np.array([[-63], [69], [-39], [45]])
    b24 = np.array([[-75], [81], [-39], [45]])
    b25 = np.array([[-87], [96], [-39], [45]])
    
    b26 = np.array([[-0], [9], [-51], [57]])
    b27 = np.array([[-15], [21], [-51], [57]])
    b28 = np.array([[-27], [33], [-51], [57]])
    b29 = np.array([[-39], [45], [-51], [57]])
    b30 = np.array([[-51], [57], [-51], [57]])
    b31 = np.array([[-63], [69], [-51], [57]])
    b32 = np.array([[-75], [81], [-51], [57]])
    b33 = np.array([[-87], [96], [-51], [57]])
    
    b34 = np.array([[-0], [9], [-63], [69]])
    b35 = np.array([[-15], [21], [-63], [69]])
    b36 = np.array([[-39], [45], [-63], [69]])
    b37 = np.array([[-51], [57], [-63], [69]])
    b38 = np.array([[-75], [81], [-63], [69]])
    b39 = np.array([[-87], [96], [-63], [69]])
    
    b40 = np.array([[-0], [9], [-75], [81]])
    b41 = np.array([[-15], [21], [-75], [81]])
    b42 = np.array([[-27], [33], [-75], [81]])
    b43 = np.array([[-39], [45], [-75], [81]])
    b44 = np.array([[-51], [57], [-75], [81]])
    b45 = np.array([[-63], [69], [-75], [81]])
    b46 = np.array([[-75], [81], [-75], [81]])
    b47 = np.array([[-87], [96], [-75], [81]])
    
    b48 = np.array([[-15], [21], [-87], [96]])
    b49 = np.array([[-27], [33], [-87], [96]])
    b50 = np.array([[-39], [45], [-87], [96]])
    b51 = np.array([[-51], [57], [-87], [96]])
    b52 = np.array([[-63], [69], [-87], [96]])
    b53 = np.array([[-75], [81], [-87], [96]])

    b54 = np.array([[-27],[33],[0],[21]])
    b55 = np.array([[-75],[81],[0],[33]])
    b56 = np.array([[-15],[21],[-15],[33]])
    b57 = np.array([[-39],[57],[-15],[33]])
    b58 = np.array([[0],[21],[-27],[33]])
    b59 = np.array([[-27],[33],[-27],[57]])
    b60 = np.array([[-75],[96],[-39],[45]])
    b61 = np.array([[0],[21],[-51],[57]])
    b62 = np.array([[-27],[45],[-51],[57]])
    b63 = np.array([[-51],[57],[-51],[69]])
    b64 = np.array([[-51],[69],[-51],[57]])
    b65 = np.array([[-15],[21],[-63],[81]])
    b66 = np.array([[-39],[57],[-63],[96]])
    b67 = np.array([[0],[21],[-75],[81]])
    b68 = np.array([[-27],[33],[-75],[96]])
    b69 = np.array([[-63],[69],[-75],[96]])
    b70 = np.array([[-75],[96],[-75],[81]])
    b71 = np.array([[-15],[33],[-39],[45]])
    b72 = np.array([[-51],[57],[-0],[45]])
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
                 (A_rect, b22),
                 (A_rect, b23),
                 (A_rect, b24),
                 (A_rect, b25),
                 (A_rect, b26),
                 (A_rect, b27),
                 (A_rect, b28),
                 (A_rect, b29),
                 (A_rect, b30),
                 (A_rect, b31),
                 (A_rect, b32),
                 (A_rect, b33),
                 (A_rect, b34),
                 (A_rect, b35),
                 (A_rect, b36),
                 (A_rect, b37),
                 (A_rect, b38),
                 (A_rect, b39),
                 (A_rect, b40),
                 (A_rect, b41),
                 (A_rect, b42),
                 (A_rect, b43),
                 (A_rect, b44),
                 (A_rect, b45),
                 (A_rect, b46),
                 (A_rect, b47),
                 (A_rect, b48),
                 (A_rect, b49),
                 (A_rect, b50),
                 (A_rect, b51),
                 (A_rect, b52),
                 (A_rect, b53),
                 (A_rect, b54),
                 (A_rect, b55),
                 (A_rect, b56),
                 (A_rect, b57),
                 (A_rect, b58),
                 (A_rect, b59),
                 (A_rect, b60),
                 (A_rect, b61),
                 (A_rect, b62),
                 (A_rect, b63),
                 (A_rect, b64),
                 (A_rect, b65),
                 (A_rect, b66),
                 (A_rect, b67),
                 (A_rect, b68),
                 (A_rect, b69),
                 (A_rect, b70),
                 (A_rect, b71),
                 (A_rect, b72)]

    #TODO: Create initial set
    b0 = np.array([[-5.5], [6.5], [-41.5], [42.5]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg1 = np.array([[-15], [21], [-3], [9]])
    bg2 = np.array([[-39], [45], [-3], [9]])
    bg3 = np.array([[-87], [93], [-3], [9]])
    bg4 = np.array([[-3], [9], [-15], [21]])
    bg5 = np.array([[-63], [69], [-27], [33]])
    bg6 = np.array([[-39], [45], [-39], [45]])
    bg7 = np.array([[-27], [33], [-63], [69]])
    bg8 = np.array([[-63], [69], [-63], [69]])
    bg9 = np.array([[-3], [9], [-87], [93]])
    bg10 = np.array([[-87], [93], [-87], [93]])
    goal = [(A_rect, bg1),
            (A_rect, bg2),
            (A_rect, bg3),
            (A_rect, bg4),
            (A_rect, bg5),
            (A_rect, bg6),
            (A_rect, bg7),
            (A_rect, bg8),
            (A_rect, bg9),
            (A_rect, bg10)]
    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    for A,b in obs:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')

    for A,b in goal:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'green')

    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(Theta[0],Theta[1]), color = 'blue')
    plt.xlim(0, 96)
    plt.ylim(0, 96)
    plt.grid()
    plt.show()
