import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt
import polytope as pc

def test_func():
    b1 = np.array([[0,0],[0,1],[1,1],[1,0]])
    b1 = pc.qhull(b1)
    A = b1.A
    b = b1.b
    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')
    plt.show()

test_func()