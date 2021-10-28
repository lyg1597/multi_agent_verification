# Plot polytope in 3d
# Written by: Kristina Miller

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3

from scipy.spatial import ConvexHull
import pypoman as ppm
import polytope as pc
import pyvista as pv

def plot_polytope_3d(A, b, ax = None, color = 'red', trans = 0.2, edge = True):
    if ax is None:
        ax = pv.Plotter() 
    poly = pc.Polytope(A = A, b = b)
    vertices = pc.extreme(poly)
    cloud = pv.PolyData(vertices)
    volume = cloud.delaunay_3d()
    shell = volume.extract_geometry()
    ax.add_mesh(shell, opacity=trans, color=color)
    if edge:
        edges = shell.extract_feature_edges(20)
        ax.add_mesh(edges, color="k", line_width=1)

def plot_mesh(mesh, ax = None, color = 'red', trans=0.2, edge=True):
    if ax is None:
        ax = pv.Plotter() 
    ax.add_mesh(mesh, opacity=trans, color=color)
    if edge:
        edges = mesh.extract_feature_edges(20)
        ax.add_mesh(edges, color="k", line_width=1)

def plot_line_3d(start, end, ax = None, color = 'blue', line_width = 1):
    if ax is None:
        ax = pv.Plotter() 

    a = start
    b = end

    line = pv.Line(a, b)

    ax.add_mesh(line, color=color, line_width=line_width)

if __name__ == '__main__':
    A = np.array([[-1, 0, 0],
                  [1, 0, 0],
                  [0, -1, 0],
                  [0, 1, 0],
                  [0, 0, -1],
                  [0, 0, 1]])
    b = np.array([[1], [1], [1], [1], [1], [1]])
    b2 = np.array([[-1], [2], [-1], [2], [-1], [2]])
    ax1 = a3.Axes3D(plt.figure())
    plot_polytope_3d(A, b, ax = ax1, color = 'red')
    plot_polytope_3d(A, b2, ax = ax1, color = 'green')
    plt.show()