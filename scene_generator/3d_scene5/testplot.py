# Plot polytope in 3d
# Written by: Kristina Miller

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3

from scipy.spatial import ConvexHull
import pypoman as ppm

class Faces():
    def __init__(self,tri, sig_dig=12, method="convexhull"):
        self.method=method
        self.tri = np.around(np.array(tri), sig_dig)
        self.grpinx = list(range(len(tri)))
        norms = np.around([self.norm(s) for s in self.tri], sig_dig)
        _, self.inv = np.unique(norms,return_inverse=True, axis=0)

    def norm(self,sq):
        cr = np.cross(sq[2]-sq[0],sq[1]-sq[0])
        return np.abs(cr/np.linalg.norm(cr))

    def isneighbor(self, tr1,tr2):
        a = np.concatenate((tr1,tr2), axis=0)
        return len(a) == len(np.unique(a, axis=0))+2

    def order(self, v):
        if len(v) <= 3:
            return v
        v = np.unique(v, axis=0)
        n = self.norm(v[:3])
        y = np.cross(n,v[1]-v[0])
        y = y/np.linalg.norm(y)
        c = np.dot(v, np.c_[v[1]-v[0],y])
        if self.method == "convexhull":
            h = ConvexHull(c)
            return v[h.vertices]
        else:
            mean = np.mean(c,axis=0)
            d = c-mean
            s = np.arctan2(d[:,0], d[:,1])
            return v[np.argsort(s)]

    def simplify(self):
        for i, tri1 in enumerate(self.tri):
            for j,tri2 in enumerate(self.tri):
                if j > i:
                    if self.isneighbor(tri1,tri2) and \
                       self.inv[i]==self.inv[j]:
                        self.grpinx[j] = self.grpinx[i]
        groups = []
        for i in np.unique(self.grpinx):
            u = self.tri[self.grpinx == i]
            u = np.concatenate([d for d in u])
            u = self.order(u)
            groups.append(u)
        return groups

def plot_polytope_3d(A, b, ax = None, color = 'red', trans = 0.2):
    verts = np.array(ppm.compute_polytope_vertices(A, b))
    # compute the triangles that make up the convex hull of the data points
    hull = ConvexHull(verts)
    triangles = [verts[s] for s in hull.simplices]
    # combine co-planar triangles into a single face
    faces = Faces(triangles, sig_dig=1).simplify()
    # plot
    if ax == None:
        ax = a3.Axes3D(plt.figure())

    pc = a3.art3d.Poly3DCollection(faces,
                                   facecolor=color,
                                   edgecolor="k", alpha=trans)
    ax.add_collection3d(pc)
    # define view
    yllim, ytlim = ax.get_ylim()
    xllim, xtlim = ax.get_xlim()
    zllim, ztlim = ax.get_zlim()
    x = verts[:,0]
    x = np.append(x, [xllim, xtlim])
    y = verts[:,1]
    y = np.append(y, [yllim, ytlim])
    z = verts[:,2]
    z = np.append(z, [zllim, ztlim])
    ax.set_xlim(np.min(x)-1, np.max(x)+1)
    ax.set_ylim(np.min(y)-1, np.max(y)+1)
    ax.set_zlim(np.min(z)-1, np.max(z)+1)

def problem():
    A = np.array([[-1, 0, 0],
                  [1, 0, 0],
                  [0, -1, 0],
                  [0, 1, 0],
                  [0, 0, -1],
                  [0, 0, 1]])
    b = np.array([[0], [9], [0], [9],[0], [18]])
    b1 = np.array([[-27], [33], [0], [9], [0], [18]])
    b2 = np.array([[-51], [57], [0], [9], [0], [18]])
    b3 = np.array([[-63], [69], [0], [9], [0], [18]])
    b4 = np.array([[-75], [81], [0], [9], [0], [18]])
    b5 = np.array([[-15], [21], [-15], [21],[0], [18]])
    b6 = np.array([[-27], [33], [-15], [21], [0], [18]])
    b7 = np.array([[-39], [45], [-15], [21], [0], [18]])
    b8 = np.array([[-51], [57], [-15], [21], [0], [18]])
    b9 = np.array([[-63], [69], [-15], [21], [0], [18]])
    b10 = np.array([[-75], [81], [-15], [21], [0], [18]])
    b11 = np.array([[-87], [96], [-15], [21], [0], [18]])
    b12 = np.array([[-0], [9], [-27], [33], [0], [18]])
    b13 = np.array([[-15], [21], [-27], [33], [0], [18]])
    b14 = np.array([[-27], [33], [-27], [33], [0], [18]])
    b15 = np.array([[-39], [45], [-27], [33], [0], [18]])
    b16 = np.array([[-51], [57], [-27], [33], [0], [18]])
    b17 = np.array([[-75], [81], [-27], [33], [0], [18]])
    b18 = np.array([[-87], [96], [-27], [33], [0], [18]])
    b19 = np.array([[-15], [21], [-39], [45], [0], [18]])
    b20 = np.array([[-27], [33], [-39], [45], [0], [18]])
    b21 = np.array([[-51], [57], [-39], [45], [0], [18]]) 
    b22 = np.array([[-63], [69], [-39], [45], [0], [18]])
    b23 = np.array([[-75], [81], [-39], [45], [0], [18]])
    b24 = np.array([[-87], [96], [-39], [45], [0], [18]])
    b25 = np.array([[ -0], [9], [-51], [57], [0], [18]])
    b26 = np.array([[-15], [21], [-51], [57], [0], [18]])
    b27 = np.array([[-27], [33], [-51], [57], [0], [18]])
    b28 = np.array([[-39], [45], [-51], [57], [0], [18]])
    b29 = np.array([[-51], [57], [-51], [57], [0], [18]])
    b30 = np.array([[-63], [69], [-51], [57], [0], [18]])
    b31 = np.array([[-75], [81], [-51], [57], [0], [18]])
    b32 = np.array([[-87], [96], [-51], [57], [0], [18]])
    b33 = np.array([[-0], [9], [-63], [69], [0], [18]])
    b34 = np.array([[-15], [21], [-63], [69], [0], [18]])
    b35 = np.array([[-39], [45], [-63], [69], [0], [18]])
    b36 = np.array([[-51], [57], [-63], [69], [0], [18]])
    b37 = np.array([[-75], [81], [-63], [69], [0], [18]])
    b38 = np.array([[-87], [96], [-63], [69], [0], [18]])
    b39 = np.array([[-0], [9], [-75], [81], [0], [18]])
    b40 = np.array([[-15], [21], [-75], [81], [0], [18]])
    b41 = np.array([[-27], [33], [-75], [81], [0], [18]])
    b42 = np.array([[-39], [45], [-75], [81], [0], [18]])
    b43 = np.array([[-51], [57], [-75], [81], [0], [18]])
    b44 = np.array([[-63], [69], [-75], [81], [0], [18]])
    b45 = np.array([[-75], [81], [-75], [81], [0], [18]])
    b46 = np.array([[-87], [96], [-75], [81], [0], [18]])
    b47 = np.array([[-15], [21], [-87], [96], [0], [18]])
    b48 = np.array([[-27], [33], [-87], [96], [0], [18]])
    b49 = np.array([[-39], [45], [-87], [96], [0], [18]])
    b50 = np.array([[-51], [57], [-87], [96], [0], [18]])
    b51 = np.array([[-63], [69], [-87], [96], [0], [18]])
    b52 = np.array([[-75], [81], [-87], [96], [0], [18]])
    b53 = np.array([[-27],[33],[0],[21], [0], [18]])
    b54 = np.array([[-75],[81],[0],[33], [0], [18]])
    b55 = np.array([[-15],[21],[-15],[33], [0], [18]])
    b56 = np.array([[-39],[57],[-15],[33], [0], [18]])
    b57 = np.array([[0],[21],[-27],[33], [0], [18]])
    b58 = np.array([[-27],[33],[-27],[57], [0], [18]])
    b59 = np.array([[-75],[96],[-39],[45], [0], [18]])
    b60 = np.array([[0],[21],[-51],[57], [0], [18]])
    b61 = np.array([[-27],[45],[-51],[57], [0], [18]])
    b62 = np.array([[-51],[57],[-51],[69], [0], [18]])
    b63 = np.array([[-51],[69],[-51],[57], [0], [18]])
    b64 = np.array([[-15],[21],[-63],[81], [0], [18]])
    b65 = np.array([[-39],[57],[-63],[96], [0], [18]])
    b66 = np.array([[0],[21],[-75],[81], [0], [18]])
    b67 = np.array([[-27],[33],[-75],[96], [0], [18]])
    b68 = np.array([[-63],[69],[-75],[96], [0], [18]])
    b69 = np.array([[-75],[96],[-75],[81], [0], [18]])
    b70 = np.array([[-15],[33],[-39],[45], [0], [18]])
    b71 = np.array([[-51],[57],[-0],[45], [0], [18]])

    obstacle = [(A, b),
                 (A, b1),
                 (A, b2),
                 (A, b3),
                 (A, b4),
                 (A, b5),
                 (A, b6),
                 (A, b7),
                 (A, b8),
                 (A, b9),
                 (A, b10),
                 (A, b11),
                 (A, b12),
                 (A, b13),
                 (A, b14),
                 (A, b15),
                 (A, b16),
                 (A, b17),
                 (A, b18),
                 (A, b19),
                 (A, b20),
                 (A, b21),
                 (A, b22),
                 (A, b23),
                 (A, b24),
                 (A, b25),
                 (A, b26),
                 (A, b27),
                 (A, b28),
                 (A, b29),
                 (A, b30),
                 (A, b31),
                 (A, b32),
                 (A, b33),
                 (A, b34),
                 (A, b35),
                 (A, b36),
                 (A, b37),
                 (A, b38),
                 (A, b39),
                 (A, b40),
                 (A, b41),
                 (A, b42),
                 (A, b43),
                 (A, b44),
                 (A, b45),
                 (A, b46),
                 (A, b47),
                 (A, b48),
                 (A, b49),
                 (A, b50),
                 (A, b51),
                 (A, b52),
                 (A, b53),
                 (A, b54),
                 (A, b55),
                 (A, b56),
                 (A, b57),
                 (A, b58),
                 (A, b59),
                 (A, b60),
                 (A, b61),
                 (A, b63),
                 (A, b63),
                 (A, b64),
                 (A, b65),
                 (A, b66),
                 (A, b67),
                 (A, b68),
                 (A, b69),
                 (A, b70),
                 (A, b71)
                 ]
    
    bg1 = np.array([[-15], [21], [-3], [9], [-13], [18]]) 
    bg2 = np.array([[-39], [45], [-3], [9], [-13], [18]])
    bg3 = np.array([[-87], [93], [-3], [9], [-13], [18]])
    bg4 = np.array([[-3], [9], [-15], [21], [-13], [18]])
    bg5 = np.array([[-63], [69], [-27], [33], [-13], [18]])
    bg6 = np.array([[-39], [45], [-39], [45], [-13], [18]])
    bg7 = np.array([[-27], [33], [-63], [69], [-13], [18]])
    bg8 = np.array([[-63], [69], [-63], [69], [-13], [18]])
    bg9 = np.array([[-3], [9], [-87], [93], [-13], [18]])
    bg10 = np.array([[-87], [93], [-87], [93], [-13], [18]])

    goal = [(A, bg1),
            (A, bg2),
            (A, bg3),
            (A, bg4),
            (A, bg5),
            (A, bg6),
            (A, bg7),
            (A, bg8),
            (A, bg9),
            (A, bg10)]


    b0 = np.array([[-5.5], [9.5], [-41.5], [45.5], [-13], [18]])
    b01 = np.array([[-119.5], [123.5], [-20.5], [24.5], [-13], [18]])
    b02 = np.array([[-119.5], [123.5], [-41.5], [45.5], [-13], [18]])
    b03 = np.array([[-119.5], [123.5], [-50], [54], [-13], [18]])
    b04 = np.array([[-5.5], [9.5], [-70.5], [74.5], [-13], [18]])
    b05 = np.array([[-10], [14], [0], [4], [-13], [18]])
    b06 = np.array([[-11], [15], [-14], [18], [-13], [18]])
    b07 = np.array([[-11], [15], [-83], [87], [-13], [18]])
    b08 = np.array([[-119.5], [123.5], [-60], [64], [-13], [18]])
    b09 = np.array([[-119.5], [123.5], [-74.5], [78.5], [-13], [18]])

    Theta = [ (A, b0),
              (A, b01),
              (A, b02),
              (A, b03),
              (A, b04),
              (A, b05),
              (A, b06),
              (A, b07),
              (A, b08),
              (A, b09),
    ]

    


    
    
    

    return obstacle , Theta, goal



if __name__ == '__main__':
    obstacle, goal, Theta = problem()
    ax1 = a3.Axes3D(plt.figure())

    for A,b in obstacle:
        plot_polytope_3d(A, b,  ax = ax1, color = 'red')
    for A,b in Theta:
        plot_polytope_3d(A, b,  ax = ax1, color = 'blue')
    for A,b in goal:
        plot_polytope_3d(A, b,  ax = ax1, color = 'green')
    plt.show()