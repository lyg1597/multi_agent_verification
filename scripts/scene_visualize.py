import json
import numpy as np
import matplotlib.pyplot as plt
import polytope as pc
import pypoman as ppm
import os 

def scene_visualize(fn, title, x_lim, y_lim):
    f = open(fn,'r')
    config = json.load(f)

    unsafe_set = []
    if "unsafeSet" in config:
        unsafe_set = config["unsafeSet"]
    
    goal_set = []
    if "goalSet" in config:
        goal_set = config["goalSet"]

    mode_list = config["agents"][0]["mode_list"]

    initial_set = config["agents"][0]["initialSet"]

    edge_list = config["agents"][0]["edge_list"]

    time_list = config["agents"][0]["timeHorizons"]

    for i in range(len(unsafe_set)):
        if unsafe_set[i][0] == "Box":
            box = unsafe_set[i][1]
            poly = pc.box2poly(np.array(box).T)
        elif unsafe_set[i][0] == "Matrix":
            mats = unsafe_set[i][1]
            poly = pc.Polytope(np.array(mats[0]),np.array(mats[1]))
        elif unsafe_set[i][0] == "Vertices":
            vertices =  np.array(unsafe_set[i][1])
            poly = pc.qhull(np.array(vertices))  
        poly = pc.projection(poly,[1,2])
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#d9d9d9', alpha = 1)

    for i in range(len(goal_set)):
        box = goal_set[i][1]
        poly = pc.box2poly(np.array(box).T)
        poly = pc.projection(poly,[1,2])
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#8dd3c7')

    for i in range(len(mode_list)):
        mode = mode_list[i][1]
        if len(mode) == 4:
            # plt.plot([mode[0],mode[2]],[mode[1],mode[3]],'k.')
            plt.plot([mode[0],mode[2]],[mode[1],mode[3]],'k')
        elif len(mode) == 6:
            # plt.plot([mode[0],mode[3]],[mode[1],mode[4]],'k.')
            plt.plot([mode[0],mode[3]],[mode[1],mode[4]],'k')
    
    box = initial_set[1]
    poly = pc.box2poly(np.array(box).T)
    poly = pc.projection(poly,[1,2])
    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = '#80b1d3', alpha = 1, linewidth=1)
            
    print("Number of modes:", len(mode_list))
    print("Number of edges:", len(edge_list))
    print("Number of obsatles:", len(unsafe_set))
    print("Total time horizon:", sum(time_list))
    plt.xlim(x_lim[0],x_lim[1])
    plt.ylim(y_lim[0],y_lim[1])
    # plt.title("title")
    plt.show()

if __name__ == "__main__":
    fn = "scenarios/c_s3_dr.json"
    fn = os.path.abspath(fn)
    title = "c_s4a-3"
    x_lim = [-6,250]
    y_lim = [-6,150]
    scene_visualize(fn, title, x_lim, y_lim)
