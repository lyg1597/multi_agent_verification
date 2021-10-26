# Demo
# Written by: Kristina Miller
import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt
from plot_polytope3d import *
import copy
import polytope as pc 
import pickle 
import json

def parse_city_json(fn, scale = 500):
    # ax1 = a3.Axes3D(plt.figure())
    with open(fn,'r') as f:
        city_json = json.load(f)
    vertices = city_json["vertices"]
    CityObjects = city_json['CityObjects']
    # if 'geographicalExtent' in city_json['metadata']:
    #     geographicalExtent = city_json['metadata']['geographicalExtent']
    #     ax1.set_xlim(geographicalExtent[0]/scale, geographicalExtent[3]/scale+1)
    #     ax1.set_ylim(geographicalExtent[1]/scale, geographicalExtent[4]/scale+1)
    #     ax1.set_zlim(geographicalExtent[2]/scale, geographicalExtent[5]/scale+1)
    # else:
    #     ax1.set_xlim(84639/scale, 84639/scale+1)
    #     ax1.set_ylim(176246/scale, 176246/scale+1)
    #     ax1.set_zlim(12232/scale, 12232/scale+1)
        
    # for object_key in CityObjects:
    #     building = CityObjects[object_key]
    #     object_type = building['geometry'][0]['type']
    #     if object_type != "MultiSurface":
    #         continue
    #     boundaries = building['geometry'][0]['boundaries']
    #     for surface in boundaries:
    #         exterior_boundary = surface[0]
    #         x = []
    #         y = []
    #         z = []
    #         vertex_list = []
    #         for vertex in exterior_boundary:
    #             val = vertices[vertex]
    #             x.append(val[0])
    #             y.append(val[1])
    #             z.append(val[2])
    #             vertex_list.append(val)

    #         surface_plot(x,y,z,fig)
    poly_list = []
    for object_key in CityObjects:
        building = CityObjects[object_key]
        if len(building['geometry']) == 0:
            continue
        object_type = building['geometry'][0]['type']
        if object_type == "MultiSurface":
            boundaries = building['geometry'][0]['boundaries']

            x_max = -float('inf')
            x_min = float('inf')
            y_max = -float('inf')
            y_min = float('inf')
            z_max = -float('inf')
            z_min = float('inf')
            vertex_list = []
            for surface in boundaries:
                exterior_boundary = surface[0]
                x = []
                y = []
                z = []
                for vertex in exterior_boundary:
                    val = vertices[vertex]
                    x.append(val[0])
                    y.append(val[1])
                    z.append(val[2])
                    if val[0] > x_max:
                        x_max = val[0] 
                    if val[0] < x_min:
                        x_min = val[0]
                    if val[1] > y_max:
                        y_max = val[1] 
                    if val[1] < y_min:
                        y_min = val[1]
                    if val[2] > z_max:
                        z_max = val[2] 
                    if val[2] < z_min:
                        z_min = val[2]

                    vertex_list.append(val)

            object_poly = pc.qhull(np.array(vertex_list)/scale)
            # object_poly.b = object_poly.b/scale
            # plot_polytope_3d(object_poly, ax = ax1)
            poly_list.append(object_poly)
        elif object_type == "Solid":
            boundaries = building['geometry'][0]['boundaries'][0]

            x_max = -float('inf')
            x_min = float('inf')
            y_max = -float('inf')
            y_min = float('inf')
            z_max = -float('inf')
            z_min = float('inf')
            vertex_list = []
            for surface in boundaries:
                exterior_boundary = surface[0]
                x = []
                y = []
                z = []
                for vertex in exterior_boundary:
                    val = vertices[vertex]
                    x.append(val[0])
                    y.append(val[1])
                    z.append(val[2])
                    if val[0] > x_max:
                        x_max = val[0] 
                    if val[0] < x_min:
                        x_min = val[0]
                    if val[1] > y_max:
                        y_max = val[1] 
                    if val[1] < y_min:
                        y_min = val[1]
                    if val[2] > z_max:
                        z_max = val[2] 
                    if val[2] < z_min:
                        z_min = val[2]

                    vertex_list.append(val)

            object_poly = pc.qhull(np.array(vertex_list)/scale)
            # object_poly.b = object_poly.b/scale
            # plot_polytope_3d(object_poly, ax = ax1)
            poly_list.append(object_poly)
    # plt.show()
    return poly_list

def problem():
    A_rect = np.array([[-1,0,0],
                       [1,0,0],
                       [0,-1,0],
                       [0,1,0],
                       [0,0,-1],
                       [0,0,1]])

    fn = './d778e50a-cef4-4243-811c-955597acd779.json'
    # with open(fn, 'rb') as f:
    #     obstacle_poly_list = pickle.load(f)

    obstacle_poly_list = parse_city_json(fn, scale = 1000)

    obstacles = []
    for poly in obstacle_poly_list:
        obstacles.append((poly.A, np.expand_dims(poly.b,1)))

    #TODO: Create initial set
    # [70,190],[75,240]
    b00 = [
        np.array([[-61], [64], [-200], [203], [-4], [7]]),
        np.array([[-71], [74], [-210], [213], [-4], [7]]),
        np.array([[-71], [74], [-240], [243], [-4], [7]]),
        np.array([[-227], [230], [-221], [224], [-4], [7]]),
        np.array([[-218], [221], [-14], [17], [-4], [7]]), # 2D plan
        np.array([[-71], [74], [-200], [203], [-4], [7]]),
        np.array([[-71], [74], [-220], [223], [-4], [7]]),
        np.array([[-71], [74], [-230], [233], [-4], [7]]),
        np.array([[-40], [43], [-14], [17], [-4], [7]]),
        np.array([[-40], [43], [-24], [27], [-4], [7]]),
        np.array([[-40], [43], [-34], [37], [-4], [7]]),
        np.array([[-40], [43], [-44], [47], [-4], [7]]),
        np.array([[-40], [43], [-54], [57], [-4], [7]]),
        np.array([[-40], [43], [-64], [67], [-4], [7]]),
        np.array([[-218], [221], [-24], [27], [-4], [7]]),
        np.array([[-218], [221], [-34], [37], [-4], [7]]),
        np.array([[-218], [221], [-44], [47], [-4], [7]]),
        np.array([[-218], [221], [-54], [57], [-4], [7]]),
        np.array([[-222], [225], [-229], [232], [-4], [7]]),
        np.array([[-222], [225], [-239], [242], [-4], [7]]),
    ]
    Theta = []
    for b in b00:
        Theta.append((A_rect, b))
    #TODO: Create goal set

    tmp_g = [
        np.array([[-218], [224], [-11], [17], [-4], [10]]),
        np.array([[-232], [238], [-26], [32], [-4], [10]]),
        np.array([[-232], [238], [-84], [90], [-4], [10]]),
        np.array([[-40], [46], [-14], [20], [-4], [10]]),
        np.array([[-68], [74], [-200], [206], [-4], [10]]), # 2D plan
        np.array([[-192], [198], [-103], [109], [-4], [10]]),
        np.array([[-158], [164], [-87], [93], [-4], [10]]),
        np.array([[-118], [124], [-81], [87], [-14], [20]]),
        np.array([[-163], [169], [-127], [133], [-4], [10]]),
        np.array([[-144], [150], [-207], [213], [-4], [10]]),
        np.array([[-185], [191], [-202], [208], [-14], [20]]),
        np.array([[-139], [145], [-127], [133], [-14], [20]]),
        np.array([[-175], [181], [-183], [189], [-4], [10]]),
        np.array([[-200], [206], [-198], [204], [-14], [20]]),
        np.array([[-62], [68], [-116], [122], [-24], [30]]),
        np.array([[-138], [144], [-177], [183], [-4], [10]]),
        np.array([[-99], [105], [-207], [213], [-4], [10]]),
        np.array([[-89], [95], [-57], [63], [-4], [10]]),
        np.array([[-110], [116], [-51], [57], [-14], [20]]),
        np.array([[-84], [90], [-31], [37], [-14], [20]]),
    ]
    goal = []
    for i in range(len(tmp_g)):
        goal.append((A_rect,tmp_g[i]))

    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    fig = plt.figure()
    axes = fig.add_subplot(111, projection='3d')
    # ax1.set_xlim(84639/scale, 84639/scale+1)
    # ax1.set_ylim(176246/scale, 176246/scale+1)
    # ax1.set_zlim(12232/scale, 12232/scale+1)
    x_min = float('inf')
    x_max = -float('inf')
    y_min = float('inf')
    y_max = -float('inf')
    z_min = float('inf')
    z_max = -float('inf')
    for A,b in obs:
        verts = pc.extreme(pc.Polytope(A=A, b=b))
        x = verts[:,0]
        y = verts[:,1]  
        z = verts[:,2]
        x_min = min(x_min, np.min(x))
        x_max = max(x_max, np.max(x))
        y_min = min(y_min, np.min(y))
        y_max = max(y_max, np.max(y))
        z_min = min(z_min, np.min(z))
        z_max = max(z_max, np.max(z))
    
    axes.set_xlim(x_min-1, x_max+1)
    axes.set_ylim(y_min-1, y_max+1)
    axes.set_zlim(z_min-1, z_max+1)

    for A,b in obs:
        # ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')
        plot_polytope_3d(A, b, ax = axes, color = '#d9d9d9', trans = 0.5)

    for A,b in goal:
        plot_polytope_3d(A, b, ax = axes, color = 'green')

    for A,b in Theta:
        plot_polytope_3d(A, b, ax = axes, color = 'blue')

    axes.set_xlim(0, 250)
    axes.set_ylim(0, 250)
    axes.set_zlim(0, 30)

    plot_line_3d([0,0,0], [0,0,18],ax = axes)
    # plt.xlim(0, 24)
    # plt.ylim(0, 24)
    plt.grid()
    plt.show()
