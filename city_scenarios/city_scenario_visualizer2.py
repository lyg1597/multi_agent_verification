import json
from types import DynamicClassAttribute 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import polytope as pc
import numpy as np
from plot_polytope3d import *
import pypoman as ppm
import pickle 
import pyvista as pv

def visualize_city(fn, scale = 1):
    p = pv.Plotter()
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
    #     ax1.set_xlim(300578/scale, 300618/scale+1)
    #     ax1.set_ylim(5041258/scale, 5041289/scale+1)
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

            cloud = pv.PolyData(np.array(vertex_list)/scale)
            volume = cloud.delaunay_3d(alpha=10000000)
            shell = volume.extract_geometry()
            p.add_mesh(shell, opacity=1, color="#d9d9d9")
            # object_poly = pc.qhull(np.array(vertex_list)/scale)
            # # object_poly.b = object_poly.b/scale
            # plot_polytope_3d(object_poly, ax = ax1)
            # poly_list.append(object_poly)
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

            cloud = pv.PolyData(np.array(vertex_list)/scale)
            volume = cloud.delaunay_3d()
            shell = volume.extract_geometry()
            edges = shell.extract_feature_edges(20)
            p.add_mesh(shell, opacity=1, color="#d9d9d9")
            p.add_mesh(edges, color="k", line_width=1)
            # object_poly = pc.qhull(np.array(vertex_list)/scale)
            # # object_poly.b = object_poly.b/scale
            # plot_polytope_3d(object_poly, ax = ax1)
            # poly_list.append(object_poly)
    p.show()
    return poly_list
    
if __name__ == "__main__":
    fn = './d778e50a-cef4-4243-811c-955597acd779.json'
    output_fn = './city_scenario1/obstacle_list'
    # scale = 500
    poly_list = visualize_city(fn, scale=500)
    # with open(output_fn, 'wb+') as f:
    #     pickle.dump(poly_list, f)