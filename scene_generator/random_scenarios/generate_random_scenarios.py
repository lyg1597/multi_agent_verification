import numpy as np
import polytope as pc
import json
import pypoman as ppm
import matplotlib.pyplot as plt 
import pickle 
import time

def generate_scenarios(fn, x_max, y_max, num_obstacles):
    poly_list = []
    box_list = [
        # [[-2, -4], [2, 4]],
        [[-3, -3], [3, 3]],
        # [[-2, -6], [2, 6]],
    ]

    init_list = []
    initbox = [[-7, -7],[7, 7]]
    i = 0
    for center_x in range(0, int(x_max/2), 40):
        j = 0
        for center_y in range(0, y_max, 40):
            poly = pc.box2poly(np.array(initbox).T)
            if i == 0 and j == 0:
                poly = poly.translation(np.array([center_x + 5, center_y + 5]))
            elif j == 0:
                poly = poly.translation(np.array([center_x, center_y + 5]))    
            elif i == 0:
                poly = poly.translation(np.array([center_x + 5, center_y]))  
            else:  
                poly = poly.translation(np.array([center_x, center_y]))  
            init_list.append(poly)
            j += 1
        i += 1

    center_y += 40
    i = 0
    for center_x in range(0, int(x_max/2), 40):
        poly = pc.box2poly(np.array(initbox).T)
        if i == 0:
            poly = poly.translation(np.array([center_x + 5, center_y - 5]))
        else:
            poly = poly.translation(np.array([center_x, center_y - 5]))
        init_list.append(poly)
        i += 1
    
    goal_list = []
    goalbox = [[-7, -7],[7, 7]]
    i = 0
    for center_x in range(0, int(x_max/2), 40):
        j = 0
        for center_y in range(0, y_max, 40):
            poly = pc.box2poly(np.array(goalbox).T)
            if i == 0 and j == 0:
                poly = poly.translation(np.array([(x_max-center_x) - 5, center_y + 5]))
            elif j == 0:
                poly = poly.translation(np.array([(x_max-center_x), center_y + 5]))    
            elif i == 0:
                poly = poly.translation(np.array([(x_max-center_x) - 5, center_y]))  
            else:  
                poly = poly.translation(np.array([(x_max-center_x), center_y]))  
            goal_list.append(poly)
            j += 1
        i += 1

    center_y += 40
    i = 0
    for center_x in range(0, int(x_max/2), 40):
        poly = pc.box2poly(np.array(goalbox).T)
        if i == 0:
            poly = poly.translation(np.array([(x_max-center_x) - 5, center_y - 5]))
        else:
            poly = poly.translation(np.array([(x_max-center_x), center_y - 5]))
        goal_list.append(poly)
        i += 1

    fixed_box = [[-10, -10], [10, 10]]
    fixed_poly_list = []
    for center_x in range(20, x_max, 40):
        for center_y in range(20, y_max, 40):
            # theta = np.random.uniform(0,360)*np.pi/180
            poly = pc.box2poly(np.array(fixed_box).T)
            # poly = poly.rotation(i=0, j=1, theta=theta)
            poly = poly.translation(np.array([center_x, center_y]))
            fixed_poly_list.append(poly)
        

    static_obstacle_region = pc.Region(list_poly = fixed_poly_list)
    init_region = pc.Region(list_poly = init_list)
    goal_region = pc.Region(list_poly = goal_list)

    region_dim = 40
    num_region = x_max*y_max/(region_dim*region_dim)
    num_obstacle_region = int(num_obstacles / num_region)

    for i in range(int(x_max/region_dim)):
        for j in range(int(y_max/region_dim)):
            tmp = num_obstacle_region
            if (i == 0 and j == 0) or \
                (i == 0 and j ==int(y_max/region_dim)-1)  or \
                (i == int(x_max/region_dim)-1 and j == 0) or \
                (i == int(x_max/region_dim)-1 and j == int(y_max/region_dim)-1):
                tmp = max(tmp-2,0)
            elif i == 0 or i == int(x_max/region_dim)-1 or j == 0 or j == int(y_max/region_dim)-1:
                tmp = max(tmp-1,0)
            for k in range(tmp):
                time.sleep(0.01)
                center_x = np.random.uniform(0,region_dim) + i*region_dim
                center_y = np.random.uniform(0,region_dim) + j*region_dim
                while static_obstacle_region.contains(np.array([[center_x, center_y]]).T)[0] or \
                    init_region.contains(np.array([[center_x, center_y]]).T)[0] or \
                    goal_region.contains(np.array([[center_x, center_y]]).T)[0]:
                    time.sleep(0.01)
                    center_x = np.random.uniform(0,x_max)
                    center_y = np.random.uniform(0,y_max)        
                box_idx = np.random.randint(0,len(box_list))
                box = box_list[int(box_idx)]
                theta = np.random.uniform(0,360)*np.pi/180
                poly = pc.box2poly(np.array(box).T)
                poly = poly.rotation(i=0, j=1, theta=theta)
                poly = poly.translation(np.array([center_x, center_y]))
                poly_list.append(poly)
    
    poly_list += fixed_poly_list
    for poly in init_list:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = 'blue')

    for poly in goal_list:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = 'green')

    for poly in poly_list:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(poly.A,poly.b), color = 'red')
    plt.xlim(0, x_max)
    plt.ylim(0, y_max)
    plt.show()
    with open(fn, 'wb+') as f:
        pickle.dump((poly_list, init_list, goal_list), f)
    return poly_list, init_list, goal_list
    # pass

if __name__ == "__main__":
    fn = './scenario_obstacles'
    x_max = 480
    y_max = 120     
    num_obstacles = 120
    obstacle_list, init_list, goal_list = generate_scenarios(fn, x_max, y_max, num_obstacles)