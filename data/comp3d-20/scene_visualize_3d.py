from plot_polytope3d_vista import *
import polytope as pc 
import json 
# import matplotlib.pyplot as plt 
# import mpl_toolkits.mplot3d as a3
import pyvista as pv

def scene_visualize_3d(fn):
    with open(fn,'r') as f:
        scene_config = json.load(f)
    ax = pv.Plotter()
    unsafe_list = scene_config['unsafeSet']
    for unsafe in unsafe_list:
        unsafe_type = unsafe[0]
        if unsafe_type == 'Matrix':
            A = np.array(unsafe[1][0])
            b = np.array(unsafe[1][1])
            plot_polytope_3d(A = A[:-6,:3], b = b[:-6,:], ax = ax, color = '#d9d9d9', trans=1, edge=True)
    
    agent = [0,10]
    for agent_idx in agent:
        agent_config = scene_config['agents'][agent_idx]
        mode_list = agent_config['mode_list']
        for i in range(1,len(mode_list)):
            mode = mode_list[i]
            plot_line_3d(mode[1][:3], mode[1][3:], ax = ax, line_width = 10)

    ax.show()

if __name__ == "__main__":
    fn = './comp3d-20.json'
    scene_visualize_3d(fn)