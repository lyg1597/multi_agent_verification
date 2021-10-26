from plot_polytope3d import *
import polytope as pc 
import json 
import matplotlib.pyplot as plt 
import mpl_toolkits.mplot3d as a3

def scene_visualize_3d(fn):
    with open(fn,'r') as f:
        scene_config = json.load(f)
    ax = a3.Axes3D(plt.figure())    
    unsafe_list = scene_config['unsafeSet']
    for unsafe in unsafe_list:
        unsafe_type = unsafe[0]
        if unsafe_type == 'Matrix':
            A = np.array(unsafe[1][0])
            b = np.array(unsafe[1][1])
            plot_polytope_3d(A = A[:-6,:3], b = b[:-6,:], ax = ax, color = 'red')
    
    for agent_config in scene_config['agents']:
        mode_list = agent_config['mode_list']
        for i in range(1,len(mode_list)):
            mode = mode_list[i]
            plot_line_3d(mode[1][:3], mode[1][3:], ax = ax)

    
    plt.show()

if __name__ == "__main__":
    fn = './scene_3d_complex.json'
    scene_visualize_3d(fn)