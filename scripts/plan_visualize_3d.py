from plot_polytope3d_vista import *
import polytope as pc 
import json 
# import matplotlib.pyplot as plt 
# import mpl_toolkits.mplot3d as a3
import pyvista as pv

def scene_visualize_3d(fn):
    with open(fn,'r') as f:
        scene_config = json.load(f)
    # for agent0 in range(5, 20):
    #     for agent1 in range(agent0+1, 20):
    #         print(f"plotting {agent0}_{agent1}")
        pv.global_theme.background = 'white'
        ax = pv.Plotter(off_screen = False)
        unsafe_list = scene_config['unsafeSet']
        for unsafe in unsafe_list:
            unsafe_type = unsafe[0]
            if unsafe_type == 'Matrix':
                A = np.array(unsafe[1][0])
                b = np.array(unsafe[1][1])
                plot_polytope_3d(A = A[:-6,:3], b = b[:-6,:], ax = ax, color = '#bdbdbd', trans=1, edge=True)
        A_rect = np.array([[-1,0,0],
                        [1,0,0],
                        [0,-1,0],
                        [0,1,0],
                        [0,0,-1],
                        [0,0,1]])
        b = np.array([[50], [300], [50], [300], [2], [0]])
        plot_polytope_3d(A = A_rect, b = b, ax = ax, color = '#ffffff', trans=1, edge=True)
        
        # agent = [9,11]
        # agent = [10, 12]
        # agent = [10, 13]

        agent = [10,15]
        # agent = [9, 10, 12, 13, 15, 16]
        # agent = [12, 15]
        # color = ['#fdb462', '#ffed6f', '#fb8072', '#ffffb3', '#bebada', '#b3de69']
        color = ['#fdb462', '#ffed6f', '#fb8072', '#ffffb3', '#bebada', '#b3de69']
        for j in range(len(agent)):
            agent_idx = agent[j]
            agent_config = scene_config['agents'][agent_idx]
        # for agent_config in scene_config['agents']:
            mode_list = agent_config['mode_list']
            for i in range(1,len(mode_list)):
                mode = mode_list[i]
                plot_line_3d(mode[1][:3], mode[1][3:], ax = ax, line_width = 10, color = color[j])
            
            initialSet = agent_config['initialSet']
            poly = pc.box2poly(np.array(initialSet[1])[:,:3].T)
            plot_polytope_3d(A = poly.A, b = poly.b, ax = ax, color = 'b')

            goalSet = agent_config['goalSet']
            poly = pc.box2poly(np.array(goalSet[1])[:,:3].T)
            plot_polytope_3d(A = poly.A, b = poly.b, ax = ax, color = 'g')
        ax.show()
        # ax.show(screenshot=f'{agent0}_{agent1}.png')
        # ax.clear()

if __name__ == "__main__":
    fn = './data/comp3d-20-v1/comp3d-20-v1.json'
    scene_visualize_3d(fn)