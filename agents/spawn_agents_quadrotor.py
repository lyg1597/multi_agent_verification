from multiprocessing import Process
import threading
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import time
import copy
import sys
import json

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from verification_msg.msg import StateVisualizeMsg, ReachtubeMsg, VerificationResultMsg
from verification_msg.srv import UnsafeSetSrv, UnsafeSetSrvResponse

from agent_car import AgentCar
from agent_quadrotor import AgentQuadrotor
from common.Waypoint import Waypoint


class AgentData:
    def __init__(self, num_agent, unsafeset_list):
        self.num_agent = num_agent
        self.agent_state_dict = {}
        for i in range(num_agent):
            self.agent_state_dict[i] = []
        self.agent_plan_dict = {}
        for i in range(num_agent):
            self.agent_plan_dict[i] = [[],[]]
        self.state_subscribers = self.create_subscriber()
        self.reachtube_subscriber = rospy.Subscriber('/verifier/reachtube', ReachtubeMsg,self.reachtube_handler, queue_size = 10)

        self.tube_plan = {}
        self.tube = {}
        self.tube_cached = {}

        self.plotted_points = {}
        self.plotted_plan = {}
        self.plotted_tube = {}
        self.unsafeset_list = unsafeset_list

        self.result_subscriber = rospy.Subscriber('/verifier/results', VerificationResultMsg, self.result_handler, queue_size = 10)
        self.results = {}
        self.done_list = []
        for i in range(num_agent):
            self.done_list.append(False)

    def result_handler(self, data):
        idx = data.idx
        num_hit = data.num_hit 
        total_length = data.total_length
        result = data.result
        # total_time = data.total_time 
        # segment_time = data.segment_time 
        verification_time = data.verification_time 
        reachtube_time = data.reachtube_time 
        service_time = data.service_time
        safety_checking_time = data.safety_checking_time
        segment_time = data.segment_time
        num_refine = data.num_refine
        refine_time = data.refine_time
        server_verify_time = data.server_verify_time
        self.results[idx] = {
            "num_hit":num_hit,
            "total_length":total_length,
            "verification_time":verification_time, 
            "reachtube_time":reachtube_time ,
            "server_time":service_time,
            "safety_checking_time":safety_checking_time,
            "segment_time":segment_time,
            "num_refine":num_refine,
            "refine_time":refine_time,
            "server_verify_time":server_verify_time,
            "result":result
        }
        self.done_list[idx] = True

    def state_handler(self, msg, idx):
        state = msg.state
        plan = msg.plan
        self.agent_state_dict[idx].append(state)
        # self.agent_plan_dict[idx][0].append(plan[0])
        # self.agent_plan_dict[idx][0].append(plan[2])
        # self.agent_plan_dict[idx][1].append(plan[1])
        # self.agent_plan_dict[idx][1].append(plan[3])
        self.agent_plan_dict[idx].append([[plan[0],plan[3]],[plan[1],plan[4]]])  
        # print(self.agent_state_dict[idx][-1])

    def reachtube_handler(self, msg):
        idx = msg.idx 
        shape = (msg.tube.layout.dim[0].size, msg.tube.layout.dim[1].size, msg.tube.layout.dim[2].size)
        tube = np.array(msg.tube.data).reshape(shape).tolist() 
        plan = msg.plan 
        from_cache = msg.from_cache
        self.tube_plan[idx] = plan
        self.tube[idx] = tube
        self.tube_cached[idx] = from_cache
        # print(tube)
        pass

    def create_subscriber(self):
        # Create state subscribers
        subscriber_list = []
        for i in range(self.num_agent):
            sub = rospy.Subscriber(f'/agent{i}/state_visualize', StateVisualizeMsg, lambda msg, idx=i:self.state_handler(msg, idx), queue_size=10)
            subscriber_list.append(sub)
        return subscriber_list
        
    def plot_tube(self, ax, tube, from_cache = 0):
        # print(len(tube))
        for i in range(0,len(tube),5):
            box = tube[i]
            x = min(box[0][0], box[1][0])
            y = min(box[0][1], box[1][1])
            width = abs(box[0][0] - box[1][0])
            height = abs(box[0][1] - box[1][1])
            # print(i, x, y, width, height)
            if from_cache != 0:
                rect = patches.Rectangle((x,y), width, height, edgecolor = '#ffed6f', facecolor = '#ffed6f')
            else:
                rect = patches.Rectangle((x,y), width, height, edgecolor = '#b3de69', facecolor = '#b3de69')
            ax.add_patch(rect)
    
    def plot_unsafe(self, ax):
        for i in range(len(self.unsafeset_list)):
            box = self.unsafeset_list[i]
            x = min(box[0][0], box[1][0])
            y = min(box[0][1], box[1][1])
            width = abs(box[0][0] - box[1][0])
            height = abs(box[0][1] - box[1][1])
            rect = patches.Rectangle((x,y), width, height, edgecolor = '#d9d9d9', facecolor = '#d9d9d9')
            ax.add_patch(rect)

    def visualize_agent_data(self):
        i = 0
        ax = plt.gca()
        self.plot_unsafe(ax)
        while True:
            # plt.clf()
            # for idx in range(self.num_agent):
            #     agent_trajectory = self.agent_state_dict[idx]
            #     agent_plan_x = self.agent_plan_dict[idx][0]
            #     agent_plan_y = self.agent_plan_dict[idx][1]
            #     # print(agent_trajectory)
            #     if agent_trajectory != []:
            #         # print("plot_trajectory")
            #         point = agent_trajectory[-1]
            #         plt.plot(point[0], point[1], 'r.')
            #         plt.plot(agent_plan_x, agent_plan_y, 'b')

            for idx in range(self.num_agent):
                agent_trajectory = self.agent_state_dict[idx]
                # agent_plan_x = self.agent_plan_dict[idx][0]
                # agent_plan_y = self.agent_plan_dict[idx][1]
                if agent_trajectory != []:
                    point = agent_trajectory[-1]
                    # if idx in self.plotted_points:
                    #     prev_plotted_point = self.plotted_points[idx].pop(0)
                    #     prev_plotted_point.remove()
                    self.plotted_points[idx] = plt.plot(point[0], point[1], color = '#fb8072', marker = '.')

                    if idx in self.plotted_plan:
                        if self.plotted_plan[idx] != self.agent_plan_dict[idx][-1]:
                            # print(f'agent{idx} plot plan')
                            agent_plan_x = self.agent_plan_dict[idx][-1][0]
                            agent_plan_y = self.agent_plan_dict[idx][-1][1]
                            plt.plot(agent_plan_x, agent_plan_y, '#8dd3c7')
                            self.plotted_plan[idx] = self.agent_plan_dict[idx][-1]
                    else:
                        # print(f'agent{idx} plot plan')
                        agent_plan_x = self.agent_plan_dict[idx][-1][0]
                        agent_plan_y = self.agent_plan_dict[idx][-1][1]
                        plt.plot(agent_plan_x, agent_plan_y, '#8dd3c7')
                        self.plotted_plan[idx] = self.agent_plan_dict[idx][-1]

                    if idx in self.tube:
                        if idx in self.plotted_tube:
                            if self.plotted_tube[idx] != self.tube_plan[idx]:
                                # print(f'agent{idx} plot tube')
                                tube = self.tube[idx]
                                from_cache = self.tube_cached[idx]
                                self.plot_tube(ax, tube, from_cache)
                                self.plotted_tube[idx] = self.tube_plan[idx]
                        else:
                            # print(f'agent{idx} plot tube')
                            tube = self.tube[idx]
                            from_cache = self.tube_cached[idx]
                            self.plot_tube(ax, tube, from_cache)
                            self.plotted_tube[idx] = self.tube_plan[idx]
                    
            plt.pause(0.000001)
            i+=1
            time.sleep(0.2)
            if np.all(self.done_list): 
                print("all agents finished")
                print(self.results)
                plt.savefig('./res_fig.png')
                plt.close()
                f = open('res.json', 'w+')
                json.dump(self.results, f)
                return 

            if rospy.is_shutdown():
                print(60)
                plt.close()
                return

def get_edge(edge_list, src_id):
    for edge in edge_list:
        if edge[0] == src_id:
            return edge[0], edge[1]
    return None, None 

def get_waypoints(init_mode_id, edge_list, mode_list):
    wp = []
    id = init_mode_id
    while id is not None:
        mode = mode_list[id]
        wp.append(Waypoint('follow_waypoint',mode[1],3.0,0))
        src, dest = get_edge(edge_list, id)
        id = dest    
    return wp

if __name__ == "__main__":
    rospy.init_node('spawn_agents')

    wp_list = []
    unsafeset_list = []
    if len(sys.argv) > 1:
        fn = sys.argv[1]
        f = open(fn, 'r')
        agent_data = json.load(f)
        num_agents = len(agent_data['agents'])
        # Handle unsafe sets
        unsafe_sets = agent_data['unsafeSet']
        for unsafe in unsafe_sets:
            unsafeset_list.append(unsafe[1])
        # Handle waypoints
        for i in range(num_agents):
            agent = agent_data['agents'][i]
            init_mode_id = agent['initialModeID']
            edge_list = agent['edge_list']
            mode_list = agent['mode_list']
            wp = get_waypoints(init_mode_id, edge_list, mode_list)
            wp_list.append(wp)
    else: 
        num_agents = 5
        
        # raw_wp_list = [
        #     [20.0, 5.0, 20.0, 10.0],
        #     [20.0, 10.0, 20.0, 15.0],
        #     [20.0, 15.0, 25.0, 15.0],
        #     [25.0, 15.0, 30.0, 15.0],
        #     [30.0, 15.0, 35.0, 15.0],
        #     [35.0, 15.0, 35.0, 20.0],
        # ]
        # raw_unsafeset_list = [
        #     ["Box", [[23,5,-100],[27,12,100]]],
        #     ["Box", [[35.8,18,-100],[43,20,100]]],
        # ]

        raw_wp_list = [
            [20.0, 5.0, 0, 20.0, 10.0, 0],
            [20.0, 10.0, 0, 20.0, 15.0, 0],
            [20.0, 15.0, 0, 25.0, 15.0, 0],
            [25.0, 15.0, 0, 30.0, 15.0, 0],
            [30.0, 15.0, 0, 35.0, 15.0, 0],
            [35.0, 15.0, 0, 35.0, 20.0, 0],
        ]
        raw_unsafeset_list = [
            ["Box", [[23,5,-100,-100,-100,-100],[27,12,100,100,100,100]]],
            ["Box", [[35.8,18,-100,-100,-100,-100],[43,20,100,100,100,100]]],
        ]

        # raw_wp_list = [
        #     [0, 0, -5, 0],
        #     [-5, 0, -10, 0],
        #     [-10, 0, -10, 5],
        #     [-10, 5, -15, 5],
        #     [-15, 5, -15, 0],
        #     [-15, 0, -15, -5]
        # ]
        # raw_unsafeset_list = [
        #     [[-10, -5, -100],[0,-3,100]],
        # ]
        
        for i in range(num_agents):
            wp1 = []
            for j in range(len(raw_wp_list)):
                raw_wp = copy.deepcopy(raw_wp_list[j])
                raw_wp[0] += i*12
                raw_wp[3] += i*12
                wp1.append(Waypoint('follow_waypoint',raw_wp,4.0,0))
            for j in range(len(raw_unsafeset_list)):
                raw_unsafeset = copy.deepcopy(raw_unsafeset_list[j][1])
                raw_unsafeset[0][0] += i*12
                raw_unsafeset[1][0] += i*12
                unsafeset_list.append(raw_unsafeset)
            wp_list.append(wp1)

    # tmp = unsafeset_list
    # unsafeset_list = []
    set_unsafeset = rospy.ServiceProxy('set_unsafe', UnsafeSetSrv)
    unsafe_array = np.array(unsafeset_list)
    unsafe_shape = unsafe_array.shape
    dim_list = []
    for i in range(len(unsafe_shape)):
        dim = MultiArrayDimension()
        dim.size = unsafe_shape[i]
        dim_list.append(dim)
    unsafe_msg = Float32MultiArray()
    unsafe_msg.layout.dim = dim_list 
    unsafe_msg.data = unsafe_array.flatten().tolist()

    set_unsafeset(unsafe_list=unsafe_msg, type = 'Box')

    # unsafeset_list = tmp
    agent_data = AgentData(num_agents, unsafeset_list)
    visualize_process = threading.Thread(target = agent_data.visualize_agent_data)
    visualize_process.start()

    safety_checking_lock = threading.Lock()
    agent_process_list = []
    for i in range(num_agents):
        # theta = np.arctan2(
        #     wp_list[i][0].mode_parameters[3] - wp_list[i][0].mode_parameters[1],
        #     wp_list[i][0].mode_parameters[2] - wp_list[i][0].mode_parameters[0]
        # )
        # # x_init = np.random.uniform(wp_list[i][0].mode_parameters[0] - 1, wp_list[i][0].mode_parameters[0] + 1)
        # # y_init = np.random.uniform(wp_list[i][0].mode_parameters[1] - 1, wp_list[i][0].mode_parameters[1] + 1)
        # # theta_init = np.random.uniform(np.pi/2-0.5,np.pi/2+0.5)
        # x_init = wp_list[i][0].mode_parameters[0]
        # y_init = wp_list[i][0].mode_parameters[1]
        # theta_init = theta
        # init_state = [x_init, y_init, theta_init]

        # agent = AgentCar(i, wp_list[i], init_state)

        x_init = wp_list[i][0].mode_parameters[0]
        y_init = wp_list[i][0].mode_parameters[1]
        z_init = wp_list[i][0].mode_parameters[2]
        vx_init = 0
        vy_init = 0
        vz_init = 0
        init_state = [x_init, y_init, z_init, vx_init, vy_init, vz_init]
        agent = AgentQuadrotor(i, wp_list[i], init_state)

        # p = Process(target=agent.execute_plan)
        # p.start()
        p = threading.Thread(target = agent.execute_plan)
        p.start()
        agent_process_list.append(p)
    
    for p in agent_process_list:
        p.join()

    visualize_process.join()