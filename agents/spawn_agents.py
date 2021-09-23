from multiprocessing import Process
import threading
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import time

import rospy
from std_msgs.msg import Float32MultiArray
from verification_msg.msg import StateVisualizeMsg, ReachtubeMsg

try:
    from src.agent_car import AgentCar
except:
    from agent_car import AgentCar
try:
    from common.Waypoint import Waypoint
except:
    from Waypoint import Waypoint


class AgentData:
    def __init__(self, num_agent):
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

        self.plotted_points = {}
        self.plotted_plan = {}
        self.plotted_tube = {}

    def state_handler(self, msg, idx):
        state = msg.state
        plan = msg.plan
        self.agent_state_dict[idx].append(state)
        # self.agent_plan_dict[idx][0].append(plan[0])
        # self.agent_plan_dict[idx][0].append(plan[2])
        # self.agent_plan_dict[idx][1].append(plan[1])
        # self.agent_plan_dict[idx][1].append(plan[3])
        self.agent_plan_dict[idx].append([[plan[0],plan[2]],[plan[1],plan[3]]])  
        # print(self.agent_state_dict[idx][-1])

    def reachtube_handler(self, msg):
        idx = msg.idx 
        shape = (msg.tube.layout.dim[0].size, msg.tube.layout.dim[1].size, msg.tube.layout.dim[2].size)
        tube = np.array(msg.tube.data).reshape(shape).tolist() 
        plan = msg.plan 
        self.tube_plan[idx] = plan
        self.tube[idx] = tube
        # print(tube)
        pass

    def create_subscriber(self):
        # Create state subscribers
        subscriber_list = []
        for i in range(self.num_agent):
            sub = rospy.Subscriber(f'/agent{i}/state_visualize', StateVisualizeMsg, lambda msg, idx=i:self.state_handler(msg, idx), queue_size=1)
            subscriber_list.append(sub)
        return subscriber_list
        
    def plot_tube(self, ax, tube):
        print(len(tube))
        for i in range(len(tube)):
            box = tube[i]
            x = min(box[0][0], box[1][0])
            y = min(box[0][1], box[1][1])
            width = abs(box[0][0] - box[1][0])
            height = abs(box[0][1] - box[1][1])
            # print(i, x, y, width, height)
            rect = patches.Rectangle((x,y), width, height, edgecolor = 'b', facecolor = 'b')
            ax.add_patch(rect)
        pass
    
    def visualize_agent_data(self):
        i = 0
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
                    self.plotted_points[idx] = plt.plot(point[0], point[1], 'r.')

                    if idx in self.plotted_plan:
                        if self.plotted_plan[idx] != self.agent_plan_dict[idx][-1]:
                            print('plot plan')
                            agent_plan_x = self.agent_plan_dict[idx][-1][0]
                            agent_plan_y = self.agent_plan_dict[idx][-1][1]
                            plt.plot(agent_plan_x, agent_plan_y, 'b')
                            self.plotted_plan[idx] = self.agent_plan_dict[idx][-1]
                    else:
                        print('plot plan')
                        agent_plan_x = self.agent_plan_dict[idx][-1][0]
                        agent_plan_y = self.agent_plan_dict[idx][-1][1]
                        plt.plot(agent_plan_x, agent_plan_y, 'b')
                        self.plotted_plan[idx] = self.agent_plan_dict[idx][-1]

                    if idx in self.tube:
                        if idx in self.plotted_tube:
                            if self.plotted_tube[idx] != self.tube_plan[idx]:
                                print('plot tube')
                                ax = plt.gca()
                                tube = self.tube[idx]
                                self.plot_tube(ax, tube)
                                self.plotted_tube[idx] = self.tube_plan[idx]
                        else:
                            print('plot tube')
                            tube = self.tube[idx]
                            ax = plt.gca()
                            self.plot_tube(ax, tube)
                            self.plotted_tube[idx] = self.tube_plan[idx]
                    
            plt.pause(0.000001)
            i+=1
            time.sleep(0.1)
            if rospy.is_shutdown():
                print(60)
                plt.close()
                return

if __name__ == "__main__":
    rospy.init_node('spawn_agents')

    num_agents = 3
    
    wp1 = [
        Waypoint('follow_waypoint',[20.4142135623735, 0.4142135623721741, 20.4142135623735, 5.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 5.4142135623721741, 20.4142135623735, 10.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 10.4142135623721741, 20.4142135623735, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 15.4142135623721741, 25.4142135623735, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[25.4142135623735, 15.4142135623721741, 30.4142135623735, 15.4142135623721668],3.0,0),
    ]

    wp2 = [
        Waypoint('follow_waypoint',[20.4142135623735+5, 0.4142135623721741, 20.4142135623735+5, 5.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735+5, 5.4142135623721741, 20.4142135623735+5, 10.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735+5, 10.4142135623721741, 20.4142135623735+5, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735+5, 15.4142135623721741, 25.4142135623735+5, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[25.4142135623735+5, 15.4142135623721741, 30.4142135623735+5, 15.4142135623721668],3.0,0),
    ]

    wp3 = [
        Waypoint('follow_waypoint',[20.4142135623735-5, 0.4142135623721741, 20.4142135623735-5, 5.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735-5, 5.4142135623721741, 20.4142135623735-5, 10.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735-5, 10.4142135623721741, 20.4142135623735-5, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735-5, 15.4142135623721741, 25.4142135623735-5, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[25.4142135623735-5, 15.4142135623721741, 30.4142135623735-5, 15.4142135623721668],3.0,0),
    ]

    wp_list = [wp1, wp2, wp3]

    agent_data = AgentData(num_agents)
    visualize_process = threading.Thread(target = agent_data.visualize_agent_data)
    visualize_process.start()

    agent_process_list = []
    for i in range(num_agents):
        x_init = np.random.uniform(wp_list[i][0].mode_parameters[0] - 1, wp_list[i][0].mode_parameters[0] + 1)
        y_init = np.random.uniform(wp_list[i][0].mode_parameters[1] - 1, wp_list[i][0].mode_parameters[1] + 1)
        # theta_init = np.random.uniform(np.pi/2-0.5,np.pi/2+0.5)
        theta_init = np.pi/2
        init_set = [x_init, y_init, theta_init]

        agent = AgentCar(i, wp_list[i], init_set)
        # p = Process(target=agent.execute_plan)
        # p.start()
        p = threading.Thread(target = agent.execute_plan)
        p.start()
        agent_process_list.append(p)
    
    for p in agent_process_list:
        p.join()

    visualize_process.join()