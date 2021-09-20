from multiprocessing import Process
import threading
import numpy as np
import matplotlib.pyplot as plt 
import time

import rospy
from std_msgs.msg import Float32MultiArray
from verification_msg.msg import StateVisualizeMsg

try:
    from src.agent_car import AgentCar
except:
    from agent_car import AgentCar
try:
    from src.Waypoint import Waypoint
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
        self.subscribers = self.create_subscriber()

    def state_handler(self, msg, idx):
        state = msg.state
        plan = msg.plan
        self.agent_state_dict[idx].append(state)
        self.agent_plan_dict[idx][0].append(plan[0])
        self.agent_plan_dict[idx][0].append(plan[2])
        self.agent_plan_dict[idx][1].append(plan[1])
        self.agent_plan_dict[idx][1].append(plan[3])
        # print(self.agent_state_dict[idx][-1])

    def create_subscriber(self):
        # Create state subscribers
        subscriber_list = []
        for i in range(self.num_agent):
            sub = rospy.Subscriber(f'/agent{i}/state_visualize', StateVisualizeMsg, lambda msg, idx=i:self.state_handler(msg, idx), queue_size=1)
            subscriber_list.append(sub)
        return subscriber_list
        
    def visualize_agent_data(self):
        i = 0
        while True:
            plt.clf()
            for idx in range(self.num_agent):
                agent_trajectory = self.agent_state_dict[idx]
                agent_plan_x = self.agent_plan_dict[idx][0]
                agent_plan_y = self.agent_plan_dict[idx][1]
                # print(agent_trajectory)
                if agent_trajectory != []:
                    # print("plot_trajectory")
                    point = agent_trajectory[-1]
                    plt.plot(point[0], point[1], 'r.')
                    plt.plot(agent_plan_x, agent_plan_y, 'b')
            plt.pause(0.000001)
            i+=1
            # time.sleep(0.1)
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
        theta_init = np.random.uniform(np.pi/2-0.5,np.pi/2+0.5)
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