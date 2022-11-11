import numpy as np
from typing import Optional, List, Tuple
import matplotlib.pyplot as plt
import time
# from verifier_interface import verifier

try:
    from common.Waypoint import Waypoint
except:
    from Waypoint import Waypoint

import rospy
from verification_msg.msg import StateVisualizeMsg, VerificationResultMsg
from std_msgs.msg import Int64
from verification_msg.srv import VerifierSrv, VerifierSrvResponse

class AgentCar:
    """
        AgentCar implements the ground vehicle agent used in the experiment
    """
    def __init__(self, idx: int, waypoints: List[Waypoint], init_state: List[float], factor = 5):
        self.idx = idx
        self.waypoint_list: List[Waypoint] = waypoints
        self.init_state: List[float] = init_state
        print(f'Publish states to /agent{self.idx}/state_visualize')
        self.state_publisher = rospy.Publisher(f'/agent{self.idx}/state_visualize', StateVisualizeMsg, queue_size=10)
        self.verification_time = []
        self.reachtube_time = []
        self.safety_checking_time = []
        self.total_time = []
        self.result_publisher = rospy.Publisher('/verifier/results', VerificationResultMsg, queue_size = 10)
        # self.safety_checking_lock = lock
        # self.status_publisher = rospy.Publisher(f'/agent{self.idx}/status', Int64, queue_size=10)
        self.num_hit = 0
        self.num_tube_computed = 0
        self.segment_time = []
        self.num_refine = []
        self.refine_time = []
        self.server_verify_time = []
        self.retry_threshold = 10
        self.factor = factor
        self.num_collision = 0

    def dynamics(self, t, state, mode_parameters):
        v = mode_parameters[2]
        omega = mode_parameters[3]

        theta = state[2]

        dx = v*np.cos(theta)
        dy = v*np.sin(theta)
        dtheta = omega

        dxref = mode_parameters[0]
        dyref = mode_parameters[1]
        dthetaref = 0

        return [dx,dy,dtheta,dxref,dyref,dthetaref]

    def run_simulation(self, init, parameters, time_bound, time_step, curr_plan):
        trajectory = [np.array(init)]
        t = 0
        trace = [[t]]
        trace[0].extend(init[0:3])
        i = 0
        theta_transform = 0
        while t < time_bound and not rospy.is_shutdown():
            state = trajectory[-1]

            x = state[0]
            y = state[1]
            theta = state[2]
            xref = state[3]
            yref = state[4]
            thetaref = state[5]

            vxref = parameters[0]
            vyref = parameters[1]

            k1 = 1
            k2 = 10
            k3 = 10

            xe = np.cos(theta) * (xref - x) + np.sin(theta) * (yref - y)
            ye = -np.sin(theta) * (xref - x) + np.cos(theta) * (yref - y)
            thetae = thetaref - theta

            xe_rotated = np.cos(theta_transform) * xe + np.sin(theta_transform) * ye
            ye_rotated = -np.sin(theta_transform) * xe + np.cos(theta_transform) * ye

            v = 1 * np.cos(thetae) + k1 * xe_rotated
            omega = 0 + 1 * (k2 * ye_rotated + k3 * np.sin(thetae))

            # r = ode(self.dynamics)
            # r.set_initial_value(state)

            # r.set_f_params([vxref, vyref, v, omega])

            # val = r.integrate(r.t + time_step)

            mode_parameters = [vxref, vyref, v, omega]
            v = mode_parameters[2]
            omega = mode_parameters[3]

            theta = state[2]

            dx = v*np.cos(theta)
            dy = v*np.sin(theta)
            dtheta = omega

            dxref = mode_parameters[0]
            dyref = mode_parameters[1]
            dthetaref = 0

            # return [dx,dy,dz,dtheta,dxref,dyref,dthetaref]

            val = [
                state[0] + dx*time_step,
                state[1] + dy*time_step,
                state[2] + dtheta*time_step,
                state[3] + dxref*time_step,
                state[4] + dyref*time_step,
                state[5] + dthetaref*time_step,
            ]

            trajectory.append(val)
            t += time_step
            i += 1
            trace.append([t])
            trace[i].extend(val[0:3])
            time.sleep(time_step*self.factor)

            curr_state = StateVisualizeMsg()
            curr_state.state = [val[0], val[1]]
            curr_state.plan = curr_plan
            # print(f"agent{self.idx}: publish state: {curr_state}")
            self.state_publisher.publish(curr_state)

        trace = np.array(trace)
        return trace

    # mode, initial_state, time_step, time_bound, x_d
    def TC_Simulate(self, Mode,initialCondition,time_bound):
        # print("TC simulate")
        time_step = 0.01
        time_bound = float(time_bound)
        # Mode = Mode[1:-1]
        # mode_parameters = Mode.split(";")
        # mode_parameters = [float(x) for x in mode_parameters]
        mode_parameters = Mode
        number_points = int(np.ceil(time_bound / time_step))
        t = [i * time_step for i in range(0, number_points)]
        if t[-1] != time_step:
            t.append(time_bound)
        newt = []
        for step in t:
            newt.append(float(format(step, '.4f')))
        t = np.array(newt)

        ref_vx = (mode_parameters[2] - mode_parameters[0]) / time_bound
        ref_vy = (mode_parameters[3] - mode_parameters[1]) / time_bound
        ref_theta = np.arctan2(ref_vy, ref_vx)

        # initialCondition = initialCondition.tolist()

        init = initialCondition+mode_parameters[0:2]+[ref_theta]
        # _, sym_rot_angle = get_transform_information(waypoint)
        trace = self.run_simulation(init, [ref_vx, ref_vy], time_bound, time_step, mode_parameters)
        # print("Dryvr trace: ", trace)
        # trace = runModel(mode_parameters[0:2] + [0] + list(initialCondition), time_bound, time_step, [ref_vx, ref_vy])
        trace = trace[:,0:4]
        return np.array(trace)

    def verifier(self, idx, plan: Waypoint, init_set):
        rospy.wait_for_service('verifyQuery')
        verify = rospy.ServiceProxy('verifyQuery', VerifierSrv)
        dynamics = "dryvr_dynamics/NN_car_TR_noNN"
        time_horizon = plan.time_bound
        variables_list = ['x', 'y', 'theta']

        # self.safety_checking_lock.acquire(blocking=True)
        res = verify(
            initset_lower = init_set[0], 
            initset_upper = init_set[1], 
            plan = plan.mode_parameters, 
            time_horizon = time_horizon,
            idx = idx, 
            dynamics = dynamics,
            variables_list = variables_list,
            initset_resolution = [0.5, 0.5, 0.1]
        )
        # self.safety_checking_lock.release()
        reachtube_time = res.rt_time 
        self.reachtube_time.append(reachtube_time)
        safety_checking_time = res.sc_time 
        self.safety_checking_time.append(safety_checking_time)
        tt_time = res.tt_time 
        self.total_time.append(tt_time)
        self.num_refine.append(res.num_ref)
        self.refine_time.append(res.refine_time)
        self.server_verify_time.append(res.verification_time)
        if res.from_cache > 0:
            self.num_hit += 1
        self.num_tube_computed += 1
        if res.res == 1:
            return 'Safe'
        else:
            return 'Unsafe'

    def execute_plan(self):
        """
            execute_plan(self)
            Description: This function execute given plan for an agent
        """
        print(f'Running agent {self.idx}')
        curr_init_set = self.init_state 
        all_trace = []
        res = "Safe"
        i = 0
        j = 0
        
        # Loop through each segment in the plan
        while i < len(self.waypoint_list):
            start_time = time.time()
            current_plan = self.waypoint_list[i]
            print(f'Start verifying plan for agent {self.idx}')
            verifier_start = time.time()
            # Call the verifier to verify if the plan is safe
            res = self.verifier(
                idx = self.idx, 
                plan = current_plan, 
                init_set = [
                    [curr_init_set[0]-1, curr_init_set[1]-1, curr_init_set[2]-0.01],
                    [curr_init_set[0]+1, curr_init_set[1]+1, curr_init_set[2]+0.01]
                ]
            )
            verification_time = time.time() - verifier_start
            self.verification_time.append(verification_time)
            print(f'Done verifying plan for agent {self.idx}, time {verification_time}')

            # If the plan is not safe, wait and retry
            if res != 'Safe':
                if j == 0:
                    self.num_collision += 1
                if j > self.retry_threshold:
                    print(f"agent{self.idx} plan unsafe")
                    self.stop_agent()
                    break
                time.sleep(15)
                self.segment_time.append(time.time() - start_time)
                j += 1
                continue
            else:
                j = 0
            
            # If the plan is safe, drive the agent to execute the plan
            trace = self.TC_Simulate(current_plan.mode_parameters, curr_init_set, current_plan.time_bound)
            # plt.plot(trace[:,1], trace[:,2])
            # plt.plot([current_plan.mode_parameters[0], current_plan.mode_parameters[2]], [current_plan.mode_parameters[1], current_plan.mode_parameters[3]], 'r')
            # plt.show()
            all_trace += trace.tolist()
            curr_init_set = [trace[-1][1], trace[-1][2], trace[-1][3]]

            self.segment_time.append(time.time() - start_time)
            i += 1

        result = VerificationResultMsg()
        result.idx = self.idx
        result.num_hit = self.num_hit
        result.total_length = self.num_tube_computed
        if res != "Safe":
            result.result = 0
        else:
            result.result = 1
        result.verification_time = self.verification_time
        result.reachtube_time = self.reachtube_time
        result.service_time = self.total_time 
        result.safety_checking_time = self.safety_checking_time
        result.segment_time = self.segment_time
        result.num_refine = self.num_refine
        result.refine_time = self.refine_time 
        result.server_verify_time = self.server_verify_time
        result.num_collision = self.num_collision
        self.result_publisher.publish(result)

        print(f'Done agent{self.idx} Verification Time', self.verification_time)
        print(f'Done agent{self.idx} Reachtube Time', self.reachtube_time)
        print(f'Done agent{self.idx} SafetyChecking Time', self.safety_checking_time)
        print(f'Done agent{self.idx} ServiceTotal Time', self.total_time)
        
        return all_trace

    def stop_agent(self):
        pass



if __name__ == "__main__":
    # wp = Waypoint('follow_waypoint',[20.4142135623735, 0.4142135623721741, 20.4142135623735, 3.4142135623721668],1.0,0)
    # plt.figure(1)
    # plt.figure(2)
    # plt.figure(3)
    # plt.figure(4)
    # for i in range(10):
    #     x_init = np.random.uniform(20.4142135623735-1, 20.4142135623735+1)
    #     y_init = np.random.uniform(0.4142135623721741-1,0.4142135623721741+1)
    #     theta_init = np.random.uniform(np.pi/2-0.5,np.pi/2+0.5)
    #     trace = TC_Simulate([20.4142135623735, 0.4142135623721741, 20.4142135623735, 2.4142135623721668], [x_init,y_init,theta_init], 3)
    #     trace = np.array(trace)
    #     plt.figure(1)
    #     plt.plot(trace[:,1],trace[:,2])
    #     # plt.plot([4.164213562373057,4.664213562373057,4.664213562373057,4.164213562373057,4.164213562373057],
    #     #         [22.16421356237288, 22.16421356237288, 22.66421356237288, 22.66421356237288, 22.16421356237288])
    #     plt.figure(2)
    #     plt.plot(trace[:,0],trace[:,1])
    #     plt.figure(3)
    #     plt.plot(trace[:,0],trace[:,2])
    #     plt.figure(4)
    #     plt.plot(trace[:,0],trace[:,3])
    # plt.show()

    # # res = get_flowstar_parameters([5.5, 5.5, 7.621320343559585, 7.621320343559585], np.array([[5.4, 5.4, -0.1], [5.6, 5.6, 0.1]]), 0.01, 3, "follow_waypoint")
    # print(res)
    rospy.init_node('agent_test')

    wp = [
        Waypoint('follow_waypoint',[20.4142135623735, 0.4142135623721741, 20.4142135623735, 5.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 5.4142135623721741, 20.4142135623735, 10.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 10.4142135623721741, 20.4142135623735, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[20.4142135623735, 15.4142135623721741, 25.4142135623735, 15.4142135623721668],3.0,0),
        Waypoint('follow_waypoint',[25.4142135623735, 15.4142135623721741, 30.4142135623735, 15.4142135623721668],3.0,0),
    ]

    x_init = np.random.uniform(20.4142135623735-1, 20.4142135623735+1)
    y_init = np.random.uniform(0.4142135623721741-1,0.4142135623721741+1)
    theta_init = np.random.uniform(np.pi/2-0.5,np.pi/2+0.5)
    car = AgentCar(0, wp, [x_init,y_init,theta_init])
    trace = car.execute_plan()
    trace = np.array(trace)

    plt.figure(1)
    plt.plot(trace[:,1],trace[:,2])
    plt.plot(trace[:,1],trace[:,2], 'r.')
    plt.figure(2)
    plt.plot(trace[:,0],trace[:,1])
    plt.plot(trace[:,0],trace[:,1], 'r.')
    plt.figure(3)
    plt.plot(trace[:,0],trace[:,2])
    plt.plot(trace[:,0],trace[:,2], 'r.')
    plt.figure(4)
    plt.plot(trace[:,0],trace[:,3])
    plt.plot(trace[:,0],trace[:,3], 'r.')
    plt.show()
    # print(os.getcwd())
    # print(os.path.realpath(__file__))