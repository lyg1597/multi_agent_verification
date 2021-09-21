import torch
from scipy.integrate import ode
import numpy as np
import polytope as pc
from typing import Optional, List, Tuple
import math
try:
    from src.Waypoint import Waypoint
except:
    from Waypoint import Waypoint
import matplotlib.pyplot as plt
import time 
# from verifier_interface import verifier
import rospy

from verification_msg.msg import StateVisualizeMsg
from verification_msg.srv import VerifierSrv, VerifierSrvResponse

class AgentCar:
    def __init__(self, idx: int, waypoints: List[Waypoint], init_set: List[float]):
        self.idx = idx
        self.waypoint_list: List[Waypoint] = waypoints
        self.init_set: List[float] = init_set
        print(f'Publish states to /agent{self.idx}/state_visualize')
        self.state_publisher = rospy.Publisher(f'/agent{self.idx}/state_visualize', StateVisualizeMsg, queue_size=1)

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

            r = ode(self.dynamics)
            r.set_initial_value(state)

            r.set_f_params([vxref, vyref, v, omega])

            val = r.integrate(r.t + time_step)
            trajectory.append(val)
            t += time_step
            i += 1
            trace.append([t])
            trace[i].extend(val[0:3])
            time.sleep(time_step*10)

            curr_state = StateVisualizeMsg()
            curr_state.state = [val[0], val[1]]
            curr_state.plan = curr_plan
            # print(f"agent{self.idx}: publish state: {curr_state}")
            self.state_publisher.publish(curr_state)

        trace = np.array(trace)
        return trace

    def get_transform_information(self, waypoint: Waypoint) -> Tuple[np.array, float]:
        # mode: str = waypoint.mode
        mode_parameters: Optional[List[float]] = waypoint.mode_parameters
        # time_bound = waypoint.time_bound
        if mode != "follow_waypoint":
            raise NotImplementedError("haven't implemented modes other than follow waypoint for these dynamics")
        # old_center = prev_mode_parameters
        dot = (mode_parameters[2] - mode_parameters[0])
        det = (mode_parameters[3] - mode_parameters[1])
        dir_angle = math.atan2(det, dot)
        # ref_vx = (mode_parameters[3] - mode_parameters[0]) / time_bound
        # ref_vy = (mode_parameters[4] - mode_parameters[1]) / time_bound
        translation_vector: np.array = np.zeros((3,))
        translation_vector[:2] = -1 * np.array(mode_parameters[2:])
        translation_vector[2] = -dir_angle
        return translation_vector, -dir_angle

    # mode, initial_state, time_step, time_bound, x_d
    def TC_Simulate(self, Mode,initialCondition,time_bound):
        # print("TC simulate")
        time_step = 0.01;
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

    def transform_poly_to_virtual(self, poly, transform_information):
        translation_vector, new_system_angle = transform_information
        poly_out: pc.Polytope = poly.translation(translation_vector)
        # box = PolyUtils.get_bounding_box(poly_out)
        # poly_out = pc.box2poly(box.T)
        return  poly_out.rotation(i=0, j=1, theta=new_system_angle)


    def transform_mode_to_virtual(self, waypoint: Waypoint, transform_information):
        point = waypoint.mode_parameters
        xs1 = point[0]  # x_i
        ys1 = point[1]  # y_i
        xd1 = point[2]
        yd1 = point[3]
        translation_vector, sc = transform_information
        x_n = translation_vector[0]
        y_n = translation_vector[1]
        xs2 = (xs1 + x_n) * math.cos(sc) - (ys1 + y_n) * math.sin(sc) # xs1 + x_n #
        ys2 = (xs1 + x_n) * math.sin(sc) + (ys1 + y_n) * math.cos(sc) #  ys1 + y_n #
        xd2 = (xd1 + x_n) * math.cos(sc) - (yd1 + y_n) * math.sin(sc) # xd1 + x_n #
        yd2 = (xd1 + x_n) * math.sin(sc) + (yd1 + y_n) * math.cos(sc) # yd1 + y_n #
        xs2 = round(xs2)
        ys2 = round(ys2)
        xd2 = round(xd2)
        yd2 = round(yd2)
        return Waypoint(waypoint.mode, [xs2, ys2,  xd2, yd2], waypoint.time_bound, waypoint.id)


    def transform_poly_from_virtual(self, poly, transform_information):
        new_system_angle = -1 * transform_information[1]
        translation_vector = -1 * transform_information[0]
        out_poly = poly.rotation(i=0, j=1, theta=new_system_angle)
        return out_poly.translation(translation_vector)


    def transform_mode_from_virtual(self, waypoint: Waypoint, transform_information):
        point = waypoint.mode_parameters
        sc = -1 * transform_information[1]
        translation_vector = -1 * transform_information[0]
        xs1 = point[0]  # x_i
        ys1 = point[1]  # y_i
        xd1 = point[3]
        yd1 = point[4]
        x_n = translation_vector[0]
        y_n = translation_vector[1]
        xs2 = (xs1) * math.cos(sc) - (ys1) * math.sin(sc) + x_n # xs1 + x_n #
        ys2 = (xs1) * math.sin(sc) + (ys1) * math.cos(sc) + y_n # ys1 + y_n #
        xd2 = (xd1) * math.cos(sc) - (yd1) * math.sin(sc) + x_n # xd1 + x_n #
        yd2 = (xd1) * math.sin(sc) + (yd1) * math.cos(sc) + y_n # yd1 + y_n #
        xs2 = round(xs2)
        ys2 = round(ys2)
        xd2 = round(xd2)
        yd2 = round(yd2)
        return Waypoint(waypoint.mode,[xs2, ys2, xd2, yd2], waypoint.time_bound, waypoint.id)


    def transform_state_from_then_to_virtual_dryvr_string(self, point, transform_information_from, transform_information_to):
        pass


    def get_virtual_mode_parameters(self):
        pass

    def get_sherlock_parameters(self, mode_parameters: List[float], initial_set: np.array, time_step: float, time_bound: float,
                                mode: str):
        if mode != "follow_waypoint":
            raise NotImplementedError("These quadrotor dynamics only support waypoint following mode")
        num_nn_outputs = 1
        num_nn_inputs = initial_set.shape[1]
        num_vars = num_nn_inputs + num_nn_outputs
        order = 4
        nn_ctrl_file_path = "../systems_with_networks/Ex_Quadrotor/trial_controller_3"
        hyper_params = [str(num_vars), str(time_step), str(time_bound), str(order), nn_ctrl_file_path, num_nn_inputs,
                        num_nn_outputs]
        cur_list: List[str] = hyper_params[:]
        var_names: List[str] = ["t", "x", "y", "z"]
        cur_list.extend(var_names[:num_vars])
        ode_rhs = ["1"]
        ode_rhs.extend([' + '.join([str(val) + " * (" + var_names[ind + 1] + ' - ' + "(" + str(mode_parameters[ind]) + "))"
                                    for ind, val in enumerate(A_row)]) for A_row in A])
        cur_list.extend(ode_rhs)
        # time initset lowerbound
        cur_list.append('0')
        cur_list.extend([str(val) for val in initial_set[0, :]])
        # time initset upperbound
        cur_list.append('0')
        cur_list.extend([str(val) for val in initial_set[1, :]])
        return cur_list

    def get_flowstar_parameters(self, mode_parameters: List[float], initial_set: np.array, time_step: float, time_bound: float, mode: str):
        initial_condition = {'x':[-0.2,-0.2],
                            'y':[-0.2,-0.2],
                            'theta':[0,0],
                            'xref':[0,0],
                            'yref':[0,0],
                            'thetaref':[0,0],
                            't':[0,0]}

        num_variables = initial_set.shape[1]
        ref_vx = (mode_parameters[2] - mode_parameters[0]) / time_bound
        ref_vy = (mode_parameters[3] - mode_parameters[1]) / time_bound
        theta_ref = np.arctan2(mode_parameters[3] - mode_parameters[1], mode_parameters[2] - mode_parameters[0])
        ref_v = np.sqrt(ref_vx**2 + ref_vy**2)
        # Initial condition for verisig is ex,ey,ez,vx,vy,vz,x,y,z
        initial_condition['x'][0] = initial_set[0,0]
        initial_condition['x'][1] = initial_set[1,0]
        initial_condition['y'][0] = initial_set[0,1]
        initial_condition['y'][1] = initial_set[1,1]
        initial_condition['theta'][0] = initial_set[0,2]
        initial_condition['theta'][1] = initial_set[1,2]
        initial_condition['xref'][0] = mode_parameters[0]
        initial_condition['xref'][1] = mode_parameters[0]
        initial_condition['yref'][0] = mode_parameters[1]
        initial_condition['yref'][1] = mode_parameters[1]
        initial_condition['thetaref'][0] = theta_ref
        initial_condition['thetaref'][1] = theta_ref
        initial_condition["t"][0] = 0
        initial_condition["t"][1] = 0

        dynamics_string = f"x' = ({ref_v}*cos(thetaref - theta) + 1*(cos(theta)*(xref-x) + sin(theta)*(yref-y)))*cos(theta)\n\
        y' = ({ref_v}*cos(thetaref - theta) + 1*(cos(theta)*(xref-x) + sin(theta)*(yref-y)))*sin(theta)\n\
        theta' = thetaref + {ref_v}*(10*(-sin(theta)*(xref-x) + cos(theta)*(yref-y))+10*sin(thetaref - theta))\n\
        xref' = {ref_vx}\n\
        yref' = {ref_vy}\n\
        thetaref' = 0\n\
        t' = 1\n"

        model_string = f"continuous reachability\n\
        {{\n\
            state var x, y, theta, xref, yref, thetaref, t\n\n\
            setting\n\
            {{\n\
                fixed steps {time_step}\n\
                time {time_bound}\n\
                remainder estimation 1e-2\n\
                identity precondition\n\
                gnuplot interval x,y\n\
                adaptive orders {{min 4,max 100}}\n\
                cutoff 1e-6\n\
                precision 100\n\
                output flowstar_tube\n\
                print off\n\
            }}\n\n\
            nonpoly ode {{20}}\n\
            {{\n\
                {dynamics_string}\
            }}\n\
            init\n\
            {{\n\
                x in [{initial_condition['x'][0]},{initial_condition['x'][1]}]\n\
                y in [{initial_condition['y'][0]},{initial_condition['y'][1]}]\n\
                theta in [{initial_condition['theta'][0]},{initial_condition['theta'][1]}]\n\
                xref in [{initial_condition['xref'][0]},{initial_condition['xref'][1]}]\n\
                yref in [{initial_condition['yref'][0]},{initial_condition['yref'][1]}]\n\
                thetaref in [{initial_condition['thetaref'][0]},{initial_condition['thetaref'][1]}]\n\
                t in [{initial_condition['t'][0]},{initial_condition['t'][1]}]\n\
            }}\n\
        }}"

        return [model_string,num_variables]

    def transform_trace_from_virtual(self):
        pass

    def verifier(self, idx, plan, init_set):
        rospy.wait_for_service('verify')
        verify = rospy.ServiceProxy('verify', VerifierSrv)
        res = verify(init_set = init_set, plan = plan, idx = idx)
        if res.res == 0:
            return 'Unsafe'
        else:
            return 'Safe'

    def execute_plan(self):
        print(f'Running agent {self.idx}')
        curr_init_set = self.init_set 
        all_trace = []
        for i in range(len(self.waypoint_list)):
            current_plan = self.waypoint_list[i]
            res = self.verifier(self.idx, current_plan.mode_parameters, curr_init_set)
            if res != 'Safe':
                self.stop_agent()
                return
            
            trace = self.TC_Simulate(current_plan.mode_parameters, curr_init_set, current_plan.time_bound)
            # plt.plot(trace[:,1], trace[:,2])
            # plt.plot([current_plan.mode_parameters[0], current_plan.mode_parameters[2]], [current_plan.mode_parameters[1], current_plan.mode_parameters[3]], 'r')
            # plt.show()
            all_trace += trace.tolist()
            curr_init_set = [trace[-1][1], trace[-1][2], trace[-1][3]]

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