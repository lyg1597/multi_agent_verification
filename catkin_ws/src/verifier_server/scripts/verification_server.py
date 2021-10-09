#!/usr/bin/env python

from __future__ import print_function

from verification_msg.srv import VerifierSrv,VerifierSrvResponse, UnsafeSetSrv, UnsafeSetSrvResponse
from dryvr_utils.dryvr_utils import DryVRUtils
from std_msgs.msg import MultiArrayDimension

import rospy
from verification_msg.msg import ReachtubeMsg

import polytope as pc 
import time
import os 
import numpy as np
import threading 
import importlib
import sys
from scipy.integrate import ode

class AgentCar:
        # self.status_publisher = rospy.Publisher(f'/agent{self.idx}/status', Int64, queue_size=10)

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

        trace = np.array(trace)
        return trace

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

class ReachTubeTreeNode():
    def __init__(self, children = []):
        self.initset_union_center = np.array([])
        self.children = children
        self.num_tubes = 0
        if children != []:
            center_list = []
            for i in range(len(children)):
                center_list.append(children[i].initset_union_center)
                self.num_tubes += children[i].num_tubes
            self.initset_union_center = np.mean(np.array(center_list), axis = 0)

    # def in_cache(self, initset_virtual, plan_virtual):
    #     initset_center = np.mean(np.array(initset_virtual), axis = 0)
    #     initset_center0 = self.children[0].initset_union_center
    #     initset_center1 = self.children[1].initset_union_center
    #     diff0 = np.linalg.norm(initset_center - initset_center0)
    #     diff1 = np.linalg.norm(initset_center - initset_center1)
    #     if diff0 < diff1:
    #         res0 = self.children[0].in_cache(initset_virtual, plan_virtual)
    #         if res0:
    #             return True
    #         res1 = self.children[1].in_cache(initset_virtual, plan_virtual)
    #         return res1
    #     else:
    #         res1 = self.children[1].in_cache(initset_virtual, plan_virtual)
    #         if res1:
    #             return res1
    #         res0 = self.children[0].in_cache(initset_virtual, plan_virtual)
    #         return res0

    # def get(self, initset_virtual, plan_virtual):
    #     initset_center = np.mean(np.array(initset_virtual), axis = 0)
    #     initset_center0 = self.children[0].initset_union_center
    #     initset_center1 = self.children[1].initset_union_center
    #     diff0 = np.linalg.norm(initset_center - initset_center0)
    #     diff1 = np.linalg.norm(initset_center - initset_center1)
    #     if diff0 < diff1:
    #         res0 = self.children[0].get(initset_virtual, plan_virtual)
    #         if res0 is not None:
    #             return res0
    #         res1 = self.children[1].get(initset_virtual, plan_virtual)
    #         return res1
    #     else:
    #         res1 = self.children[1].get(initset_virtual, plan_virtual)
    #         if res1 is not None:
    #             return res1
    #         res0 = self.children[0].get(initset_virtual, plan_virtual)
    #         return res0

    def in_cache(self, initset_virtual, plan_virtual):
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        initset_center0 = self.children[0].initset_union_center
        initset_center1 = self.children[1].initset_union_center
        diff0 = np.linalg.norm(initset_center - initset_center0)
        diff1 = np.linalg.norm(initset_center - initset_center1)
        if diff0 < diff1:
            res0 = self.children[0].in_cache(initset_virtual, plan_virtual)
            return res0
        else:
            res1 = self.children[1].in_cache(initset_virtual, plan_virtual)
            return res1

    def get(self, initset_virtual, plan_virtual):
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        initset_center0 = self.children[0].initset_union_center
        initset_center1 = self.children[1].initset_union_center
        diff0 = np.linalg.norm(initset_center - initset_center0)
        diff1 = np.linalg.norm(initset_center - initset_center1)
        if diff0 < diff1:
            res0 = self.children[0].get(initset_virtual, plan_virtual)
            return res0
        else:
            res1 = self.children[1].get(initset_virtual, plan_virtual)
            return res1

    def add(self, initset_virtual, plan_virtual, tube_virtual):
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        initset_center0 = self.children[0].initset_union_center
        initset_center1 = self.children[1].initset_union_center
        diff0 = np.linalg.norm(initset_center - initset_center0)
        diff1 = np.linalg.norm(initset_center - initset_center1)
        if diff0 < diff1:
            self.children[0].add(initset_virtual, plan_virtual, tube_virtual)
        else:
            self.children[1].add(initset_virtual, plan_virtual, tube_virtual)
        self.initset_union_center = (self.initset_union_center * self.num_tubes + initset_center)/(self.num_tubes+1)
        self.num_tubes += 1

    def split(self, initset_virtual):
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        initset_center0 = self.children[0].initset_union_center
        initset_center1 = self.children[1].initset_union_center
        diff0 = np.linalg.norm(initset_center - initset_center0)
        diff1 = np.linalg.norm(initset_center - initset_center1)
        if diff0 < diff1:
            self.children[0] = self.children[0].split(initset_virtual)
        else:
            self.children[1] = self.children[1].split(initset_virtual)
        return self

class ReachTubeUnion():
    def __init__(self, initset = [], plan = [], tube = [], initset_center = []):
        # if len(initset_list) != len(tube_list):
        #     print(f"initset_list {len(initset_list)} != tube_list {len(tube_list)}")
        #     raise ValueError
        # self.tube_list = tube_list
        # if len(initset_list) > 1:
        #     max_tube_length = 0
        #     for tube in tube_list:
        #         max_tube_length = max(max_tube_length, len(tube))
        #     combined_tube = []
        #     for i in range(max_tube_length):
        #         upper = []
        #         lower = , plan_virtual_min = float("INF")
        #             val_max = -float("INF")
        #             for k in range(len(tube_list[0][0][0])):
        #                 if j < len(tube_list[i]):
        #                     if tube_list[i][j][0][k] < val_min:
        #                         val_min = tube_list[i][j][0][k]
        #                     if tube_list[i][j][1][k] > val_max:
        #                         val_max = tube_list[i][j][1][k]
        #             lower.append(val_min)
        #             upper.append(val_max)
        #         combined_tube.append([lower,upper])
        #     self.combined_tube = combined_tube
        # else:
        #     self.combined_tube = tube_list
        # self.initset_list = initset_list 
        # self.initset_center_list = []
        # initset_union_poly = []
        # for initset in initset_list:
        #     self.initset_center_list.append(np.mean(np.array(initset), axis = 0))
        #     initset_union_poly.append(pc.box2poly(np.array(initset).T))
        # self.initset_union = pc.Region(list_poly = initset_union_poly)
        # self.initset_union_center = np.mean(np.array(self.initset_center_list), axis = 0)

        if tube != []:
            self.tube_list = [tube]
            self.combined_tube = tube 
            self.initset_list = [initset]
            self.initset_center_list = [np.mean(np.array(initset), axis = 0)]
            initset_union_poly = pc.box2poly(np.array(initset).T)
            self.initset_union = pc.Region(list_poly = [initset_union_poly])
            self.initset_union_center = np.mean(np.array(initset),axis=0)

            self.plan = plan
            self.num_tubes = 1
        elif initset_center != []:
            self.tube_list = []
            self.combined_tube = []
            self.initset_list = []
            self.initset_center_list = []
            self.initset_union = pc.Region(list_poly = [])
            self.initset_union_center = initset_center
            self.plan = plan 
            self.num_tubes = 0

    def in_cache(self, initset_virtual, plan_virtual):
        initset_virtual_poly = pc.box2poly(np.array(initset_virtual).T)
        # print(initset_virtual, pc.bounding_box(self.initset_union))
        if pc.is_subset(initset_virtual_poly, self.initset_union):
            return True
        return False  

    def add(self, initset_virtual, plan_virtual, tube_virtual):
        initset_virtual_poly = pc.box2poly(np.array(initset_virtual).T)
        self.initset_union = pc.union(self.initset_union, initset_virtual_poly)
        self.initset_union_center = \
            (self.initset_union_center*len(self.initset_list)+\
            np.mean(np.array(initset_virtual),axis=0))\
            /(len(self.initset_list)+1)
        self.initset_list.append(initset_virtual)
        self.tube_list.append(tube_virtual)
        tube_length = min(len(tube_virtual), len(self.combined_tube))
        for i in range(tube_length):
            combined_box = self.combined_tube[i]
            box = tube_virtual[i]
            for j in range(len(combined_box[0])):
                # Update lower bound
                self.combined_tube[i][0][j] = min(combined_box[0][j], box[0][j])
                # Update upper bound 
                self.combined_tube[i][1][j] = max(combined_box[1][j], box[1][j])
        if len(self.combined_tube) == 0:
            i = 0
        if len(tube_virtual) > len(self.combined_tube):
            self.combined_tube += tube_virtual[i:]    
        self.initset_center_list.append(np.mean(np.array(initset_virtual), axis = 0))
        self.num_tubes += 1

    def get(self, initset_virtual, plan_virtual):
        if self.in_cache(initset_virtual, plan_virtual):
            return self.combined_tube
        else:
            return None

    def split(self, initset_virtual):
        center1 = np.mean(np.array(initset_virtual), axis = 0)
        center2 = self.initset_union_center
        cluster1 = []
        cluster2 = []
        for i in range(10):
            for center in self.initset_center_list:
                dist1 = np.linalg.norm(center-center1)
                dist2 = np.linalg.norm(center-center2)
                if dist1 < dist2:
                    cluster1.append(center)
                else:
                    cluster2.append(center)
            center1 = np.mean(np.array(cluster1), axis = 0)
            center2 = np.mean(np.array(cluster2), axis = 0)
        # initset_list1 = []
        # initset_list2 = []
        # tube_list1 = []
        # tube_list2 = []
        tmp_dict = {}
        for i, center in enumerate(self.initset_center_list):
            dist1 = np.linalg.norm(center-center1)
            dist2 = np.linalg.norm(center-center2)
            if dist1 < dist2:
                if 1 not in tmp_dict:
                    tmp_dict[1] = ReachTubeUnion(self.initset_list[i], self.plan, self.tube_list[i])
                else:
                    tmp_dict[1].add(self.initset_list[i], self.plan, self.tube_list[i])
            else:
                if 2 not in tmp_dict:
                    tmp_dict[2] = ReachTubeUnion(self.initset_list[i], self.plan, self.tube_list[i])
                else:
                    tmp_dict[2].add(self.initset_list[i], self.plan, self.tube_list[i])
        if 1 not in tmp_dict and 2 not in tmp_dict:
            print("Something wrong in the k-means algorithm")
            print(initset_virtual)
            print(center1)
            print(center2)
            print(self.initset_center_list)
            print(self.initset_union_center)
            print(self.in_cache(initset_virtual, []))
        if 1 not in tmp_dict:
            tmp_dict[1] = ReachTubeUnion(initset_center = np.mean(np.array(initset_virtual), axis = 0))
        return ReachTubeTreeNode([tmp_dict[1], tmp_dict[2]])

class DryVRRunner:
    def run_dryvr(self, init_set, plan, idx, variables_list, time_horizon, agent_dynamics):
        # print(os.getcwd())

        # init_set = [params.initset_lower, params.initset_upper]
        # plan = params.plan
        # idx = params.idx
        # agent_dynamics = params.dynamics
        # variables_list = params.variables_list
        # time_horizon = params.time_horizon
        # print(init_set)
        dryvrutils = DryVRUtils(
            variables_list = variables_list, 
            dynamics_path = agent_dynamics, 
            seed = 4
        )
        dryvr_input = dryvrutils.construct_mode_dryvr_input(
            init_set, 
            plan, 
            time_horizon
        )
        # print(dryvr_input)
        tube, trace = dryvrutils.run_dryvr_input(dryvr_input) # (dryvr_path, json_output_file)
        return tube, trace

    def get_tube(self, params: VerifierSrv):
        tube, trace = self.run_dryvr(params)
        return tube

class TubeCache:
    def __init__(self):
        self.cache_dict = {}
        self.agent_dynamics_dict = {}
        self.tube_computer = DryVRRunner()

    def get_agent_dynamics(self, path: str):
        if path in self.agent_dynamics_dict:
            return self.agent_dynamics_dict[path]
        sys.path.append(os.path.abspath(path))
        mod_name = path.replace('/', '.')
        module = importlib.import_module(mod_name)
        sys.path.pop()
        self.agent_dynamics_dict[path] = module 
        return module 

    def in_cache(self, initset_virtual, plan_virtual, agent_dynamics):
        # return False
        if (tuple(plan_virtual), agent_dynamics) not in self.cache_dict:
            return False 

        res = self.cache_dict[tuple(plan_virtual), agent_dynamics].in_cache(initset_virtual, plan_virtual)
        return res 

    def add(self, initset_virtual, plan_virtual, agent_dynamics, tube_virtual):
        if (tuple(plan_virtual), agent_dynamics) not in self.cache_dict:
            self.cache_dict[(tuple(plan_virtual), agent_dynamics)] = ReachTubeUnion(initset_virtual, plan_virtual, tube_virtual)
        else:
            self.cache_dict[(tuple(plan_virtual), agent_dynamics)].add(initset_virtual, plan_virtual, tube_virtual)

    def get(self, initset_virtual, plan_virtual, agent_dynamics):
        return self.cache_dict[(tuple(plan_virtual), agent_dynamics)].get(initset_virtual, plan_virtual)

    def compute_tube(self, init_set, plan, idx, variables_list, time_horizon, agent_dynamics):
        tube, trace = self.tube_computer.run_dryvr(init_set, plan, idx, variables_list, time_horizon, agent_dynamics)
        # self.add(init_set, plan, agent_dynamics, tube)
        return tube, trace

    def transform_tube_from_virtual(self, tube, transform_information, dynamics_funcs):
        transformed_tube = []
        for box in tube:
            poly = pc.box2poly(np.array(box).T)
            transformed_poly = dynamics_funcs.transform_poly_from_virtual(poly, transform_information)
            transformed_box = np.column_stack(transformed_poly.bounding_box).T
            transformed_tube.append(transformed_box)
        return transformed_tube
        
    def transform_tube_to_virtual(self, tube, transform_information, dynamics_funcs):
        transformed_tube = []
        for box in tube:
            poly = pc.box2poly(np.array(box).T)
            transformed_poly = dynamics_funcs.transform_poly_to_virtual(poly, transform_information)
            transformed_box = np.column_stack(transformed_poly.bounding_box).T
            transformed_tube.append(transformed_box)
        return transformed_tube

    def refine(self, key, initset_virtual):
        # return
        self.cache_dict[key] = self.cache_dict[key].split(initset_virtual)

class MultiAgentVerifier:
    def __init__(self):
        self.cache = TubeCache()
        self.curr_segments = {}
        self.unsafeset_list = []
        self.reachtube_publisher = rospy.Publisher('/verifier/reachtube', ReachtubeMsg, queue_size=10)
        self.safety_checking_lock = threading.Lock()
        self.refine_threshold = 10

    def run_dryvr(self, params: VerifierSrv):
        # print(os.getcwd())

        init_set = [list(params.initset_lower), list(params.initset_upper)]
        plan = params.plan
        idx = params.idx
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon
        # print(init_set)
        init_set = self.bloat_initset(init_set, [0.5,0.5,0.1])
        dryvrutils = DryVRUtils(
            variables_list = variables_list, 
            dynamics_path = agent_dynamics, 
            seed = 4
        )
        dryvr_input = dryvrutils.construct_mode_dryvr_input(
            init_set, 
            plan, 
            time_horizon
        )
        # print(dryvr_input)
        tube, trace = dryvrutils.run_dryvr_input(dryvr_input) # (dryvr_path, json_output_file)
        return tube, trace

    def check_static_safety(self, tube):
        bloated_tube = self.seg_bloat_tube(tube, 1)
        for unsafe_poly in self.unsafeset_list:
            intersect = False 
            for rect in bloated_tube:
                intersect = self.check_intersection(rect, unsafe_poly)
                if intersect:
                    break
            if intersect:
                for rect in tube:
                    intersect = self.check_intersection(rect, unsafe_poly)
                    if intersect: 
                        return False 
        return True

    def check_dynamic_safety(self, idx, plan, tube):
        for key in self.curr_segments:
            if key != idx:
                other_tube = self.curr_segments[key][1]
                other_plan = self.curr_segments[key][0]
                plan_dist = np.linalg.norm(np.array(plan) - np.array(other_plan))
                if plan_dist > 100:
                    continue
                seg_safe = False
                for i in range(10):
                    bloated_tube = self.seg_bloat_tube(tube, i+1)
                    bloated_other_tube = self.seg_bloat_tube(other_tube, i+1)
                    intersect = False
                    for rect in bloated_tube:
                        for other_rect in bloated_other_tube:
                            intersect = self.check_intersection(rect, other_rect)
                            if intersect:
                                break 
                        if intersect:
                            break
                    if not intersect:
                        seg_safe = True
                        break
                if not seg_safe:
                    return False 

        return True

    def seg_bloat_tube(self, tube, num_seg):
        if tube == []:
            return []
        tube_length = len(tube)
        dim = len(tube[0][0])
        seg_length = int(tube_length/num_seg)+1
        bloated_segs = []
        for i in range(0, tube_length, seg_length):
            tube_seg = np.array(tube[i:i+seg_length])
            seg_max = []
            seg_min = []
            for j in range(dim):
                var_max = np.amax(tube_seg[:,:,j])
                var_min = np.amin(tube_seg[:,:,j])
                seg_max.append(var_max)
                seg_min.append(var_min)
            bloated_segs.append([seg_min, seg_max])
        return bloated_segs

    def check_intersection(self, rect1, rect2):
        # print(rect1, rect2)
        p1 = rect1
        if not isinstance(rect1, pc.Polytope):
            p1 = pc.box2poly(np.array(rect1).T)
        p2 = rect2
        if not isinstance(rect2, pc.Polytope):
            p2 = pc.box2poly(np.array(rect2).T)

        if pc.is_empty(pc.intersect(p1, p2)):
            return False
        else:
            return True

    def bloat_initset(self, initset, resolution = [0.5,0.5,0.1]):
        print(initset)
        res = initset 
        for i in range(len(res[0])):
            if resolution[i] != 0:
                res[0][i] = np.floor(res[0][i]/resolution[i])*resolution[i]
                res[1][i] = np.ceil(res[1][i]/resolution[i])*resolution[i]
        return res

    def verify_full(self, params: VerifierSrv):
        self.safety_checking_lock.acquire(blocking=True)
        for _ in range(self.refine_threshold):
            res, key, initset_virtual = self.verify_cached(params)
            if res.res == 1 or res.res == -1:
                self.safety_checking_lock.release()
                return res
            self.cache.refine(key, initset_virtual)
        self.safety_checking_lock.release()
        return res

    def verify_cached(self, params: VerifierSrv):
        total_time = time.time()
        init_set = [list(params.initset_lower), list(params.initset_upper)]
        idx = params.idx
        plan = params.plan
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon

        init_set = self.bloat_initset(init_set)

        compute_reachtube_start = time.time()
        dynamics_funcs = self.cache.get_agent_dynamics(agent_dynamics)
        transform_information = dynamics_funcs.get_transform_information(plan)
        plan_virtual = dynamics_funcs.transform_mode_to_virtual(plan, transform_information)
        initset_poly = pc.box2poly(np.array(init_set).T)
        initset_virtual_poly = dynamics_funcs.transform_poly_to_virtual(initset_poly, transform_information)
        initset_virtual = np.column_stack(initset_virtual_poly.bounding_box).T
        
        # print(initset_virtual)
        from_cache = False 
        if self.cache.in_cache(initset_virtual, plan_virtual, agent_dynamics):
            print("cache hit")
            tube_virtual = self.cache.get(initset_virtual, plan_virtual, agent_dynamics)
            from_cache = True
            tube = self.cache.transform_tube_from_virtual(tube_virtual, transform_information, dynamics_funcs)
        else:
            tube, trace = self.cache.compute_tube(init_set, plan, idx, variables_list, time_horizon, agent_dynamics)
            tube_virtual = self.cache.transform_tube_to_virtual(tube, transform_information, dynamics_funcs)
            # tube_virtual, trace = self.cache.compute_tube(initset_virtual, plan_virtual, idx, variables_list, time_horizon, agent_dynamics)
            self.cache.add(initset_virtual, plan_virtual, agent_dynamics, tube_virtual)
            from_cache = False 
            # self.cache.add(initset_virtual, plan_virtual, agent_dynamics, tube)
        compute_reachtube_time = time.time() - compute_reachtube_start

        # print(tube_virtual)
        # tube = self.cache.transform_tube_from_virtual(tube_virtual, transform_information, dynamics_funcs)
        self.publish_reachtube(idx, tube, plan, int(from_cache))
        safety_checking_start = time.time()
        print(f"start checking static safety for agent {idx}")
        res = self.check_static_safety(tube)
        if not res:
            safety_checking_time = time.time() - safety_checking_start
            if from_cache:
                return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(from_cache)), (tuple(plan_virtual), agent_dynamics), initset_virtual
            else:
                return VerifierSrvResponse(res = -1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(from_cache)), (tuple(plan_virtual), agent_dynamics), initset_virtual
 
        self.curr_segments[idx] = (plan, tube)
        print(f"start checking dynamic safety for agent {idx}")
        res = self.check_dynamic_safety(idx, plan, tube)
        safety_checking_time = time.time() - safety_checking_start
        print(idx, f"verification finished for plan {plan}")
        tt_time = time.time() - total_time

        if not res:
            self.curr_segments[idx] = (plan, [])
            if from_cache:
                return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache)), (tuple(plan_virtual), agent_dynamics), initset_virtual
            else:
                return VerifierSrvResponse(res = -1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache)), (tuple(plan_virtual), agent_dynamics), initset_virtual

        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache)), (tuple(plan_virtual), agent_dynamics), initset_virtual

    def verify(self, params: VerifierSrv):
        # init_set = params.init_set
        # plan = params.plan
        self.safety_checking_lock.acquire(blocking=True)
        total_time = time.time()
        idx = params.idx
        plan = params.plan
        agent_dynamics = params.dynamics
        print(idx, os.getcwd())
        # agent_dynamics = params.dynamics
        # variables_list = params.variables_list
        # time_horizon = params.time_horizon
        # if self.cache.in_cache(init_set, plan, agent_dynamics):
        #     tube = self.cache.get(init_set, plan)
        # else:
        compute_reachtube_start = time.time()
        tube, trace = self.run_dryvr(params)
            # self.cache.add(init_set, plan, tube)
        self.publish_reachtube(idx, tube, plan, int(False))
        compute_reachtube_time = time.time() - compute_reachtube_start

        safety_checking_start = time.time()
        res = self.check_static_safety(tube)
        res = True
        if not res:
            safety_checking_time = time.time() - safety_checking_start
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(False))
 
        self.curr_segments[idx] = (plan, tube)
        res = self.check_dynamic_safety(idx, plan, tube)
        self.safety_checking_lock.release()
        safety_checking_time = time.time() - safety_checking_start
        if not res:
            self.curr_segments[idx] = (plan, [])
            tt_time = time.time() - total_time
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(False))

        # print(f"Verifying init set {init_set}, plan {plan}, for agent{idx}")
        # print(tube)
        print(idx, "verification finished")
        tt_time = time.time() - total_time
        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(False))

    def process_unsafeset(self, params: UnsafeSetSrv):
        print("Unsafe set received")
        print("Clear previous unsafe set")
        self.unsafeset_list = []
        unsafe_type = params.type
        unsafe_msg = params.unsafe_list
        shape = []
        for i in range(len(unsafe_msg.layout.dim)):
            shape.append(unsafe_msg.layout.dim[i].size)
        shape = tuple(shape)
        unsafe_list = np.array(unsafe_msg.data).reshape(shape).tolist()
        if unsafe_type == "Box" or unsafe_type == "box":
            for box in unsafe_list:
                poly = pc.box2poly(np.array(box).T)
                self.unsafeset_list.append(poly)
        else:
            print('Unknown unsafe set type. Return')
            return UnsafeSetSrvResponse(res = 0)
        print('Unsafe set successfuly set')
        return UnsafeSetSrvResponse(res = 1)

    def start_verifier_server(self):
        rospy.init_node('verifier_server')
        print(os.path.realpath(__file__))
        verify_service = rospy.Service('verify', VerifierSrv, self.verify)
        print("Verification Server Started")
        unsafe_service = rospy.Service('set_unsafe', UnsafeSetSrv, self.process_unsafeset)
        rospy.spin()
    
    def publish_reachtube(self, idx, tube, plan, from_cache):
        msg = ReachtubeMsg()
        msg.idx = int(idx)
        msg.from_cache = int(from_cache)
        tmp = np.array(tube)
        tmp_shape = tmp.shape
        dim_list = []
        for i in range(len(tmp_shape)):
            dim = MultiArrayDimension()
            dim.size = tmp_shape[i]
            dim_list.append(dim)
        msg.tube.layout.dim = dim_list
        msg.tube.data = tmp.flatten().tolist()
        msg.plan = plan

        self.reachtube_publisher.publish(msg)

if __name__ == "__main__":
    print(os.getcwd())

    verifier = MultiAgentVerifier()
    verifier.start_verifier_server()
    # raw_wp_list = [
    #     [20.0, 5.0, 20.0, 10.0, np.pi/2],
    #     [20.0, 10.0, 20.0, 15.0, np.pi/2],
    #     [20.0, 15.0, 25.0, 15.0, 0],
    #     [25.0, 15.0, 30.0, 15.0, 0],
    #     [30.0, 15.0, 35.0, 15.0, 0],
    #     [35.0, 15.0, 35.0, 20.0, np.pi/2],
    # ]

    # # dynamics_funcs = verifier.cache.get_agent_dynamics('agents/NN_car_TR_noNN')

    # agent = AgentCar()

    # x = raw_wp_list[0][0]
    # y = raw_wp_list[0][1]
    # theta = raw_wp_list[0][4]

    # for wp in raw_wp_list:
    #     params = VerifierSrv()
    #     params.initset_lower = [x-1, y-1, theta-0.1] 
    #     params.initset_upper = [x+1, y+1, theta+0.1] 
    #     params.plan = wp[:4]
    #     params.time_horizon = 4
    #     params.idx = 0
    #     params.dynamics = 'dryvr_dynamics/NN_car_TR_noNN'
    #     params.variables_list = ['x','y','theta']
    #     verifier.verify_cached(params)

    #     trace = agent.TC_Simulate(
    #         wp[:-1], 
    #         [x,y,theta],
    #         4
    #     )
    #     end = trace[-1]
    #     x = end[1]
    #     y = end[2]
    #     theta = end[3]