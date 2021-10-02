#!/usr/bin/env python

from __future__ import print_function

from verification_msg.srv import VerifierSrv,VerifierSrvResponse
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

class ReachTubeTreeNode():
    def __init__(self):
        self.initset_center = []
        self.children = []

    def in_cache(self):
        pass 

    def add(self):
        pass 

    def get(self):
        pass 

class ReachTubeUnion():
    def __init__(self, initset, plan, tube):
        self.tube_list = [tube]
        self.combined_tube = tube 
        self.initset_list = [initset]
        initset_union_poly = pc.box2poly(np.array(initset).T)
        self.initset_union = pc.Region(list_poly = [initset_union_poly])
        self.initset_union_center = initset_union_poly.chebXc
        self.plan = plan

    def in_cache(self, initset_virtual, plan_virtual):
        initset_virtual_poly = pc.box2poly(np.array(initset_virtual).T)
        if pc.is_subset(initset_virtual_poly, self.initset_union):
            return True
        return False  

    def add(self, initset_virtual, plan_virtual, tube_virtual):
        self.initset_union = pc.union(self.initset_union, initset_virtual)
        self.initset_union_center = (self.initset_union_center*len(self.initset_list)+np.array(initset_virtual))\
            /(len(self.initset_list)+1)
        self.initset_list.append(initset_virtual)
        self.tube_list.append(tube_virtual)
        tube_length = min(len(tube_virtual), len(self.combined_tube))
        for i in range(len(tube_length)):
            combined_box = self.combined_tube[i]
            box = tube_virtual[i]
            for j in range(combined_box[i][0]):
                # Update lower bound
                self.combined_tube[i][0][j] = min(combined_box[0][j], box[0][j])
                # Update upper bound 
                self.combined_tube[i][1][j] = max(combined_box[1][j], box[1][j])
        if len(tube_virtual) > len(self.combined_tube):
            self.combined_tube += tube_virtual[i:]    

    def get(self, initset_virtual, plan_virtual):
        return self.combined_tube

    def split(self):
        pass

class DryVRRunner:
    def run_dryvr(self, init_set, plan, idx, variables_list, time_horizon, agent_dynamics):
        # print(os.getcwd())

        # init_set = [params.init_set_lower, params.init_set_upper]
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
        self.add(init_set, plan, agent_dynamics, tube)
        return tube, trace

    def transform_tube_from_virtual(self, tube, transform_information, dynamics_funcs):
        transformed_tube = []
        for box in tube:
            poly = pc.box2poly(np.array(box).T)
            transformed_poly = dynamics_funcs.transform_poly_from_virtual(poly, transform_information)
            transformed_box = np.column_stack(transformed_poly.bounding_box).T
            transformed_tube.append(transformed_box)
        return transformed_tube
        
class MultiAgentVerifier:
    def __init__(self):
        self.cache = TubeCache()
        self.curr_segments = {}
        self.unsafeset_list = []
        self.reachtube_publisher = rospy.Publisher('/verifier/reachtube', ReachtubeMsg, queue_size=10)
        self.safety_checking_lock = threading.Lock()

    def run_dryvr(self, params: VerifierSrv):
        # print(os.getcwd())

        init_set = [params.init_set_lower, params.init_set_upper]
        plan = params.plan
        idx = params.idx
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon
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

    def check_static_safety(self, tube):
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
        p1 = pc.box2poly(np.array(rect1).T)
        p2 = pc.box2poly(np.array(rect2).T)
        if pc.is_empty(pc.intersect(p1, p2)):
            return False
        else:
            return True

    def verify_cached(self, params: VerifierSrv):
        self.safety_checking_lock.acquire(blocking=True)
        init_set = [params.init_set_lower, params.init_set_upper]
        total_time = time.time()
        idx = params.idx
        plan = params.plan
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon

        compute_reachtube_start = time.time()
        dynamics_funcs = self.cache.get_agent_dynamics(agent_dynamics)
        transform_information = dynamics_funcs.get_transform_information(plan)
        plan_virtual = dynamics_funcs.transform_mode_to_virtual(plan, transform_information)
        initset_poly = pc.box2poly(np.array(init_set).T)
        initset_virtual_poly = dynamics_funcs.transform_poly_to_virtual(initset_poly, transform_information)
        initset_virtual = np.column_stack(initset_virtual_poly.bounding_box).T
        
        if self.cache.in_cache(initset_virtual, plan_virtual, agent_dynamics):
            print("cache hit")
            tube_virtual = self.cache.get(initset_virtual, plan_virtual, agent_dynamics)
        else:
            tube_virtual, trace = self.cache.compute_tube(initset_virtual, plan_virtual, idx, variables_list, time_horizon, agent_dynamics)
            # self.cache.add(initset_virtual, plan_virtual, agent_dynamics, tube)
        compute_reachtube_time = time.time() - compute_reachtube_start

        # print(tube_virtual)
        tube = self.cache.transform_tube_from_virtual(tube_virtual, transform_information, dynamics_funcs)
        self.publish_reachtube(idx, tube, plan)
        safety_checking_start = time.time()
        res = self.check_static_safety(tube)
        if not res:
            safety_checking_time = time.time() - safety_checking_start
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time)
 
        self.curr_segments[idx] = (plan, tube)
        res = self.check_dynamic_safety(idx, plan, tube)
        safety_checking_time = time.time() - safety_checking_start
        print(idx, "verification finished")
        tt_time = time.time() - total_time
        self.safety_checking_lock.release()

        if not res:
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time)

        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time)

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
        tube = self.run_dryvr(params)
            # self.cache.add(init_set, plan, tube)
        self.publish_reachtube(idx, tube, plan)
        compute_reachtube_time = time.time() - compute_reachtube_start

        safety_checking_start = time.time()
        res = self.check_static_safety(tube)
        if not res:
            safety_checking_time = time.time() - safety_checking_start
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time)
 
        self.curr_segments[idx] = (plan, tube)
        res = self.check_dynamic_safety(idx, plan, tube)
        self.safety_checking_lock.release()
        safety_checking_time = time.time() - safety_checking_start
        if not res:
            tt_time = time.time() - total_time
            return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time)

        # print(f"Verifying init set {init_set}, plan {plan}, for agent{idx}")
        # print(tube)
        print(idx, "verification finished")
        tt_time = time.time() - total_time
        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time)

    def start_verifier_server(self):
        rospy.init_node('verifier_server')
        print(os.path.realpath(__file__))
        s = rospy.Service('verify', VerifierSrv, self.verify_cached)
        print("Verification Server Started")
        rospy.spin()
    
    def publish_reachtube(self, idx, tube, plan):
        msg = ReachtubeMsg()
        msg.idx = idx
        tmp = np.array(tube)
        tmp_shape = tmp.shape
        dim_list = []
        for i in range(3):
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
    # params = VerifierSrv()
    # params.init_set = [[-1, -1, np.pi/2],[1, 1, np.pi/2]] 
    # params.plan = [0,0,0,5]
    # params.time_horizon = 5 
    # params.idx = 0, 
    # params.dynamics = 'dryvr_dynamics/NN_car_TR_noNN'
    # params.variables_list = ['x','y','theta']
    # verifier.verify(params)
    verifier.start_verifier_server()