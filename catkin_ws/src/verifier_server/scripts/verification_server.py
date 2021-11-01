#!/usr/bin/env python

from __future__ import print_function

from polytope import polytope

from verification_msg.srv import VerifierSrv,VerifierSrvResponse, UnsafeSetSrv, UnsafeSetSrvResponse, CacheInfoSrv, CacheInfoSrvResponse
from dryvr_utils.dryvr_utils import DryVRUtils
from std_msgs.msg import MultiArrayDimension
import rospy
from verification_msg.msg import ReachtubeMsg, Obstacle

try:
    from common.Waypoint import Waypoint
except:
    from Waypoint import Waypoint

import polytope as pc 
import time
import os 
import numpy as np
import math
import threading 
import importlib
import sys
from scipy.integrate import ode
import random
from typing import List
import argparse

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
            node, res = self.children[0].split(initset_virtual)
            self.children[0] = node
        else:
            node, res = self.children[1].split(initset_virtual)
            self.children[1] = node 
        return self, res 

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
        self.children = None
        self.depth = 0
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
        elif len(initset_center) != 0:
            self.tube_list = []
            self.combined_tube = []
            self.initset_list = []
            self.initset_center_list = []
            self.initset_union = pc.Region(list_poly = [])
            self.initset_union_center = initset_center
            self.plan = plan 
            self.num_tubes = 0

    def in_cache(self, initset_virtual, plan_virtual):
        if pc.is_empty(self.initset_union):
            return False
        initset_virtual_poly = pc.box2poly(np.array(initset_virtual).T)
        # initset_union_box = np.column_stack(pc.bounding_box(self.initset_union)).T
        # initset_union_poly = pc.box2poly(initset_union_box.T)
        # if pc.is_subset(initset_virtual_poly, self.initset_union, abs_tol = 1e-10):
        if pc.is_subset(initset_virtual_poly, self.initset_union):
            # print(initset_virtual, pc.bounding_box(self.initset_union), True)
            return True
        # initset_union_box = np.column_stack(pc.bounding_box(self.initset_union)).T
        # if PolyUtils.does_rect_contain(initset_virtual, initset_union_box):
        #     print(initset_virtual, initset_union_box, True)
        #     return True
        # print(initset_virtual, initset_union_box, False)
        # print(initset_virtual, pc.bounding_box(self.initset_union), False)
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

    def split2(self, initset_virtual):
        if self.num_tubes <= 1:
            return None, None, False

        # Use K-means++ to determine the new centroids
        k = 2
        data = np.array(self.initset_center_list)
        centroids = []
        centroids.append(data[np.random.randint(
                data.shape[0]), :])
        # plot(data, np.array(centroids))
    
        ## compute remaining k - 1 centroids
        for c_id in range(k - 1):
            
            ## initialize a list to store distances of data
            ## points from nearest centroid
            dist = []
            for i in range(data.shape[0]):
                point = data[i, :]
                d = sys.maxsize
                
                ## compute distance of 'point' from each of the previously
                ## selected centroid and store the minimum distance
                for j in range(len(centroids)):
                    temp_dist = np.linalg.norm(point - centroids[j])
                    d = min(d, temp_dist)
                dist.append(d)
                
            ## select data point with maximum distance as our next centroid
            dist = np.array(dist)
            next_centroid = data[np.argmax(dist), :]
            centroids.append(next_centroid)
            dist = []
            # plot(data, np.array(centroids))
        center1 = centroids[0]
        center2 = centroids[1]
        
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
        if 1 not in tmp_dict or 2 not in tmp_dict:
            return None, ReachTubeUnion(initset_center = np.mean(np.array(initset_virtual), axis = 0)), False
        return tmp_dict[1], tmp_dict[2], True

class DryVRRunner:
    def run_dryvr(self, init_set, plan, idx, variables_list, time_horizon, agent_dynamics):
        # print(os.getcwd())
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
        tree_root, res = self.cache_dict[key].split(initset_virtual)
        self.cache_dict[key] = tree_root
        return res 

    def in_cache2(self, initset_virtual, plan_virtual, agent_dynamics):
        # return False
        key = (tuple(plan_virtual), agent_dynamics)
        # print(key)
        # print(self.cache_dict)
        if key not in self.cache_dict:
            return False, None 

        reachtube_union_list = self.cache_dict[key]
        closest_center_idx = -1
        closest_center_val = float('INF')
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        for i in range(len(reachtube_union_list)):
            reachtube_union:ReachTubeUnion = reachtube_union_list[i]
            dist = np.linalg.norm(initset_center - reachtube_union.initset_union_center)
            if dist<closest_center_val:
                closest_center_idx = i
                closest_center_val = dist

        res = self.cache_dict[key][closest_center_idx].in_cache(initset_virtual, plan_virtual)
        if res:
            return res, closest_center_idx

        return res, None

    def get2(self, initset_virtual, plan_virtual, agent_dynamics, closest_center_idx = None):
        key = (tuple(plan_virtual), agent_dynamics)
        if closest_center_idx is not None:
            return self.cache_dict[key][closest_center_idx].get(initset_virtual, plan_virtual)
        else:
            reachtube_union_list = self.cache_dict[key]
            closest_center_idx = -1
            closest_center_val = float('INF')
            initset_center = np.mean(np.array(initset_virtual), axis = 0)
            for i in range(len(reachtube_union_list)):
                reachtube_union:ReachTubeUnion = reachtube_union_list[i]
                dist = np.linalg.norm(initset_center - reachtube_union.initset_union_center)
                if dist<closest_center_val:
                    closest_center_idx = i
                    closest_center_val = dist
            return self.cache_dict[key][closest_center_idx].get(initset_virtual, plan_virtual)

    def add2(self, initset_virtual, plan_virtual, agent_dynamics, tube_virtual):
        key = (tuple(plan_virtual), agent_dynamics)
        if key not in self.cache_dict:
            self.cache_dict[key] = [ReachTubeUnion(initset_virtual, plan_virtual, tube_virtual)]
        else:
            reachtube_union_list = self.cache_dict[key]
            closest_center_idx = -1
            closest_center_val = float('INF')
            initset_center = np.mean(np.array(initset_virtual), axis = 0)
            for i in range(len(reachtube_union_list)):
                reachtube_union:ReachTubeUnion = reachtube_union_list[i]
                dist = np.linalg.norm(initset_center - reachtube_union.initset_union_center)
                if dist<closest_center_val:
                    closest_center_idx = i
                    closest_center_val = dist

            self.cache_dict[key][closest_center_idx].add(initset_virtual, plan_virtual, tube_virtual)

    def refine2(self, key, initset_virtual):
        reachtube_union_list = self.cache_dict[key]
        closest_center_idx = -1
        closest_center_val = float('INF')
        initset_center = np.mean(np.array(initset_virtual), axis = 0)
        for i in range(len(reachtube_union_list)):
            reachtube_union:ReachTubeUnion = reachtube_union_list[i]
            dist = np.linalg.norm(initset_center - reachtube_union.initset_union_center)
            if dist<closest_center_val:
                closest_center_idx = i
                closest_center_val = dist

        union1, union2, res = self.cache_dict[key][closest_center_idx].split2(initset_virtual)
        if not res:
            if union2 is not None:
                self.cache_dict[key].append(union2)
            # print(f"refine failed {len(self.cache_dict[key])}, {closest_center_idx}")
            return res
            
        self.cache_dict[key][closest_center_idx] = union1 
        self.cache_dict[key].append(union2)
        # print(f"done refinement {len(self.cache_dict[key])}, {closest_center_idx}")
        return res

    def print_cache_info(self, key):
        root = self.cache_dict[key]
        expansion_queue = [root]
        print(f">>>>>> {key}")
        while expansion_queue != []:
            node = expansion_queue.pop(0)
            if node.children is None:
                print(node.depth, node.num_tubes)
            else:
                expansion_queue += node.children

    def print_cache_info_2(self, key):
        reachtube_union_list: List[ReachTubeUnion] = self.cache_dict[key]
        print(f">>>>>> {key}, {len(reachtube_union_list)}")
        for reachtube_union in reachtube_union_list:
            # node = expansion_queue.pop(0)
            # if node.children is None:
            print(reachtube_union.num_tubes, reachtube_union.initset_union_center)
            # else:
            #     expansion_queue += node.children

class MultiAgentVerifier:
    def __init__(self):
        self.cache = TubeCache()
        self.curr_segments = {}
        self.unsafeset_list = []
        self.reachtube_publisher = rospy.Publisher('/verifier/reachtube', ReachtubeMsg, queue_size=10)
        self.cache_lock = threading.Lock()
        self.reachtube_computation_lock = threading.Lock()
        self.refine_threshold = 10

    def run_dryvr(self, params: VerifierSrv):
        # print(os.getcwd())

        init_set = [list(params.initset_lower), list(params.initset_upper)]
        plan = params.plan
        idx = params.idx
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon
        initset_resolution = params.initset_resolution
        # print(init_set)
        init_set = self.bloat_initset(init_set, resolution=initset_resolution)
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
            # tmp = pc.projection(unsafe_poly, [1,2,3])
            for rect in bloated_tube:
                tmp2 = [rect[0][:3],rect[1][:3]]
                intersect = self.check_intersection(tmp2, unsafe_poly)
                if intersect:
                    break
            if intersect:
                tmp2 = [rect[0][:3],rect[1][:3]]
                for rect in tube:
                    intersect = self.check_intersection(tmp2, unsafe_poly)
                    if intersect: 
                        return False 
        return True

    def check_dynamic_safety(self, idx, plan, tube):
        for key in self.curr_segments:
            if key != idx:
                other_tube = self.curr_segments[key][1]
                other_plan = self.curr_segments[key][0]
                # print(plan, other_plan)
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
                            tmp1 = [rect[0][:3],rect[1][:3]]
                            tmp2 = [other_rect[0][:3],other_rect[1][:3]]
                            intersect = self.check_intersection(tmp1, tmp2)
                            if intersect:
                                break 
                        if intersect:
                            break
                    if not intersect:
                        seg_safe = True
                        break
                if not seg_safe:
                    return False, key 
        return True, -1

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
        # print(initset)
        res = initset 
        for i in range(len(res[0])):
            if resolution[i] != 0:
                res[0][i] = np.floor(res[0][i]/resolution[i])*resolution[i]
                res[1][i] = np.ceil(res[1][i]/resolution[i])*resolution[i]
        return res

    def verification_server2(self, params: VerifierSrv):        
        self.cache_lock.acquire(blocking=True)
        print(params.idx, f"verification start for plan {params.plan}")
        verification_start = time.time()
        verification_time = 0
        refine_time = 0
        compute = False 
        for i in range(self.refine_threshold):
            verification_start = time.time()
            if i == self.refine_threshold-1:
                compute = True
            res, key, initset_virtual = self.verify_cache2(params, compute)
            compute = False 
            verification_time += (time.time() - verification_start)
            if res.res == 1 or res.res == -1 or res.res == -2:
                res.tt_time = time.time() - verification_start
                res.num_ref = i
                res.refine_time = refine_time
                res.verification_time = verification_time
                print(params.idx, f"verification finished for plan {params.plan}, time {verification_time}, cache {res.from_cache}")
                self.cache_lock.release()
                return res
            refine_start = time.time()
            success = self.cache.refine2(key, initset_virtual)
            if not success:
                compute = True 
            refine_time += (time.time() - refine_start)
        res.num_ref = i
        res.tt_time = time.time() - verification_start
        res.refine_time = refine_time
        res.verification_time = verification_time
        print(params.idx, f"verification finished for plan {params.plan}, time {verification_time}, cache {res.from_cache}")
        self.cache_lock.release()
        return res

    def verify_cache2(self, params: VerifierSrv, compute = False):
        total_time = time.time()
        init_set = [list(params.initset_lower), list(params.initset_upper)]
        idx = params.idx
        plan = params.plan
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon
        initset_resolution = params.initset_resolution

        compute_reachtube_start = time.time()
        dynamics_funcs = self.cache.get_agent_dynamics(agent_dynamics)
        wp = Waypoint("follow_waypoint",plan,time_horizon,0)
        transform_information = dynamics_funcs.get_transform_information(wp)
        plan_virtual = dynamics_funcs.transform_mode_to_virtual(wp, transform_information)
        initset_poly = pc.box2poly(np.array(init_set).T)
        initset_virtual_poly = dynamics_funcs.transform_poly_to_virtual(initset_poly, transform_information)
        initset_virtual = np.column_stack(initset_virtual_poly.bounding_box).T
        
        # print(initset_virtual)
        from_cache = False 
        in_cache, closest_center_idx = self.cache.in_cache2(initset_virtual, plan_virtual, agent_dynamics)
        if not compute and in_cache:
            # print("cache hit")
            tube_virtual = self.cache.get2(initset_virtual, plan_virtual, agent_dynamics, closest_center_idx)
            from_cache = True
            tube = self.cache.transform_tube_from_virtual(tube_virtual, transform_information, dynamics_funcs)
        else:
            init_set = self.bloat_initset(init_set, resolution = initset_resolution)
            print("############")
            print(f'agent idx {idx}')
            print(f"initial condition compute: {init_set}")
            print(f"plan: {plan}")
            tube, trace = self.cache.compute_tube(init_set, plan, idx, variables_list, time_horizon, agent_dynamics)
            tube_virtual = self.cache.transform_tube_to_virtual(tube, transform_information, dynamics_funcs)
            # tube_virtual, trace = self.cache.compute_tube(initset_virtual, plan_virtual, idx, variables_list, time_horizon, agent_dynamics)
            initset_poly = pc.box2poly(np.array(init_set).T)
            initset_virtual_poly = dynamics_funcs.transform_poly_to_virtual(initset_poly, transform_information)
            initset_virtual = np.column_stack(initset_virtual_poly.bounding_box).T
            self.cache.add2(initset_virtual, plan_virtual, agent_dynamics, tube_virtual)
            from_cache = False 
            # self.cache.add(initset_virtual, plan_virtual, agent_dynamics, tube)
        compute_reachtube_time = time.time() - compute_reachtube_start

        # print(tube_virtual)
        # self.publish_reachtube(idx, tube, plan, int(from_cache), res = -1)
        # tube = self.cache.transform_tube_from_virtual(tube_virtual, transform_information, dynamics_funcs)
        safety_checking_start = time.time()
        print(f"start checking static safety for agent {idx}")
        res = self.check_static_safety(tube)
        if not res:
            print(f"static unsafe {idx}")
            safety_checking_time = time.time() - safety_checking_start
            if from_cache:
                return VerifierSrvResponse(res = 0, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(from_cache), num_ref = 0), (tuple(plan_virtual), agent_dynamics), initset_virtual
            else:
                self.publish_reachtube(idx, tube, plan, int(from_cache), res = -1)
                return VerifierSrvResponse(res = -2, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(from_cache), num_ref = 0), (tuple(plan_virtual), agent_dynamics), initset_virtual
 
        self.curr_segments[idx] = (plan, tube)
        print(f"start checking dynamic safety for agent {idx}")
        res, key = self.check_dynamic_safety(idx, plan, tube)
        safety_checking_time = time.time() - safety_checking_start
        print(idx, f"verification finished for plan {plan}")
        tt_time = time.time() - total_time

        if not res:
            print(f"dynamic unsafe {idx}, {key}")
            self.curr_segments[idx] = (plan, [])
            self.publish_reachtube(idx, tube, plan, int(from_cache), res = -1)
            if from_cache:
                return VerifierSrvResponse(res = -1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache), num_ref = 0), (tuple(plan_virtual), agent_dynamics), initset_virtual
            else:
                return VerifierSrvResponse(res = -1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache), num_ref = 0), (tuple(plan_virtual), agent_dynamics), initset_virtual

        self.publish_reachtube(idx, tube, plan, int(from_cache), res = 1)
        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(from_cache), num_ref = 0), (tuple(plan_virtual), agent_dynamics), initset_virtual

    def verification_server_nocache(self, params: VerifierSrv):
        self.cache_lock.acquire(blocking=True)
        total_time = time.time()
        idx = params.idx
        plan = params.plan
        print(idx, os.getcwd())
        compute_reachtube_start = time.time()
        tube, trace = self.run_dryvr(params)
        compute_reachtube_time = time.time() - compute_reachtube_start

        safety_checking_start = time.time()
        res = self.check_static_safety(tube)
        res = True
        if not res:
            self.publish_reachtube(idx, tube, plan, int(False),-1)
            safety_checking_time = time.time() - safety_checking_start
            return VerifierSrvResponse(res = -1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, from_cache = int(False))
 
        self.curr_segments[idx] = (plan, tube)
        res,_ = self.check_dynamic_safety(idx, plan, tube)
        self.cache_lock.release()
        safety_checking_time = time.time() - safety_checking_start
        if not res:
            self.publish_reachtube(idx, tube, plan, int(False),-1)
            self.curr_segments[idx] = (plan, [])
            tt_time = time.time() - total_time
            return VerifierSrvResponse(res = -2, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(False))

        self.publish_reachtube(idx, tube, plan, int(False),1)
        print(idx, "verification finished")
        tt_time = time.time() - total_time
        return VerifierSrvResponse(res = 1, idx = idx, rt_time = compute_reachtube_time, sc_time = safety_checking_time, tt_time = tt_time, from_cache = int(False))

    def process_unsafeset(self, params: UnsafeSetSrv):
        print("Unsafe set received")
        print("Clear previous unsafe set")
        self.curr_segments = {}
        self.unsafeset_list = []
        # unsafe_type = params.type
        # unsafe_msg = params.unsafe_list
        unsafe_list = params.obstacle_list
        for unsafe in unsafe_list:
            shape = []
            for i in range(len(unsafe.obstacle.layout.dim)):
                shape.append(unsafe.obstacle.layout.dim[i].size)
            shape = tuple(shape)
            obstacle = np.array(unsafe.obstacle.data).reshape(shape)
            unsafe_type = unsafe.obstacle_type
            if unsafe_type == "Box" or unsafe_type == "box":
                # for box in unsafe_list:
                    # print(box)
                poly = pc.box2poly(obstacle[:,:3].T)
                # poly = pc.projection(poly,[1,2,3])
                self.unsafeset_list.append(poly)
            elif unsafe_type == "Vertices" or unsafe_type == "vertices":
                poly = pc.qhull(obstacle[:,:3])
                # poly = pc.projection(poly,[1,2,3])
                self.unsafeset_list.append(poly)
            elif unsafe_type == "Matrix" or unsafe_type == "matrix":
                A = obstacle[:,:-1]
                b = obstacle[:,-1]
                A = A[:-6,:3]
                b = b[:-6]
                poly = pc.Polytope(A=A, b=b)
                # poly = pc.projection(poly,[1,2,3])
                self.unsafeset_list.append(poly)
            else:
                print('Unknown unsafe set type. Return')
                return UnsafeSetSrvResponse(res = 0)
        print('Unsafe set successfuly set')
        return UnsafeSetSrvResponse(res = 1)

    def start_verifier_server(self, no_cache = True):
        rospy.init_node('verifier_server')
        print(os.path.realpath(__file__))
        if no_cache:
            verify_service = rospy.Service('verify', VerifierSrv, self.verification_server_nocache)
        else:
            verify_service = rospy.Service('verify', VerifierSrv, self.verification_server2)
        print("Verification Server Started")
        unsafe_service = rospy.Service('set_unsafe', UnsafeSetSrv, self.process_unsafeset)
        rospy.Service('print_tree', CacheInfoSrv,self.print_cache_info)
        rospy.spin()
    
    def publish_reachtube(self, idx, tube, plan, from_cache, res):
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
        msg.res = res
        print(f"reachtube publish time {time.time()}")
        msg.reachtube_start_time = time.time()

        self.reachtube_publisher.publish(msg)

    def print_cache_info(self, params):
        print("Print cache info")
        for key in self.cache.cache_dict:
            self.cache.print_cache_info_2(key)
        return CacheInfoSrvResponse()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('-no_cache', action='store_true')
    args = parser.parse_args()

    verifier = MultiAgentVerifier()
    verifier.start_verifier_server(args.no_cache)