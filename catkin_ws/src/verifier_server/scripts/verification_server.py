#!/usr/bin/env python

from __future__ import print_function

from verification_msg.srv import VerifierSrv,VerifierSrvResponse
from dryvr_utils.dryvr_utils import DryVRUtils
from std_msgs.msg import MultiArrayDimension

import rospy
from verification_msg.msg import ReachtubeMsg
import polytope as pc 

import os 
import numpy as np

import threading 

class TubeCache:
    def __init__(self):
        self.cache_dict = {}

class MultiAgentVerifier:
    def __init__(self):
        self.cache = {}
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

    def verify(self, params: VerifierSrv):
        # init_set = params.init_set
        # plan = params.plan

        self.safety_checking_lock.acquire(blocking=True)
        idx = params.idx
        plan = params.plan
        print(idx, os.getcwd())
        # agent_dynamics = params.dynamics
        # variables_list = params.variables_list
        # time_horizon = params.time_horizon
        
        
        tube, trace = self.run_dryvr(params)
        self.publish_reachtube(idx, tube, plan)

        res = self.check_static_safety(tube)
        if not res:
            return VerifierSrvResponse(0, idx)
 
        self.curr_segments[idx] = (plan, tube)
        res = self.check_dynamic_safety(idx, plan, tube)
        self.safety_checking_lock.release()
        if not res:
            return VerifierSrvResponse(0, idx)

        # print(f"Verifying init set {init_set}, plan {plan}, for agent{idx}")
        # print(tube)
        print(idx, "verification finished")
        return VerifierSrvResponse(1, idx)

    def start_verifier_server(self):
        rospy.init_node('verifier_server')
        print(os.path.realpath(__file__))
        s = rospy.Service('verify', VerifierSrv, self.verify)
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