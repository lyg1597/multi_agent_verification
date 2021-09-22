#!/usr/bin/env python

from __future__ import print_function

from verification_msg.srv import VerifierSrv,VerifierSrvResponse
from dryvr_utils.dryvr_utils import DryVRUtils
import rospy
import os 
import numpy as np

class TubeCache:
    def __init__(self):
        self.cache_dict = {}

class MultiAgentVerifier:
    def __init__(self):
        self.cache = {}
        self.curr_segments = {}

    def verify(self, params: VerifierSrv):
        init_set = params.init_set
        plan = params.plan
        idx = params.idx
        agent_dynamics = params.dynamics
        variables_list = params.variables_list
        time_horizon = params.time_horizon
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
        tube, trace = dryvrutils.run_dryvr_input(dryvr_input) # (dryvr_path, json_output_file)
        print(tube)

        print(f"Verifying init set {init_set}, plan {plan}, for agent{idx}")
        return VerifierSrvResponse(1, idx)

    def start_verifier_server(self):
        rospy.init_node('verifier_server')
        print(os.path.realpath(__file__))
        s = rospy.Service('verify', VerifierSrv, self.verify)
        print("Verification Server Started")
        rospy.spin()

if __name__ == "__main__":
    verifier = MultiAgentVerifier()
    params = VerifierSrv()
    params.init_set = [[-1, -1, np.pi/2],[1, 1, np.pi/2]] 
    params.plan = [0,0,0,5], 
    params.time_horizon = 5, 
    params.idx = 0, 
    params.dynamics = '/home/younger/work/multi_agent_verification/agents/dryvr_dynamics/NN_car_TR_noNN'
    params.variables_list = ['x','y','theta']
    verifier.verify(params)
    # verifier.start_verifier_server()