#!/usr/bin/env python

from __future__ import print_function

from verification_msg.srv import VerifierSrv,VerifierSrvResponse
from dryvr_utils.dryvr_utils import DryVRUtils
import rospy

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
        print(f"Verifying init set {init_set}, plan {plan}, for agent{idx}")
        return VerifierSrvResponse(1, idx)

    def start_verifier_server(self):
        rospy.init_node('verifier_server')
        s = rospy.Service('verify', VerifierSrv, self.verify)
        print("Verification Server Started")
        rospy.spin()

if __name__ == "__main__":
    verifier = MultiAgentVerifier()
    verifier.start_verifier_server()