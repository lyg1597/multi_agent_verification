This is the repeatability evaluation package for the tool paper "Swerve: Efficient Runtime Verification of Multi-Agent Systems Using Dynamical Symmetries and Cloud Computing" by: Yangge Li, Hussein Sibai, and Sayan Mitra, submitted to HSCC'22.

The link to this artifact VM on figshare is: ???

The link to this artifact VM on Google drive is: ???

The link to the source code git repository for Swerve is: https://github.com/lyg1597/multi_agent_verification/tree/hscc_artifact

Swerve with its source codes is provided with this VM. There is no tool or library that should be downloaded before usage, and all tools can be used in this VM without an internet connection. The root password of the VM is HSCC2022.

###########################

This file explains how to replicate the results presented in the paper.

Swerve consists of a Verification Server and a set of agents. To simplify the repeatability evaluation process, the Verification Server and all agents are running on the same virtual machine. However, because of the nature of ROS, the Verification Server and agents can easily be deployed on different machines. 

The results of the experiments shown in Table 1 and 2 are stochastic and change between runs due to the following reasons:

    1- Since each agent is running independently, the sequence of how different agents request the verification service is non-determinitic. This non deterministic can influence the response time for each agent's request. 

    2- The non-determinism in the sequence of verification request can also influence the number of calls to the reachability engine. 

    3- The response time is machine specific. The machine specifications used for generating the results in the paper are shown at the end of this file.

However, the vary in results will not influence the observations that we made in the paper. 

###########################

Dependencies: All dependencies for the repeatability package are already installed in the Virtual Machine
Ubuntu 20.04
ROS Noetic
Python 3.8 or newer versions
numpy
scipy
polytope
pypoman
PyTorch
pyvista
matplotlib

To replicate our experiments from the paper please run commands on our example files from the root directory of our local repo. The statics will appear in the terminal as the scenario finishes.
1. To generate Figure 1: 
    Change directory by running the following command:
        cd ~/Swerve
    and then run following commands to visualize the scenario:
        python3 scripts/plan_visualize_3d.py

2. For the experiment results in Table 1:
    Swerve with symmetry caching: 
        Start three terminals,
        In the first terminal
            run following command to start roscore: 
                roscore 

        In the second terminal, 
            change directory by running the following command:
                cd ~/Swerve 
            and then set necessary ROS variables:
                source catkin_ws/devel/setup.bash
            and then start the Verification Server with symmetry and caching by using command:
                rosrun verification_server verification_server.py

        In the third terminal,
            change directory by running the following command:
                cd ~/Swerve 
            and then set necessary ROS variables:
                source catkin_ws/devel/setup.bash
            and then run the scenarios by using following commands: 
                Map1-2d-50      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map1-2d-50.json 
                Map2-2d-12      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-12.json
                Map2-2d-17      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json
                Map3-2d-34      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map3-2d-34.json
                Map1-3d-50      python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map1-3d-50.json 
                Map2-3d-17      python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map2-3d-17.json 
                Map4-3d-20(Q)   python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map4-3d-20.json 
                Map4-3d-20(C&Q) python3 agents/spawn_agents_mixed.py -scenario ./scenario_hscc/map4-3d-20.json
            Note that while generating data for table 1, between running each of the above scenarios, we restart the verification server to reset the cache. 
            Due to the complexity of the scenarios, Map3-2d-34, Map4-3d-20(Q), Map4-3d-20(C&Q) make take more than an hour to finish. 

    Swerve without symmetry caching: 
        Start three terminals,
        
        In the first terminal
            run following command to start roscore: 
                roscore 

        In the second terminal, 
            change directory by running the following command:
                cd ~/Swerve 
            and then set necessary ROS variables:
                source catkin_ws/devel/setup.bash
            and then start the Verification Server without symmetry and caching by using command:
                rosrun verification_server verification_server.py -no_cache

        In the third terminal,
            change directory by running the following command:
                cd ~/Swerve 
            and then set necessary ROS variables:
                source catkin_ws/devel/setup.bash
            and then run the scenarios by using following commands: 
                Map1-2d-50      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map1-2d-50.json 
                Map2-2d-12      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-12.json
                Map2-2d-17      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json
                Map3-2d-34      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map3-2d-34.json
                Map1-3d-50      python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map1-3d-50.json 
                Map2-3d-17      python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map2-3d-17.json 
                Map4-3d-20(Q)   python3 agents/spawn_agents_quadrotor.py -scenario ./scenario_hscc/map4-3d-20.json 
                Map4-3d-20(C&Q) python3 agents/spawn_agents_mixed.py -scenario ./scenario_hscc/map4-3d-20.json
            Due to the complexity of the scenarios, Map3-2d-34, Map4-3d-20(Q), Map4-3d-20(C&Q) make take more than an hour to finish. 

2. For the experiment results in Table 2:
    Start three terminals,
    In the first terminal
        run following command to start roscore: 
            roscore 

    In the second terminal, 
        change directory by running the following command:
            cd ~/Swerve 
        and then set necessary ROS variables:
            source catkin_ws/devel/setup.bash
        and then start the Verification Server with symmetry and caching by using command:
            rosrun verification_server verification_server.py

    In the third terminal,
        change directory by running the following command:
            cd ~/Swerve 
        and then set necessary ROS variables:
            source catkin_ws/devel/setup.bash
        and then run the scenarios by using following commands: 
            Map2-2d-17(15)  python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json
            Map2-2d-12      python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-12.json
            Map2-2d-6       python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-6.json
            Map2-2d-17(20)  python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json -Ts 20
            Map2-2d-17(10)  python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json -Ts 10
            Map2-2d-17(5)   python3 agents/spawn_agents_car.py -scenario ./scenario_hscc/map2-2d-17.json -Ts 5
        Note that while generating data for table 1, between running each of the above scenarios, we restart the verification server to reset the cache. 

3. To generate Figure 3: figure 3 can be obtained by visualizing the result from a experiment run. 
    Change directory by running the following command:
        cd ~/Swerve
    and then run following commands to visualize the result:
        python3 scripts/res_vis_segment.py

The statistics are displayed at the end in terminal runing the scenario after each scenario finishes. The important statistics of Swerve are: 
|O|: Number of obstacles
|S|: Total number of plan segments
Rc: Number of calls to the reachability subroutine
ARt: Average response time for each service call
MRt: Max response time
MRt-90: Max response time of 90% of agents
ASt: The average traversal time for each path segment 

The computer that we used to do the testing has the following specifications:
32 GB RAM
Ubuntu 20.04
AMD Ryzen 7 5800X CPU @ 3.80GHz

The main file that implements the verification server is catkin_ws/src/verification_server/scripts/verification_server.py.

The agents that requests for verification service are implemented in agents/agent_car.py, agents/agent_car3d.py, and agents/agent_quadrotor. 

The spawners for agents and visualizer of runtime data are implemented in agents/spawn_agent_car.py, agents/spawn_agents_mixed.py, and agents/spawn_agents_quadrotor.py

