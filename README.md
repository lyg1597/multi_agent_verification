# Multi-Agent Verification
## Software Requirements

## Setup the experiments
1. In the root directory of the software package, execute command 
```
echo 'export PYTHONPATH=$PYTHONPATH:$(pwd)' >> ~/.bashrc
source ~/.bashrc 
```
to add the current directory to python path     

2. Run command 
```
cd catkin_ws
catkin_make
cd ..
```
to build the ros workspace

3. Run command 
```
roscore
```
to start the ros master node 

## Run server 
To start the verification server, in the root directory of the software package, execute command 
``` 
source ./catkin_ws/devel/setup.bash 
rosrun verifier_server verification_server.py
```

## Run agents 
To run the agents for the experiment, in the root directory of the software package, execute command 
```
source ./catkin_ws/devel/setup.bash 
python3 agents/spawn_agents.py
```
