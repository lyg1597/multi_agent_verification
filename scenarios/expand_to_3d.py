import numpy as np 
import json 

def expand_to_3d(input_fn, output_fn):
    f = open(input_fn, 'r')
    config = json.load(f)
    f.close()

    # Expand unsafeset
    unsafe_set = config['unsafeSet']
    if len(unsafe_set[0][1][0]) == 2:
        for i in range(len(unsafe_set)):
            for j in range(4):
                unsafe_set[i][1][0].append(-100)
                unsafe_set[i][1][1].append(100)
    elif len(unsafe_set[0][1][0]) == 3:
        for i in range(len(unsafe_set)):
            unsafe_set[i][1][0][-1] = -100
            unsafe_set[i][1][1][-1] = 100
            for j in range(3):
                unsafe_set[i][1][0].append(-100)
                unsafe_set[i][1][1].append(100)
    config["unsafeSet"] = unsafe_set

    # Expand waypoint for each agents
    agent_list = config['agents']
    new_agent_list = []
    for agent in agent_list:
        mode_list = agent['mode_list']
        for i in range(len(mode_list)):
            mode_list[i][1].insert(2,0)
            mode_list[i][1].append(0)
        agent['mode_list'] = mode_list
        new_agent_list.append(agent)
    config['agents'] = new_agent_list

    with open(output_fn, 'w+') as f:
        json.dump(config, f)

if __name__ == "__main__":
    input_fn = './scene_complex_v1.json'
    output_fn = './scene_complex_v1_3d.json'
    expand_to_3d(input_fn, output_fn)