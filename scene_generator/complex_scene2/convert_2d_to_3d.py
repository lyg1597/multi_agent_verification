import json 
import numpy as np

def convert_to_3d(fn):
    config = {}
    with open(fn,'r') as f:
        config = json.load(f)

    # guard_list = config["agents"][0]["guards"]
    unsafe_set = config['unsafeSet']
    if len(unsafe_set[0][1][0]) == 2:
        for i in range(len(unsafe_set)):
            for j in range(4):
                unsafe_set[i][1][0].append(-100)
                unsafe_set[i][1][1].append(100)
    config["unsafeSet"] = unsafe_set

    mode_list = config['agents'][0]['mode_list']
    for i in range(len(mode_list)):
        mode_list[i][1].insert(2,0)
        mode_list[i][1].append(0)
    config['agents'][0]['mode_list'] = mode_list

    guards = config['agents'][0]['guards']
    for i in range(len(guards)):
        guards[i][1][0][-1] = -0.5
        guards[i][1][1][-1] = 0.5
        for j in range(3):
            guards[i][1][0].append(-100)
            guards[i][1][1].append(100)
    config['agents'][0]['guards'] = guards

    with open(fn,'w') as f:     
        json.dump(config,f)    

if __name__ == "__main__":
    fn = "./scene_complex2_quadrotor.json"
    convert_to_3d(fn)