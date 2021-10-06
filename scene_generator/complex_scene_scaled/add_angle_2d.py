import json
import numpy as np

def add_angle(fn, rad = 0.5):
    config = {}
    with open(fn,'r') as f:
        config = json.load(f)

    edge_list = config["agents"][0]["edge_list"]
    # guard_list = config["agents"][0]["guards"]

    for i in range(len(edge_list)):
        edge = edge_list[i]
        if edge[0] == 40 and edge[1] == 45:
            print("here")
        src = config["agents"][0]["mode_list"][edge[0]][1]

        orientation = np.arctan2(src[3]-src[1],src[2]-src[0])
        config["agents"][0]["guards"][i][1][0][2] = orientation - rad
        config["agents"][0]["guards"][i][1][1][2] = orientation + rad

    with open(fn,'w') as f:     
        json.dump(config,f)    

if __name__ == "__main__":
    fn = "./scene_complex.json"
    add_angle(fn)