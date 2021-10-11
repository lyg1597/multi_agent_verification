import numpy as np 
import json 

fn = './data/res_car_nounsafe.json'
with open(fn, 'r') as f:
    res = json.load(f)

total_length = []
total_hit = []
verification_time = []
segment_time = []
safe = 0
num_agent = 0
for key, agent in res.items():
    total_length.append(agent['total_length'])
    total_hit.append(agent['num_hit'])
    verification_time += agent['verification_time']
    segment_time += agent['segment_time']
    safe += agent['result']
    num_agent += 1

total_length = np.array(total_length)
total_hit = np.array(total_hit) 
verification_time = np.array(verification_time)
segment_time = np.array(segment_time) 


segments_sum = np.sum(total_length)
print(f"total number of segments {segments_sum}")
cache_hit = np.sum(total_hit)
print(f"total number of cache hit {cache_hit}")
avg_verification_time = np.mean(verification_time)
print(f"average verification time {avg_verification_time}")
max_verification_time = np.amax(verification_time)
print(f"max verification time {max_verification_time}")
avg_segment_time = np.mean(segment_time)
print(f"average segment time {avg_segment_time}")
print(f"{safe} agents safe, within {num_agent} agents")