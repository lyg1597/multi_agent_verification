import numpy as np 
import json 

# fn = './data/res_car3_nocache.json'
fn = './res.json'
with open(fn, 'r') as f:
    res = json.load(f)

total_length = []
total_hit = []
verification_time = []
segment_time = []
safe = 0
num_agent = 0
verification_time_later = []
for key, agent in res.items():
    total_length.append(agent['total_length'])
    total_hit.append(agent['num_hit'])
    verification_time_later += agent['verification_time'][5:]
    verification_time += agent['verification_time']
    segment_time += agent['segment_time']
    safe += agent['result']
    num_agent += 1

total_length = np.array(total_length)
total_hit = np.array(total_hit) 
verification_time = np.array(verification_time)
segment_time = np.array(segment_time) 
sorted_verification_time = np.sort(verification_time)
sorted_len = len(sorted_verification_time)

segments_sum = np.sum(total_length)
print(f"total number of segments {segments_sum}")
cache_hit = np.sum(total_hit)
print(f"total number of cache hit {cache_hit}")
Rc = segments_sum - cache_hit 
print(f"Rc {Rc}")

avg_verification_time = np.mean(verification_time)
print(f"average verification time {avg_verification_time}")
avg_verification_time_later = np.mean(verification_time_later)
print(f"average verification time after first 5 steps {avg_verification_time_later}")
percent_verification_time = sorted_verification_time[int(sorted_len*0.9)]
print(f"90% verification time {percent_verification_time}")
avg_verification_time_later = np.mean(verification_time_later)
max_verification_time = np.amax(verification_time)
print(f"max verification time {max_verification_time}")
max_verification_time_later = np.amax(verification_time_later)
print(f"max verification time after first 5 steps {max_verification_time_later}")
avg_segment_time = np.mean(segment_time)
print(f"average segment time {avg_segment_time}")
print(f"{safe} agents safe, within {num_agent} agents")
max_segment_time = np.amax(segment_time)
print(f"max segment time {max_segment_time}")