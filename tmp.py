import json 
import matplotlib.pyplot as plt 

fn = 'res.json'
with open(fn, 'r') as f:
    tmp = json.load(f)
legend_list = []
for agent_idx in tmp:
    legend_list.append(agent_idx)
    agent = tmp[agent_idx]
    plt.plot(agent['verification_time'], label = agent_idx)
    plt.plot(agent['verification_time'], '.')
plt.legend()
plt.show()


for agent_idx in tmp:
    agent = tmp[agent_idx]
    horiz = []
    vert = []
    val = 0
    for i in range(len(agent['segment_time'])):
        horiz.append(val)
        val += agent['segment_time'][i]
        vert.append(i)
    plt.plot(horiz, vert, label = agent_idx)
    plt.plot(horiz, vert, '.')
plt.legend()
plt.show()
