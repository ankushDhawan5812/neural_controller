from collections import OrderedDict
import json
import torch

model_verb_cpu = torch.load("/home/pi/policy_lstm_1.pt", map_location="cpu").eval()
od1 = model_verb_cpu.state_dict()
od1 = OrderedDict({k: od1[k].detach().cpu().tolist() for k in od1})
od1 = json.dumps(od1)
with open('/home/pi/model_weights.json', 'w') as outfile:
    outfile.write(od1)