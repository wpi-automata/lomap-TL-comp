from __future__ import print_function


import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))
sys.path.append(os.path.abspath(os.getcwd())+'/src')

from TL_GridWorld import TLGridWorld
from boolean_composition.four_rooms.library import *
from value_iteration import q_iteration
from itertools import product

import networkx as nx
from lomap import Fsa, Ts, ts_times_fsa, ts_times_ts

TL_MAP = "1 1 1 1 1 1 1 1 1 1 1 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 1 1 1 1 1 1 1 1 1 1 1"

goal_reward = 2
env = TLGridWorld(MAP=TL_MAP)

labels = ['A','B']
label_regions = {'A': [(3, 3), (4, 4), (3, 4), (4, 3), (9, 9), (8, 8), (9, 8), (8, 9)],
                 'B': [(2, 2), (3, 2), (4, 2), (5, 2), (2, 3), (2, 4), (2, 5), (3, 5), (4, 5), (5, 5), (5, 3), (5, 4)]}

"""
        "1 1 1 1 1 1 1 1 1 1 1 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 B B B B 0 0 0 0 0 1\n" \
        "1 0 B A A B 0 0 0 0 0 1\n" \
        "1 0 B A A B 0 0 0 0 0 1\n" \
        "1 0 B B B B 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 0 0 0 0 0 0 0 A A 0 1\n" \
        "1 0 0 0 0 0 0 0 A A 0 1\n" \
        "1 0 0 0 0 0 0 0 0 0 0 1\n" \
        "1 1 1 1 1 1 1 1 1 1 1 1"
"""

label_map = defaultdict(set)
for label, region in label_regions.items():
    for state in region:
        label_map[state].add(label)
label_map = dict(label_map)

print(f"label map: {label_map}")

tl_goals = ['A','B']
### Learning min/max Q functions
env_max = TLGridWorld(tl_goals=tl_goals, goal_reward=goal_reward, MAP=TL_MAP, labels=labels, label_map=label_map)
env_min = TLGridWorld(tl_goals=tl_goals, goal_reward=env.step_reward, MAP=TL_MAP, labels=labels, label_map=label_map,rmax=goal_reward)
EQ_max, _ = q_iteration(env_max)
EQ_min, _ = q_iteration(env_min)
P_max = EQ_P(EQ_max)
V_max = EQ_V(EQ_max)
env_max.render(P=P_max,V=V_max) #Isnt this A and B?

### Learning base TL task A
tl_goals = ['A']
# goals_A = [[pos,pos] for pos in goals]
env_A = TLGridWorld(tl_goals=tl_goals, goal_reward=goal_reward, MAP=TL_MAP, labels=labels, label_map=label_map)
EQ_A, P_A = q_iteration(env_A)
V_A = EQ_V(EQ_A)
env_A.render(P=P_A,V=V_A)

### Learning base TL task B
tl_goals = ['B']
# goals_A = [[pos,pos] for pos in goals]
env_B = TLGridWorld(tl_goals=tl_goals, goal_reward=goal_reward, MAP=TL_MAP, labels=labels, label_map=label_map)
EQ_B, P_B = q_iteration(env_B)
V_B = EQ_V(EQ_B)
env_B.render(P=P_B,V=V_B)

EQ_nB = NOT(EQ_B, EQ_max=EQ_max, EQ_min=EQ_min)
env_B.render(P=EQ_P(EQ_nB), V=EQ_V(EQ_nB))

# A and B -- not well-defined
# A or B -- well-defined
env.render(P=EQ_P(OR(EQ_A,EQ_B)),V=EQ_V(OR(EQ_A,EQ_B)))

# A and NOT B
env.render( P=EQ_P(AND(EQ_A,NOT(EQ_B, EQ_max=EQ_max, EQ_min=EQ_min))), V = EQ_V(AND(EQ_A,NOT(EQ_B,EQ_max=EQ_max, EQ_min=EQ_min))), title='A and NOT B')

# NOT A and B
env.render( P=EQ_P(AND(EQ_B,NOT(EQ_A, EQ_max=EQ_max, EQ_min=EQ_min))), V = EQ_V(AND(EQ_B,NOT(EQ_A,EQ_max=EQ_max, EQ_min=EQ_min))), title='NOT A and B')
