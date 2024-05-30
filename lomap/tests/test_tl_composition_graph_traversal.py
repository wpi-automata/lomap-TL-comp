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

def tl():

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

        return EQ_P(OR(EQ_A,EQ_B)) #A OR B

def make_fsa(specs):
    for spec in specs:
        aut = Fsa(multi=False)
        aut.from_formula(spec)
        print(aut)
    return aut

def construct_ts():
    ts = Ts(directed=True, multi=False)
    ts.g = nx.grid_2d_graph(4, 3)

    ts.init[(1, 1)] = 1

    ts.g.add_node((0, 0), attr_dict={'prop': set(['a'])})
    ts.g.add_node((3, 2), attr_dict={'prop': set(['b'])})

    ts.g.add_edges_from(ts.g.edges(), weight=1)

    return ts

def is_word_accepted_verbose(fsa, word):
    s_current = next(iter(fsa.init))
    for symbol in word:
        s_next = fsa.next_state(s_current, symbol)
        print('state:', s_current, 'symbol:', symbol, 'next_state:', s_next)
        if s_next is None:
            print('blocked on (state, symbol):', (s_current, symbol))
            return
        s_current = s_next
    print('terminal state', s_current, 'Accept word:', s_current in fsa.final)


def create_parseable_path(fsa, symbols_produced):
    props = {v: k for k, v in fsa.props.items()}

    new_path = []

    for s in symbols_produced:
        binary = decimal_to_binary(s)
        numerical_props = numerical_binary(binary)
        alphabetical_props = []
        for numerical_prop in numerical_props:
            alphabetical_props.append(props.get(numerical_prop))
        new_path.append(set(alphabetical_props))

    return new_path


# First number always 1, need to go right to left to parse. Ex:
# decimalToBinary(8) -> 1000
# decimalToBinary(18) -> 10010  
# decimalToBinary(7) -> 111
def decimal_to_binary(num):
     
    # if num >= 1:
    #     return decimal_to_binary(num // 2)
    # # print(num % 2, end = '')
    return bin(num).replace("0b", "")



def numerical_binary(binary):
    str_binary = str(binary)[::-1]
    symbols = []
    for i in range(len(str_binary)):
        if str_binary[i]=="1":
            symbols.append(math.pow(2,i))
    return symbols


def main():
    fsa = make_fsa(['F a && F !b'])
    
    print('Is FSA deterministic:', fsa.is_deterministic())

    # Load the map
    grid, start, goal = load_map('map_multiple_symbols.csv')

    # Search
    astar_path, astar_symbols_produced, aster_steps = astar(grid, start, goal)

    print(f"Symbols produced: {astar_symbols_produced}")

    # Show result
    draw_path(grid, start, goal, astar_path, 'A*')
    plt.show()

    word = create_parseable_path(fsa, astar_symbols_produced)

    print(f"word: {word}")
    """
    word: [set(), set(), set(), set(), set(), set(), set(), set(), set(), set(), set(), {'b', 'a'}, {'b'}]
    """

    is_word_accepted_verbose(fsa, word)
    print('Word accepted (Method):', fsa.is_word_accepted(word))

    print('Language empty?:', fsa.is_language_empty())

if __name__ == '__main__':
    main()
