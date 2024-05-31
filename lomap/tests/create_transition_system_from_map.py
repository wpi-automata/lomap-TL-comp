import sys
import os

sys.path.append(os.path.abspath(os.getcwd()))
sys.path.append(os.path.abspath(os.getcwd())+'/src')

import string 

from a_star import *
from create_graph_from_map import *
import networkx as nx
import math
import copy
from test_map_word_accepted_randomized_occupancy_grid import *
from lomap.algorithms.product import ts_times_buchi
from lomap.classes import Buchi, Ts

def create_propositions(props_numerical):
    props = dict()
    # for i in props_numerical:
    number_power_of_twos = sum(1 for i in props_numerical if isPowerOfTwo(i))
    # props_alphabetical = list(map(chr, range(number_power_of_twos))) #creates alphabetical rep 
    props_alphabetical = list(string.ascii_lowercase)[0:number_power_of_twos]
    for i in range(len(props_alphabetical)):
        props[props_alphabetical[i]] = 2**i
    return props

# def power_of_two(x):
#     return (x and (not(x & (x - 1))) )
 
# Function to check
# if x is power of 2
def isPowerOfTwo(n):
    # return (math.log(n)/math.log(2)).is_integer()
    return (n & (n-1) == 0) and n != 0


def create_transitions(G, edges, nodes):

    spl = dict(nx.all_pairs_shortest_path_length(G))
    print(f"Shortest path length: {spl}")

    labels = dict()

    for edge in edges:
        for node in nodes:
            key = str(edge)
            start_edge = spl[edge[0]][node]
            end_edge = spl[edge[1]][node]
            # print(f"Start edge dist from {edge[0]} to {node}: {start_edge}")
            # print(f"end edge dist from {edge[1]} to {node}: {end_edge}")
            if start_edge>end_edge:
                if not labels.get(key):
                    label = list()
                    label.append(node)
                    labels[key] = label
                else:
                    labels.get(key).append(node)

    print(f"labels: {labels}")
    return labels

    #Other potentially important functions:
    # sp = dict(nx.all_pairs_shortest_path(G))
    # print(f"Shortest path: {sp}")
    # nx.all_simple_paths(G, source=0, target=3)
    # eg = nx.ego_graph(G, '4', radius=10, center=True, undirected=False, distance=None)
    # print(f"Ego graph: {eg}")
    # fig, axes = plt.subplots(1,1,dpi=72)
    # nx.draw(eg, pos=nx.spring_layout(eg), ax=axes, with_labels=True)
    # plt.show()

def prune_labels(nodes, labels):

    label_keys = list(labels.keys())

    #for these purposes, need to remove every sub label e.g. 1.1 1.2 and replace it with 1, since same symbol in map
    for node in nodes:
        for label_key in label_keys:
            pruned_label = list(set(map(lambda x: str(math.floor(float(x))),labels.get(label_key)))) #set to remove duplicates
            labels[label_key] = pruned_label
    
    pruned_labels = copy.deepcopy(labels)

    for node in nodes:
        node_outgoing_labels = dict()
        simplfied_node_rep = str(math.floor(float(node)))

        for label_key in label_keys:

            label_key_list = label_key.strip('][\'\"').split('\', \'')
            if label_key_list[0] == node:
                node_outgoing_labels[label_key_list[1]] = labels.get(label_key)

        new_transition_dict = dict()

        case_1(node, node_outgoing_labels, new_transition_dict)
        case_2(simplfied_node_rep, node_outgoing_labels)        

        if not new_transition_dict:
            step_transition_dict = {k:[item for item in v] for (k,v) in node_outgoing_labels.items()}
            new_transition_dict.update(step_transition_dict)

        if new_transition_dict:
            for key in new_transition_dict.keys():
                new_key = str([node, key])
                pruned_labels[new_key] = new_transition_dict.get(key)

    return pruned_labels

'''
CASES:
1) if all outgoing edges same share transition and all go to node containing shared transition - remove transition from all (maybe need check to make sure not empty transition?)
2) if there is an outgoing edge containing same label as node remove it 
3) if some outgoing edges go to node containing shared transition but others don't, remove transition from others
4) if 2 of the same node symbols (e.g. 1.0 and 1.1) are connected, there should be no transitions between them that contain the symbol on the transition
etc
'''

def case_1(node, node_outgoing_labels, new_transition_dict):
    if len(list(node_outgoing_labels.keys())) > 1: #if there is more than one outgoing edge
        node_outgoing_label_intersections = set.intersection(*[set(x) for x in node_outgoing_labels.values()])
        print(f"outgoing edge intersection for node {node}: {node_outgoing_label_intersections}")
        if len(node_outgoing_label_intersections) > 0: #if more than one reduced (eg. 1.1, 1.0 = 1) label on different outgoing edges match
            print(f"node_outgoing_labels {node}: {node_outgoing_labels}")

            #TODO: check if works with multiple intersection values
            for node_outgoing_label_intersection in node_outgoing_label_intersections:

                print(f"node_outgoing_label_intersection: {node_outgoing_label_intersection}")

                #transitions that go to node that shares same value as intersection between transitions
                transitions_that_go_to_intersection_node = {k:v for (k,v) in node_outgoing_labels.items() if str(math.floor(float(k))) == node_outgoing_label_intersection}
                print(f"transitions_that_go_to_intersection_node dict: {transitions_that_go_to_intersection_node}")

                #transitions that share same value as intersection
                transitions_that_share_same_value = {k:v for (k,v) in node_outgoing_labels.items() for item in v if item == node_outgoing_label_intersection}
                # transitions_that_share_same_value = {k:[item for item in v if item==str(node_outgoing_label_intersection)] for (k,v) in node_outgoing_labels.items()}

                print(f"transitions_that_share_same_value dict: {transitions_that_share_same_value}")

                if transitions_that_go_to_intersection_node==transitions_that_share_same_value:
                    # if all(len(l) > 1 for l in list(transitions_that_go_to_intersection_node.values())): #all transition labels longer than 1 symbol
                    step_transition_dict = {k:[item for item in v if item!=str(node_outgoing_label_intersection)] for (k,v) in node_outgoing_labels.items()}
                    new_transition_dict.update(step_transition_dict)
                    print(f"new_transition_dict: {new_transition_dict}")     

def case_2(simplfied_node_rep, node_outgoing_labels):
    for key in node_outgoing_labels.keys():
            if simplfied_node_rep in node_outgoing_labels.get(key):
                node_outgoing_labels.get(key).remove(simplfied_node_rep)    


def construct_ts(G):
    ts = Ts(directed=True, multi=False)
    ts.g = G
    # ts.g = nx.grid_2d_graph(4, 3)

    # ts.init[(1, 1)] = 1

    # ts.g.add_node((0, 0), attr_dict={'prop': set(['a'])})
    # ts.g.add_node((3, 2), attr_dict={'prop': set(['b'])})

    # ts.g.add_edges_from(ts.g.edges(), weight=1)

    return ts           


def main():
    grid, start, goal = load_map('maps/map_multiple_symbols_BCD_level3.csv')
    # draw_path(grid, start, goal, [], 'Map')
    clusters = create_clusters(np.asarray(grid))
    print(f"Clusters: {clusters}")
    unique_clusters = each_cluster_uuid(clusters)
    print(f"Unique Clusters: {unique_clusters}")
    edges = create_graph(unique_clusters)
    print(f"Edges: {edges}")
    reversed_edges = [sublist[::-1] for sublist in edges[::-1]]
    print(f"Reversed Edges: {reversed_edges}")
    edges.extend(reversed_edges)

    G = nx.DiGraph()
    G.add_edges_from(edges)
    
    labels = create_transitions(G, edges, list(unique_clusters.keys()))
    labels = prune_labels(list(unique_clusters.keys()), labels)

    G.remove_edges_from(edges)

    for edge in edges:
        edge.append({'pi':labels.get(str(edge))})
    G.add_edges_from(edges)
    draw_graph(G)

    ts = construct_ts(G)

    spec = 'G (F a && F g && !e)'
    buchi = Buchi()
    buchi.from_formula(spec)
    print('Created Buchi automaton of size', buchi.size())
    buchi.visualize(draw='matplotlib')
    plt.show()

    pa = ts_times_buchi(ts, buchi)
    print('Created product automaton of size', pa.size())
    pa.visualize(draw='matplotlib')
    plt.show()

if __name__ == '__main__':
    
    main()