from a_star import *
from create_graph_from_map import *
import networkx as nx
import math
import copy
import unittest
import string
from collections import Counter
from test_map_word_accepted_randomized_occupancy_grid import *
from lomap.classes import Buchi, Ts

EMPTY_SYMBOL='0'
WALL_SYMBOL = '#'

# Load map, start and goal point.
def load_symbol_map(file_path):
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                parsed_row = [col for col in row]
                grid.append(parsed_row)
    return grid, start, goal

def assign_props(grid):
    props = dict()
    reduced = list(set(i for j in grid for i in j))
    reduced.sort()

    single_chars=[x for x in reduced if (len(x)==1 and x!=EMPTY_SYMBOL and x!=WALL_SYMBOL)]
    props['{}'] = 0

    for i in range(len(single_chars)):
        props[single_chars[i]] = 2**i

    return props

def create_numerical_grid(props, symbol_grid):

    grid = copy.deepcopy(symbol_grid)

    for row in range(len(symbol_grid)):
        for col in range(len(symbol_grid[0])):
            val = symbol_grid[row][col]
            if val in list(props.keys()):
                grid[row][col] = props.get(val)
            elif len(val) > 1:
                sum = 0
                for letter in val:
                    sum += props.get(letter)
                grid[row][col] = sum
                props[val] = sum
            else:
                print("ERROR parsing grid from symbols to binary numerical translation")
    
    return grid


def create_transitions(G, edges, nodes, spl):

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

def prune_labels(nodes, edges, labels, spl):

    #for these purposes, need to remove every sub label e.g. 1.1 1.2 and replace it with 1, since same symbol in map
    for node in nodes:
        for label_key in labels.keys():
            pruned_label = list(set(map(lambda x: str(math.floor(float(x))),labels.get(label_key)))) #set to remove duplicates
            labels[label_key] = pruned_label
    
    pruned_labels = copy.deepcopy(labels)

    case_4(nodes, edges, pruned_labels)

    for node in nodes:
        simplfied_node_rep = str(math.floor(float(node)))

        node_incoming_labels, node_outgoing_labels = get_node_edge_labels(node, pruned_labels)

        case_1(node, node_outgoing_labels, spl)
        case_2(simplfied_node_rep, node_outgoing_labels)
        case_3(node_outgoing_labels)
        case_5(node_outgoing_labels)

        # prune "finalized" labels according to updated outgoing labels for each node
        for key in node_outgoing_labels.keys():
            new_key = str([node, key])
            pruned_labels[new_key] = node_outgoing_labels.get(key)

    case_5(pruned_labels)

    return pruned_labels

'''
CASES:
1) if any outgoing edges share transition label, only keep label on the closest edge(s)
2) if there is an outgoing edge containing same label as node remove it 
3) remove empty symbol from transition
4) if nodes and transitions are the same, combine them
5) remove all empty transitions
potential others:
- if some outgoing edges go to node containing shared transition but others don't, remove transition from others
- if 2 of the same node symbols (e.g. 1.0 and 1.1) are connected, there should be no transitions between them that contain the symbol on the transition
etc
'''

def case_1(node, node_outgoing_labels, spl):
    if len(list(node_outgoing_labels.keys())) > 1: #if there is more than one outgoing edge

        #if item duplicated in list of all edge labels concanated then it occurs on at least more than one edge
        list_of_all_edge_values = [elem for sublist in list(node_outgoing_labels.values()) for elem in sublist]
        node_outgoing_label_intersections = [k for k,v in Counter(list_of_all_edge_values).items() if v>1]

        # node_outgoing_label_intersections = set.intersection(*[set(x) for x in node_outgoing_labels.values()]) #this does not work because if 2/3 edges share label it won't recognize
        print(f"outgoing edge intersection for node {node}: {node_outgoing_label_intersections}")
        if len(node_outgoing_label_intersections) > 0: #if more than one reduced (eg. 1.1, 1.0 = 1) label on different outgoing edges match
            print(f"node_outgoing_labels {node}: {node_outgoing_labels}")

            # loop through all the shared transition labels 
            for shared in node_outgoing_label_intersections:
                distance_to_shared = dict()
                for connected_node in node_outgoing_labels.keys():
                    # ensure the shared transition label is included in the label to this node
                    if shared in node_outgoing_labels[connected_node]:
                        #because the shared nodes are the reduced node value, e.g. '1' instead of '1.0' and '1.1' we need to go through and compare all floored values. take the min of either
                        connected_nodes_connections_path_length = spl[connected_node]
                        # need to loop through all connections and do a check due to the reduced node representation
                        for key in connected_nodes_connections_path_length.keys():
                            if int(float(key)) == int(shared):
                                if connected_node in distance_to_shared:
                                    if connected_nodes_connections_path_length[key] < distance_to_shared[connected_node]:
                                        distance_to_shared[connected_node] = connected_nodes_connections_path_length[key] #update
                                else:
                                    distance_to_shared[connected_node] = connected_nodes_connections_path_length[key] #init
                # filter the list so that the only nodes remaining are the nodes with the shortest length to the node with the same label as the shared transition
                nodes_with_shortest_path_to_shared = [key for key in distance_to_shared if distance_to_shared[key] == min(distance_to_shared.values())] 
                # if the list of nodes with the shortest path length to the node with the same label as the shared transition label natches the list of all nodes with shared transition, then don't remove those nodes since they are equally important
                if len(nodes_with_shortest_path_to_shared) <=1 and nodes_with_shortest_path_to_shared != list(distance_to_shared.keys()): 
                    for k in nodes_with_shortest_path_to_shared:
                        distance_to_shared.pop(k, None)
                # now distance_to_shared_sorted only contains nodes that have a longer path to the shared node. The shared node label will be removed from the transition from node to each of these connected nodes
                # if len(distance_to_shared.keys()) > 1:
                for k in distance_to_shared.keys():
                    node_outgoing_labels[k].remove(shared)

def case_2(simplfied_node_rep, node_outgoing_labels):
    for key in node_outgoing_labels.keys():
            if simplfied_node_rep in node_outgoing_labels.get(key):
                node_outgoing_labels.get(key).remove(simplfied_node_rep)    

def case_3(node_outgoing_labels):
    for key in node_outgoing_labels.keys():
        empty_symbols = [s for s in node_outgoing_labels.get(key) if EMPTY_SYMBOL in s]
        for s in empty_symbols:
            node_outgoing_labels.get(key).remove(s)

def case_4(nodes, edges, pruned_labels):
    #ASSUMPTION: 2 nodes are the same if the input and output labels are the same

    to_remove = dict()
    replacements = list()

    for i in range(len(nodes)):
        node = nodes[i]
        if node not in to_remove:
            node_incoming_labels, node_outgoing_labels = get_node_edge_labels(node, pruned_labels)
            for other_node in nodes[i+1:]:
                if other_node not in to_remove:
                    if int(float(node)) == int(float(other_node)): # reduced node labels are the same
                        other_node_incoming_labels, other_node_outgoing_labels = get_node_edge_labels(other_node, pruned_labels)
                        if node_incoming_labels==other_node_incoming_labels and node_outgoing_labels==other_node_outgoing_labels:
                            key = int(float(node))
                            if key in to_remove:
                                to_remove_nodes = to_remove[int(float(node))]
                            else:
                                to_remove_nodes = set()
                            to_remove_nodes.add(node)
                            to_remove_nodes.add(other_node) 
                            to_remove[int(float(node))] = to_remove_nodes
    
    for reduced_node in to_remove.keys():
        sorted_equivalent_nodes = sorted(to_remove[reduced_node]) 
        # keep the lowest value node. e.g. from [1.0,1.1,1.2], keep 1.0 remove 1.1 and 1.2
        for node_to_remove in sorted_equivalent_nodes[1:]:
            nodes.remove(node_to_remove)
            # remove all instances of the deleted nodes from the nodes and the edge labels
            pruned_labels_updated = {k: list(filter(lambda x: node_to_remove not in x, v)) for k, v in pruned_labels.items() if node_to_remove not in k}
            pruned_labels.clear()
            pruned_labels.update(pruned_labels_updated) #to keep same object for cleanliness
            # edges_updated = list(list(filter(lambda x: node_to_remove not in x, e)) for e in edges)
            edges_updated = [e for e in edges if node_to_remove not in e]
            edges.clear()
            edges.extend(edges_updated)

def case_5(node_outgoing_labels):
    node_outgoing_labels_updated = {k: v for k, v in node_outgoing_labels.items() if v != []}
    node_outgoing_labels.clear()
    node_outgoing_labels.update(node_outgoing_labels_updated) #to keep same object for cleanliness

def get_node_edge_labels(node, labels):
    node_incoming_labels = dict()
    node_outgoing_labels = dict()

    for label_key in labels.keys():

            label_key_list = label_key.strip('][\'\"').split('\', \'')
            if label_key_list[0] == node:
                node_outgoing_labels[label_key_list[1]] = labels.get(label_key)
            if label_key_list[1] == node:
                node_incoming_labels[label_key_list[0]] = labels.get(label_key)

    return node_incoming_labels, node_outgoing_labels
      
def convert_edges_and_add_labels_alphabetical(labels, props, edges):

    num_to_alpha_prop = {v: k for k, v in props.items()}
    label_mapping = dict()
    edges_to_remove = list()

    for edge in edges:
        pi = labels.get(str(edge))
        if pi:
            for i in range(len(pi)):
                pi[i] = num_to_alpha_prop.get(int(pi[i]))

            label_mapping[edge[0]] = num_to_alpha_prop.get(int(float(edge[0])))
            label_mapping[edge[1]] = num_to_alpha_prop.get(int(float(edge[1])))

            edge.append({'pi':pi, 'weight': 0})
        else:
            edges_to_remove.append(edge)
    
    edges = [e for e in edges if e not in edges_to_remove]

    return edges, label_mapping

def add_edge_labels(labels, edges):
    for edge in edges:
        edge.append({'pi':labels.get(str(edge))})

def clean_clusters(clusters):
    if WALL_SYMBOL in clusters.keys():
        del clusters[WALL_SYMBOL]

def create_ts(map_path = "maps/alphabetical_maps/map_multiple_alpha_symbols_complex.csv", init_node='a'):
    '''
    Map must only contain empty set '{}' or alphabetical values
    '''
    symbol_grid, start, goal = load_symbol_map(map_path)
    props = assign_props(symbol_grid)
    grid = create_numerical_grid(props, symbol_grid)

    print(f"replaced grid: {grid}")

    # draw_path(grid, start, goal, [], 'Map')
    clusters = create_clusters(np.asarray(grid))
    clean_clusters(clusters)
    print(f"Clusters: {clusters}")
    unique_clusters = each_cluster_uuid(clusters)
    print(f"Unique Clusters: {unique_clusters}")
    edges = create_graph(unique_clusters)
    print(f"Edges: {edges}")
    reversed_edges = [sublist[::-1] for sublist in edges[::-1]]
    print(f"Reversed Edges: {reversed_edges}")
    edges.extend(reversed_edges)

    intermediate_G = nx.DiGraph()
    intermediate_G.add_edges_from(edges)

    spl = dict(nx.all_pairs_shortest_path_length(intermediate_G))
    print(f"Shortest path length: {spl}")
    
    labels = create_transitions(intermediate_G, edges, list(unique_clusters.keys()), spl)
    labels = prune_labels(list(unique_clusters.keys()), edges, labels, spl)

    G = nx.DiGraph()

    alphabetical_edges, label_mapping = convert_edges_and_add_labels_alphabetical(labels, props, edges)

    G.add_edges_from(alphabetical_edges)

    inv_props = {v: k for k, v in props.items()}

    ts = Ts(directed=True, multi=False)
    for key in copy.deepcopy(G.nodes.keys()):
        G.add_node(key, prop=inv_props[int(float(key))])
    ts.g = G
    ts.init = {init_node}

    draw_graph(G, label_mapping)

    return ts, props

class TestTSCreation(unittest.TestCase):

    def test_example_1(self):
        nodes = ['0', '1.1', '1.0', '2', '4']
        out_edges = [('0', '1.0'), ('0', '1.1'), ('1.0', '2'), ('1.0', '0'), ('1.1', '4'), ('1.1', '0'), ('2', '1.0'), ('4', '1.1')]
        in_edges = [('1.1', '0'), ('1.0', '0'), ('0', '1.0'), ('2', '1.0'), ('0', '1.1'), ('4', '1.1'), ('1.0', '2'), ('1.1', '4')]
        adj = {'0': {'1.0': {'pi': ['b'], 'weight': 0}, '1.1': {'pi': ['c'], 'weight': 0}}, '1.0': {'2': {'pi': ['b'], 'weight': 0}, '0': {'pi': ['c'], 'weight': 0}}, '1.1': {'4': {'pi': ['c'], 'weight': 0}, '0': {'pi': ['b'], 'weight': 0}}, '2': {'1.0': {'pi': ['c', 'a'], 'weight': 0}}, '4': {'1.1': {'pi': ['b', 'a'], 'weight': 0}}}
        ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example1.csv')
        self.assertEqual(ts.g.number_of_nodes(), 5)
        self.assertEqual(ts.g.number_of_edges(), 8)
        self.assertEqual(set(ts.g.nodes()), set(nodes))
        self.assertEqual(set(ts.g.out_edges()), set(out_edges))
        self.assertEqual(set(ts.g.in_edges()), set(in_edges))
        self.assertEqual(sorted(dict(ts.g.adjacency())), sorted(adj))

    def test_example_2(self):
        nodes = ['0.0', '1', '4', '2', '0.1']
        out_edges = [('0.0', '1'), ('0.0', '4'), ('0.0', '2'), ('1', '0.0'), ('4', '0.0'), ('2', '0.0'), ('0.1', '1'), ('0.1', '2')]
        in_edges = [('2', '0.0'), ('4', '0.0'), ('1', '0.0'), ('0.0', '1'), ('0.1', '1'), ('0.0', '4'), ('0.0', '2'), ('0.1', '2')]
        adj = {'0.0': {'1': {'pi': ['a'], 'weight': 0}, '4': {'pi': ['c'], 'weight': 0}, '2': {'pi': ['b'], 'weight': 0}}, '1': {'0.0': {'pi': ['c'], 'weight': 0}}, '4': {'0.0': {'pi': ['a', 'b'], 'weight': 0}}, '2': {'0.0': {'pi': ['c'], 'weight': 0}}, '0.1': {'1': {'pi': ['a'], 'weight': 0}, '2': {'pi': ['b'], 'weight': 0}}}
        ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example2.csv')
        self.assertEqual(ts.g.number_of_nodes(), 5)
        self.assertEqual(ts.g.number_of_edges(), 8)
        self.assertEqual(set(ts.g.nodes()), set(nodes))
        self.assertEqual(set(ts.g.out_edges()), set(out_edges))
        self.assertEqual(set(ts.g.in_edges()), set(in_edges))
        self.assertEqual(sorted(dict(ts.g.adjacency())), sorted(adj))

    def test_example_3(self):
        nodes = ['0', '1.0']
        out_edges = [('0', '1.0')]
        in_edges = [('0', '1.0')]
        adj = {'0': {'1.0': {'pi': ['a'], 'weight': 0}}, '1.0': {}}
        ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example3.csv')
        self.assertEqual(ts.g.number_of_nodes(), 2)
        self.assertEqual(ts.g.number_of_edges(), 1)
        self.assertEqual(set(ts.g.nodes()), set(nodes))
        self.assertEqual(set(ts.g.out_edges()), set(out_edges))
        self.assertEqual(set(ts.g.in_edges()), set(in_edges))
        self.assertEqual(sorted(dict(ts.g.adjacency())), sorted(adj))

    def test_example_4(self):
        nodes = ['4.0', '0', '1.0', '4.1', '2.1', '2.0', '1.1']
        out_edges = [('4.0', '1.0'), ('1.0', '2.0'), ('1.0', '4.0'), ('4.1', '2.1'), ('2.1', '1.1'), ('2.1', '4.1'), ('2.0', '1.0'), ('1.1', '2.1'), ('0', '4.1'), ('0', '4.0')]
        in_edges = [('1.0', '4.0'), ('0', '4.0'), ('4.0', '1.0'), ('2.0', '1.0'), ('2.1', '4.1'), ('0', '4.1'), ('4.1', '2.1'), ('1.1', '2.1'), ('1.0', '2.0'), ('2.1', '1.1')]
        adj = {'4.0': {'1.0': {'pi': ['b', 'a'], 'weight': 0}}, '1.0': {'2.0': {'pi': ['b'], 'weight': 0}, '4.0': {'pi': ['c'], 'weight': 0}}, '4.1': {'2.1': {'pi': ['b', 'a'], 'weight': 0}}, '2.1': {'1.1': {'pi': ['a'], 'weight': 0}, '4.1': {'pi': ['c'], 'weight': 0}}, '2.0': {'1.0': {'pi': ['c', 'a'], 'weight': 0}}, '1.1': {'2.1': {'pi': ['c', 'b'], 'weight': 0}}, '0': {'4.1': {'pi': ['b'], 'weight': 0}, '4.0': {'pi': ['a'], 'weight': 0}}}
        ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example4.csv')
        self.assertEqual(ts.g.number_of_nodes(), 7)
        self.assertEqual(ts.g.number_of_edges(), 10)
        self.assertEqual(set(ts.g.nodes()), set(nodes))
        self.assertEqual(set(ts.g.out_edges()), set(out_edges))
        self.assertEqual(set(ts.g.in_edges()), set(in_edges))
        self.assertEqual(sorted(dict(ts.g.adjacency())), sorted(adj))

    def test_example_5(self):
        nodes = ['4.0', '8', '2.1', '4.1', '1.0', '4.2', '2.2', '2.0', '1.1']
        out_edges = [('4.0', '8'), ('4.0', '2.1'), ('8', '4.1'), ('2.1', '4.0'), ('4.1', '8'), ('4.1', '1.0'), ('1.0', '2.0'), ('1.0', '4.1'), ('4.2', '8'), ('4.2', '2.2'), ('2.2', '1.1'), ('2.2', '4.2'), ('2.0', '1.0'), ('1.1', '2.2')]
        in_edges = [('2.1', '4.0'), ('4.0', '8'), ('4.1', '8'), ('4.2', '8'), ('4.0', '2.1'), ('1.0', '4.1'), ('8', '4.1'), ('4.1', '1.0'), ('2.0', '1.0'), ('2.2', '4.2'), ('4.2', '2.2'), ('1.1', '2.2'), ('1.0', '2.0'), ('2.2', '1.1')]
        adj = {'4.0': {'8': {'pi': ['d', 'a'], 'weight': 0}, '2.1': {'pi': ['b'], 'weight': 0}}, '8': {'4.1': {'pi': ['a'], 'weight': 0}}, '2.1': {'4.0': {'pi': ['d', 'c', 'a'], 'weight': 0}}, '4.1': {'8': {'pi': ['d'], 'weight': 0}, '1.0': {'pi': ['b', 'a'], 'weight': 0}}, '1.0': {'2.0': {'pi': ['b'], 'weight': 0}, '4.1': {'pi': ['d', 'c'], 'weight': 0}}, '4.2': {'8': {'pi': ['d'], 'weight': 0}, '2.2': {'pi': ['b', 'a'], 'weight': 0}}, '2.2': {'1.1': {'pi': ['a'], 'weight': 0}, '4.2': {'pi': ['d', 'c'], 'weight': 0}}, '2.0': {'1.0': {'pi': ['d', 'c', 'a'], 'weight': 0}}, '1.1': {'2.2': {'pi': ['d', 'c', 'b'], 'weight': 0}}}
        ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example5.csv')
        self.assertEqual(ts.g.number_of_nodes(), 9)
        self.assertEqual(ts.g.number_of_edges(), 14)
        self.assertEqual(set(ts.g.nodes()), set(nodes))
        self.assertEqual(set(ts.g.out_edges()), set(out_edges))
        self.assertEqual(set(ts.g.in_edges()), set(in_edges))
        self.assertEqual(sorted(dict(ts.g.adjacency())), sorted(adj))

if __name__ == '__main__':
    unittest.main()
    # ts, _ = create_ts('maps/alphabetical_maps/office_world.csv')
    # ts, _ = create_ts('maps/unit_test_maps/alphabetical_maps/example5.csv')