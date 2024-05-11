from a_star import *
from create_graph_from_map import *
import networkx as nx
import math
import copy

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

    # map(lambda x: map(lambda y: math.floor(float(y)), x), list(labels.values()))
    # print(f"cleaned: {labels}")
    # map(lambda x: map(lambda y: y + 1, x), [[1], [1, 2], [1, 2, 3]])

    pruned_labels = copy.deepcopy(labels)
    label_keys = list(labels.keys())

    for node in nodes:
        node_outgoing_labels = dict()

        for label_key in label_keys:

            #for these purposes, need to remove every sub label e.g. 1.1 1.2 and replace it with 1, since same symbol in map
            #maybe clean with similar : new_dict = {math.floor(float(key)):value for (key,value) in outgoing_labels_for_node.items()}
            pruned_label = list(map(lambda x: str(math.floor(float(x))),labels.get(label_key)))
            labels[label_key] = pruned_label

            # label_key_list = label_key.strip('][','').split(', ')
            label_key_list = label_key.strip('][\'\"').split('\', \'')
            if label_key_list[0] == node:
                node_outgoing_labels[label_key_list[1]] = labels.get(label_key)

        new_transition_dict = dict()

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

                    '''
                    CASES:
                    1) if all outgoing edges same share transition and all go to node containing shared transition - remove transition from all (maybe need check to make sure not empty transition?)
                    2) if some outgoing edges go to node containing shared transition but others don't, remove transition from others
                    etc
                    '''

                    #CASE 1:
                    if transitions_that_go_to_intersection_node==transitions_that_share_same_value:
                        if all(len(l) > 1 for l in list(transitions_that_go_to_intersection_node.values())): #all transition labels longer than 1 symbol
                            new_transition_dict = {k:[item for item in v if item!=str(node_outgoing_label_intersection)] for (k,v) in node_outgoing_labels.items()}
                            print(f"new_transition_dict: {new_transition_dict}")

                    #TODO: other cases
                    #TODO: more work on removing 1.1 and 1.0
        
        if new_transition_dict:
            for key in new_transition_dict.keys():
                new_key = str([node, key])
                pruned_labels[new_key] = new_transition_dict.get(key)

    return pruned_labels


def main():
    grid, start, goal = load_map('maps/map_multiple_symbols_BCD.csv')
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

    #TODO: replace numerical labels with alphabetical labels from FSA


if __name__ == '__main__':
    
    # unittest.main()
    main()