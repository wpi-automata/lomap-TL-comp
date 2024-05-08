from a_star import *
import numpy as np
import math
import unittest
import networkx as nx

def create_graph(clusters):
    edges = list()
    keys = list(clusters.keys())

    for i in range(len(keys)-1):
        key = keys[i]
        cluster = clusters.get(key)
        for j in range(i+1, len(keys)):
            comparison_key = keys[j]
            comparison_cluster = clusters.get(comparison_key)
            
            if cluster_connected(cluster, comparison_cluster):
                edges.append([key, comparison_key])
    
    return edges
                    

def cluster_connected(cluster, comparison_cluster):
    for point in cluster:
        for comparison_point in comparison_cluster:
            if connected(point, comparison_point):
                return True
    return False

def each_cluster_uuid(clusters):
    unique_clusters = dict()

    for key in clusters.keys():
        key_clusters = clusters.get(key)
        if len(key_clusters)>1:
            for i in range(len(key_clusters)):
                new_key = str(str(key)+"."+str(i))
                unique_clusters[new_key] = key_clusters[i]
        else:
            unique_clusters[key] = key_clusters[0]

    return unique_clusters

def create_clusters(grid):
    clusters = dict()
    mappings = dict() #used for when duplicates of labels. E.G 2 seperate regions of Bs, we will have mapping B: (B1, B2)
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            val = grid[r,c]
            cluster(r, c, val, clusters, mappings)
    return clusters
            

def cluster(r, c, val, clusters, mappings):
    first_in_list = [[r,c]]
    if str(val) in clusters.keys():
        val_clusters = clusters.get(str(val))
        for i in range(len(val_clusters)):
            cluster = val_clusters[i]
            if is_neighbor(r,c,cluster):
                cluster.append([r,c])
                val_clusters[i]=cluster
                return
        val_clusters.append(first_in_list)
        return
    clusters[str(val)]= [first_in_list]

#uses 4 connected neighbor check
def is_neighbor(r,c,cluster):
    for node in cluster:
        if connected(node, [r,c]):
            return True
    return False
        
def connected(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2) == 1. #if they are next to each other they will be 1 unit away

def draw_graph(nx_graph):
    fig, axes = plt.subplots(1,1,dpi=72)
    nx.draw(nx_graph, pos=nx.spring_layout(nx_graph), ax=axes, with_labels=True)
    plt.show()

def main():
    grid, start, goal = load_map('map_multiple_symbols.csv')
    clusters = create_clusters(np.asarray(grid))
    print(f"Clusters: {clusters}")
    unique_clusters = each_cluster_uuid(clusters)
    print(f"Unique Clusters: {unique_clusters}")
    edges = create_graph(unique_clusters)
    print(f"Edges: {edges}")

    G = nx.Graph()
    G.add_edges_from(edges)

    draw_graph(G)


class TestStringMethods(unittest.TestCase):

    def test_edges_symbol_not_touching_empty(self):
        self.assertEqual(create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('unit_test_maps/map_2_encased.csv')[0])))), [['-1', '0'], ['0', '3'], ['3', '2']])

    def test_clusters_multiple_groupings_same_symbol(self):
        self.assertEqual(len(create_clusters(np.asarray(load_map('unit_test_maps/map_multiple_2_groups.csv')[0])).get('2')), 2)

    def test_unique_clusters_multiple_groupings_same_symbol(self):
        self.assertEqual(len(list(each_cluster_uuid(create_clusters(np.asarray(load_map('unit_test_maps/map_multiple_2_groups.csv')[0]))).keys())), 5)

    def test_graph_symbol_not_touching_empty(self):
        edges = create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('unit_test_maps/map_2_encased.csv')[0]))))
        G = nx.Graph()
        G.add_edges_from(edges)
        self.assertEqual(G.number_of_nodes(), 4)
        self.assertEqual(G.number_of_edges(), 3)

    def test_graph_multiple_groupings_same_symbol(self):
        edges = create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('unit_test_maps/map_multiple_2_groups.csv')[0]))))
        G = nx.Graph()
        G.add_edges_from(edges)
        self.assertEqual(G.number_of_nodes(), 5)
        self.assertEqual(G.number_of_edges(), 4)


if __name__ == '__main__':
    
    unittest.main()
    # main()