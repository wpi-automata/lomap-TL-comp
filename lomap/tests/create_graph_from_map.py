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
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            val = grid[r,c]
            cluster(r, c, val, clusters)
    return clusters

def cluster(r, c, val, clusters):
    first_in_list = [[r,c]]
    if str(val) in clusters.keys():
        val_clusters = clusters.get(str(val))
        neighbor_clusters = []
        for i in range(len(val_clusters)):
            cluster = val_clusters[i]
            if is_neighbor(r,c,cluster):
                neighbor_clusters.append(cluster)
        if neighbor_clusters:
            for cluster in neighbor_clusters:
                val_clusters.remove(cluster)
            merged_neighbor_clusters = [element for nestedlist in neighbor_clusters for element in nestedlist]
            merged_neighbor_clusters.append([r,c])
            val_clusters.append(merged_neighbor_clusters)
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

def draw_graph(nx_graph, labels=None):
    
    pos = nx.nx_agraph.graphviz_layout(nx_graph, prog="neato")
    fig, ax = plt.subplots()
    nx.draw_networkx_nodes(nx_graph, pos, ax=ax)
    nx.draw_networkx_labels(nx_graph, pos, labels=labels, ax=ax)

    curved_edges = [edge for edge in nx_graph.edges() if reversed(edge) in nx_graph.edges()]
    straight_edges = list(set(nx_graph.edges()) - set(curved_edges))
    nx.draw_networkx_edges(nx_graph, pos, ax=ax, edgelist=straight_edges)
    arc_rad = 0.25
    nx.draw_networkx_edges(nx_graph, pos, ax=ax, edgelist=curved_edges, connectionstyle=f'arc3, rad = {arc_rad}')

    edge_labels = nx.get_edge_attributes(nx_graph,'pi')
    curved_edge_labels = {edge: edge_labels[edge] for edge in curved_edges}
    straight_edge_labels = {edge: edge_labels[edge] for edge in straight_edges}
    my_draw_networkx_edge_labels(nx_graph, pos, ax=ax, edge_labels=curved_edge_labels,rotate=False,rad = arc_rad)
    nx.draw_networkx_edge_labels(nx_graph, pos, ax=ax, edge_labels=straight_edge_labels,rotate=False)
    
    plt.show()


def my_draw_networkx_edge_labels(
    G,
    pos,
    edge_labels=None,
    label_pos=0.5,
    font_size=10,
    font_color="k",
    font_family="sans-serif",
    font_weight="normal",
    alpha=None,
    bbox=None,
    horizontalalignment="center",
    verticalalignment="center",
    ax=None,
    rotate=True,
    clip_on=True,
    rad=0
):
    """Draw edge labels.
    Source: https://stackoverflow.com/questions/22785849/drawing-multiple-edges-between-two-nodes-with-networkx

    Parameters
    ----------
    G : graph
        A networkx graph

    pos : dictionary
        A dictionary with nodes as keys and positions as values.
        Positions should be sequences of length 2.

    edge_labels : dictionary (default={})
        Edge labels in a dictionary of labels keyed by edge two-tuple.
        Only labels for the keys in the dictionary are drawn.

    label_pos : float (default=0.5)
        Position of edge label along edge (0=head, 0.5=center, 1=tail)

    font_size : int (default=10)
        Font size for text labels

    font_color : string (default='k' black)
        Font color string

    font_weight : string (default='normal')
        Font weight

    font_family : string (default='sans-serif')
        Font family

    alpha : float or None (default=None)
        The text transparency

    bbox : Matplotlib bbox, optional
        Specify text box properties (e.g. shape, color etc.) for edge labels.
        Default is {boxstyle='round', ec=(1.0, 1.0, 1.0), fc=(1.0, 1.0, 1.0)}.

    horizontalalignment : string (default='center')
        Horizontal alignment {'center', 'right', 'left'}

    verticalalignment : string (default='center')
        Vertical alignment {'center', 'top', 'bottom', 'baseline', 'center_baseline'}

    ax : Matplotlib Axes object, optional
        Draw the graph in the specified Matplotlib axes.

    rotate : bool (deafult=True)
        Rotate edge labels to lie parallel to edges

    clip_on : bool (default=True)
        Turn on clipping of edge labels at axis boundaries

    Returns
    -------
    dict
        `dict` of labels keyed by edge

    Examples
    --------
    >>> G = nx.dodecahedral_graph()
    >>> edge_labels = nx.draw_networkx_edge_labels(G, pos=nx.spring_layout(G))

    Also see the NetworkX drawing examples at
    https://networkx.org/documentation/latest/auto_examples/index.html

    See Also
    --------
    draw
    draw_networkx
    draw_networkx_nodes
    draw_networkx_edges
    draw_networkx_labels
    """
    import matplotlib.pyplot as plt
    import numpy as np

    if ax is None:
        ax = plt.gca()
    if edge_labels is None:
        labels = {(u, v): d for u, v, d in G.edges(data=True)}
    else:
        labels = edge_labels
    text_items = {}
    for (n1, n2), label in labels.items():
        (x1, y1) = pos[n1]
        (x2, y2) = pos[n2]
        (x, y) = (
            x1 * label_pos + x2 * (1.0 - label_pos),
            y1 * label_pos + y2 * (1.0 - label_pos),
        )
        pos_1 = ax.transData.transform(np.array(pos[n1]))
        pos_2 = ax.transData.transform(np.array(pos[n2]))
        linear_mid = 0.5*pos_1 + 0.5*pos_2
        d_pos = pos_2 - pos_1
        rotation_matrix = np.array([(0,1), (-1,0)])
        ctrl_1 = linear_mid + rad*rotation_matrix@d_pos
        ctrl_mid_1 = 0.5*pos_1 + 0.5*ctrl_1
        ctrl_mid_2 = 0.5*pos_2 + 0.5*ctrl_1
        bezier_mid = 0.5*ctrl_mid_1 + 0.5*ctrl_mid_2
        (x, y) = ax.transData.inverted().transform(bezier_mid)

        if rotate:
            # in degrees
            angle = np.arctan2(y2 - y1, x2 - x1) / (2.0 * np.pi) * 360
            # make label orientation "right-side-up"
            if angle > 90:
                angle -= 180
            if angle < -90:
                angle += 180
            # transform data coordinate angle to screen coordinate angle
            xy = np.array((x, y))
            trans_angle = ax.transData.transform_angles(
                np.array((angle,)), xy.reshape((1, 2))
            )[0]
        else:
            trans_angle = 0.0
        # use default box of white with white border
        if bbox is None:
            bbox = dict(boxstyle="round", ec=(1.0, 1.0, 1.0), fc=(1.0, 1.0, 1.0))
        if not isinstance(label, str):
            label = str(label)  # this makes "1" and 1 labeled the same

        t = ax.text(
            x,
            y,
            label,
            size=font_size,
            color=font_color,
            family=font_family,
            weight=font_weight,
            alpha=alpha,
            horizontalalignment=horizontalalignment,
            verticalalignment=verticalalignment,
            rotation=trans_angle,
            transform=ax.transData,
            bbox=bbox,
            zorder=1,
            clip_on=clip_on,
        )
        text_items[(n1, n2)] = t

    ax.tick_params(
        axis="both",
        which="both",
        bottom=False,
        left=False,
        labelbottom=False,
        labelleft=False,
    )

    return text_items


def main():
    grid, start, goal = load_map('maps/numerical_maps/map_multiple_symbols_BCD.csv')
    draw_path(grid, start, goal, [], 'Map')
    clusters = create_clusters(np.asarray(grid))
    print(f"Clusters: {clusters}")
    unique_clusters = each_cluster_uuid(clusters)
    print(f"Unique Clusters: {unique_clusters}")
    edges = create_graph(unique_clusters)
    print(f"Edges: {edges}")
    reversed_edges = [sublist[::-1] for sublist in edges[::-1]]
    print(f"Reversed Edges: {reversed_edges}")
    edges.extend(reversed_edges)
    
    for edge in edges:
        edge.append({'pi':str(edge[0])+"->"+str(edge[1])})

    G = nx.DiGraph()
    G.add_edges_from(edges)
    draw_graph(G)


class TestStringMethods(unittest.TestCase):

    def test_edges_symbol_not_touching_empty(self):
        self.assertEqual(create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('maps/unit_test_maps/numerical_maps/map_2_encased.csv')[0])))), [['-1', '0'], ['0', '3'], ['3', '2']])

    def test_clusters_multiple_groupings_same_symbol(self):
        self.assertEqual(len(create_clusters(np.asarray(load_map('maps/unit_test_maps/numerical_maps/map_multiple_2_groups.csv')[0])).get('2')), 2)

    def test_unique_clusters_multiple_groupings_same_symbol(self):
        self.assertEqual(len(list(each_cluster_uuid(create_clusters(np.asarray(load_map('maps/unit_test_maps/numerical_maps/map_multiple_2_groups.csv')[0]))).keys())), 5)

    def test_graph_symbol_not_touching_empty(self):
        edges = create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('maps/unit_test_maps/numerical_maps/map_2_encased.csv')[0]))))
        G = nx.Graph()
        G.add_edges_from(edges)
        self.assertEqual(G.number_of_nodes(), 4)
        self.assertEqual(G.number_of_edges(), 3)

    def test_graph_multiple_groupings_same_symbol(self):
        edges = create_graph(each_cluster_uuid(create_clusters(np.asarray(load_map('maps/unit_test_maps/numerical_maps/map_multiple_2_groups.csv')[0]))))
        G = nx.Graph()
        G.add_edges_from(edges)
        self.assertEqual(G.number_of_nodes(), 5)
        self.assertEqual(G.number_of_edges(), 4)


if __name__ == '__main__':
    
    # unittest.main()
    main()