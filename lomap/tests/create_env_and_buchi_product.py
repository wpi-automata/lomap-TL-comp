from lomap.tests.create_transition_system_from_map import *
from lomap.algorithms.product import ts_times_buchi
import networkx as nx
from ast import literal_eval

def create_product(map_path, start_state, spec, prune=False, display=True):

    #TODO: need to be explicit about where start. If 2 a's, there needs to be a way at this step to differentiate. For now, will just pick symbol.0.

    ts, ts_props, clusters = create_ts(map_path, start_state, prune, display)

    buchi = Buchi()
    buchi.from_formula(spec)
    buchi_edge_labels = {(u, v): d['attr_dict']['label'] for u, v, d in buchi.g.edges(data=True) if ('attr_dict' in d and 'label' in d['attr_dict'])}
    print('Created Buchi automaton of size', buchi.size())
    if display:
        buchi.visualize(edgelabel=buchi_edge_labels, draw='matplotlib')
        plt.show()

    pa = ts_times_buchi(ts, buchi, ts_props, multi=True)  # Enable multiple edges between nodes
    print('Created product automaton of size', pa.size())
    print('Product automaton edges:', pa.g.edges(data=True))
    print('Product automaton nodes:', pa.g.nodes(data=True))

    #     # Explore node labels and their corresponding strings
    # print("\n=== NODE LABELS ===")
    # for node, node_data in pa.g.nodes(data=True):
    #     if 'attr_dict' in node_data:
    #         attr = node_data['attr_dict']
    #         prop_set = attr.get('prop', set())  # This is the set of proposition strings (e.g., {'a'}, {'b'}, set())
    #         abbrev_label = attr.get('abbrev_label', '')  # String representation
    #         full_label = attr.get('label', '')  # More detailed string
    #         print(f"Node: {node}")
    #         print(f"  prop (proposition strings): {prop_set}")
    #         print(f"  abbrev_label: {abbrev_label}") # This is what we need to embed 
    #         print(f"  label: {full_label}")
    #         print()

    # # Explore edge labels and their corresponding strings
    # print("\n=== EDGE LABELS ===")
    # for u, v, edge_data in pa.g.edges(data=True):
    #     if 'attr_dict' in edge_data:
    #         attr = edge_data['attr_dict']
    #         weight = attr.get('weight', None)
    #         control = attr.get('control', None)
    #         pi = attr.get('pi', None)  # Proposition set on edge (if present)
    #         print(f"Edge: {u} -> {v}")
    #         print(f"  weight: {weight}")
    #         print(f"  control: {control}")
    #         if pi is not None:
    #             print(f"  pi (proposition strings): {pi}") 
    #         print()


    if display:
        pa.visualize(draw='matplotlib')
        plt.show()

    print('Product initial states:', pa.init) # initial states
    print('Product accepting states:', pa.final) # final states 

    labels = {n: d['attr_dict']['abbrev_label'] for n, d in pa.g.nodes.items() if ('attr_dict' in d and 'abbrev_label' in d['attr_dict'])}
    '''
    Because we are taking the product between an LTL spec buchi and a TS, we have only one input state, but many possible final states.
    To find the shortest possible trajectory, we must compare all possible input state to final state pairs and the path length they produce.
    '''
    shortest_trajectory = None
    for f in iter(pa.final):
        trajectory = nx.shortest_path(pa.g, source=next(iter(pa.init)), target=f)
        if (not shortest_trajectory or len(trajectory) < len(shortest_trajectory)) and \
        (literal_eval(labels.get(f))[0] != '{}'): # has !{} because can't run policy for {} so can't get to final node if {}
            shortest_trajectory = trajectory

    print('Shortest Trajectory: ', shortest_trajectory)

    if not shortest_trajectory:
        print(f"No trajectory found, returning None")
        return None
    
    #Add policy labels from transition system to product automaton - THIS CAUSES ERRORS BECAUSE MIGHT NOT ADHERE TO SPEC - UNSAFE!
    #Uncomment below 4 lines and remove labels from word_from_trajectory to run old shortest word code
    # ts_edge_policies = {n: d['pi'] for n, d in ts.g.edges.items() if 'pi' in d}
    # for edge in pa.g.edges():
    #     if (edge[0][0], edge[1][0]) in ts_edge_policies.keys():
    #         pa.g[edge[0]][edge[1]]['pi'] = ts_edge_policies[(edge[0][0], edge[1][0])]
    shortest_word = pa.word_from_trajectory(shortest_trajectory, labels=labels)

    print('Accepted word:', shortest_word)

    return shortest_word, pa, clusters
    
if __name__ == '__main__':

    # spec = '(F (f & X (F d))) & (G ! g) '
    # spec = '(F h) & (! h U f)'
    # shortest_word = create_product('maps/alphabetical_maps/office_world.csv', '{}', spec)

    spec = '(F b)'
    shortest_word = create_product('maps/unit_test_maps/alphabetical_maps/example9.csv', '{}', spec)