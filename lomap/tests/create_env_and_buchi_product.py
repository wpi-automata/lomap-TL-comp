from lomap.tests.create_transition_system_from_map import *
from lomap.algorithms.product import ts_times_buchi
import networkx as nx

def main():

    #TODO: need to be explicit about where start. If 2 a's, there needs to be a way at this step to differentiate. For now, will just pick symbol.0.

    # ts, ts_props = create_ts('maps/alphabetical_maps/office_world.csv', '{}')
    ts, ts_props = create_ts('maps/unit_test_maps/alphabetical_maps/example8.csv', '{}')

    # spec = '(F (f & X (F d))) & (G ! g) '
    # spec = '(F h) & (! h U f)'
    spec = 'F e'
    buchi = Buchi()
    buchi.from_formula(spec)
    print('Created Buchi automaton of size', buchi.size())
    buchi.visualize(draw='matplotlib')
    plt.show()

    pa = ts_times_buchi(ts, buchi, ts_props, multi=False)
    print('Created product automaton of size', pa.size())

    #Add policy labels from transition system to product automaton
    labels = {n: d['attr_dict']['abbrev_label'] for n, d in pa.g.nodes.items() if ('attr_dict' in d and 'abbrev_label' in d['attr_dict'])}
    ts_edge_policies = {n: d['pi'] for n, d in ts.g.edges.items() if 'pi' in d}

    for edge in pa.g.edges():
        if (edge[0][0], edge[1][0]) in ts_edge_policies.keys():
            pa.g[edge[0]][edge[1]]['pi'] = ts_edge_policies[(edge[0][0], edge[1][0])]

    print('Product initial states:', pa.init) # initial states
    print('Product accepting states:', pa.final) # final states 
    '''
    Because we are taking the product between an LTL spec buchi and a TS, we have only one input state, but many possible final states.
    To find the shortest possible trajectory, we must compare all possible input state to final state pairs and the path length they produce.
    '''
    # shortest_trajectory = None
    # for f in iter(pa.final):
    #     trajectory = nx.shortest_path(pa.g, source=next(iter(pa.init)), target=f)
    #     if not shortest_trajectory or len(trajectory) < len(shortest_trajectory):
    #         shortest_trajectory = trajectory

    # print('Shortest Trajectory: ', shortest_trajectory)

    # print('Accepted word:', pa.word_from_trajectory(shortest_trajectory))

    draw_graph(pa.g, labels=labels)
    plt.show()
    
if __name__ == '__main__':
    
    main()