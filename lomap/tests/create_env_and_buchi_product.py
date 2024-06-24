from create_transition_system_from_map import *

def main():

    #TODO: need to be explicit about where start. If 2 a's, there needs to be a way at this step to differentiate. For now, will just pick symbol.0.

    ts, ts_props = create_ts('maps/unit_test_maps/alphabetical_maps/example1.csv', 'b')

    spec = 'F a'
    buchi = Buchi()
    buchi.from_formula(spec)
    print('Created Buchi automaton of size', buchi.size())
    buchi.visualize(draw='matplotlib')
    plt.show()

    pa = ts_times_buchi(ts, buchi, ts_props)
    print('Created product automaton of size', pa.size())

    e = pa.g.edges()
    ts_edge_policies = {n: d['pi'] for n, d in ts.g.edges.items() if 'pi' in d}
    pa_edges = dict()
    for edge in e:
        if (edge[0][0], edge[1][0]) in ts_edge_policies.keys():
            pa_edges[edge] = ts_edge_policies[(edge[0][0], edge[1][0])]

    pa.visualize(draw='matplotlib', edgelabel=pa_edges)
    plt.show()


if __name__ == '__main__':
    
    main()