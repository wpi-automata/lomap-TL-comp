from create_transition_system_from_map import *

def main():

    #TODO: need to be explicit about where start. If 2 a's, there needs to be a way at this step to differentiate. For now, will just pick symbol.0.

    ts, ts_props = create_ts('maps/unit_test_maps/alphabetical_maps/example1.csv', 'a')

    spec = 'F a'
    buchi = Buchi()
    buchi.from_formula(spec)
    print('Created Buchi automaton of size', buchi.size())
    buchi.visualize(draw='matplotlib')
    plt.show()

    pa = ts_times_buchi(ts, buchi, ts_props)
    print('Created product automaton of size', pa.size())
    pa.visualize(draw='matplotlib') #TODO: edgelabel prarameter can be used to add policies on edges
    plt.show()


if __name__ == '__main__':
    
    main()