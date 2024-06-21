from create_transition_system_from_map import *

def main():

    ts = create_ts()

    spec = 'F a'
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