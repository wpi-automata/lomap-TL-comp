#! /usr/bin/python

# Copyright (C) 2020, Cristian-Ioan Vasile (cvasile@lehigh.edu)
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

from __future__ import print_function

import networkx as nx

from lomap import Fsa, Ts, ts_times_fsa, ts_times_ts

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
from lomap.tests.a_star import *
import math

import sys

DEFAULT_PERCENT_EMPTY_OPEN = 0.9

def make_fsa(specs):
    for spec in specs:
        aut = Fsa(multi=False)
        aut.from_formula(spec)
        print(aut)
    return aut

def construct_ts():
    ts = Ts(directed=True, multi=False)
    ts.g = nx.grid_2d_graph(4, 3)

    ts.init[(1, 1)] = 1

    ts.g.add_node((0, 0), attr_dict={'prop': set(['a'])})
    ts.g.add_node((3, 2), attr_dict={'prop': set(['b'])})

    ts.g.add_edges_from(ts.g.edges(), weight=1)

    return ts

def is_word_accepted_verbose(fsa, word):
    s_current = next(iter(fsa.init))
    for symbol in word:
        s_next = fsa.next_state(s_current, symbol)
        print('state:', s_current, 'symbol:', symbol, 'next_state:', s_next)
        if s_next is None:
            print('blocked on (state, symbol):', (s_current, symbol))
            return
        s_current = s_next
    print('terminal state', s_current, 'Accept word:', s_current in fsa.final)


def create_parseable_path(props, symbols_produced):

    new_path = []

    for s in symbols_produced:
        binary = decimal_to_binary(s)
        numerical_props = numerical_binary(binary)
        alphabetical_props = []
        for numerical_prop in numerical_props:
            alphabetical_props.append(props.get(numerical_prop))
        new_path.append(set(alphabetical_props))

    return new_path


# First number always 1, need to go right to left to parse. Ex:
# decimalToBinary(8) -> 1000
# decimalToBinary(18) -> 10010  
# decimalToBinary(7) -> 111
def decimal_to_binary(num):
     
    # if num >= 1:
    #     return decimal_to_binary(num // 2)
    # # print(num % 2, end = '')
    return bin(num).replace("0b", "")



def numerical_binary(binary):
    str_binary = str(binary)[::-1]
    symbols = []
    for i in range(len(str_binary)):
        if str_binary[i]=="1":
            symbols.append(math.pow(2,i))
    return symbols


def main(percent_empty_open):
    fsa = make_fsa(['F a && F !b'])  # WARNING!!! FSA randomly assigns numbers to A and B, and since map CSV uses numerical values, ensure map representation matches props
    props = {v: k for k, v in fsa.props.items()}

    fsa.visualize(edgelabel='props', draw='matplotlib') #this only shows states not the transitions between
    plt.show()

    print('Is FSA deterministic:', fsa.is_deterministic())

    # Load the map
    grid, start, goal = create_map(percent_empty_open, props)

    # Search
    astar_path, astar_symbols_produced, aster_steps = astar(grid, start, goal)

    print(f"Symbols produced: {astar_symbols_produced}")

    # Show result
    draw_grid(grid, 'A*')
    plt.show()
    draw_path(grid, start, goal, astar_path, 'A*')
    plt.show()

    word = create_parseable_path(props, astar_symbols_produced)

    print(f"word: {word}")
    """
    word: [set(), set(), set(), set(), set(), set(), set(), set(), set(), set(), set(), {'b', 'a'}, {'b'}]
    """

    is_word_accepted_verbose(fsa, word)
    print('Word accepted (Method):', fsa.is_word_accepted(word))

    print('Language empty?:', fsa.is_language_empty())

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        percent_empty_open = sys.argv[1] if sys.argv[1] else DEFAULT_PERCENT_EMPTY_OPEN
    else:
        percent_empty_open = DEFAULT_PERCENT_EMPTY_OPEN
    main(percent_empty_open)

