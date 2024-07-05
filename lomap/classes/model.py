# Copyright (C) 2012-2015, Alphan Ulusoy (alphan@bu.edu)
#               2015-2017, Cristian-Ioan Vasile (cvasile@mit.edu)
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

import networkx as nx

# TODO: always use safe load
from yaml import load, dump #, safe_load as load
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper


def graph_constructor(directed, multi):
    '''Returns the class to construct the appropriate graph type.'''
    if directed:
        if multi:
            constructor = nx.MultiDiGraph
        else:
            constructor = nx.DiGraph
    else:
        if multi:
            constructor = nx.MultiGraph
        else:
            constructor = nx.Graph
    return constructor


class Model(object):
    """
    Base class for various system models.
    """

    yaml_tag = u'!Model'

    def __init__(self, name='Unnamed model', directed=True, multi=True):
        """
        Empty LOMAP Model object constructor.
        """
        self.name = name
        self.init = dict()
        self.current = None
        self.final = set()
        graph_type = graph_constructor(directed, multi)
        self.g = graph_type()
        self.directed = directed
        self.multi = multi

    def __eq__(self, other):
        '''Equality testing, which includes data stored on nodes and edges.
        The name and current state are not checked for equality.
        '''
        return (isinstance(other, Model)
            and self.directed == other.directed and self.multi == other.multi
            and self.init == other.init and self.final == other.final
            #FIXME: Incompatible with nx2.0
            and self.g.node == other.g.node and self.g.edge == other.g.edge)

    def __ne__(self, other):
        '''Equality testing. See `Model.__eq__()`.'''
        return not self.__eq__(other)

    def nodes_w_prop(self, propset):
        """
        Returns the set of nodes with given properties.
        """
        nodes_w_prop = set()
        for node, data in self.g.nodes(data=True):
            if propset <= data.get('prop',set()):
                nodes_w_prop.add(node)
        return nodes_w_prop

    def size(self):
        return (self.g.number_of_nodes(), self.g.number_of_edges())

    def visualize(self, edgelabel=None, draw='pygraphviz'):
        """
        Visualizes a LOMAP system model
        """
        if draw == 'pygraphviz':
            A = nx.nx_agraph.to_agraph(self.g)
            nx.draw(A)
        elif draw == 'matplotlib':
            pos = nx.spring_layout(self.g)

            nx.draw_networkx_nodes(self.g, pos)
            nx.draw_networkx_edges(self.g, pos, edgelist=self.g.edges())
        
            labels = {n: d['attr_dict']['abbrev_label'] for n, d in self.g.nodes.items() if ('attr_dict' in d and 'abbrev_label' in d['attr_dict'])}

            if labels:
                nx.draw_networkx_labels(self.g, pos=pos, labels=labels)
            else:            
                nx.draw_networkx_labels(self.g, pos=pos)
            
            if edgelabel:
                nx.draw_networkx_edge_labels(self.g, pos, edge_labels=edgelabel,rotate=False)

        else:
            raise ValueError('Expected parameter draw to be either:'
                             + '"pygraphviz" or "matplotlib"!')

    @classmethod
    def load(cls, filename):
        '''Load model from file in YAML format.'''
        with open(filename, 'r') as fin:
            return load(fin, Loader=Loader)

    def save(self, filename):
        '''Save the model to file in YAML format.'''
        with open(filename, 'w') as fout:
            dump(self, fout, Dumper=Dumper)

    def word_from_trajectory(self, trajectory):
        possible_words = []
        word = []

        for state, next_state in zip(trajectory, trajectory[1:]):
            symbols = self.g[state][next_state]['pi']
            possible_words.append(symbols) # list of lists of possible policies to take on each transition edge

        #simplify the trajectory by seeing if there is an intersection (same policy) on subsequent edges
        for possible_word, next_possible_word in zip(possible_words, possible_words[1:]):
            intersection = list(set(possible_word) & set(next_possible_word))
            if intersection:
                word.append(intersection[0])
            else:
                word.append(possible_word[0])
        
        #need to add final symbol
        word.append(possible_words[-1][0])
        #remove duplicates
        word = list(dict.fromkeys(word))
        return word