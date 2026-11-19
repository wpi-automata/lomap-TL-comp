from lomap.tests.create_transition_system_from_map import *
import itertools


# def create_combined_symbols_overlapping_objects(collision_pairings):
#     for pairing in collision_pairings:
#         symbol_collision_pairings = ""
#         for object in pairing:
#             symbol_collision_pairings += get_object_symbols(object)


def get_object_symbols(object, sprites_to_symbols, delimiter):
    object_symbols = ""
    object_sprite = object.split(delimiter)
    for sprite in object_sprite:
        object_symbols += sprites_to_symbols.get(sprite)
    return object_symbols


def create_edges(objects_as_binary):
    edges = []
    # creating edges between all objects in the environment
    # vals = list(objects_as_binary.values())
    # for obj1, obj2 in itertools.combinations(vals, 2):
    #     edges.append([obj1, obj2])
    # return edges

    # creating edges between empty and all objects in the environment
    for obj in list(objects_as_binary.values()):
        edges.append([str(int(EMPTY_SYMBOL_NUMERIC)), str(obj)])
    return edges



def get_objects_as_symbols(obs_locations, sprites_to_symbols, delimiter):
    unconnected_objects_as_symbols = []
    for object in obs_locations.keys():
        unconnected_objects_as_symbols.append(get_object_symbols(object, sprites_to_symbols, delimiter))  # blue_box -> be

    unconnected_objects_as_symbols.append(EMPTY_SYMBOL)
    return unconnected_objects_as_symbols


def extend_props(objects_as_symbols, props):
    objects_as_binary_combo_props = {}
    for symbols in objects_as_symbols:
        if len(symbols) > 1:
            objects_as_binary_combo_props[symbols] = (combine_concat_symbols_into_new_props(symbols, props))
    return objects_as_binary_combo_props

'''
TODO: the following env->prop logic was created for a continuous environment with no connected object, 
all objects are connected to empty space, and each object has a shape and color.
The remainder of the logic makes no such assumptions.
'''
def create_ts_from_continuous_env(obs_locations, sprites_to_symbols, delimiter, init_node):
    unconnected_objects_as_symbols = get_objects_as_symbols(obs_locations, sprites_to_symbols, delimiter)
    props = assign_props(unconnected_objects_as_symbols)
    objects_as_binary = extend_props(unconnected_objects_as_symbols, props)
    # if objects are not connected to empty space, remove
    # objects_as_binary[EMPTY_SYMBOL] = int(EMPTY_SYMBOL_NUMERIC)

    edges = create_edges(objects_as_binary)
    unique_clusters_labels = [str(o) for o in list(objects_as_binary.values())]
    return create_ts_from_edges(edges, unique_clusters_labels, props, init_node)
