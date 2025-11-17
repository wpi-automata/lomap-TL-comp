#Author: Taylor Bergeron
# UPDATED VERSION: Fixed LTL state transitions with proper symbol mapping

import os
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import numpy as np
import random

# Travel constant for A* cost calculation
TRAVEL_CONSTANT = 1.0

# Global symbol mapping: grid integers to proposition names
# This should match the encoding used when creating the grid
SYMBOL_MAP = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e'}


def state_aware_astar(grid, start, goal, weight_map, LTL_expression, state_history = {}, symbol_map=None):
    '''Return a path found by A* alogirhm w/ regards to an LTL statement
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]
    weight_map - Path to CSV file with cell weights
    LTL_expression - FSA/Buchi automaton for LTL formula
    state_history - (unused, kept for compatibility)
    symbol_map - Dict mapping grid integers to proposition names (e.g., {1: 'a', 2: 'b'})
                 If None, uses global SYMBOL_MAP

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    # Use provided symbol_map or fall back to global
    if symbol_map is None:
        symbol_map = SYMBOL_MAP
    
    queue = list()
    visited = list()
    parent_dict = {}
    path = []
    symbols_produced = []
    steps = 0

    min_dimension = 0
    max_dimension_row = len(grid) - 1
    max_dimension_col = len(grid[0]) - 1

    if start == goal:
        print("start the same as goal")
        return (path, symbols_produced, steps)

    # Get initial LTL state if expression is provided
    if LTL_expression:
        # Get the initial state from the FSA's init dictionary
        initial_ltl_state = next(iter(LTL_expression.init.keys()))
    else:
        initial_ltl_state = None
    
    start_node = (start, initial_ltl_state)
    
    queue.append(start_node)
    visited.append(start_node)
    
    # Load weights once at the start instead of on every node expansion
    weights = load_weights(weight_map)

    path, symbols_produced, steps = iterative_a_star(grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col,
                                     steps, weights, LTL_expression, state_history, symbol_map)
    if path != []:
        print(f"It takes {steps} steps to find a path using A*")
        path.reverse()
        symbols_produced.reverse()
        return (path, symbols_produced, steps)

    else:
        print("No path found")
        return path, symbols_produced, steps


def iterative_a_star(grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col, steps, weights, LTL_expression, state_history, symbol_map):
    previous_node = None

    gx_dict = dict()
    # Initialize with start node (which is now a tuple of (position, ltl_state))
    start_node = queue[0] if queue else None
    if start_node:
        gx_dict[str(start_node)] = 0

    while queue:

        # this is a dictionary of the f(x) values for everything in this queue, which is why it is reset every time a new
        # node is popped
        fx = dict()

        steps += 1

        if previous_node is not None:
            for n in queue:
                # Extract position from node tuple (position, ltl_state)
                pos = n[0]
                
                gx = gx_dict.get(str(n), 0)  # Default to 0 if not found
                dx_goal = abs(goal[1] - pos[1])
                dy_goal = abs(goal[0] - pos[0])
                hx = dx_goal + dy_goal

                # calculate f(x) and store it in a dict with the key as node for future reference
                fx[str(pos[0]) + ',' + str(pos[1]) + ',' + str(n[1])] = gx+hx

            fx_sorted = dict(sorted(fx.items(), key=lambda item: item[1]))

            queue = list()

            # rearrange the queue based on the f(x) values for each node in the queue
            for key in fx_sorted.keys():
                node_info = key.split(",")
                # Reconstruct node as (position, ltl_state)
                pos = [int(node_info[0]), int(node_info[1])]
                ltl_state = node_info[2] if len(node_info) > 2 else None
                queue.append((pos, ltl_state))

        node = queue.pop(0)
        visited.append(node)
        
        # Extract position from node tuple
        pos = node[0]

        if pos == goal:
            path = list()
            symbols_produced = list()
            path.append(pos)
            parent = parent_dict.get(str(node))
            symbols_produced.append(grid[pos[0]][pos[1]])
            
            # Check if parent exists before accessing
            if parent is None:
                # Goal reached from start directly or parent not found
                return path, symbols_produced, steps
            
            path.append(parent[0])  # Extract position from parent node
            symbols_produced.append(grid[parent[0][0]][parent[0][1]])
            node = parent

            # back track using the dictionary of parents as links between nodes to find the path
            while node[0] != start:
                parent = parent_dict.get(str(node))
                if parent is None:
                    # Reached start or missing parent
                    break
                path.append(parent[0])  # Extract position from parent node
                symbols_produced.append(grid[parent[0][0]][parent[0][1]])
                node = parent

            return path, symbols_produced, steps

        else:
            previous_node = node

            # updated queue, parent dictionary that contains the parents of each node (who added it or a neighbor that
            # gives it a lesser g(x) value, and the dictionary of each nodes g(x) value
            queue, parent_dict, gx_dict = four_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, parent_dict, gx_dict, goal, weights, LTL_expression, symbol_map)

    return ([], [], steps)


# NODE EXPANSION HELPER FUNCTIONS
# by doing != -1 I can produce a symbol path while still checking for occupancy.
# Assumption: location with symbol is never occupied
def four_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, parent_dict, gx_dict, goal, weights, LTL_expression, symbol_map):

    # Extract position and LTL state from node tuple
    pos = node[0]
    current_ltl_state = node[1]
    
    row = pos[0]
    col = pos[1]
    
    # Get parent's g(x) cost
    parent_gx = gx_dict.get(str(node), 0)
    
    # Define four-connected neighbors: [row_offset, col_offset]
    neighbors = [
        [0, 1],   # right
        [1, 0],   # down
        [0, -1],  # left
        [-1, 0]   # up
    ]
    
    # Process each neighbor
    for offset in neighbors:
        new_row = row + offset[0]
        new_col = col + offset[1]
        
        # Check bounds
        if new_row < min_dimension or new_row > max_dimension_row:
            continue
        if new_col < min_dimension or new_col > max_dimension_col:
            continue
        
        # Check if cell is traversable (not obstacle)
        if grid[new_row][new_col] == -1:
            continue
        
        # Get the symbol in the new cell
        cell_value = grid[new_row][new_col]
        
        # Build the set of propositions for this cell
        # UPDATED: Map grid integers to proposition names
        cell_props = set()
        if cell_value not in [-1, 0] and cell_value != {}:
            # Map integer to proposition name (e.g., 1 -> 'a', 2 -> 'b')
            if cell_value in symbol_map:
                cell_props.add(symbol_map[cell_value])
            else:
                # If no mapping, use the integer directly (backward compatibility)
                cell_props.add(cell_value)
        
        # Determine the next LTL state
        next_ltl_state = current_ltl_state
        # See if moving to cell would change the state of the LTL expression
        if LTL_expression is not None:
            next_ltl_state = LTL_expression.next_state(current_ltl_state, cell_props)
            if next_ltl_state is None:
                # Moving to this cell would violate the LTL expression
                continue
        
        # Create node with position and LTL state
        new_pos = [new_row, new_col]
        node_to_add = (new_pos, next_ltl_state)
        
        # Check if not already visited
        if node_to_add in visited:
            continue
        
        # Get cell weight from weights dictionary (default to 0 if not found)
        cell_weight = weights.get((new_row, new_col), 0)
        
        # Calculate gx using formula: gx = parent_gx + cell_weight + travel_constant
        gx = parent_gx + cell_weight + TRAVEL_CONSTANT
        
        # Add to queue if not present, or update if this path is better
        if node_to_add not in queue:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif gx < gx_dict[str(node_to_add)]:
            # Found a better path to this node
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx
    
    return queue, parent_dict, gx_dict

# Load map, start and goal point.
def load_map(file_path):
    print(os.getcwd())
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                parsed_row = [int(col) for col in row]
                grid.append(parsed_row)
    return grid, start, goal

#TODO: allow for user to choose dimensions of map
# Create map with randomized start and goal point, using input of percent empty cells produce no symbols
def create_map(percent_empty_open, props):

    numerical_props = list(props.keys())

    dim = 12 #width and height of map

    start = [random.randint(1, 10), random.randint(1, 10)]
    goal = [random.randint(1, 10), random.randint(1, 10)]

    # # Creating a dimxdim matrix filled with -1s
    # grid = np.full((dim, dim), -1)

    # # Setting the inner values of the matrix 'x' (excluding the borders) to 0 using slicing
    # grid[1:-1, 1:-1] = 0

    number_symbols = int((1-percent_empty_open)*(dim*dim))

    inner_map_length = (dim-2)**2
    inner_map = np.zeros((inner_map_length)) #map inside -1 boundaries

    for i in range(number_symbols):
        inner_map[random.randint(0, inner_map_length-1)] = random.choice(numerical_props)

    inner_map = inner_map.reshape((dim-2, dim-2))

    grid = np.pad(inner_map, pad_width=1, mode='constant',constant_values=-1)

    print(grid)

    return grid.astype(int), start, goal

# Draw final results
def draw_grid(grid, title):
    create_grid(grid)
    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()

# Draw final results
def draw_path(grid, start, goal, path, title):
    fig, ax = create_grid(grid)
    # Visualization of the found path using matplotlib
    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='grey'))          # path
    ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings
    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()

def create_grid(grid):
    fig, ax = plt.subplots(1)
    ax.margins()
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j]==-1: 
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            elif grid[i][j]==1:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='m'))  # A
            elif grid[i][j]==2:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='c'))  # B
            elif grid[i][j]==3:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='blueviolet'))  # A&B
            elif grid[i][j]==4:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='y'))  # A&B
            else:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
    return fig, ax

def load_weights(file_path):
    """Load weights from CSV file. Supports two formats:
    1. Grid format: Matrix of weights (skips first 2 rows for start/goal)
    2. Coordinate format: x,y,weight per row
    """
    weights = dict()
    with open(file_path, 'r') as weights_file:
        reader = csv.reader(weights_file)
        rows = list(reader)
        
        # Check if this is grid format (first row contains 'start') or coordinate format
        if len(rows) > 0 and rows[0][0] == 'start':
            # Grid format: skip first 2 rows, then build weight dict from grid
            for i, row in enumerate(rows[2:]):  # Skip start and goal rows
                for j, val in enumerate(row):
                    try:
                        weight = float(val)
                        if weight != -1:  # Don't store obstacles
                            weights[(i, j)] = weight
                    except ValueError:
                        pass  # Skip non-numeric values
        else:
            # Coordinate format: x,y,weight
            for row in rows:
                try:
                    x = int(row[0])
                    y = int(row[1])
                    weight = float(row[2])
                    weights[(x, y)] = weight
                except (ValueError, IndexError):
                    pass  # Skip malformed rows
    
    return weights

if __name__ == "__main__":
    # Load the map
    symbol_grid, start, goal = load_map('maps/unit_test_maps/alphabetical_maps/example8_map.csv')
    weight_file = 'maps/test_weights.csv'
    # Search
    astar_path, astar_symbols_produced, aster_steps = state_aware_astar(symbol_grid, start, goal, weight_file, LTL_expression=None)

    print(f"Symbols produced: {astar_symbols_produced}")

    # Show result
    draw_path(symbol_grid, start, goal, astar_path, 'A*')
    plt.show()
