#Author: Taylor Bergeron
# UPDATED VERSION: Fixed LTL state transitions with proper symbol mapping

import os
from lomap.classes.automata_updated import Fsa
import csv
import numpy as np
import random

# Travel constant for A* cost calculation
TRAVEL_CONSTANT = 1.0
OBS_THRESHOLD = 80
def state_aware_astar(start, goal, symbol_grid, occupancy_grid, weights, LTL_expression, use_8_neighbors=True):
    '''Return a path found by A* alogirhm w/ regards to an LTL statement
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]
    weight_map - Path to CSV file with cell weights
    LTL_expression - FSA/Buchi automaton for LTL formula
    use_8_neighbors - If True (default), use 8-connected grid (includes diagonals).
                      If False, use 4-connected grid (only up/down/left/right).

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
    # Merge occupancy grid obstacles into symbol grid
    # If occupancy grid cell > OBS_THRESHOLD, mark as obstacle (-1) in symbol grid
    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] > OBS_THRESHOLD:
                symbol_grid[i][j] = -1
    
    queue = list()
    visited = list()
    parent_dict = {}
    path = []
    symbols_produced = []
    steps = 0

    min_dimension = 0
    max_dimension_row = len(symbol_grid) - 1
    max_dimension_col = len(symbol_grid[0]) - 1

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
    

    path, symbols_produced, steps, final_state = iterative_a_star(symbol_grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col,
                                     steps, weights, LTL_expression, use_8_neighbors)
    if path != []:
        print(f"It takes {steps} steps to find a path using A*")
        path.reverse()
        symbols_produced.reverse()
        return (path, symbols_produced, steps, final_state)

    else:
        print("No path found")
        return path, symbols_produced, steps, final_state


def iterative_a_star(symbol_grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col, steps, weights, LTL_expression, use_8_neighbors=True):
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
            symbols_produced.append(symbol_grid[pos[0]][pos[1]])
            final_state = node[1]  # Extract LTL state from node tuple
            # Check if parent exists before accessing
            if parent is None:
                # Goal reached from start directly or parent not found
                return path, symbols_produced, steps, final_state
            
            path.append(parent[0])  # Extract position from parent node
            symbols_produced.append(symbol_grid[parent[0][0]][parent[0][1]])
            node = parent

            # back track using the dictionary of parents as links between nodes to find the path
            while node[0] != start:
                parent = parent_dict.get(str(node))
                if parent is None:
                    # Reached start or missing parent
                    break
                path.append(parent[0])  # Extract position from parent node
                symbols_produced.append(symbol_grid[parent[0][0]][parent[0][1]])
                node = parent

            return path, symbols_produced, steps, final_state

        else:
            previous_node = node

            # updated queue, parent dictionary that contains the parents of each node (who added it or a neighbor that
            # gives it a lesser g(x) value, and the dictionary of each nodes g(x) value
            if use_8_neighbors:
                queue, parent_dict, gx_dict = eight_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, symbol_grid, parent_dict, gx_dict, goal, weights, LTL_expression)
            else:
                queue, parent_dict, gx_dict = four_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, symbol_grid, parent_dict, gx_dict, goal, weights, LTL_expression)

    return ([], [], steps, LTL_expression)


# NODE EXPANSION HELPER FUNCTIONS
# by doing != -1 I can produce a symbol path while still checking for occupancy.
# Assumption: location with symbol is never occupied
def four_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, symbol_grid, parent_dict, gx_dict, goal, weights, LTL_expression):

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
        if symbol_grid[new_row][new_col] == -1:
            continue
        
        # Get the symbol in the new cell
        cell_value = symbol_grid[new_row][new_col]
        
        # Build the set of propositions for this cell
        cell_props = set()
        if cell_value not in [-1, 0] and cell_value != {}:
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


def eight_connected_with_gx_check_state_restraint(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, symbol_grid, parent_dict, gx_dict, goal, weights, LTL_expression):
    """
    Eight-connected neighbor expansion (includes diagonals).
    Diagonal moves cost sqrt(2) * TRAVEL_CONSTANT to reflect actual distance.
    """
    # Extract position and LTL state from node tuple
    pos = node[0]
    current_ltl_state = node[1]
    
    row = pos[0]
    col = pos[1]
    
    # Get parent's g(x) cost
    parent_gx = gx_dict.get(str(node), 0)
    
    # Define eight-connected neighbors: [row_offset, col_offset, is_diagonal]
    neighbors = [
        [0, 1, False],    # right
        [1, 0, False],    # down
        [0, -1, False],   # left
        [-1, 0, False],   # up
        [1, 1, True],     # down-right (diagonal)
        [1, -1, True],    # down-left (diagonal)
        [-1, 1, True],    # up-right (diagonal)
        [-1, -1, True]    # up-left (diagonal)
    ]
    
    # Process each neighbor
    for offset in neighbors:
        new_row = row + offset[0]
        new_col = col + offset[1]
        is_diagonal = offset[2]
        
        # Check bounds
        if new_row < min_dimension or new_row > max_dimension_row:
            continue
        if new_col < min_dimension or new_col > max_dimension_col:
            continue
        
        # Check if cell is traversable (not obstacle)
        if symbol_grid[new_row][new_col] == -1:
            continue
        
        # For diagonal moves, check that both adjacent cells are also free
        # This prevents "cutting corners" through obstacles
        if is_diagonal:
            # Check the two adjacent cells
            if symbol_grid[row][new_col] == -1 or symbol_grid[new_row][col] == -1:
                continue
        
        # Get the symbol in the new cell
        cell_value = symbol_grid[new_row][new_col]
        
        # Build the set of propositions for this cell
        cell_props = set()
        if cell_value not in [-1, 0] and cell_value != {}:
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
        
        # Calculate gx with diagonal distance consideration
        # Diagonal moves cost sqrt(2) ≈ 1.414 times the normal travel constant
        import math
        travel_cost = TRAVEL_CONSTANT * math.sqrt(2) if is_diagonal else TRAVEL_CONSTANT
        gx = parent_gx + cell_weight + travel_cost
        
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

#from A* path, reconstruct the LTL state trace and path
def reconstruct_state_trace(path, grid, LTL_expression):
    """Reconstruct the LTL state trace from a path through the grid."""
    state_trace = []
    if not path:
        return state_trace
    
    current_ltl_state = next(iter(LTL_expression.init.keys())) if LTL_expression else None
    
    for pos in path:
        cell_value = grid[pos[0]][pos[1]]
        
        cell_props = set()
        if cell_value not in [-1, 0, '', '{}']:
            cell_props.add(str(cell_value))
        
        if LTL_expression:
            next_state = LTL_expression.next_state(current_ltl_state, cell_props)
            if next_state is not None:
                current_ltl_state = next_state
        
        state_trace.append((pos, current_ltl_state, cell_value))
    
    return state_trace

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


def load_maps(og_file_path, sym_file_path, weight_file_path=None):
    """Load occupancy grid, symbol grid, and optional weights from CSV files.
    
    All files must be in grid format with comma-separated values.
    
    Args:
        og_file_path: Path to occupancy grid CSV file (grid format: comma-separated integers)
        sym_file_path: Path to symbol grid CSV file (grid format: comma-separated strings)
        weight_file_path: Optional path to weights CSV file (grid format: comma-separated floats)
        
    Returns:
        occupancy_grid: 2D list of integers (occupancy values)
        symbol_grid: 2D list of strings (symbol values preserved as-is)
        weights: Dictionary mapping (row,col) tuples to weight values
        
    Raises:
        FileNotFoundError: If any required file doesn't exist
        ValueError: If grids have mismatched dimensions
    """
    # Validate file paths exist
    if not os.path.exists(og_file_path):
        raise FileNotFoundError(f"Occupancy grid file not found: {og_file_path}")
    if not os.path.exists(sym_file_path):
        raise FileNotFoundError(f"Symbol grid file not found: {sym_file_path}")
    if weight_file_path and not os.path.exists(weight_file_path):
        raise FileNotFoundError(f"Weight file not found: {weight_file_path}")
    
    occupancy_grid = []
    symbol_grid = []    
    weights = dict()

    # Load occupancy grid
    with open(og_file_path, 'r') as og_file:
        reader = csv.reader(og_file)
        for row in reader:
            # Parse grid values
            parsed_row = []
            for col in row:
                if col == '' or col.strip() == '':
                    parsed_row.append(0)
                else:
                    try:
                        parsed_row.append(int(col))
                    except ValueError:
                        parsed_row.append(0)
            if parsed_row:  # Only add non-empty rows
                occupancy_grid.append(parsed_row)
    
    # Load symbol grid - preserve values as-is from file
    with open(sym_file_path, 'r') as sym_file:
        reader = csv.reader(sym_file)
        for row in reader:
            # Keep original values from file without parsing
            parsed_row = []
            for col in row:
                # Keep empty cells as empty strings, everything else as-is
                parsed_row.append(col)
            if parsed_row:  # Only add non-empty rows
                symbol_grid.append(parsed_row)    # Validate that grids have the same dimensions
    if len(occupancy_grid) != len(symbol_grid):
        raise ValueError(
            f"Grid dimension mismatch: occupancy grid has {len(occupancy_grid)} rows, "
            f"symbol grid has {len(symbol_grid)} rows"
        )
    
    # Check column dimensions
    if len(occupancy_grid) > 0 and len(symbol_grid) > 0:
        og_cols = len(occupancy_grid[0])
        sym_cols = len(symbol_grid[0])
        if og_cols != sym_cols:
            raise ValueError(
                f"Grid dimension mismatch: occupancy grid has {og_cols} columns, "
                f"symbol grid has {sym_cols} columns"
            )
        
        # Verify all rows have consistent width
        for i, row in enumerate(occupancy_grid):
            if len(row) != og_cols:
                raise ValueError(
                    f"Inconsistent row width in occupancy grid at row {i}: "
                    f"expected {og_cols}, got {len(row)}"
                )
        
        for i, row in enumerate(symbol_grid):
            if len(row) != sym_cols:
                raise ValueError(
                    f"Inconsistent row width in symbol grid at row {i}: "
                    f"expected {sym_cols}, got {len(row)}"
                )

    # Load weights if provided - always in grid format (comma-separated)
    if weight_file_path:
        with open(weight_file_path, 'r') as weight_file:
            reader = csv.reader(weight_file)
            row_idx = 0
            for row in reader:
                # Grid format: each row contains comma-separated weight values
                for col_idx, cell in enumerate(row):
                    try:
                        weight = float(cell)
                        weights[(row_idx, col_idx)] = weight
                    except ValueError:
                        # If parsing fails, default to 0.0
                        weights[(row_idx, col_idx)] = 0.0
                row_idx += 1
    
    print(f"Loaded grids: {len(occupancy_grid)}x{len(occupancy_grid[0]) if occupancy_grid else 0}")
    
    return occupancy_grid, symbol_grid, weights

if __name__ == "__main__":
    # Load the map
    occupancy_file = 'maps/og/random_occupancy_grid.csv'
    symbol_file = 'maps/symbols/symbol.csv'
    weight_file = 'maps/weights/weights.csv'
    
    # Define start and goal
    start = [0, 0]
    goal = [7, 7]
    
    try:
        occupancy_grid, symbol_grid, weights = load_maps(occupancy_file, symbol_file, weight_file)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error loading maps: {e}")
        exit(1)
    ltl_expression = Fsa()
    ltl_expression.from_formula("G !b")
    # Search
    astar_path, astar_symbols_produced, astar_steps, final_ltl = state_aware_astar(start, goal, symbol_grid, occupancy_grid, weights, ltl_expression)

    print(f"Symbols produced: {astar_symbols_produced}")
    print(f"Path: {astar_path}")
