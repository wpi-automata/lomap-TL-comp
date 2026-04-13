# Author: Taylor Bergeron
# UPDATED VERSION: Fixed LTL state transitions with proper symbol mapping

import os
import sys
import heapq
import itertools

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from lomap.classes.automata_updated import Fsa
import csv
import numpy as np
import random
import yaml
from PIL import Image

# Travel constant for A* cost calculation
TRAVEL_CONSTANT = 1.0
OBS_THRESHOLD = 80


def memoized_next_ltl_state(
    current_ltl_state, cell_props, ltl_expression, ltl_state_cache
):
    if ltl_expression is None:
        return current_ltl_state

    props_key = frozenset(cell_props)
    try:
        cache_key = (current_ltl_state, props_key)
        if cache_key in ltl_state_cache:
            return ltl_state_cache[cache_key]
    except TypeError:
        cache_key = (repr(current_ltl_state), tuple(sorted(props_key)))
        if cache_key in ltl_state_cache:
            return ltl_state_cache[cache_key]

    next_state = ltl_expression.next_state(current_ltl_state, cell_props)
    ltl_state_cache[cache_key] = next_state
    return next_state


def world_to_grid(x, y, origin, resolution):
    """Convert world coordinates (meters) to grid indices.

    Args:
        x: X coordinate in meters (world frame)
        y: Y coordinate in meters (world frame)
        origin: [x, y, theta] origin offset from YAML file
        resolution: meters per pixel

    Returns:
        [row, col]: Grid indices where rows are X-axis, columns are Y-axis
    """
    grid_row = int((x - origin[0]) / resolution)  # X maps to rows
    grid_col = int((y - origin[1]) / resolution)  # Y maps to columns
    return [grid_row, grid_col]


def grid_to_world(row, col, origin, resolution):
    """Convert grid indices to world coordinates (meters).

    Args:
        row: Grid row index (represents X-axis)
        col: Grid column index (represents Y-axis)
        origin: [x, y, theta] origin offset from YAML file
        resolution: meters per pixel

    Returns:
        [x, y]: World coordinates in meters
    """
    x = row * resolution + origin[0]  # Rows represent X
    y = col * resolution + origin[1]  # Columns represent Y
    return [x, y]


def state_aware_astar(
    start,
    goal,
    symbol_grid,
    occupancy_grid,
    weights,
    LTL_expression,
    use_8_neighbors=True,
    origin=None,
    resolution=None,
):
    """Return a path found by A* alogirhm w/ regards to an LTL statement
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
    """
    # Convert world coordinates to grid indices if origin and resolution are provided
    return_world_coords = False
    # overrode
    if origin is not None and resolution is not None and False:
        return_world_coords = True
        start_grid = world_to_grid(start[0], start[1], origin, resolution)
        goal_grid = world_to_grid(goal[0], goal[1], origin, resolution)
    else:
        start_grid = tuple(start)
        goal_grid = tuple(goal)

    # Merge occupancy grid obstacles into symbol grid
    # If occupancy grid cell > OBS_THRESHOLD, mark as obstacle (-1) in symbol grid
    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] > OBS_THRESHOLD:
                symbol_grid[i][j] = -1

    visited = set()
    parent_dict = {}
    path = []
    symbols_produced = []
    steps = 0

    min_dimension = 0
    max_dimension_row = len(symbol_grid) - 1
    max_dimension_col = len(symbol_grid[0]) - 1

    if start_grid == goal_grid:
        print("start the same as goal")
        return (path, symbols_produced, steps, None)

    # Get initial LTL state if expression is provided
    if LTL_expression:
        # Get the initial state from the FSA's init dictionary
        initial_ltl_state = next(iter(LTL_expression.init.keys()))
    else:
        initial_ltl_state = None

    ltl_state_cache = {}

    start_node = (start_grid, initial_ltl_state)

    path, symbols_produced, steps, final_state = iterative_a_star(
        symbol_grid,
        start_node,
        goal_grid,
        parent_dict,
        visited,
        min_dimension,
        max_dimension_row,
        max_dimension_col,
        steps,
        weights,
        LTL_expression,
        ltl_state_cache,
        use_8_neighbors,
    )
    if path != []:
        print(f"It takes {steps} steps to find a path using A*")
        path.reverse()
        symbols_produced.reverse()

        # Convert path back to world coordinates if needed
        if return_world_coords:
            path = [grid_to_world(pos[0], pos[1], origin, resolution) for pos in path]

        return (path, symbols_produced, steps, final_state)

    else:
        print("No path found")
        return path, symbols_produced, steps, final_state


def iterative_a_star(
    symbol_grid,
    start_node,
    goal,
    parent_dict,
    visited,
    min_dimension,
    max_dimension_row,
    max_dimension_col,
    steps,
    weights,
    LTL_expression,
    ltl_state_cache,
    use_8_neighbors=True,
):
    previous_node = None

    gx_dict = {}
    counter = itertools.count()
    # Initialize with start node (which is now a tuple of (position, ltl_state))
    gx_dict[start_node] = 0

    heap = []
    start_pos = start_node[0]
    start_hx = abs(goal[1] - start_pos[1]) + abs(goal[0] - start_pos[0])
    start_fx = gx_dict[start_node] + start_hx
    entry = (start_fx, start_hx, next(counter), start_node)
    heapq.heappush(heap, entry)

    while heap:
        steps += 1

        _, _, _, node = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)

        # Extract position from node tuple
        pos = node[0]

        if pos == goal:
            path = list()
            symbols_produced = list()
            path.append(list(pos))
            parent = parent_dict.get(node)
            symbols_produced.append(symbol_grid[pos[0]][pos[1]])
            final_state = node[1]  # Extract LTL state from node tuple
            # Check if parent exists before accessing
            if parent is None:
                # Goal reached from start directly or parent not found
                return path, symbols_produced, steps, final_state

            path.append(list(parent[0]))  # Extract position from parent node
            symbols_produced.append(symbol_grid[parent[0][0]][parent[0][1]])
            node = parent

            # back track using the dictionary of parents as links between nodes to find the path
            while node[0] != start_node[0]:
                parent = parent_dict.get(node)
                if parent is None:
                    # Reached start or missing parent
                    break
                path.append(list(parent[0]))  # Extract position from parent node
                symbols_produced.append(symbol_grid[parent[0][0]][parent[0][1]])
                node = parent

            return path, symbols_produced, steps, final_state

        else:
            previous_node = node

            # updated heap, parent dictionary that contains the parents of each node
            # gives it a lesser g(x) value, and the dictionary of each nodes g(x) value
            if use_8_neighbors:
                heap, parent_dict, gx_dict = (
                    eight_connected_with_gx_check_state_restraint(
                        node,
                        visited,
                        min_dimension,
                        max_dimension_row,
                        max_dimension_col,
                        heap,
                        symbol_grid,
                        parent_dict,
                        gx_dict,
                        goal,
                        weights,
                        LTL_expression,
                        ltl_state_cache,
                        counter,
                    )
                )
            else:
                heap, parent_dict, gx_dict = (
                    four_connected_with_gx_check_state_restraint(
                        node,
                        visited,
                        min_dimension,
                        max_dimension_row,
                        max_dimension_col,
                        heap,
                        symbol_grid,
                        parent_dict,
                        gx_dict,
                        goal,
                        weights,
                        LTL_expression,
                        ltl_state_cache,
                        counter,
                    )
                )

    return ([], [], steps, LTL_expression)


# NODE EXPANSION HELPER FUNCTIONS
# by doing != -1 I can produce a symbol path while still checking for occupancy.
# Assumption: location with symbol is never occupied
def four_connected_with_gx_check_state_restraint(
    node,
    visited,
    min_dimension,
    max_dimension_row,
    max_dimension_col,
    heap,
    symbol_grid,
    parent_dict,
    gx_dict,
    goal,
    weights,
    LTL_expression,
    ltl_state_cache,
    counter,
):
    # Extract position and LTL state from node tuple
    pos = node[0]
    current_ltl_state = node[1]

    row = pos[0]
    col = pos[1]

    # Get parent's g(x) cost
    parent_gx = gx_dict.get(node, 0)

    # Define four-connected neighbors: [row_offset, col_offset]
    neighbors = [
        [0, 1],  # right
        [1, 0],  # down
        [0, -1],  # left
        [-1, 0],  # up
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

        next_ltl_state = memoized_next_ltl_state(
            current_ltl_state,
            cell_props,
            LTL_expression,
            ltl_state_cache,
        )
        if next_ltl_state is None:
            # Moving to this cell would violate the LTL expression
            continue

        # Create node with position and LTL state
        new_pos = (new_row, new_col)
        node_to_add = (new_pos, next_ltl_state)

        # Get cell weight from weights dictionary (default to 0 if not found)
        cell_weight = weights.get((new_row, new_col), 0)

        # Calculate gx using formula: gx = parent_gx + cell_weight + travel_constant
        gx = parent_gx + cell_weight + TRAVEL_CONSTANT

        if  gx < gx_dict.get(node_to_add, float("inf")):
            parent_dict[node_to_add] = node
            gx_dict[node_to_add] = gx
            # Reopen this exact (position, LTL state) node if it was already closed.
            if node_to_add in visited:
                visited.remove(node_to_add)
            dx_goal = abs(goal[1] - new_col)
            dy_goal = abs(goal[0] - new_row)
            hx = dx_goal + dy_goal
            fx = gx + hx
            heapq.heappush(heap, (fx, hx, next(counter), node_to_add))

    return heap, parent_dict, gx_dict


def eight_connected_with_gx_check_state_restraint(
    node,
    visited,
    min_dimension,
    max_dimension_row,
    max_dimension_col,
    heap,
    symbol_grid,
    parent_dict,
    gx_dict,
    goal,
    weights,
    LTL_expression,
    ltl_state_cache,
    counter,
):
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
    parent_gx = gx_dict.get(node, 0)

    # Define eight-connected neighbors: [row_offset, col_offset, is_diagonal]
    neighbors = [
        [0, 1, False],  # right
        [1, 0, False],  # down
        [0, -1, False],  # left
        [-1, 0, False],  # up
        [1, 1, True],  # down-right (diagonal)
        [1, -1, True],  # down-left (diagonal)
        [-1, 1, True],  # up-right (diagonal)
        [-1, -1, True],  # up-left (diagonal)
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

        next_ltl_state = memoized_next_ltl_state(
            current_ltl_state,
            cell_props,
            LTL_expression,
            ltl_state_cache,
        )
        if next_ltl_state is None:
            # Moving to this cell would violate the LTL expression
            continue

        # Create node with position and LTL state
        new_pos = (new_row, new_col)
        node_to_add = (new_pos, next_ltl_state)


        # Get cell weight from weights dictionary (default to 0 if not found)
        cell_weight = weights.get((new_row, new_col), 0)

        # Calculate gx with diagonal distance consideration
        # Diagonal moves cost sqrt(2) ≈ 1.414 times the normal travel constant
        import math

        travel_cost = TRAVEL_CONSTANT * math.sqrt(2) if is_diagonal else TRAVEL_CONSTANT
        gx = parent_gx + cell_weight + travel_cost

        if gx < gx_dict.get(node_to_add, float("inf")):
            parent_dict[node_to_add] = node
            gx_dict[node_to_add] = gx
            # Reopen this exact (position, LTL state) node if it was already closed.
            if node_to_add in visited:
                visited.remove(node_to_add)
            dx_goal = abs(goal[1] - new_col)
            dy_goal = abs(goal[0] - new_row)
            hx = dx_goal + dy_goal
            fx = gx + hx
            heapq.heappush(heap, (fx, hx, next(counter), node_to_add))

    return heap, parent_dict, gx_dict


# from A* path, reconstruct the LTL state trace and path
def reconstruct_state_trace(path, grid, LTL_expression, origin=None, resolution=None):
    """Reconstruct the LTL state trace from a path through the grid.

    Args:
        path: List of positions. Can be grid indices [row, col] or world coords [x, y]
        grid: Symbol grid
        LTL_expression: LTL automaton
        origin: Optional [x, y, theta] for converting world coords to grid indices
        resolution: Optional meters per pixel for converting world coords to grid indices

    Returns:
        List of (position, ltl_state, cell_value) tuples
    """
    state_trace = []
    if not path:
        return state_trace

    current_ltl_state = (
        next(iter(LTL_expression.init.keys())) if LTL_expression else None
    )

    for pos in path:
        # Convert world coordinates to grid indices if needed
        if origin is not None and resolution is not None and False:
            # pos is in world coordinates [x, y], convert to grid indices
            grid_pos = world_to_grid(pos[0], pos[1], origin, resolution)
            row, col = grid_pos[0], grid_pos[1]
        else:
            # pos is already in grid indices [row, col]
            row, col = int(pos[0]), int(pos[1])
        print(f"Row: {row}, Col: {col}")
        cell_value = grid[row][col]

        cell_props = set()
        if cell_value not in [-1, 0, "", "{}"]:
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
    with open(file_path, "r") as map_file:
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


# TODO: allow for user to choose dimensions of map
# Create map with randomized start and goal point, using input of percent empty cells produce no symbols
def create_map(percent_empty_open, props):
    numerical_props = list(props.keys())

    dim = 12  # width and height of map

    start = [random.randint(1, 10), random.randint(1, 10)]
    goal = [random.randint(1, 10), random.randint(1, 10)]

    # # Creating a dimxdim matrix filled with -1s
    # grid = np.full((dim, dim), -1)

    # # Setting the inner values of the matrix 'x' (excluding the borders) to 0 using slicing
    # grid[1:-1, 1:-1] = 0

    number_symbols = int((1 - percent_empty_open) * (dim * dim))

    inner_map_length = (dim - 2) ** 2
    inner_map = np.zeros((inner_map_length))  # map inside -1 boundaries

    for i in range(number_symbols):
        inner_map[random.randint(0, inner_map_length - 1)] = random.choice(
            numerical_props
        )

    inner_map = inner_map.reshape((dim - 2, dim - 2))

    grid = np.pad(inner_map, pad_width=1, mode="constant", constant_values=-1)

    print(grid)

    return grid.astype(int), start, goal


def load_maps(map_yaml_path, sym_file_path, weight_file_path=None):
    """Load occupancy grid from ROS map files (.yaml + .pgm), symbol grid from CSV, and optional weights.

    Args:
        map_yaml_path: Path to ROS map .yaml file (contains metadata and references .pgm image)
        sym_file_path: Path to symbol grid CSV file (grid format: comma-separated strings)
        weight_file_path: Optional path to weights CSV file (grid format: comma-separated floats)

    Returns:
        occupancy_grid: 2D list of integers (occupancy values: 0=free, 100=occupied)
        symbol_grid: 2D list of strings (symbol values preserved as-is)
        weights: Dictionary mapping (row,col) tuples to weight values
        origin: [x, y, theta] offset from map metadata
        resolution: meters per pixel

    Raises:
        FileNotFoundError: If any required file doesn't exist
        ValueError: If grids have mismatched dimensions
    """
    # Validate yaml file exists
    if not os.path.exists(map_yaml_path):
        raise FileNotFoundError(f"Map YAML file not found: {map_yaml_path}")
    if not os.path.exists(sym_file_path):
        raise FileNotFoundError(f"Symbol grid file not found: {sym_file_path}")
    if weight_file_path and not os.path.exists(weight_file_path):
        raise FileNotFoundError(f"Weight file not found: {weight_file_path}")

    # Load map metadata from YAML
    with open(map_yaml_path, "r") as yaml_file:
        map_data = yaml.safe_load(yaml_file)

    # Extract parameters
    image_filename = map_data["image"]
    resolution = map_data["resolution"]
    origin = map_data["origin"]  # [x, y, theta]
    occupied_thresh = map_data.get("occupied_thresh", 0.65)

    # Load PGM image
    map_dir = os.path.dirname(map_yaml_path)
    pgm_path = os.path.join(map_dir, image_filename)

    if not os.path.exists(pgm_path):
        raise FileNotFoundError(f"PGM image file not found: {pgm_path}")

    # Load image and convert to numpy array
    img = Image.open(pgm_path)
    img_array = np.array(img)

    # Convert image to occupancy grid
    # White pixels (254,254,254 or high values) = free space (0)
    # Other pixels = occupied (100)
    occupancy_grid = []

    if len(img_array.shape) == 3:  # RGB image
        # Convert to grayscale if needed
        img_gray = np.mean(img_array, axis=2)
    else:  # Already grayscale
        img_gray = img_array

    # Threshold: white pixels (>250) = free (0), others = occupied (100)
    for row in img_gray:
        og_row = []
        for pixel in row:
            if pixel > 250:  # White = free space
                og_row.append(0)
            else:  # Non-white = occupied
                og_row.append(100)
        occupancy_grid.append(og_row)

    # Load symbol grid - preserve values as-is from file
    symbol_grid = []
    with open(sym_file_path, "r") as sym_file:
        reader = csv.reader(sym_file)
        for row in reader:
            # Keep original values from file without parsing
            parsed_row = []
            for col in row:
                # Keep empty cells as empty strings, everything else as-is
                parsed_row.append(col)
            if parsed_row:  # Only add non-empty rows
                symbol_grid.append(parsed_row)

    # Validate that grids have the same dimensions
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
    weights = dict()
    if weight_file_path:
        with open(weight_file_path, "r") as weight_file:
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

    print(f"Loaded map from {map_yaml_path}")
    print(
        f"  Grid dimensions: {len(occupancy_grid)}x{len(occupancy_grid[0]) if occupancy_grid else 0}"
    )
    print(f"  Resolution: {resolution} m/pixel")
    print(f"  Origin: {origin}")

    return occupancy_grid, symbol_grid, weights, origin, resolution


def visualize_astar_result(
    occupancy_grid,
    symbol_grid,
    weights,
    path,
    start,
    goal,
    title="A* Path Result",
    save_path=None,
):
    """
    Convenience function to visualize A* search results.

    Args:
        occupancy_grid: 2D list/array of occupancy values
        symbol_grid: 2D list/array of symbol labels
        weights: Dictionary or 2D array of weights
        path: List of [row, col] coordinates from A* search
        start: [row, col] start position
        goal: [row, col] goal position
        title: Title for the plot
        save_path: Optional path to save the figure
    """
    try:
        from lomap.tests.visualize_path import visualize_path

        visualize_path(
            occupancy_grid=occupancy_grid,
            symbol_grid=symbol_grid,
            weights=weights,
            path=path,
            start=start,
            goal=goal,
            title=title,
            save_path=save_path,
            show_labels=True,
            show_weights=True,
        )
    except ImportError:
        print(
            "Warning: visualize_path module not available. Install matplotlib to enable visualization."
        )


if __name__ == "__main__":
    # Load the map from ROS map format
    map_yaml_file = "/home/colette/MQP/reliable_robot_llms/maps/nav2_local_map.yaml"
    symbol_file = "/home/colette/MQP/reliable_robot_llms/maps/nav2_local_map_labels.csv"
    weight_file = (
        "/home/colette/MQP/reliable_robot_llms/maps/nav2_local_map_weights.csv"
    )

    # Define start and goal
    start = [25, 25]
    goal = [50, 50]

    try:
        occupancy_grid, symbol_grid, weights, origin, resolution = load_maps(
            map_yaml_file, symbol_file, weight_file
        )
        print(f"Map origin offset: {origin}")
        print(f"Map resolution: {resolution} m/pixel")
    except (FileNotFoundError, ValueError) as e:
        print(f"Error loading maps: {e}")
        exit(1)
    ltl_expression = Fsa()
    ltl_expression.from_formula("G !b")
    # Search
    astar_path, astar_symbols_produced, astar_steps, final_ltl = state_aware_astar(
        start, goal, symbol_grid, occupancy_grid, weights, ltl_expression
    )

    print(f"Symbols produced: {astar_symbols_produced}")
    print(f"Path: {astar_path}")
