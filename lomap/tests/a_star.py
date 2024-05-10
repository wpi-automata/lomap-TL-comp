#Author: Taylor Bergeron

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import numpy as np
import random


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

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
        return (path, steps)

    queue.append(start)
    visited.append(start)

    path, symbols_produced, steps = iterative_a_star(grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col,
                                     steps)
    if path != []:
        print(f"It takes {steps} steps to find a path using A*")
        path.reverse()
        symbols_produced.reverse()
        return (path, symbols_produced, steps)

    else:
        print("No path found")
        return path, symbols_produced, steps


def iterative_a_star(grid, queue, start, goal, parent_dict, visited, min_dimension, max_dimension_row, max_dimension_col, steps):
    previous_node = None

    gx_dict = dict()
    gx_dict[str(start)] = 0

    while queue:

        # this is a dictionary of the f(x) values for everything in this queue, which is why it is reset every time a new
        # node is popped
        fx = dict()

        steps += 1

        if previous_node is not None:
            for n in queue:

                gx = gx_dict.get(str(n))
                dx_goal = abs(goal[1] - n[1])
                dy_goal = abs(goal[0] - n[0])
                hx = dx_goal + dy_goal

                # calculate f(x) and store it in a dict with the key as node for future reference
                fx[str(n[0]) + ',' + str(n[1])] = gx+hx

            fx_sorted = dict(sorted(fx.items(), key=lambda item: item[1]))

            queue = list()

            # rearrange the queue based on the f(x) values for each node in the queue
            for key in fx_sorted.keys():
                node_info = key.split(",")
                queue.append([int(node_info[0]), int(node_info[1])])

        node = queue.pop(0)
        visited.append(node)

        if node == goal:
            path = list()
            symbols_produced = list()
            path.append(node)
            parent = parent_dict.get(str(node))
            symbols_produced.append(grid[node[0]][node[1]])
            path.append(parent)
            symbols_produced.append(grid[parent[0]][parent[1]])
            node = parent

            # back track the using the dictonary of parents as links between nodes to find the path
            while node != start:
                parent = parent_dict.get(str(node))
                path.append(parent)
                symbols_produced.append(grid[parent[0]][parent[1]])
                node = parent

            return path, symbols_produced, steps

        else:
            previous_node = node

            # updated queue, parent dictionary that contains the parents of each node (who added it or a neighbor that
            # gives it a lesser g(x) value, and the dictionary of each nodes g(x) value
            queue, parent_dict, gx_dict = four_connected_with_gx_check(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, parent_dict, gx_dict, goal)

    return ([], [], steps)


# NODE EXPANSION HELPER FUNCTIONS
# by doing != -1 I can produce a symbol path while still checking for occupancy.
# Assumption: location with symbol is never occupied
def four_connected_with_gx_check(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, parent_dict, gx_dict, goal):

    row = node[0]
    col = node[1]

    # upper left case
    if node[0] == min_dimension and node[1] == min_dimension:

        node_to_add = [min_dimension, min_dimension + 1]
        gx = gx_dict.get(str(node))+1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [min_dimension + 1, min_dimension]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    # lower right case
    elif node[0] == max_dimension_row and node[1] == max_dimension_col:
        node_to_add = [max_dimension_row, max_dimension_col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [max_dimension_row - 1, max_dimension_col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    # upper right case
    elif node[0] == min_dimension and node[1] == max_dimension_col:

        node_to_add = [min_dimension + 1, max_dimension_col]

        # adds 1 to the dictionary of g(x) values since any node to be added is 1 manhattan grid away from the
        # node adding it
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx

        # checks for the case that the node adding it makes it closer to the start then its previous parent, so a
        # lower g(x) value.  If so, it replaces its parent and updates its g(x) value
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [min_dimension, max_dimension_col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    # lower left case
    elif node[0] == max_dimension_row and node[1] == min_dimension:

        node_to_add = [max_dimension_row, min_dimension + 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [max_dimension_row - 1, min_dimension]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    #left case
    elif node[0] == min_dimension:

        node_to_add = [row, col + 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row + 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row, col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    # upper case
    elif node[1] == min_dimension:

        node_to_add = [row, col + 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row + 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row - 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx


    # right case
    elif node[0] == max_dimension_row:

        node_to_add = [row, col + 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx


        node_to_add = [row, col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx


        node_to_add = [row - 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    # upper case
    elif node[1] == max_dimension_col:

        node_to_add = [row + 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row, col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row - 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx


    # all other "interior grid" cases
    else:
        node_to_add = [row, col + 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row + 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row, col - 1]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

        node_to_add = [row - 1, col]
        gx = gx_dict.get(str(node)) + 1
        if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] != -1 and node_to_add not in visited:
            parent_dict[str(node_to_add)] = node
            queue.append(node_to_add)
            gx_dict[str(node_to_add)] = gx
        elif node_to_add in queue and gx < gx_dict[str(node_to_add)]:
            parent_dict[str(node_to_add)] = node
            gx_dict[str(node_to_add)] = gx

    return queue, parent_dict, gx_dict

# Load map, start and goal point.
def load_map(file_path):
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

if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('maps/map.csv')

    # Search
    astar_path, astar_symbols_produced, aster_steps = astar(grid, start, goal)

    print(f"Symbols produced: {astar_symbols_produced}")

    # Show result
    draw_path(grid, start, goal, astar_path, 'A*')
    plt.show()