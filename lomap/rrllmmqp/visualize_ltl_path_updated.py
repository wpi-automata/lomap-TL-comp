#!/usr/bin/env python
"""
Visualization for A* paths with LTL state tracking.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import json

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from lomap.classes.automata_updated import Fsa
from lomap.rrllmmqp.a_star_state_aware import grid_to_world, state_aware_astar, load_maps, OBS_THRESHOLD, reconstruct_state_trace


def _color_to_rgb(c):
    """Convert matplotlib color to (r,g,b) in [0,1]."""
    from matplotlib.colors import to_rgb
    return to_rgb(c)


def visualize_ltl_aware_path(grid, start, goal, path, state_trace, ltl_formula, 
                            occupancy_grid=None, weights=None, save_path='ltl_path_visualization.png', dpi=150):
    """Create visualization showing spatial path, LTL states, and transitions.
    Uses a single raster image for the grid (fast) instead of one patch per cell."""
    fig = plt.figure(figsize=(18, 8))
    ax1 = plt.subplot(1, 2, 1)
    ax2 = plt.subplot(1, 2, 2)
    
    ax1.set_title(f'Spatial Path with LTL States\nFormula: {ltl_formula}', fontsize=14, fontweight='bold')
    
    # Check if grid is empty or invalid
    if grid is None:
        raise ValueError("Grid is None")
    
    # Handle numpy arrays
    if isinstance(grid, np.ndarray):
        if grid.size == 0 or len(grid.shape) < 2:
            raise ValueError(f"Grid is empty or invalid shape: {grid.shape}")
        row, col = grid.shape[0], grid.shape[1]
    else:
        if not grid or len(grid) == 0:
            raise ValueError("Grid is empty or invalid")
        if not grid[0] or (hasattr(grid[0], '__len__') and len(grid[0]) == 0):
            raise ValueError("Grid has no columns")
        row = len(grid)
        col = len(grid[0]) if hasattr(grid[0], '__len__') else 1
    
    symbol_colors = {
        'a': _color_to_rgb('magenta'),
        'b': _color_to_rgb('cyan'),
        'c': _color_to_rgb('blueviolet'),
        'd': _color_to_rgb('yellow'),
        'e': _color_to_rgb('orange'),
        'f': _color_to_rgb('lime'),
        'g': _color_to_rgb('pink'),
        'h': _color_to_rgb('lightblue'),
        '{}': _color_to_rgb('lightyellow')
    }
    black = (0.0, 0.0, 0.0)
    white = (1.0, 1.0, 1.0)
    gray = (0.5, 0.5, 0.5)

    # Build grid as single RGB image (one array instead of row*col patches)
    rgb = np.zeros((row, col, 3), dtype=np.float32)
    max_weight = max(weights.values()) if weights else 13.0
    min_weight = min(weights.values()) if weights else 0.0
    weight_range = max_weight - min_weight if max_weight > min_weight else 1.0

    for i in range(row):
        for j in range(col):
            if isinstance(grid, np.ndarray):
                cell_value = grid[i, j]
            else:
                cell_value = grid[i][j]
            is_obstacle = False
            if occupancy_grid is not None and len(occupancy_grid) > 0 and len(occupancy_grid[0]) > 0 and i < len(occupancy_grid) and j < len(occupancy_grid[0]):
                is_obstacle = occupancy_grid[i][j] > OBS_THRESHOLD
            if cell_value == -1 or is_obstacle:
                rgb[i, j] = black
            elif cell_value == 0 or cell_value == '' or cell_value == '{}':
                if weights is not None and (i, j) in weights:
                    weight = weights[(i, j)]
                    normalized_weight = (weight - min_weight) / weight_range if weight_range else 0.0
                    if normalized_weight < 0.01:
                        rgb[i, j] = white
                    elif normalized_weight < 0.25:
                        t = normalized_weight / 0.25
                        rgb[i, j] = (1.0, 1.0, 1.0 - t * 0.3)
                    elif normalized_weight < 0.5:
                        t = (normalized_weight - 0.25) / 0.25
                        rgb[i, j] = (1.0, 1.0 - t * 0.2, 0.7 - t * 0.7)
                    elif normalized_weight < 0.75:
                        t = (normalized_weight - 0.5) / 0.25
                        rgb[i, j] = (1.0, 0.8 - t * 0.5, 0.0)
                    else:
                        t = (normalized_weight - 0.75) / 0.25
                        rgb[i, j] = (1.0 - t * 0.3, 0.3 - t * 0.3, 0.0)
                else:
                    rgb[i, j] = white
            else:
                symbol = str(cell_value)
                rgb[i, j] = symbol_colors.get(symbol, _color_to_rgb('lightgray'))
                # blend with white for alpha effect
                rgb[i, j] = 0.6 * rgb[i, j] + 0.4 * white

    # extent: (left, right, bottom, top) in data coords; origin=upper so row 0 at top
    ax1.imshow(rgb, origin='upper', extent=(-0.5, col - 0.5, row - 0.5, -0.5), aspect='equal', interpolation='nearest')

    # Draw path as single line (fast) plus sparse arrows
    if path and len(path) > 1:
        path_cols = [p[1] for p in path]
        path_rows = [p[0] for p in path]
        ax1.plot(path_cols, path_rows, color='blue', linewidth=2, alpha=0.8, zorder=2)
        # Arrow every N segments for direction
        step = max(1, (len(path) - 1) // 8)
        for i in range(0, len(path) - 1, step):
            ax1.annotate('', xy=(path[i + 1][1], path[i + 1][0]), xytext=(path[i][1], path[i][0]),
                         arrowprops=dict(arrowstyle='->', color='blue', lw=1.5), annotation_clip=True)

    # Start and goal markers
    ax1.plot(start[1], start[0], 's', color='green', markersize=12, markeredgecolor='darkgreen', markeredgewidth=2, zorder=3)
    ax1.plot(goal[1], goal[0], 's', color='red', markersize=12, markeredgecolor='darkred', markeredgewidth=2, zorder=3)
    ax1.text(start[1], start[0], 'S', ha='center', va='center', fontsize=9, fontweight='bold', color='white', zorder=4)
    ax1.text(goal[1], goal[0], 'G', ha='center', va='center', fontsize=9, fontweight='bold', color='white', zorder=4)

    ax1.set_xlim(-0.5, col - 0.5)
    ax1.set_ylim(row - 0.5, -0.5)
    ax1.set_xlabel('Column', fontsize=12)
    ax1.set_ylabel('Row', fontsize=12)
    ax1.grid(True, alpha=0.3)

    # Legend
    legend_elements = [
        mpatches.Patch(facecolor='black', edgecolor='k', label='Obstacles'),
        mpatches.Patch(facecolor='white', edgecolor='gray', label='Weight: min'),
        mpatches.Patch(facecolor=(1.0, 1.0, 0.7), edgecolor='gray', label='Weight: low'),
        mpatches.Patch(facecolor=(1.0, 0.9, 0.0), edgecolor='gray', label='Weight: medium'),
        mpatches.Patch(facecolor=(1.0, 0.3, 0.0), edgecolor='gray', label='Weight: high'),
        mpatches.Patch(facecolor=(0.7, 0.0, 0.0), edgecolor='gray', label='Weight: max'),
        mpatches.Patch(facecolor='green', edgecolor='green', label='Start', alpha=0.5),
        mpatches.Patch(facecolor='red', edgecolor='red', label='Goal', alpha=0.5),
        mpatches.Patch(facecolor='blue', edgecolor='blue', label='Path', alpha=0.8)
    ]
    ax1.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
               fontsize=8, framealpha=0.9, borderaxespad=0)
    
    # State timeline
    ax2.set_title('LTL State Progression Over Time', fontsize=14, fontweight='bold')
    
    # Normalize state_trace: accept (pos, ltl_state, symbol), 4+ tuples (take first 3), or symbols-only (zip with path)
    _trace = []
    if state_trace and len(state_trace) > 0:
        first = state_trace[0]
        try:
            n = len(first) if isinstance(first, (list, tuple)) else 1
        except TypeError:
            n = 1
        if n >= 3:
            for item in state_trace:
                _trace.append((item[0], item[1], item[2]))
        elif path and (n == 1 or not isinstance(first, (list, tuple))):
            # Symbols-only list (e.g. from state_aware_astar's symbols_produced)
            for i, sym in enumerate(state_trace):
                pos = path[i] if i < len(path) else (0, 0)
                _trace.append((pos, None, sym))
        else:
            _trace = [(item[0], item[1], item[2]) for item in state_trace if isinstance(item, (list, tuple)) and len(item) >= 3]
    state_trace = _trace

    if state_trace and len(state_trace) > 0:
        states_seen = []
        state_changes = []
        
        prev_state = None
        for i, (pos, ltl_state, symbol) in enumerate(state_trace):
            if ltl_state != prev_state:
                state_changes.append((i, prev_state, ltl_state, symbol))
                if ltl_state not in [s[1] for s in states_seen]:
                    states_seen.append((len(states_seen), ltl_state))
                prev_state = ltl_state
        
        if len(states_seen) > 0:
            steps = list(range(len(state_trace)))
            state_indices = []
            
            for i, (pos, ltl_state, symbol) in enumerate(state_trace):
                for idx, (state_idx, state_name) in enumerate(states_seen):
                    if state_name == ltl_state:
                        state_indices.append(idx)
                        break
            
            ax2.step(steps, state_indices, where='post', linewidth=2, color='darkblue')
            ax2.scatter(steps, state_indices, c='darkblue', s=50, zorder=5)
            
            # Annotate transitions
            for step, old_state, new_state, symbol in state_changes:
                if old_state is None:
                    continue
                    
                if step < len(state_trace):
                    transition_symbol = state_trace[step][2]
                    
                    if transition_symbol not in [0, '', -1, '{}']:
                        symbol_name = str(transition_symbol)
                        y_pos = None
                        for idx, (state_idx, state_name) in enumerate(states_seen):
                            if state_name == new_state:
                                y_pos = idx
                                break
                        
                        if y_pos is not None:
                            ax2.annotate(f'saw "{symbol_name}"', xy=(step, y_pos), 
                                       xytext=(step + 2, y_pos + 0.3),
                                       arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
                                       fontsize=10, color='red', fontweight='bold',
                                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
            
            ax2.set_yticks(range(len(states_seen)))
            ax2.set_yticklabels([f'State {i}\n({name})' for i, name in states_seen], fontsize=9)
            ax2.set_xlabel('Step Number', fontsize=12)
            ax2.set_ylabel('LTL Automaton State', fontsize=12)
            ax2.grid(True, alpha=0.3, axis='x')
            ax2.set_xlim(-1, len(steps))
            
            info_text = f"Total Steps: {len(path)}\n"
            info_text += f"State Transitions: {len(state_changes) - 1}\n"
            info_text += f"Symbols Encountered: {len([s for s in state_trace if s[2] != 0])}"
            ax2.text(0.98, 0.02, info_text, transform=ax2.transAxes, fontsize=10,
                    verticalalignment='bottom', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        else:
            ax2.text(0.5, 0.5, 'No LTL state tracking\n(No LTL constraints applied)', 
                    ha='center', va='center', fontsize=14, transform=ax2.transAxes)
    else:
        ax2.text(0.5, 0.5, 'No LTL state tracking\n(No LTL constraints applied)', 
                ha='center', va='center', fontsize=14, transform=ax2.transAxes)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=dpi, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {save_path}")
    plt.close(fig)
    # Return left-panel raster as (H, W, 3) uint8 for callers (e.g. Image.fromarray)
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


if __name__ == '__main__':
    # Load maps
    occupancy_grid, symbol_grid, weights, origin, resolution = load_maps(
        '/home/bdc/MQP/reliable_robot_llms/maps/uh2.yaml',
        '/home/bdc/MQP/reliable_robot_llms/maps/uh2_labels.csv',
        '/home/bdc/MQP/reliable_robot_llms/maps/uh2_weights.csv'
    )
    
    # Start and goal positions that require navigating around a/b regions
    # a: rows 180-217, cols 140-219
    # b: rows 213-243, cols 187-222
    start = [170, 170]  # Before the regions
    
    goal = [230, 210]  # Past the regions
    
    # Create LTL expression (set to None for no constraints)
    ltl_formula = "(G !a) && (G !b )"
    ltl_expression = Fsa()
    ltl_expression.from_formula(ltl_formula)
    
    # Create working copy (A* modifies the grid)
    # Use symbol_grid for state trace reconstruction, not occupancy_grid
    if isinstance(symbol_grid, np.ndarray):
        grid = symbol_grid.copy()
    elif isinstance(symbol_grid, list):
        grid = [row[:] for row in symbol_grid]
    else:
        grid = symbol_grid.copy() if hasattr(symbol_grid, 'copy') else symbol_grid
    
    # Run A* planner
    path, symbols_produced, steps, final_state = state_aware_astar(
        start, goal, symbol_grid, occupancy_grid, weights, ltl_expression, origin=origin, resolution=resolution, use_8_neighbors=True
    )
    print(f"Path: {path}")
    if grid is not None and len(grid) > 0 and len(grid[0]) > 0:
        print(f"Grid dimensions: {len(grid)}, {len(grid[0])}")
    else:
        print(f"Grid dimensions: Invalid or empty grid")
        print(f"Grid type: {type(grid)}, Grid length: {len(grid) if grid else 'None'}")
    if path:
        # Reconstruct state trace (path is in world coordinates, need to pass origin/resolution)
        # Use symbol_grid for state trace, not occupancy_grid
        state_trace = reconstruct_state_trace(path, symbol_grid, ltl_expression, origin=origin, resolution=resolution)

        # Save path data to JSON file for web visualization
        path_data = {
            'path': path,  # World coordinates
            'symbols_produced': symbols_produced,
            'steps': steps,
            'final_state': str(final_state),
            'state_trace': [(list(pos), str(state), str(symbol)) for pos, state, symbol in state_trace],
            'start': start,
            'goal': goal,
            'ltl_formula': ltl_formula,
            'origin': origin,
            'resolution': resolution,
            'grid_dimensions': [len(occupancy_grid), len(occupancy_grid[0])]
        }
        
        output_file = 'path_output.json'
        with open(output_file, 'w') as f:
            json.dump(path_data, f, indent=2)
        
        print(f"✓ Path found with {len(path)} positions in {steps} steps")
        print(f"✓ Path data saved to {output_file}")
        print(f"  Use the occupancy grid labeler HTML to visualize this path")
    else:
        print("✗ No path found")
