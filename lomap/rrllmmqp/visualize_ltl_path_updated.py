#!/usr/bin/env python
"""
Visualization for A* paths with LTL state tracking.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
import matplotlib.patches as mpatches
import json

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from lomap.classes.automata_updated import Fsa
from lomap.rrllmmqp.a_star_state_aware import grid_to_world, state_aware_astar, load_maps, OBS_THRESHOLD, reconstruct_state_trace


def visualize_ltl_aware_path(grid, start, goal, path, state_trace, ltl_formula, symbol_positions, 
                            occupancy_grid=None, weights=None, save_path='ltl_path_visualization.png'):
    """Create visualization showing spatial path, LTL states, and transitions."""
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
        'a': 'magenta',
        'b': 'cyan',
        'c': 'blueviolet',
        'd': 'yellow',
        'e': 'orange',
        'f': 'lime',
        'g': 'pink',
        'h': 'lightblue',
        '{}': 'lightyellow'
    }
    
    for i in range(row):
        for j in range(col):
            # Handle both list and numpy array access
            if isinstance(grid, np.ndarray):
                cell_value = grid[i, j]
            else:
                cell_value = grid[i][j]
            
            is_obstacle = False
            if occupancy_grid is not None and len(occupancy_grid) > 0 and len(occupancy_grid[0]) > 0 and i < len(occupancy_grid) and j < len(occupancy_grid[0]):
                is_obstacle = occupancy_grid[i][j] > OBS_THRESHOLD
            
            if cell_value == -1 or is_obstacle: 
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='k'))
            elif cell_value == 0 or cell_value == '' or cell_value == '{}':
                cell_color = 'w'
                edge_color = 'gray'
                text_color = None
                
                if weights is not None and (i, j) in weights:
                    weight = weights[(i, j)]
                    max_weight = max(weights.values()) if weights else 13.0
                    min_weight = min(weights.values()) if weights else 0.0
                    
                    if max_weight > min_weight:
                        normalized_weight = (weight - min_weight) / (max_weight - min_weight)
                    else:
                        normalized_weight = 0.0
                    
                    if normalized_weight < 0.01:
                        cell_color = (1.0, 1.0, 1.0)
                    elif normalized_weight < 0.25:
                        t = normalized_weight / 0.25
                        cell_color = (1.0, 1.0, 1.0 - t * 0.3)
                        text_color = 'gray'
                    elif normalized_weight < 0.5:
                        t = (normalized_weight - 0.25) / 0.25
                        cell_color = (1.0, 1.0 - t * 0.2, 0.7 - t * 0.7)
                        text_color = 'darkred'
                    elif normalized_weight < 0.75:
                        t = (normalized_weight - 0.5) / 0.25
                        cell_color = (1.0, 0.8 - t * 0.5, 0.0)
                        text_color = 'darkred'
                    else:
                        t = (normalized_weight - 0.75) / 0.25
                        cell_color = (1.0 - t * 0.3, 0.3 - t * 0.3, 0.0)
                        text_color = 'white'
                    
                    if weight > 0:
                        edge_color = (0.6, 0.6, 0.6)
                
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor=edge_color, 
                                       facecolor=cell_color, linewidth=0.5))
                
                if weights is not None and (i, j) in weights and weights[(i, j)] > 0:
                    if text_color is None:
                        text_color = 'darkred'
                    ax1.text(j, i, f'{weights[(i, j)]:.0f}', ha='center', va='center', 
                            fontsize=8, color=text_color, fontweight='bold')
            else:
                symbol = str(cell_value)
                color = symbol_colors.get(symbol, 'lightgray')
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor=color, alpha=0.6))
                ax1.text(j, i, symbol, ha='center', va='center', fontsize=10, fontweight='bold')
    
    # Draw path with arrows
    for i in range(len(path) - 1):
        pos_current = path[i]
        pos_next = path[i + 1]
        arrow = FancyArrowPatch((pos_current[1], pos_current[0]), (pos_next[1], pos_next[0]),
                               arrowstyle='->', color='blue', linewidth=2, mutation_scale=20, alpha=0.7)
        ax1.add_patch(arrow)
        
        if i % 3 == 0:
            mid_x = (pos_current[1] + pos_next[1]) / 2
            mid_y = (pos_current[0] + pos_next[0]) / 2
            ax1.text(mid_x, mid_y, str(i), bbox=dict(boxstyle='circle', facecolor='white', 
                    edgecolor='blue', alpha=0.8), ha='center', va='center', fontsize=8)
    
    # Start and goal
    ax1.add_patch(Rectangle((start[1]-0.5, start[0]-0.5), 1, 1, edgecolor='green', 
                           facecolor='green', linewidth=3, alpha=0.5))
    ax1.text(start[1], start[0], 'START', ha='center', va='center', fontsize=10, 
            fontweight='bold', color='white')
    ax1.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5), 1, 1, edgecolor='red', 
                           facecolor='red', linewidth=3, alpha=0.5))
    ax1.text(goal[1], goal[0], 'GOAL', ha='center', va='center', fontsize=10, 
            fontweight='bold', color='white')
    
    ax1.set_xlim(-1, col)
    ax1.set_ylim(-1, row)
    ax1.set_aspect('equal')
    ax1.invert_yaxis()
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
        mpatches.FancyArrowPatch((0, 0), (0.5, 0), arrowstyle='->', color='blue', 
                                linewidth=2, label='Path', mutation_scale=20)
    ]
    ax1.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
               fontsize=8, framealpha=0.9, borderaxespad=0)
    
    # State timeline
    ax2.set_title('LTL State Progression Over Time', fontsize=14, fontweight='bold')
    
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
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {save_path}")


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
