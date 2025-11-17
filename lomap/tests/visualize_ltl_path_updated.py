#!/usr/bin/env python
"""
Enhanced visualization for A* paths with LTL state tracking.
UPDATED VERSION: Uses automata_updated.py with NetworkX compatibility fixes

This module provides visualization tools to show:
1. The spatial path through the grid
2. The LTL automaton state at each step
3. Transitions between LTL states
4. Symbol encounters and their effects on LTL progress
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
import matplotlib.patches as mpatches

# Add parent directory to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import the UPDATED automata classes
from lomap.classes.automata_updated import Fsa
# Import the UPDATED A* with symbol mapping fix
from lomap.tests.a_star_weights_state_restraints_updated import (
    state_aware_astar,
    load_weights,
    create_grid,
    SYMBOL_MAP
)


def create_multi_constraint_map():
    """
    Create a map with multiple regions for multi-constraint LTL testing.
    
    Map layout (11x11):
    -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
    -1  0  0  0  a  0  0  b  0  0 -1
    -1  0  0  0  0  0  0  0  0  0 -1
    -1  0  0  0  0  0  0  0  0  0 -1
    -1  0  0  0  0  c  0  0  0  0 -1
    -1  0  0  0  0  0  0  0  0  0 -1
    -1  0  d  0  0  0  0  0  0  0 -1
    -1  0  0  0  0  0  0  0  0  0 -1
    -1  0  0  0  0  0  0  e  0  0 -1
    -1  0  0  0  0  0  0  0  0  0 -1
    -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
    
    Start: [1, 1], Goal: [9, 9]
    Symbols: a=1, b=2, c=3, d=4, e=5
    """
    grid = np.array([
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1,  0,  0,  0,  1,  0,  0,  2,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  0,  0,  0,  3,  0,  0,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  4,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  5,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
    ])
    
    start = [1, 1]
    goal = [9, 9]
    
    # Map symbols to propositions
    props = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e'}
    symbol_positions = {
        'a': [1, 4],
        'b': [1, 7],
        'c': [4, 5],
        'd': [6, 2],
        'e': [8, 7]
    }
    
    return grid, start, goal, props, symbol_positions


def modified_state_aware_astar(grid, start, goal, weight_map, LTL_expression, symbol_map=None):
    """
    Modified version of state_aware_astar that returns detailed state information.
    
    Args:
        symbol_map: Dict mapping grid integers to proposition names (e.g., {1: 'a', 2: 'b'})
    
    Returns:
        path: List of positions
        symbols: List of symbols encountered
        steps: Number of steps
        state_trace: List of (position, ltl_state, symbol) tuples showing the full state at each step
    """
    # Use provided symbol_map or fall back to default
    if symbol_map is None:
        symbol_map = SYMBOL_MAP
    
    # Call the updated state_aware_astar which handles symbol mapping internally
    path, symbols_produced, steps = state_aware_astar(
        grid, start, goal, weight_map, LTL_expression, 
        state_history={}, symbol_map=symbol_map
    )
    
    # Reconstruct state trace by re-tracing the path with LTL state tracking
    state_trace = []
    if path:
        # Build state trace by tracking LTL states
        current_ltl_state = next(iter(LTL_expression.init.keys())) if LTL_expression else None
        
        for i, pos in enumerate(path):
            # Get symbol at this position
            cell_value = grid[pos[0]][pos[1]]
            
            # Add current state BEFORE transition
            state_trace.append((pos, current_ltl_state, cell_value))
            
            # Update LTL state for next iteration (map integer to proposition name)
            if LTL_expression and i < len(path) - 1:
                cell_props = set()
                if cell_value not in [-1, 0]:
                    # Map integer to proposition name
                    if cell_value in symbol_map:
                        cell_props.add(symbol_map[cell_value])
                    else:
                        cell_props.add(cell_value)
                
                next_state = LTL_expression.next_state(current_ltl_state, cell_props)
                if next_state is not None:
                    current_ltl_state = next_state
        
        return (path, symbols_produced, steps, state_trace)
    else:
        return ([], [], steps, [])


def visualize_ltl_aware_path(grid, start, goal, path, state_trace, ltl_formula, symbol_positions, save_path='ltl_path_visualization.png'):
    """
    Create a comprehensive visualization showing:
    1. The spatial path on the grid
    2. The LTL state at each step
    3. Symbol encounters and state transitions
    
    Args:
        grid: The map grid
        start: Start position [row, col]
        goal: Goal position [row, col]
        path: List of positions in the path
        state_trace: List of (position, ltl_state, symbol) tuples
        ltl_formula: String representation of the LTL formula
        symbol_positions: Dict mapping symbol names to positions
        save_path: Where to save the visualization
    """
    fig = plt.figure(figsize=(18, 8))
    
    # Create two subplots: grid view and state timeline
    ax1 = plt.subplot(1, 2, 1)
    ax2 = plt.subplot(1, 2, 2)
    
    # ===== LEFT PLOT: Grid with path and states =====
    ax1.set_title(f'Spatial Path with LTL States\nFormula: {ltl_formula}', fontsize=14, fontweight='bold')
    
    # Draw the grid
    row = len(grid)
    col = len(grid[0])
    for i in range(row):
        for j in range(col):
            if grid[i][j] == -1: 
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='k'))
            elif grid[i][j] == 1:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='magenta', alpha=0.6))
                ax1.text(j, i, 'a', ha='center', va='center', fontsize=12, fontweight='bold')
            elif grid[i][j] == 2:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='cyan', alpha=0.6))
                ax1.text(j, i, 'b', ha='center', va='center', fontsize=12, fontweight='bold')
            elif grid[i][j] == 3:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='blueviolet', alpha=0.6))
                ax1.text(j, i, 'c', ha='center', va='center', fontsize=12, fontweight='bold')
            elif grid[i][j] == 4:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='yellow', alpha=0.6))
                ax1.text(j, i, 'd', ha='center', va='center', fontsize=12, fontweight='bold')
            elif grid[i][j] == 5:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='k', facecolor='orange', alpha=0.6))
                ax1.text(j, i, 'e', ha='center', va='center', fontsize=12, fontweight='bold')
            else:
                ax1.add_patch(Rectangle((j-0.5, i-0.5), 1, 1, edgecolor='gray', facecolor='w', linewidth=0.5))
    
    # Draw the path with arrows and step numbers
    for i in range(len(path) - 1):
        pos_current = path[i]
        pos_next = path[i + 1]
        
        # Draw arrow from current to next position
        arrow = FancyArrowPatch(
            (pos_current[1], pos_current[0]),
            (pos_next[1], pos_next[0]),
            arrowstyle='->', 
            color='blue', 
            linewidth=2, 
            mutation_scale=20,
            alpha=0.7
        )
        ax1.add_patch(arrow)
        
        # Add step number at every 3rd step to avoid clutter
        if i % 3 == 0:
            mid_x = (pos_current[1] + pos_next[1]) / 2
            mid_y = (pos_current[0] + pos_next[0]) / 2
            ax1.text(mid_x, mid_y, str(i), 
                    bbox=dict(boxstyle='circle', facecolor='white', edgecolor='blue', alpha=0.8),
                    ha='center', va='center', fontsize=8)
    
    # Highlight start and goal
    ax1.add_patch(Rectangle((start[1]-0.5, start[0]-0.5), 1, 1, edgecolor='green', facecolor='green', linewidth=3, alpha=0.5))
    ax1.text(start[1], start[0], 'START', ha='center', va='center', fontsize=10, fontweight='bold', color='white')
    
    ax1.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5), 1, 1, edgecolor='red', facecolor='red', linewidth=3, alpha=0.5))
    ax1.text(goal[1], goal[0], 'GOAL', ha='center', va='center', fontsize=10, fontweight='bold', color='white')
    
    ax1.set_xlim(-1, col)
    ax1.set_ylim(-1, row)
    ax1.set_aspect('equal')
    ax1.invert_yaxis()
    ax1.set_xlabel('Column', fontsize=12)
    ax1.set_ylabel('Row', fontsize=12)
    ax1.grid(True, alpha=0.3)
    
    # ===== RIGHT PLOT: State transition timeline =====
    ax2.set_title('LTL State Progression Over Time', fontsize=14, fontweight='bold')
    
    if state_trace and len(state_trace) > 0:
        # Extract unique states in order
        states_seen = []
        state_changes = []  # [(step, old_state, new_state, symbol)]
        
        prev_state = None
        for i, (pos, ltl_state, symbol) in enumerate(state_trace):
            if ltl_state != prev_state:
                state_changes.append((i, prev_state, ltl_state, symbol))
                if ltl_state not in [s[1] for s in states_seen]:
                    states_seen.append((len(states_seen), ltl_state))
                prev_state = ltl_state
        
        # Only plot if we have states to show
        if len(states_seen) > 0:
            # Create timeline visualization
            steps = list(range(len(state_trace)))
            state_indices = []
            
            # Map each step to a state index
            for i, (pos, ltl_state, symbol) in enumerate(state_trace):
                # Find which state index this corresponds to
                for idx, (state_idx, state_name) in enumerate(states_seen):
                    if state_name == ltl_state:
                        state_indices.append(idx)
                        break
            
            # Plot state progression as a step function
            ax2.step(steps, state_indices, where='post', linewidth=2, color='darkblue')
            ax2.scatter(steps, state_indices, c='darkblue', s=50, zorder=5)
            
            # Annotate state transitions with symbols
            for step, old_state, new_state, symbol in state_changes:
                if step > 0 and symbol != 0:  # Only show when symbol was encountered
                    symbol_map = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e', 6: 'f', 7: 'g', 8: 'h'}
                    symbol_name = symbol_map.get(symbol, '?')
                    
                    # Find y-position for this transition
                    y_pos = None
                    for idx, (state_idx, state_name) in enumerate(states_seen):
                        if state_name == new_state:
                            y_pos = idx
                            break
                    
                    if y_pos is not None:
                        ax2.annotate(
                            f'saw "{symbol_name}"',
                            xy=(step, y_pos),
                            xytext=(step + 2, y_pos + 0.3),
                            arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
                            fontsize=10,
                            color='red',
                            fontweight='bold',
                            bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7)
                        )
            
            # Set y-axis labels to state names
            ax2.set_yticks(range(len(states_seen)))
            ax2.set_yticklabels([f'State {i}\n({name})' for i, name in states_seen], fontsize=9)
            ax2.set_xlabel('Step Number', fontsize=12)
            ax2.set_ylabel('LTL Automaton State', fontsize=12)
            ax2.grid(True, alpha=0.3, axis='x')
            ax2.set_xlim(-1, len(steps))
            
            # Add info box
            info_text = f"Total Steps: {len(path)}\n"
            info_text += f"State Transitions: {len(state_changes) - 1}\n"
            info_text += f"Symbols Encountered: {len([s for s in state_trace if s[2] != 0])}"
            ax2.text(
                0.98, 0.02, info_text,
                transform=ax2.transAxes,
                fontsize=10,
                verticalalignment='bottom',
                horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            )
        else:
            # No states to visualize
            ax2.text(0.5, 0.5, 'No LTL state tracking\n(No LTL constraints applied)', 
                    ha='center', va='center', fontsize=14, transform=ax2.transAxes)
    else:
        # No state trace available
        ax2.text(0.5, 0.5, 'No LTL state tracking\n(No LTL constraints applied)', 
                ha='center', va='center', fontsize=14, transform=ax2.transAxes)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {save_path}")
    
    return fig


def run_multi_constraint_visualization():
    """Run a multi-constraint LTL problem and visualize the result."""
    print("="*80)
    print("MULTI-CONSTRAINT LTL PATH VISUALIZATION")
    print("(Using updated NetworkX-compatible automata module)")
    print("="*80)
    
    # Create the map
    grid, start, goal, props, symbol_positions = create_multi_constraint_map()
    
    # Test different LTL formulas
    test_cases = [
        ("!a && F b", "Eventually visit both 'a' and 'b'"),
        ("!a && !b", "Visit 'a', then eventually visit 'b'"),
        ("F a && F b && F c", "Eventually visit 'a', 'b', and 'c'"),
    ]
    
    for ltl_formula, description in test_cases:
        print(f"\n{'='*80}")
        print(f"Testing: {ltl_formula}")
        print(f"Description: {description}")
        print(f"{'='*80}")
        
        # Create LTL expression using UPDATED Fsa class
        try:
            ltl_expression = Fsa()
            ltl_expression.from_formula(ltl_formula)
            
            print(f"\nFSA Details:")
            print(f"  States: {len(ltl_expression.g.nodes())}")
            print(f"  Initial state: {list(ltl_expression.init.keys())}")
            print(f"  Accepting states: {ltl_expression.final}")
            print(f"  Alphabet: {ltl_expression.props}")
            
        except Exception as e:
            print(f"✗ Failed to create FSA: {e}")
            import traceback
            traceback.print_exc()
            continue
        
        # Create uniform weights file
        import csv
        weights_file = 'temp_visualization_weights.csv'
        weights = {}
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i, j] != -1:
                    weights[(i, j)] = 0.0
        
        with open(weights_file, 'w', newline='') as f:
            writer = csv.writer(f)
            for (x, y), weight in weights.items():
                writer.writerow([x, y, weight])
        
        # Run A* with state tracking
        print(f"\nRunning A* with LTL constraints...")
        try:
            path, symbols, steps, state_trace = modified_state_aware_astar(
                grid, start, goal, weights_file, ltl_expression
            )
            
            if path:
                print(f"\n✓ Path found!")
                print(f"  Total steps: {steps}")
                print(f"  Path length: {len(path)}")
                print(f"  Symbols encountered: {[props.get(s, s) for s in symbols if s != 0]}")
                
                # Count state transitions
                state_transitions = 0
                prev_state = None
                for pos, ltl_state, symbol in state_trace:
                    if ltl_state != prev_state and prev_state is not None:
                        state_transitions += 1
                    prev_state = ltl_state
                
                print(f"  LTL state transitions: {state_transitions}")
                
                # Create visualization
                safe_formula = ltl_formula.replace(' ', '_').replace('&', 'and').replace('|', 'or')
                viz_filename = f'ltl_path_{safe_formula}.png'
                visualize_ltl_aware_path(
                    grid, start, goal, path, state_trace, 
                    ltl_formula, symbol_positions, viz_filename
                )
                
                print(f"\n✓ Detailed path breakdown:")
                unique_states = set()
                for i, (pos, ltl_state, symbol) in enumerate(state_trace):
                    unique_states.add(ltl_state)
                    if symbol != 0:
                        symbol_name = props.get(symbol, '?')
                        print(f"    Step {i}: Position {pos} → Symbol '{symbol_name}' → LTL State: {ltl_state}")
                
                print(f"\n  Unique LTL states visited: {len(unique_states)}")
                
            else:
                print("\n✗ No path found!")
                
        except Exception as e:
            print(f"\n✗ Failed to run A*: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup
            if os.path.exists(weights_file):
                os.remove(weights_file)
    
    print(f"\n{'='*80}")
    print("Visualization complete!")
    print("="*80)


def create_avoidance_test_map():
    """
    Create a map specifically for testing avoidance behaviors.
    The map has symbolic regions that act as "hazards" to avoid.
    
    Map layout (13x13):
    -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
    -1  0  0  0  0  0  0  0  0  0  0  0 -1
    -1  0  a  a  0  0  b  b  0  0  c  0 -1
    -1  0  a  a  0  0  b  b  0  0  c  0 -1
    -1  0  0  0  0  0  0  0  0  0  0  0 -1
    -1  0  0  d  d  d  0  0  e  e  0  0 -1
    -1  0  0  d  d  d  0  0  e  e  0  0 -1
    -1  0  0  d  d  d  0  0  e  e  0  0 -1
    -1  0  0  0  0  0  0  0  0  0  0  0 -1
    -1  0  f  0  0  g  g  0  0  h  h  0 -1
    -1  0  f  0  0  g  g  0  0  h  h  0 -1
    -1  0  0  0  0  0  0  0  0  0  0  0 -1
    -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
    
    Start: [1, 1] (top-left), Goal: [11, 11] (bottom-right)
    Symbols: a=1, b=2, c=3, d=4, e=5, f=6, g=7, h=8
    """
    grid = np.array([
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1],
        [-1,  0,  1,  2,  3,  4,  0,  0,  0,  0,  5,  0, -1],
        [-1,  0,  1,  1,  4,  0,  2,  2,  0,  0,  3,  0, -1],
        [-1,  0,  1,  1,  4,  0,  2,  2,  0,  0,  3,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  3,  0, -1],
        [-1,  2,  2,  4,  4,  4,  5,  5,  5,  5,  5,  0, -1],
        [-1,  0,  0,  4,  4,  4,  0,  0,  5,  5,  0,  0, -1],
        [-1,  0,  0,  4,  4,  4,  0,  0,  5,  5,  0,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1,  0,  6,  0,  0,  7,  7,  0,  0,  8,  8,  0, -1],
        [-1,  0,  6,  0,  0,  7,  7,  0,  0,  8,  8,  0, -1],
        [-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
        [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
    ])
    
    start = [1, 1]
    goal = [11, 11]
    
    # Map symbols to propositions
    props = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e', 6: 'f', 7: 'g', 8: 'h'}
    symbol_positions = {
        'a': [[2, 2], [2, 3], [3, 2], [3, 3]],
        'b': [[2, 6], [2, 7], [3, 6], [3, 7]],
        'c': [[2, 10], [3, 10]],
        'd': [[5, 3], [5, 4], [5, 5], [6, 3], [6, 4], [6, 5], [7, 3], [7, 4], [7, 5]],
        'e': [[5, 8], [5, 9], [6, 8], [6, 9], [7, 8], [7, 9]],
        'f': [[9, 2], [10, 2]],
        'g': [[9, 5], [9, 6], [10, 5], [10, 6]],
        'h': [[9, 9], [9, 10], [10, 9], [10, 10]]
    }
    
    return grid, start, goal, props, symbol_positions


def test_avoidance_scenarios():
    """
    Test scenarios where the planner must avoid symbolic regions.
    These test cases verify that LTL constraints can force the planner
    to navigate around hazardous or forbidden areas.
    """
    print("="*80)
    print("AVOIDANCE SCENARIO TESTING")
    print("Testing: Planner avoiding symbolic regions (hazards)")
    print("="*80)
    
    # Create the avoidance test map
    grid, start, goal, props, symbol_positions = create_avoidance_test_map()
    
    print("\nMap Information:")
    print(f"  Grid size: {grid.shape}")
    print(f"  Start: {start}")
    print(f"  Goal: {goal}")
    print(f"  Symbolic regions (to potentially avoid): {list(props.values())}")
    
    # Test cases for avoidance
    avoidance_test_cases = [
        # Test 1: Avoid ALL symbolic regions - navigate only through empty cells
        ("G !a && G !b && G !c && G !d && G !e && G !f && G !g && G !h", 
         "Avoid all symbolic regions (stay in safe zones only)"),
        
        # Test 2: Avoid specific regions
        ("(!d U c) && (!c U b) && (!b U a) && G !e && G !f && G !g && G !h", 
         "Avoid regions 'd' and 'e' (the large central hazards)"),
        
        # Test 3: Avoid top regions
        ("G !a && G !b && G !c", 
         "Avoid top regions 'a', 'b', and 'c'"),
        
        # Test 4: No constraints (baseline)
        (None,
         "No LTL constraints (shortest path, may enter symbolic regions)"),
    ]
    
    import csv
    
    for i, (ltl_formula, description) in enumerate(avoidance_test_cases):
        print(f"\n{'='*80}")
        print(f"AVOIDANCE TEST {i+1}")
        print(f"Description: {description}")
        if ltl_formula:
            print(f"LTL Formula: {ltl_formula}")
        else:
            print(f"LTL Formula: None (no constraints)")
        print(f"{'='*80}")
        
        # Create LTL expression if specified
        ltl_expression = None
        if ltl_formula:
            try:
                ltl_expression = Fsa()
                ltl_expression.from_formula(ltl_formula)
                
                print(f"\nFSA Details:")
                print(f"  States: {len(ltl_expression.g.nodes())}")
                print(f"  Initial state: {list(ltl_expression.init.keys())}")
                print(f"  Accepting states: {ltl_expression.final}")
                print(f"  Alphabet size: {len(ltl_expression.props)}")
                
            except Exception as e:
                print(f"✗ Failed to create FSA: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # Create uniform weights file
        weights_file = f'temp_avoidance_test_{i}_weights.csv'
        weights = {}
        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                if grid[row, col] != -1:
                    weights[(row, col)] = 0.0
        
        with open(weights_file, 'w', newline='') as f:
            writer = csv.writer(f)
            for (x, y), weight in weights.items():
                writer.writerow([x, y, weight])
        
        # Run A* with or without LTL constraints
        print(f"\nRunning A* {'with' if ltl_expression else 'without'} LTL constraints...")
        try:
            path, symbols, steps, state_trace = modified_state_aware_astar(
                grid, start, goal, weights_file, ltl_expression
            )
            
            if path:
                print(f"\n✓ Path found!")
                print(f"  Total steps: {steps}")
                print(f"  Path length: {len(path)}")
                
                # Analyze which symbols were encountered
                symbols_encountered = [s for s in symbols if s != 0]
                unique_symbols = set(symbols_encountered)
                
                if unique_symbols:
                    symbol_names = [props.get(s, f'unknown_{s}') for s in unique_symbols]
                    print(f"  ✗ Symbolic regions entered: {symbol_names}")
                    print(f"     (Entered {len(symbols_encountered)} symbolic cells total)")
                else:
                    print(f"  ✓ NO symbolic regions entered! (Perfect avoidance)")
                
                # Count state transitions if LTL is used
                if ltl_expression and state_trace:
                    state_transitions = 0
                    prev_state = None
                    for pos, ltl_state, symbol in state_trace:
                        if ltl_state != prev_state and prev_state is not None:
                            state_transitions += 1
                        prev_state = ltl_state
                    
                    print(f"  LTL state transitions: {state_transitions}")
                    unique_states = set([s[1] for s in state_trace])
                    print(f"  Unique LTL states visited: {len(unique_states)}")
                
                # Create visualization
                if ltl_formula:
                    safe_formula = ltl_formula[:50].replace(' ', '_').replace('&', 'and').replace('!', 'not')
                    viz_filename = f'avoidance_test_{i+1}_{safe_formula}.png'
                else:
                    viz_filename = f'avoidance_test_{i+1}_no_constraints.png'
                
                visualize_ltl_aware_path(
                    grid, start, goal, path, state_trace, 
                    ltl_formula if ltl_formula else "No LTL Constraints", 
                    symbol_positions, viz_filename
                )
                
                # Detailed path analysis for avoidance verification
                if ltl_formula and "G !" in ltl_formula:
                    print(f"\n  Avoidance Verification:")
                    forbidden_symbols = set()
                    # Parse which symbols should be avoided
                    for prop_name in props.values():
                        if f"G !{prop_name}" in ltl_formula or f"!{prop_name}" in ltl_formula:
                            forbidden_symbols.add(prop_name)
                    
                    violations = []
                    for i, pos in enumerate(path):
                        cell_val = grid[pos[0]][pos[1]]
                        if cell_val in props:
                            cell_name = props[cell_val]
                            if cell_name in forbidden_symbols:
                                violations.append((i, pos, cell_name))
                    
                    if violations:
                        print(f"     ✗ VIOLATIONS FOUND: {len(violations)} forbidden cells entered")
                        for step, pos, name in violations[:5]:  # Show first 5
                            print(f"        Step {step}: Entered '{name}' at {pos}")
                        if len(violations) > 5:
                            print(f"        ... and {len(violations) - 5} more violations")
                    else:
                        print(f"     ✓ PERFECT AVOIDANCE: All forbidden regions successfully avoided!")
                
            else:
                print("\n✗ No path found!")
                print("   (LTL constraints may be too restrictive or goal unreachable)")
                
        except Exception as e:
            print(f"\n✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup
            if os.path.exists(weights_file):
                os.remove(weights_file)
    
    print(f"\n{'='*80}")
    print("Avoidance testing complete!")
    print("="*80)


if __name__ == '__main__':
    # Run avoidance tests
    test_avoidance_scenarios()
    
    # Optionally run the original multi-constraint tests
    # Uncomment the line below to run both test suites
    # run_multi_constraint_visualization()
