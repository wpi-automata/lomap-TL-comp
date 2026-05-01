"""
Visualization module for displaying paths over occupancy grids, labels, and weights.
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm

OBS_THRESHOLD = 80

def visualize_path(occupancy_grid, symbol_grid, weights, path, start=None, goal=None, 
                  title="Path Visualization", save_path=None, show_labels=True, 
                  show_weights=True, figsize=(15, 12)):
    """
    Visualize a path overlaid on occupancy grid, labels, and weights.
    
    Args:
        occupancy_grid: 2D list/array of occupancy values (0-100, >80 = obstacle)
        symbol_grid: 2D list/array of symbol labels (strings)
        weights: Dictionary mapping (row, col) tuples to weight values, or 2D array
        path: List of [row, col] coordinates representing the path
        start: Optional [row, col] start position (will be marked)
        goal: Optional [row, col] goal position (will be marked)
        title: Title for the plot
        save_path: Optional path to save the figure
        show_labels: Whether to show label colors
        show_weights: Whether to show weight heatmap
        figsize: Figure size tuple
    """
    # Convert to numpy arrays for easier manipulation
    og = np.array(occupancy_grid)
    sg = np.array(symbol_grid, dtype=object)
    
    rows, cols = og.shape
    
    # Create figure with subplots
    fig, axes = plt.subplots(1, 2, figsize=figsize)
    ax1 = axes[0]  # Combined view
    ax2 = axes[1]  # Weights only view
    
    # === Combined View (ax1) ===
    
    # Create base image: occupancy grid
    base_image = np.ones((rows, cols, 3))  # White background
    
    # Mark obstacles in black
    obstacles = og > OBS_THRESHOLD
    base_image[obstacles] = [0, 0, 0]  # Black for obstacles
    
    # Overlay labels with colors if enabled
    if show_labels:
        # Get unique symbols (excluding empty/obstacle markers)
        unique_symbols = set()
        for i in range(rows):
            for j in range(cols):
                if not obstacles[i, j]:
                    symbol = str(sg[i, j]).strip()
                    if symbol and symbol != '{}' and symbol != '-1' and symbol != '':
                        unique_symbols.add(symbol)
        
        # Create color map for symbols
        symbol_colors = {}
        if unique_symbols:
            # Use a colormap to assign colors
            cmap = cm.get_cmap('tab20')
            for idx, symbol in enumerate(sorted(unique_symbols)):
                color = cmap(idx % 20)
                symbol_colors[symbol] = color[:3]  # RGB only
        
        # Apply label colors
        for i in range(rows):
            for j in range(cols):
                if not obstacles[i, j]:
                    symbol = str(sg[i, j]).strip()
                    if symbol and symbol != '{}' and symbol != '-1' and symbol != '':
                        if symbol in symbol_colors:
                            base_image[i, j] = symbol_colors[symbol]
    
    # Overlay weights as transparency/alpha if enabled
    if show_weights and weights:
        # Convert weights dict to 2D array if needed
        if isinstance(weights, dict):
            weight_array = np.zeros((rows, cols))
            for (r, c), w in weights.items():
                if 0 <= r < rows and 0 <= c < cols:
                    weight_array[r, c] = w
        else:
            weight_array = np.array(weights)
        
        # Normalize weights to 0-1 range for alpha
        if weight_array.max() > weight_array.min():
            normalized_weights = (weight_array - weight_array.min()) / (weight_array.max() - weight_array.min())
        else:
            normalized_weights = np.zeros_like(weight_array)
        
        # Create weight overlay (red tint, darker = higher weight)
        weight_overlay = np.zeros((rows, cols, 3))
        weight_overlay[:, :, 0] = normalized_weights  # Red channel
        weight_overlay[:, :, 1] = normalized_weights * 0.3  # Some green
        weight_overlay[:, :, 2] = normalized_weights * 0.1  # Little blue
        
        # Blend with base image (weighted average)
        alpha = 0.4  # Transparency of weight overlay
        base_image = (1 - alpha) * base_image + alpha * weight_overlay
    
    # Display base image
    ax1.imshow(base_image, origin='lower', interpolation='nearest')
    
    # Draw path
    if path and len(path) > 0:
        path_array = np.array(path)
        # Extract row and column coordinates
        if path_array.ndim == 2 and path_array.shape[1] >= 2:
            path_rows = path_array[:, 0]
            path_cols = path_array[:, 1]
        else:
            # Handle different path formats
            path_rows = [p[0] for p in path if len(p) >= 2]
            path_cols = [p[1] for p in path if len(p) >= 2]
        
        # Draw path as a single line (no dots) for a cleaner look
        ax1.plot(path_cols, path_rows, color='#1D3557', linewidth=3.5, alpha=0.95, label='Path', zorder=5,
                 solid_capstyle='round', solid_joinstyle='round')
    
    # Mark start and goal (both circles, different colors)
    if start is not None and len(start) >= 2:
        ax1.scatter(start[1], start[0], c='#2A9D8F', s=48, marker='o', zorder=6,
                   edgecolors='#1D3557', linewidths=1.5, label='Start')
    if goal is not None and len(goal) >= 2:
        ax1.scatter(goal[1], goal[0], c='#457B9D', s=48, marker='o', zorder=6,
                   edgecolors='#1D3557', linewidths=1.5, label='Goal')
    
    ax1.set_title(f'{title}\n(Occupancy + Labels + Weights + Path)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Column (X)', fontsize=10)
    ax1.set_ylabel('Row (Y)', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    # origin='lower' in imshow places (0,0) at bottom-left (standard mathematical convention)
    
    # === Weights Only View (ax2) ===
    if show_weights and weights:
        # Convert weights dict to 2D array if needed
        if isinstance(weights, dict):
            weight_array = np.zeros((rows, cols))
            for (r, c), w in weights.items():
                if 0 <= r < rows and 0 <= c < cols:
                    weight_array[r, c] = w
        else:
            weight_array = np.array(weights)
        
        # Create heatmap
        im = ax2.imshow(weight_array, cmap='hot', origin='lower', interpolation='nearest')
        
        # Draw obstacles as black overlay
        obstacle_mask = og > OBS_THRESHOLD
        overlay = np.zeros((rows, cols, 4))
        overlay[obstacle_mask, :] = [0, 0, 0, 1]  # Black with full opacity
        ax2.imshow(overlay, origin='lower', interpolation='nearest')
        
        # Draw path on weights view
        if path and len(path) > 0:
            path_array = np.array(path)
            if path_array.ndim == 2 and path_array.shape[1] >= 2:
                path_rows = path_array[:, 0]
                path_cols = path_array[:, 1]
            else:
                path_rows = [p[0] for p in path if len(p) >= 2]
                path_cols = [p[1] for p in path if len(p) >= 2]
            
            ax2.plot(path_cols, path_rows, color='#00F5D4', linewidth=3.5, alpha=0.95, label='Path', zorder=5,
                     solid_capstyle='round', solid_joinstyle='round')
        
        # Mark start (circle) and goal (diamond) on weights view
        if start is not None and len(start) >= 2:
            ax2.scatter(start[1], start[0], c='#2A9D8F', s=48, marker='o', zorder=6,
                       edgecolors='#1D3557', linewidths=1.5, label='Start')
        if goal is not None and len(goal) >= 2:
            ax2.scatter(goal[1], goal[0], c='#457B9D', s=48, marker='o', zorder=6,
                       edgecolors='#1D3557', linewidths=1.5, label='Goal')
        
        ax2.set_title('Weights Heatmap + Path', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Column (X)', fontsize=10)
        ax2.set_ylabel('Row (Y)', fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right')
        # origin='lower' in imshow places (0,0) at bottom-left (standard mathematical convention)
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax2, fraction=0.046, pad=0.04)
        cbar.set_label('Weight Value', rotation=270, labelpad=15)
    else:
        ax2.axis('off')
        ax2.text(0.5, 0.5, 'Weights not available', 
                ha='center', va='center', transform=ax2.transAxes, fontsize=12)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to {save_path}")
    
    plt.show()
    
    return fig


def visualize_path_simple(occupancy_grid, symbol_grid, weights, path, start=None, goal=None,
                          title="Path Visualization", save_path=None, figsize=(12, 10)):
    """
    Simplified single-view visualization.
    
    Args:
        occupancy_grid: 2D list/array of occupancy values
        symbol_grid: 2D list/array of symbol labels
        weights: Dictionary or 2D array of weights
        path: List of [row, col] coordinates
        start: Optional [row, col] start position
        goal: Optional [row, col] goal position
        title: Title for the plot
        save_path: Optional path to save the figure
        figsize: Figure size tuple
    """
    og = np.array(occupancy_grid)
    sg = np.array(symbol_grid, dtype=object)
    rows, cols = og.shape
    
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    
    # Create base image
    base_image = np.ones((rows, cols, 3))
    
    # Mark obstacles
    obstacles = og > OBS_THRESHOLD
    base_image[obstacles] = [0, 0, 0]
    
    # Add labels
    unique_symbols = set()
    for i in range(rows):
        for j in range(cols):
            if not obstacles[i, j]:
                symbol = str(sg[i, j]).strip()
                if symbol and symbol != '{}' and symbol != '-1' and symbol != '':
                    unique_symbols.add(symbol)
    
    symbol_colors = {}
    if unique_symbols:
        cmap = cm.get_cmap('tab20')
        for idx, symbol in enumerate(sorted(unique_symbols)):
            color = cmap(idx % 20)
            symbol_colors[symbol] = color[:3]
    
    for i in range(rows):
        for j in range(cols):
            if not obstacles[i, j]:
                symbol = str(sg[i, j]).strip()
                if symbol and symbol != '{}' and symbol != '-1' and symbol != '':
                    if symbol in symbol_colors:
                        base_image[i, j] = symbol_colors[symbol]
    
    # Add weights overlay
    if weights:
        if isinstance(weights, dict):
            weight_array = np.zeros((rows, cols))
            for (r, c), w in weights.items():
                if 0 <= r < rows and 0 <= c < cols:
                    weight_array[r, c] = w
        else:
            weight_array = np.array(weights)
        
        if weight_array.max() > weight_array.min():
            normalized_weights = (weight_array - weight_array.min()) / (weight_array.max() - weight_array.min())
        else:
            normalized_weights = np.zeros_like(weight_array)
        
        weight_overlay = np.zeros((rows, cols, 3))
        weight_overlay[:, :, 0] = normalized_weights
        weight_overlay[:, :, 1] = normalized_weights * 0.3
        weight_overlay[:, :, 2] = normalized_weights * 0.1
        
        alpha = 0.3
        base_image = (1 - alpha) * base_image + alpha * weight_overlay
    
    ax.imshow(base_image, origin='lower', interpolation='nearest')
    
    # Draw path
    if path and len(path) > 0:
        path_array = np.array(path)
        if path_array.ndim == 2 and path_array.shape[1] >= 2:
            path_rows = path_array[:, 0]
            path_cols = path_array[:, 1]
        else:
            path_rows = [p[0] for p in path if len(p) >= 2]
            path_cols = [p[1] for p in path if len(p) >= 2]
        
        ax.plot(path_cols, path_rows, color='#1D3557', linewidth=3.5, alpha=0.95, label='Path', zorder=5,
                solid_capstyle='round', solid_joinstyle='round')
    
    # Mark start and goal (both circles)
    if start is not None and len(start) >= 2:
        ax.scatter(start[1], start[0], c='#2A9D8F', s=48, marker='o', zorder=6,
                  edgecolors='#1D3557', linewidths=1.5, label='Start')
    if goal is not None and len(goal) >= 2:
        ax.scatter(goal[1], goal[0], c='#457B9D', s=48, marker='o', zorder=6,
                  edgecolors='#1D3557', linewidths=1.5, label='Goal')
    
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Column (X)', fontsize=11)
    ax.set_ylabel('Row (Y)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    # origin='lower' in imshow places (0,0) at bottom-left (standard mathematical convention)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to {save_path}")
    
    plt.show()
    
    return fig

import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib.cm as cm

OBS_THRESHOLD = 80

def load_path_from_csv(csv_file):
    """
    Assumes CSV format: row,col per line
    """
    path = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) >= 2:
                path.append([int(row[0]), int(row[1])])
    return path


def visualize_multiple_paths_from_csv(
    occupancy_grid,
    symbol_grid,
    weights,
    csv_paths,          # list of csv file paths
    labels=None,        # list of names for legend
    start=None,
    goal=None,
    title="Method Comparison: F Storage Closet",
    save_path=None,
    figsize=(12, 10)
):
    og = np.array(occupancy_grid)
    sg = np.array(symbol_grid, dtype=object)

    rows, cols = og.shape

    fig, ax = plt.subplots(1, 1, figsize=figsize)

    # === Base Map (same as your left plot) ===
    base_image = np.ones((rows, cols, 3))

    # Obstacles
    obstacles = og > OBS_THRESHOLD
    base_image[obstacles] = [0, 0, 0]

    # === Label coloring (same logic) ===
    unique_symbols = set()
    for i in range(rows):
        for j in range(cols):
            if not obstacles[i, j]:
                symbol = str(sg[i, j]).strip()
                if symbol and symbol not in ['{}', '-1', '']:
                    unique_symbols.add(symbol)

    symbol_colors = {}
    if unique_symbols:
        cmap = cm.get_cmap('tab20')
        for idx, symbol in enumerate(sorted(unique_symbols)):
            symbol_colors[symbol] = cmap(idx % 20)[:3]

    for i in range(rows):
        for j in range(cols):
            if not obstacles[i, j]:
                symbol = str(sg[i, j]).strip()
                if symbol in symbol_colors:
                    base_image[i, j] = symbol_colors[symbol]

    # === Weight overlay ===
    if weights:
        if isinstance(weights, dict):
            weight_array = np.zeros((rows, cols))
            for (r, c), w in weights.items():
                if 0 <= r < rows and 0 <= c < cols:
                    weight_array[r, c] = w
        else:
            weight_array = np.array(weights)

        if weight_array.max() > weight_array.min():
            norm = (weight_array - weight_array.min()) / (weight_array.max() - weight_array.min())
        else:
            norm = np.zeros_like(weight_array)

        overlay = np.zeros((rows, cols, 3))
        overlay[:, :, 0] = norm
        overlay[:, :, 1] = norm * 0.3
        overlay[:, :, 2] = norm * 0.1

        base_image = 0.7 * base_image + 0.3 * overlay

    ax.imshow(base_image, origin='lower', interpolation='nearest')

    # === Load all paths ===
    paths = [load_path_from_csv(p) for p in csv_paths]

    # === Colors for each path ===
    cmap = cm.get_cmap('tab10')
    colors = [cmap(i % 10) for i in range(len(paths))]

    # === Labels fallback ===
    if labels is None:
        labels = [f"Path {i+1}" for i in range(len(paths))]

   # === Plot each path ===
    for idx, path in enumerate(paths):
        label = labels[idx]

        if not path or len(path) == 0:
            # Add legend entry for missing path
            ax.plot(
                [], [],  # no data
                linestyle='--',
                color='gray',
                linewidth=2,
                label=f"{label} (no path)"
            )
            continue

        path_array = np.array(path)
        rows_p = path_array[:, 0]
        cols_p = path_array[:, 1]

        ax.plot(
            cols_p,
            rows_p,
            linewidth=3.5,
            alpha=0.95,
            color=colors[idx],
            label=label,
            zorder=5,
            solid_capstyle='round',
            solid_joinstyle='round'
        )

    # === Start / Goal ===
    if start is not None:
        ax.scatter(start[1], start[0], c='#2A9D8F', s=60,
                   edgecolors='black', label='Start', zorder=6)

    if goal is not None:
        ax.scatter(goal[1], goal[0], c='#457B9D', s=60,
                   edgecolors='black', label='Goal', zorder=6)

    # === Formatting ===
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Column (X)')
    ax.set_ylabel('Row (Y)')
    ax.grid(True, alpha=0.3)

    ax.legend(loc='upper right')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {save_path}")

    plt.show()

    return fig