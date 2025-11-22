# This script will weight the edges of the product (maybe TS if needed, experiment)

# What to do when objects have multiple risk weights for different risks in output- take max or look at interacting nodes that may result in greater weight
# What to do when long objects like cables extend across the floor 

import json
from ast import literal_eval
from lomap.tests.create_env_and_buchi_product import create_product
from transformers import AutoTokenizer, CLIPTextModel
import torch
import torch.nn.functional as F
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, ArrowStyle
import numpy as np

def visualize_weighted_graph(pa, edge_weights=None, figsize=(16, 12), highlight_path=None):
    """
    Enhanced visualization for graphs with potentially overlapping edges.
    
    Args:
        pa: Product automaton object with .g attribute (NetworkX graph)
        edge_weights: Dictionary mapping edges to weight strings (optional)
        figsize: Figure size tuple
        highlight_path: List of nodes representing a path to highlight (optional)
    """
    G = pa.g
    is_multigraph = isinstance(G, (nx.MultiDiGraph, nx.MultiGraph))
    
    print(f"Graph type: {type(G)}")
    print(f"Is MultiGraph: {is_multigraph}")
    print(f"Number of nodes: {G.number_of_nodes()}")
    print(f"Number of edges: {G.number_of_edges()}")
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # Use spring layout for better node distribution
    pos = nx.spring_layout(G, k=2, iterations=50, seed=42)
    
    # Draw nodes with enhanced styling
    node_colors = []
    node_sizes = []
    for node in G.nodes():
        node_data = G.nodes[node].get('attr_dict', {})
        label = str(node_data.get('label', ''))
        
        # Highlight nodes in the path
        if highlight_path and node in highlight_path:
            node_colors.append('#FFD700')  # Gold for path nodes
            node_sizes.append(1000)
        elif '[' in label and ']' in label:
            node_colors.append('#FF6B6B')  # Red for nodes with propositions
            node_sizes.append(800)
        else:
            node_colors.append('#4ECDC4')  # Teal for other nodes
            node_sizes.append(800)
    
    nx.draw_networkx_nodes(G, pos, 
                          node_color=node_colors,
                          node_size=node_sizes,
                          alpha=0.9,
                          edgecolors='black',
                          linewidths=2,
                          ax=ax)
    
    # Draw node labels
    labels = {}
    for node in G.nodes():
        node_data = G.nodes[node].get('attr_dict', {})
        label = str(node_data.get('label', str(node)))
        if len(label) > 20:
            label = label[:17] + '...'
        labels[node] = label
    
    nx.draw_networkx_labels(G, pos, labels, font_size=8, font_weight='bold', ax=ax)
    
    # Count parallel edges between each node pair
    edge_counter = {}
    all_edges = []
    path_edges = set()
    
    # Create set of edges in the highlighted path
    if highlight_path and len(highlight_path) > 1:
        for i in range(len(highlight_path) - 1):
            path_edges.add((highlight_path[i], highlight_path[i + 1]))
    
    if is_multigraph:
        # For MultiDiGraph, collect all edges with keys
        for u, v, key in G.edges(keys=True):
            edge_key = (u, v)
            if edge_key not in edge_counter:
                edge_counter[edge_key] = []
            edge_counter[edge_key].append(key)
            all_edges.append((u, v, key))
        
        print(f"\nEdge groups (parallel edges):")
        for edge_key, keys in edge_counter.items():
            if len(keys) > 1:
                print(f"  {edge_key}: {len(keys)} parallel edges")
    else:
        # For regular DiGraph, still check for reverse edges
        for u, v in G.edges():
            edge_key = (u, v)
            if edge_key not in edge_counter:
                edge_counter[edge_key] = [0]
            all_edges.append((u, v, 0))
    
    # Draw edges with appropriate curvature
    for edge_info in all_edges:
        if is_multigraph:
            u, v, key = edge_info
            edge_data = G[u][v][key]
            edge_key = (u, v)
            keys = edge_counter[edge_key]
            num_edges = len(keys)
            idx = keys.index(key)
        else:
            u, v, _ = edge_info
            edge_data = G[u][v]
            edge_key = (u, v)
            num_edges = 1
            idx = 0
            # Check if there's a reverse edge
            if G.has_edge(v, u):
                num_edges = 2
                # Assign idx based on whether this is the forward or reverse edge
                if (v, u) in edge_counter and edge_key > (v, u):
                    idx = 1
        
        # Get weight
        weight = edge_data.get('attr_dict', {}).get('weight', None)
        
        # Check if this edge is in the highlighted path
        is_path_edge = (u, v) in path_edges
        
        # Calculate curvature
        if num_edges == 1:
            rad = 0.1  # Slight curve even for single edges
        else:
            # Spread multiple edges with different curvatures
            # Increase the rad multiplier for more visible separation
            rad = 0.4 * ((idx - (num_edges - 1) / 2) / max(1, (num_edges - 1) / 2))
        
        print(f"Edge {u}->{v} (idx={idx}/{num_edges}): rad={rad:.3f}, weight={weight}")
        
        # Get edge color based on weight and path membership
        if is_path_edge:
            color = '#FFD700'  # Gold for path edges
            alpha = 0.95
            linewidth = 4
        elif weight is not None:
            color = plt.cm.RdYlGn_r(weight)
            alpha = 0.6 + 0.4 * weight
            linewidth = 2 + 2 * weight
        else:
            color = 'gray'
            alpha = 0.4
            linewidth = 1.5
        
        # Draw curved arrow with more prominent arrowhead
        arrow = FancyArrowPatch(
            pos[u], pos[v],
            connectionstyle=f"arc3,rad={rad}",
            arrowstyle='-|>',  # Arrow style: line with filled triangle
            mutation_scale=30,  # Larger arrowhead
            linewidth=linewidth,
            color=color,
            alpha=min(alpha + 0.2, 1.0),  # More opaque
            zorder=1,
            shrinkA=18,  # Shrink from start node
            shrinkB=18,   # Shrink from end node
            capstyle='round',
            joinstyle='round'
        )
        ax.add_patch(arrow)
        
        # Add a small circle at the arrowhead for extra visibility
        # Calculate position near the end of the arrow
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        
        # Point 90% along the path for arrow marker
        dx = x2 - x1
        dy = y2 - y1
        
        # For curved edge, approximate the end point
        if abs(rad) > 0.01:
            # Adjust for curvature
            norm = np.sqrt(dx**2 + dy**2)
            if norm > 0:
                perp_x = -dy / norm
                perp_y = dx / norm
                curve_offset = rad * norm * 0.5
                arrow_x = x1 + 0.85 * dx + perp_x * curve_offset
                arrow_y = y1 + 0.85 * dy + perp_y * curve_offset
            else:
                arrow_x = x1 + 0.85 * dx
                arrow_y = y1 + 0.85 * dy
        else:
            arrow_x = x1 + 0.85 * dx
            arrow_y = y1 + 0.85 * dy
        
        # Draw small arrow marker circle
        ax.plot(arrow_x, arrow_y, 'o', 
                color=color, 
                markersize=6, 
                alpha=min(alpha + 0.3, 1.0),
                zorder=2,
                markeredgecolor='black',
                markeredgewidth=0.5)
        
        # Add edge label if weight exists
        if weight is not None:
            # Calculate label position on the curved edge
            x1, y1 = pos[u]
            x2, y2 = pos[v]
            
            # Parametric curve point at t=0.5 (midpoint)
            # For arc3, the curve is a circular arc
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            
            # Offset perpendicular to the line
            dx = x2 - x1
            dy = y2 - y1
            norm = np.sqrt(dx**2 + dy**2)
            
            if norm > 0:
                # Perpendicular vector
                perp_x = -dy / norm
                perp_y = dx / norm
                
                # Offset amount increases with rad
                offset_amount = rad * norm * 0.3
                
                label_x = mid_x + perp_x * offset_amount
                label_y = mid_y + perp_y * offset_amount
            else:
                label_x = mid_x
                label_y = mid_y
            
            # Draw label with background
            ax.text(label_x, label_y, f'{weight:.3f}',
                   bbox=dict(boxstyle='round,pad=0.3', 
                           facecolor='white', 
                           edgecolor=color,
                           alpha=0.95,
                           linewidth=1.5),
                   fontsize=8,
                   ha='center',
                   va='center',
                   weight='bold',
                   zorder=3)
    
    # Add colorbar for weight scale
    sm = plt.cm.ScalarMappable(cmap=plt.cm.RdYlGn_r, 
                               norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('Edge Weight (Risk)', rotation=270, labelpad=20, fontsize=10)
    
    graph_type = "MultiDiGraph" if is_multigraph else "DiGraph"
    title = f'Weighted Product Automaton ({graph_type})\n'
    if highlight_path:
        title += 'Gold: Shortest path | '
    title += 'Red nodes: with propositions | Teal nodes: without'
    
    ax.set_title(title, fontsize=12, fontweight='bold', pad=20)
    ax.axis('off')
    plt.tight_layout()
    
    return fig, ax

    
def embed_labels(dict, model, tokenizer):
    with torch.no_grad():
            for label in dict.keys():
                # Skip empty labels and the empty set representation
                if label and label != f'{{}}' and label != '{}':
                    encoded_input = tokenizer(label, return_tensors='pt', padding=True, truncation=True)
                    model_output = model(**encoded_input)
                    if hasattr(model_output, 'pooler_output') and model_output.pooler_output is not None:
                        embedding = model_output.pooler_output
                    else:
                        embedding = model_output.last_hidden_state.mean(dim=1)
                    embedding = F.normalize(embedding, p=2, dim=1)
                    dict[label]['embedding'] = embedding.squeeze(0)
    return dict

def parse_abbrev_label(pa):
    pa_dict = {}
    for node, data in pa.g.nodes(data=True):
        if 'attr_dict' in data:
            attr = data['attr_dict']
            prop_set = attr.get('prop', set())
            abbrev_label = attr.get('abbrev_label', '')
            
            # Extract proposition string from abbrev_label (e.g., "({'a'}, 'T0_init')" -> 'a')
            prop_string = ''
            if abbrev_label:
                try:
                    # Parse the string representation of tuple: "({'a'}, 'T0_init')" -> ({'a'}, 'T0_init')
                    parsed_tuple = literal_eval(abbrev_label)
                    # Get first element (the set) and extract the proposition(s)
                    prop_set_from_label = parsed_tuple[0] if parsed_tuple else set()
                    # Get the first (or only) proposition, or join if multiple
                    if prop_set_from_label:
                        prop_string = ', '.join(sorted(prop_set_from_label)) if len(prop_set_from_label) > 1 else next(iter(prop_set_from_label))
                except (ValueError, SyntaxError, IndexError):
                    # Fallback: use prop_set directly if parsing fails
                    prop_string = ', '.join(sorted(prop_set)) if len(prop_set) > 1 else (next(iter(prop_set)) if prop_set else '')
            else:
                # If no abbrev_label, use prop_set directly
                prop_string = ', '.join(sorted(prop_set)) if len(prop_set) > 1 else (next(iter(prop_set)) if prop_set else '')
            
            if prop_string not in pa_dict:
                pa_dict[prop_string] = {}
            

    return pa_dict

def extract_prop_from_label(label_string):
    """Extract proposition from label string like "('1', 'T0_init')\n['a']" -> 'a'
    Returns the proposition in the same format as parse_abbrev_label (sorted, comma-separated string)
    """
    label_str = str(label_string)
    
    # Method 1: Try to extract from the list part after newline
    if '\n' in label_str and '[' in label_str:
        parts = label_str.split('\n')
        if len(parts) > 1:
            list_part = parts[1].strip()
            try:
                # Parse the list: ['a'] or ['d', 'e', 'l', 'w']
                prop_list = literal_eval(list_part)
                if prop_list:
                    # Format exactly like parse_abbrev_label: sorted, comma-space separated
                    if len(prop_list) > 1:
                        return ', '.join(sorted(prop_list))
                    else:
                        return prop_list[0] if isinstance(prop_list[0], str) else str(prop_list[0])
            except (ValueError, SyntaxError, TypeError):
                pass
    
    # Method 2: Try to extract from bracket notation directly
    if '[' in label_str:
        # Find the content between [ and ]
        start_idx = label_str.rfind('[')
        end_idx = label_str.rfind(']')
        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            list_content = label_str[start_idx:end_idx + 1]
            try:
                prop_list = literal_eval(list_content)
                if prop_list:
                    if len(prop_list) > 1:
                        return ', '.join(sorted(prop_list))
                    else:
                        return prop_list[0] if isinstance(prop_list[0], str) else str(prop_list[0])
            except (ValueError, SyntaxError, TypeError):
                pass
    
    return ''

def weight_edges(pa, similarity_dict, risk_dict):
    import networkx as nx
    is_multigraph = isinstance(pa.g, (nx.MultiDiGraph, nx.MultiGraph))
    
    if is_multigraph:
        # Handle MultiDiGraph - edges have keys: (u, v, key)
        for u, v, key, edge_data in pa.g.edges(keys=True, data=True):
            print("Edge: ", (u, v, key))
            # Find the labels of the node the edge goes to 
            to_label = pa.g.nodes[v]['attr_dict']['label'] 
            print("To label: ", to_label)
            # Extract proposition from label string
            prop_string = extract_prop_from_label(to_label)
            print("Extracted prop: ", prop_string)
            # Find the similarity between the labels
            if prop_string in similarity_dict:
                # get the max similarity score
                max_similarity = max(similarity_dict[prop_string][0].values())
                max_similarity_label = max(similarity_dict[prop_string][0].keys())
                print("Max similarity: ", max_similarity)
                print("Max similarity label: ", max_similarity_label)
                # Now we need to find the object in risk_dict
                risk_label = risk_dict[max_similarity_label]['danger_score']
                print("Risk label: ", risk_label)
                # Now we need to update the weight of the edge
                pa.g[u][v][key]['attr_dict']['weight'] = risk_label
                print(f"Updated weight to: {pa.g[u][v][key]['attr_dict'].get('weight', 'N/A')}")
            else:
                print(f"No similarity found for prop '{prop_string}'")
    else:
        # Handle regular DiGraph - edges are just (u, v)
        for u, v, edge_data in pa.g.edges(data=True):
            print("Edge: ", (u, v))
            # Find the labels of the node the edge goes to 
            to_label = pa.g.nodes[v]['attr_dict']['label'] 
            print("To label: ", to_label)
            # Extract proposition from label string
            prop_string = extract_prop_from_label(to_label)
            print("Extracted prop: ", prop_string)
            # Find the similarity between the labels
            if prop_string in similarity_dict:
               # get the max similarity score
                max_similarity = max(similarity_dict[prop_string][0].values())
                max_similarity_label = max(similarity_dict[prop_string][0].keys())
                print("Max similarity: ", max_similarity)
                print("Max similarity label: ", max_similarity_label)
                # Now we need to find the object in risk_dict
                risk_label = risk_dict[max_similarity_label]['danger_score']
                print("Risk label: ", risk_label)
                # Now we need to update the weight of the edge
                pa.g[u][v][key]['attr_dict']['weight'] = risk_label
                print(f"Updated weight to: {pa.g[u][v][key]['attr_dict'].get('weight', 'N/A')}")
            else:
                print(f"No similarity found for prop '{prop_string}'")
    return pa

def main():
    # Get sample paranoia data 
    with open('paranoia.json', 'r') as file:
        data = json.load(file)

    # Put objects in dictionary with associated risk value
    risk_dict = {}
    for item in data:
        for object in item['objects']:
            if object not in risk_dict: 
                risk_dict[object] = {'danger_score': item['danger_score'], 'embedding': None}
            
            else: 
                item['danger_score'] = item['danger_score'] + risk_dict[object]['danger_score'] # Sum the danger scores for each object, also may need to change...

    print(risk_dict)

    # Define example specification and calculate product automaton
    # TODO: Change map to match paranoia output 
    spec = '(F coffee)'
    shortest_word, pa = create_product('maps/unit_test_maps/alphabetical_maps/example9 copy.csv', '{}', spec, display=False)

    # Now organize the labels in a dict so we can encode them and get their relationship
    pa_dict = parse_abbrev_label(pa)
    #print(pa_dict)

    # # Use CLIP to embed the product labels and paranoia output 
    model = CLIPTextModel.from_pretrained("openai/clip-vit-base-patch32") # TODO: Change pre-trained model if there's a better fit 
    tokenizer = AutoTokenizer.from_pretrained("openai/clip-vit-base-patch32")

    pa_dict = embed_labels(pa_dict, model, tokenizer)
    #print(pa_dict)

    risk_dict = embed_labels(risk_dict, model, tokenizer)
    #print(risk_dict)


    # Calculate cosine similarity between the two embeddings
    # If cosine similarity is less than some threshold, can assume objects not the same
    similarity_dict = {}
    for pa_label, pa_data in pa_dict.items():
        # Skip if no embedding was created (e.g., empty label)
        if 'embedding' not in pa_data:
            continue
        for risk_label, risk_data in risk_dict.items():
            # Skip if no embedding was created
            if 'embedding' not in risk_data:
                continue
            similarity = F.cosine_similarity(
                pa_data['embedding'].unsqueeze(0),
                risk_data['embedding'].unsqueeze(0)
            )
            print(f"Cosine similarity between '{pa_label}' and '{risk_label}': {similarity.item():.4f}")
            if similarity.item() > 0.65:
                print(f"Objects '{pa_label}' and '{risk_label}' are the same")
                if pa_label not in similarity_dict:
                    similarity_dict[pa_label] = []
                similarity_dict[pa_label].append({risk_label: similarity.item()})
    
    print(similarity_dict)

    # Weight the edges of the product automaton based on the cosine similarity
    # Return the weighted product automaton
    pa = weight_edges(pa, similarity_dict, risk_dict)
    
    # Extract edge weights for visualization
    # At the end of your main() function, replace the visualization with:

    import networkx as nx

    # Check what type of graph you actually have
    print(f"\\nProduct automaton graph type: {type(pa.g)}")
    print(f"Is MultiDiGraph: {isinstance(pa.g, nx.MultiDiGraph)}")

    # Get the shortest path
    labels = {n: d['attr_dict']['abbrev_label'] for n, d in pa.g.nodes.items() if ('attr_dict' in d and 'abbrev_label' in d['attr_dict'])}
    shortest_trajectory = None
    for f in iter(pa.final):
        trajectory = nx.shortest_path(pa.g, source=next(iter(pa.init)), target=f)
        if (not shortest_trajectory or len(trajectory) < len(shortest_trajectory)) and \
        (literal_eval(labels.get(f))[0] != '{}'): # has !{} because can't run policy for {} so can't get to final node if {}
            shortest_trajectory = trajectory

    print('Shortest Trajectory: ', shortest_trajectory)
    
    # Calculate path weight manually since weights are in attr_dict
    def calculate_path_weight(graph, path, weight_key='weight'):
        """Calculate total weight of a path, handling MultiDiGraph with attr_dict"""
        total_weight = 0.0
        is_multigraph = isinstance(graph, (nx.MultiDiGraph, nx.MultiGraph))
        
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            if is_multigraph:
                # For MultiDiGraph, get the minimum weight edge (or first if multiple)
                edges = graph[u][v]
                min_weight = float('inf')
                for key, edge_data in edges.items():
                    weight = edge_data.get('attr_dict', {}).get(weight_key, 1.0)
                    min_weight = min(min_weight, weight)
                total_weight += min_weight if min_weight != float('inf') else 1.0
            else:
                # For regular DiGraph
                edge_data = graph[u][v]
                weight = edge_data.get('attr_dict', {}).get(weight_key, 1.0)
                total_weight += weight
        
        return total_weight
    
    path_weight = calculate_path_weight(pa.g, shortest_trajectory)
    print('Shortest Trajectory Total Weight: ', path_weight)
    print('Shortest Trajectory Node Labels: ', [labels.get(node) for node in shortest_trajectory])

    # Visualize with the shortest path highlighted
    fig, ax = visualize_weighted_graph(pa, highlight_path=shortest_trajectory)
    plt.show()


if __name__ == '__main__':
    main()