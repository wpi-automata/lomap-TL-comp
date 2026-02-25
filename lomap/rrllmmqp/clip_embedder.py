#!/usr/bin/env python3
"""
CLIP Embedding Service
This script runs in a Python 3.9+ environment (clipenv) to handle CLIP embeddings.
It can be called from Python 3.7 scripts via subprocess.
"""
import json
import sys
import torch
import torch.nn.functional as F
from transformers import AutoTokenizer, CLIPTextModel
import numpy as np

def embed_labels(dict_data, model, tokenizer):
    """Embed labels using CLIP model"""
    with torch.no_grad():
        for label in dict_data.keys():
            # Skip empty labels and the empty set representation
            if label and label != f'{{}}' and label != '{}':
                encoded_input = tokenizer(label, return_tensors='pt', padding=True, truncation=True)
                model_output = model(**encoded_input)
                if hasattr(model_output, 'pooler_output') and model_output.pooler_output is not None:
                    embedding = model_output.pooler_output
                else:
                    embedding = model_output.last_hidden_state.mean(dim=1)
                embedding = F.normalize(embedding, p=2, dim=1)
                # Convert to numpy for JSON serialization
                dict_data[label]['embedding'] = embedding.squeeze(0).numpy().tolist()
    return dict_data

def main():
    """Main function to process embeddings via stdin/stdout"""
    if len(sys.argv) < 2:
        print("Usage: clip_embedder.py <model_name>", file=sys.stderr)
        sys.exit(1)
    
    model_name = sys.argv[1]
    
    # Load model and tokenizer
    try:
        model = CLIPTextModel.from_pretrained(model_name)
        tokenizer = AutoTokenizer.from_pretrained(model_name)
    except Exception as e:
        print(json.dumps({"error": f"Failed to load model: {str(e)}"}), file=sys.stderr)
        sys.exit(1)
    
    # Read input JSON from stdin
    try:
        input_data = json.load(sys.stdin)
    except Exception as e:
        print(json.dumps({"error": f"Failed to parse input: {str(e)}"}), file=sys.stderr)
        sys.exit(1)
    
    # Process embeddings
    try:
        result = embed_labels(input_data, model, tokenizer)
        # Output result as JSON
        print(json.dumps(result))
    except Exception as e:
        print(json.dumps({"error": f"Failed to process embeddings: {str(e)}"}), file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()

