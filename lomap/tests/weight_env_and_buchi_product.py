# This script will weight the edges of the product (maybe TS if needed, experiment)

# What to do when objects have multiple risk weights for different risks in output- take max or look at interacting nodes that may result in greater weight
# What to do when long objects like cables extend across the floor 

import json
from lomap.tests.create_env_and_buchi_product import create_product
from transformers import AutoTokenizer, CLIPTextModel
import torch

def main():
    # Get sample paranoia data 
    with open('paranoia.json', 'r') as file:
        data = json.load(file)

    # Define example specification and calculate product automaton
    # TODO: Change map to match paranoia output 
    spec = '(F b)'
    shortest_word, pa = create_product('maps/unit_test_maps/alphabetical_maps/example9.csv', '{}', spec, display=False)


    # Use CLIP to embed the product labels and paranoia output 
    model = CLIPTextModel.from_pretrained("openai/clip-vit-base-patch32") # TODO: Change pre-trained model if there's a better fit 
    tokenizer = AutoTokenizer.from_pretrained("openai/clip-vit-base-patch32")

    # Calculate cosine similarity between the two embeddings
        # If cosine similarity is less than some threshold, can assume objects not the same
    # Weight the edges of the product automaton based on the cosine similarity
    # Return the weighted product automaton



if __name__ == '__main__':
    main()