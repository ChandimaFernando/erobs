"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import numpy as np
import torch
import matplotlib.pyplot as plt
import os
import cv2
import argparse
from segment_anything import sam_model_registry, SamPredictor
from typing import Any, Dict, List
import pdb


parser = argparse.ArgumentParser(
    description=(
        'Runs prompt-based mask generation on an input image or directory of images,'
        'and outputs masks as either PNGs or COCO-style RLEs. Requires open-cv.'
    )
)

parser.add_argument(
    '--input',
    type=str,
    required=True,
    help='Path to either a single input image or folder of images.',
)

parser.add_argument(
    '--output',
    type=str,
    required=True,
    help=(
        'Path to the directory where masks will be output. Output will be either a folder '
        'of PNGs per image or a single json with COCO-style masks.'
    ),
)

parser.add_argument(
    '--model-type',
    type=str,
    default='vit_h',
    help="The type of model to load, in ['default', 'vit_h', 'vit_l', 'vit_b']",
)

parser.add_argument(
    '--point-x',
    type=int,
    default=500,
    help='This is tricky. This is the x coordinate of the object to be masked in the image.',
)

parser.add_argument(
    '--point-y',
    type=int,
    default=500,
    help='This is tricky. This is the y coordinate of the object to be masked in the image.',
)

parser.add_argument(
    '--checkpoint',
    type=str,
    default='weights/sam_vit_h_4b8939.pth',
    help="The path to the SAM checkpoint to use for mask generation.",
)

parser.add_argument('--device', type=str, default='cuda', help='The device to run generation on.')
 

def write_masks_to_folder(masks, path: str, fname: str) -> None:
    """Write masks as B/W images."""
    pdb.set_trace()
    for i, mask_data in enumerate(masks):
        filename = f"{fname}_{i}.png"
        cv2.imwrite(os.path.join(path, filename), mask_data * 255)

    return


def main(args: argparse.Namespace) -> None:
    """Do sam here. It takes two critical arguments. x-y coordinate of the location of the object in the mask"""
    print('Loading model...')
    sam = sam_model_registry[args.model_type](checkpoint=args.checkpoint)
    _ = sam.to(device=args.device)

    predictor = SamPredictor(sam)

    # Prepare the inputs here to be read
    if not os.path.isdir(args.input):
        targets = [args.input]
    else:
        targets = [
            f for f in os.listdir(args.input) if not os.path.isdir(os.path.join(args.input, f))
        ]
        targets = [os.path.join(args.input, f) for f in targets]

    os.makedirs(args.output, exist_ok=True)

    for t in targets:
        print(f"Processing '{t}'...")
        image = cv2.imread(t)
        if image is None:
            print(f"Could not load '{t}' as an image, skipping...")
            continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        predictor.set_image(image)

        input_point = np.array([[args.point_x, args.point_y]])
        input_label = np.array([1])

        masks, scores, logits = predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=True,
        )

        pdb.set_trace()
        base = os.path.basename(t)
        base = os.path.splitext(base)[0]

        write_masks_to_folder(masks, args.output, base)

    print("Done!")


if __name__ == "__main__":
    args = parser.parse_args()
    main(args)