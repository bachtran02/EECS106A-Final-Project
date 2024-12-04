#!/usr/bin/env python

import cv2
import numpy as np
from PIL import Image

def process_image(image_path, canvas_width, canvas_height):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Get original dimensions and compute aspect ratio
    original_height, original_width = img.shape
    aspect_ratio = original_width / original_height

    # Calculate new dimensions while maintaining aspect ratio
    if canvas_width / canvas_height > aspect_ratio:
        new_height = canvas_height
        new_width = int(aspect_ratio * canvas_height)
    else:
        new_width = canvas_width
        new_height = int(canvas_width / aspect_ratio)
    
    # Resize the image to fit within the canvas
    resized_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    # Convert to a PIL image and apply Floyd-Steinberg dithering
    pil_image = Image.fromarray(resized_img)
    dithered_image = pil_image.convert("1")  # Convert to black-and-white using dithering

    return dithered_image

def print_ascii_art(image):
    # Convert the image to a binary array (0 for white, 1 for black)
    binary_array = (np.array(image) == 0).astype(int)  # 0 is black in "1" mode

    # Generate the ASCII art
    ascii_art = "\n".join("".join("#" if pixel == 1 else " " for pixel in row) for row in binary_array)

    # Print the ASCII art
    print(ascii_art)

# Example usage
image_path = "../../assets/img/smiley.jpg"   # Add image path
canvas_width, canvas_height = 50, 50       # Example canvas size in pixels
dithered_image = process_image(image_path, canvas_width, canvas_height)
# dithered_image.show()
print_ascii_art(dithered_image)