import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import itertools

def create_rectangle_dots(tags: list, image_path, offset=0.5, dot_dist=0.1):

    assert len(tags) >= 3
    
    x1, y1, _ = tags[0]
    x2, y2, _ = tags[1]
    x3, y3, _ = tags[2]
    
    x4 = x2 + (x3 - x1)
    y4 = y2 + (y3 - y1)

    x12 = x1 - x2
    y12 = y1 - y2
    x13 = x1 - x3
    y13 = y1 - y3
    v12 = np.sqrt((x12)**2 + (y12)**2)
    v13 = np.sqrt((x13)**2 + (y13)**2)

    # Swap the sides if necessary, ensuring (x1, y1) to (x2, y2) is the longer side
    if v12 < v13:
        x2, y2, x3, y3 = x3, y3, x2, y2
        x12 = x1 - x2
        y12 = y1 - y2
        x13 = x1 - x3
        y13 = y1 - y3
        v12 = np.sqrt((x12)**2 + (y12)**2)
        v13 = np.sqrt((x13)**2 + (y13)**2)

    x1_ = x1 - offset*x12/v12
    y1_ = y1 - offset*y12/v12
    x2_ = x2 + offset*x12/v12
    y2_ = y2 + offset*y12/v12

    xp1 = x1_ - offset*x13/v13
    yp1 = y1_ - offset*y13/v13
    xp2 = x2_ - offset*x13/v13
    yp2 = y2_ - offset*y13/v13
    xp3 = (x1_ - x13) + offset*x13/v13
    yp3 = (y1_ - y13) + offset*y13/v13
    xp4 = (x2_ - x13) + offset*x13/v13
    yp4 = (y2_ - y13) + offset*y13/v13

    xp12 = xp1 - xp2
    yp12 = yp1 - yp2
    xp13 = xp1 - xp3
    yp13 = yp1 - yp3
    vp12 = np.sqrt((xp12)**2 + (yp12)**2)
    vp13 = np.sqrt((xp13)**2 + (yp13)**2)
    rows = np.floor(vp12/dot_dist + 1)
    cols = np.floor(vp13/dot_dist + 1)

    dithered_image = process_image(image_path, rows, cols)
    image_size = dithered_image._size
    cur_col = image_size[0]
    cur_row = image_size[1]

    #print_ascii_art(dithered_image)
    binary_array = (np.array(dithered_image) == 0).astype(int)

    new_vp12 = dot_dist * (cur_col - 1)
    new_vp13 = dot_dist * (cur_row - 1)

    xp2 = xp1 - new_vp12*xp12/vp12
    yp2 = yp1 - new_vp12*yp12/vp12
    xp3 = xp1 - new_vp13*xp13/vp13
    yp3 = yp1 - new_vp13*yp13/vp13
    xp4 = xp2 + xp3 - xp1
    yp4 = yp2 + yp3 - yp1

    d2x = (xp2-xp1)/(cur_col - 1) # horizontal small step from p1 to p2
    d2y = (yp2-yp1)/(cur_row - 1) # vertical small step from p1 to p2
    d3x = (xp3-xp1)/(cur_col - 1) # horizontal small step from p1 to p3
    d3y = (yp3-yp1)/(cur_row - 1) # vertical small step from p1 to p3
    grid = []

    for i in range(cur_row):
        for j in range(cur_col):
            x = xp1 + i * d2x + j * d3x
            y = yp1 + i * d2y + j * d3y
            grid.append([x, y])

    pts_to_plot = []
    for i in range(len(binary_array)):
        for j in range(len(binary_array[0])):
            if binary_array[i][j] == 1:
                pts_to_plot.append(grid[i * (len(binary_array[0])) + j])

    total_distance = calculate_total_distance(pts_to_plot)
    print('Total distance before ordering: ', total_distance)
    path, total_distance_ordered = tsp_path_planner(pts_to_plot)
    print('Total distance after ordering: ', total_distance_ordered)
    pts_to_plot = np.array(pts_to_plot)
    pts_to_plot = pts_to_plot[path]
    
    # code to plot image
    x_coords, y_coords = zip(*pts_to_plot)
    plt.scatter(x_coords, y_coords, color='black', label='Points')  # Plot points
    plt.scatter([x1, x2, x3, x4], [y1, y2, y3, y4], color='red', marker='o')
    plt.scatter([xp1, xp2, xp3, xp4], [yp1, yp2, yp3, yp4], color='green', marker='o')
    plt.grid(True)  # Add gridlines for better visualization
    plt.savefig("plot_image.png", dpi=300)

    return pts_to_plot

def process_image(image_path, canvas_width, canvas_height):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Get original dimensions and compute aspect ratio
    original_height, original_width = img.shape
    # Check if the image needs to be rotated
    if original_width > original_height and canvas_width < canvas_height or original_width < original_height and canvas_width > canvas_height:
        # Rotate the image by 90 degrees if necessary (swap width and height)
        img = cv2.transpose(img)
        img = cv2.flip(img, flipCode=1)  # Flip after transpose to rotate 90 degrees

        # Update dimensions after rotation
        original_height, original_width = img.shape
            
    aspect_ratio = original_width / original_height

    # Calculate new dimensions while maintaining aspect ratio
    if canvas_width / canvas_height > aspect_ratio:
        new_height = int(canvas_height)
        new_width = int(aspect_ratio * canvas_height)
    else:
        new_width = int(canvas_width)
        new_height = int(canvas_width / aspect_ratio)
    
    # Resize the image to fit within the canvas
    resized_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    # Convert to a PIL image and apply Floyd-Steinberg dithering
    pil_image = Image.fromarray(resized_img)
    dithered_image = pil_image.convert("1")  # Convert to black-and-white using dithering

    return dithered_image

def print_ascii_art(image):
    # Convert the image to a binary array (0 for white, 1 for black)
    binary_array = (np.array(image) == 0).astype(int)

    # Generate the ASCII art
    ascii_art = "\n".join("".join("#" if pixel == 1 else " " for pixel in row) for row in binary_array)

    # Print the ASCII art
    print(ascii_art)

def calculate_distance_matrix(points):
    """Compute a distance matrix for the given points."""
    n = len(points)
    dist_matrix = np.zeros((n, n))
    for i, j in itertools.combinations(range(n), 2):
        dist = np.linalg.norm(points[i] - points[j])
        dist_matrix[i, j] = dist
        dist_matrix[j, i] = dist
    return dist_matrix

def greedy_tsp(points, dist_matrix):
    """Generate an initial greedy TSP path."""
    n = len(points)
    unvisited = set(range(1, n))
    path = [0]  # Start with the first point
    while unvisited:
        last = path[-1]
        next_point = min(unvisited, key=lambda x: dist_matrix[last, x])
        path.append(next_point)
        unvisited.remove(next_point)
    return path

def calculate_total_distance(points):
    """
    Calculate the total distance between consecutive points in a given order.
    """
    points = np.array(points)
    total_distance = 0
    for i in range(len(points) - 1):
        total_distance += np.linalg.norm(points[i] - points[i + 1])
    return total_distance

def calculate_total_distance_ordered(path, dist_matrix):
    """Calculate the total distance of a TSP path."""
    return sum(dist_matrix[path[i], path[i + 1]] for i in range(len(path) - 1)) + dist_matrix[path[-1], path[0]]

def two_opt(path, dist_matrix):
    """Improve the TSP path using the 2-opt algorithm."""
    best_path = path
    best_distance = calculate_total_distance_ordered(path, dist_matrix)
    n = len(path)
    improvement = True
    while improvement:
        improvement = False
        for i in range(1, n - 2):  # Avoid breaking the first and last connections
            for j in range(i + 1, n):
                new_path = best_path[:i] + best_path[i:j][::-1] + best_path[j:]
                new_distance = calculate_total_distance_ordered(new_path, dist_matrix)
                if new_distance < best_distance:
                    best_path = new_path
                    best_distance = new_distance
                    improvement = True
    return best_path

def tsp_path_planner(points):
    """Find the best TSP path using a greedy initialization and 2-opt optimization."""
    points = np.array(points)
    dist_matrix = calculate_distance_matrix(points)
    initial_path = greedy_tsp(points, dist_matrix)
    optimized_path = two_opt(initial_path, dist_matrix)
    return optimized_path, calculate_total_distance_ordered(optimized_path, dist_matrix)
