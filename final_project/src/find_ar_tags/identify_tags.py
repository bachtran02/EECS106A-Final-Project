#!/usr/bin/env python

import sys

import cv2
from PIL import Image

import rospy
import rospkg
import roslaunch

import matplotlib.pyplot as plt
import numpy as np

import tf2_ros

def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('final_project')
        launch_path = path + '/custom_tuck/launch/custom_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    ar_tag_string = "ar_marker_{}".format(tag_number)

    retry = 0
    while True:

        try:
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            trans = tfBuffer.lookup_transform("base", ar_tag_string, rospy.Time(0), rospy.Duration(10.0))
            break
        except Exception as e:
            print(f'Failed to find {ar_tag_string}: err={e}')
        retry += 1
        
        if retry == 5:
            sys.exit(1)
        
        print("Retrying...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)


def rotate_point(x, y, angle, x0, y0):
    x_translated = x - x0
    y_translated = y - y0
    
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rotated = np.dot(rot_matrix, np.array([x_translated, y_translated]))
    
    x_rotated = rotated[0] + x0
    y_rotated = rotated[1] + y0
    
    return x_rotated, y_rotated

def create_rectangle_dots(x1, y1, x2, y2, x3, y3, offset=0.5, dot_dist=0.1):
    
    angle = (np.arctan(np.abs((x3-x1)/(y3-y1))) + np.arctan(np.abs((y2-y1)/(x2-x1))))/2
    rx2, ry2 = rotate_point(x2, y2, angle, x1, y1)
    rx3, ry3 = rotate_point(x3, y3, angle, x1, y1)

    xp1 = x1 + offset
    yp1 = y1 + offset
    xp2 = rx2 - offset
    yp2 = ry2 + offset
    xp3 = rx3 + offset
    yp3 = ry3 - offset
    xp4 = xp2
    yp4 = yp3

    x_min = min(xp1, xp2, xp3, xp4)
    x_max = max(xp1, xp2, xp3, xp4)
    y_min = min(yp1, yp2, yp3, yp4)
    y_max = max(yp1, yp2, yp3, yp4)

    cols = np.floor((x_max - x_min)/dot_dist)
    rows = np.floor((y_max - y_min)/dot_dist)

    image_path = "../../assets/img/smiley.jpg" 
    dithered_image = process_image(image_path, cols, rows)
    image_size = dithered_image._size
    cur_col = image_size[1]
    cur_row = image_size[0]
    
    x_pt = np.linspace(x_min, x_max - dot_dist*(cols - cur_col), cur_col)
    y_pt = np.linspace(y_min, y_max - dot_dist*(rows - cur_row), cur_row)
    grid = np.array([rotate_point(x, y, -angle, x1, y1) for x in x_pt for y in y_pt])


    plt.scatter(grid[:, 0], grid[:, 1], color='blue', marker='o')
    plt.scatter([x1, x2, x3], [y1, y2, y3], color='red', marker='o')
    plt.grid(True)
    plt.show()
    
    return grid, cols, rows

def process_image(image_path, canvas_width, canvas_height):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Get original dimensions and compute aspect ratio
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
    binary_array = (np.array(image) == 0).astype(int)  # 0 is black in "1" mode

    # Generate the ASCII art
    ascii_art = "\n".join("".join("#" if pixel == 1 else " " for pixel in row) for row in binary_array)

    # Print the ASCII art
    print(ascii_art)

def main():
    """
    python find_ar_tags/identify_tags.py
    """

    ar_tags = [7, 8, 6]
    
    rospy.init_node('find_tags_node')
    
    # move the camera to correct position
    # tuck()

    # # Lookup the AR tag position.
    # tags_pos = [lookup_tag(ar_tag) for ar_tag in ar_tags]
    
    # # ensure that all AR tags are found
    # assert len(tags_pos) == len(ar_tags)

    # x1, y1 = tags_pos[0][0], tags_pos[0][1]
    # x2, y2 = tags_pos[1][0], tags_pos[1][1]
    # x3, y3 = tags_pos[2][0], tags_pos[2][1]

    x1, y1 = 0, 1
    x2, y2 = 5, 1
    x3, y3 = 0, 5

    rectangle_dots, cols, rows = create_rectangle_dots(x1, y1, x2, y2, x3, y3, offset=0.5)
    print(rectangle_dots)
    print('Number of rows:', rows)
    print('Number of cols:', cols)

    image_path = "../../assets/img/bw_smiley.jpg" 
    dithered_image = process_image(image_path, cols, rows)
    print('image dimension:', dithered_image._size)
    binary_array = (np.array(dithered_image) == 0).astype(int) 
    print_ascii_art(dithered_image)

    print(rectangle_dots)

    pts_to_plot = []
    for i in range(len(binary_array)):
        for j in range(len(binary_array[0])):
            if binary_array[i][j] == 1:
                pts_to_plot.append(rectangle_dots[i * j + j])

    print(pts_to_plot)
    
    x_coords, y_coords = zip(*pts_to_plot)
    # Create a plot
    plt.scatter(x_coords, y_coords, color='blue', label='Points')  # Plot points

    # Optionally, you can add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Plotting Coordinates')
    plt.legend()

    # Show the plot
    plt.grid(True)  # Add gridlines for better visualization
    plt.show()



if __name__ == "__main__":
    main()