#! /usr/bin/env python
import sys
import rospy
import rospkg
import time

from robot_code.move_arm import tuck_arm, move_arm_plot, move_arm_probe
from robot_code.tag import lookup_tag
from robot_code.image_to_plot import create_rectangle_dots

from moveit_commander import RobotCommander

def main():

    rospack = rospkg.RosPack()
    prefix_path = rospack.get_path('final_project')
    image_path = prefix_path + '/../assets/img/dog.png'

    ar_tags = [2, 1, 5]

    # initialize node
    rospy.init_node('art_sawyer')

    # tuck robot arm
    tuck_arm()

    # find positions of AR tags
    tags_pos = [lookup_tag(ar_tag) for ar_tag in ar_tags]
    
    # Find the (x, y) positions of each point
    points_to_paint = create_rectangle_dots(tags_pos, image_path, offset=0.06, dot_dist=0.005)
    print('Number of points to plot:', len(points_to_paint))

    # Use probe to find the z of each (x, y) point
    points_to_paint = move_arm_probe(points_to_paint, paused=False, link='right_hand')

    # Save (x, y, z) points
    with open('xyz.txt', 'w') as file:
        file.write(str(points_to_paint))

    tuck_arm()

    # Change end effector from button to marker
    
    tuck_arm()
    # Use marker to paint
    move_arm_plot(points_to_paint, paused=False, link='right_hand')

if __name__ == '__main__':
    main()
