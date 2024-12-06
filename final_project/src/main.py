#! /usr/bin/env python
import sys
import rospy
import rospkg

from robot_code.move_arm import tuck_arm, move_arm_plot, move_arm_probe
from robot_code.tag import lookup_tag
from robot_code.image_to_plot import create_rectangle_dots

from moveit_commander import RobotCommander

def main():

    rospack = rospkg.RosPack()
    prefix_path = rospack.get_path('final_project')

    image_path = prefix_path + '/../assets/img/bw_smiley.jpg'
    ar_tags = [2, 1, 5]

    # initialize node
    rospy.init_node('art_sawyer')

    # tuck robot arm
    tuck_arm()

    # find positions of AR tags
    tags_pos = [lookup_tag(ar_tag) for ar_tag in ar_tags]

    points_to_paint = create_rectangle_dots(tags_pos, image_path, offset=0.04, dot_dist=0.004)
    move_arm_plot(points_to_paint, paused=False, link='right_hand')

if __name__ == '__main__':
    main()