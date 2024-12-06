import rospy

from move_arm import tuck_arm, move_arm
from tag import lookup_tag
from image_to_plot import create_rectangle_dots

def main():

    image_path = ...
    ar_tags = []

    # initialize node
    rospy.init_node('art_sawyer')

    # tuck robot arm
    tuck_arm()

    # find positions of AR tags
    tags_pos = lookup_tag(ar_tags)

    points_to_paint = create_rectangle_dots(tags_pos, image_path)
    move_arm(points=points_to_paint)

if __name__ == '__main__':
    main()