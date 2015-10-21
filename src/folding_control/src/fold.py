#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('baxter_fold')

# From takktile_ros.msg import Raw, Touch, Contact, Info.
# From sensor_msgs.msg import Image.
import rospy, sys
import baxter_vision
import fold_control
from communication_module import Client
import math
import tf


""" Main functions to run the Baxter folding.
"""

# Arm open position in (w, x, y, z)
orientation_list = \
{
     'left_pick' : [0.15514553575781437, 0.70952161496983279,
                    0.67205755566191661, -0.14438691943091997],
    'right_pick' : [0.15514553575781437, -0.70952161496983279,
                    0.67205755566191661, 0.14438691943091997]
}


def fetch_keypoint(vision, com):
    """
    Send source image and fetch key points from the Windows machine.

    Args:
        vision: Vision object for image saving.
        com: Communication module.
    Return:
    """
    fn = "test.jpg"
    vision.save_img(fn)
    com.send_file(fn)
    rospy.sleep(2)
    com.launch()
    raw_input("Press key to receive file")
    com.recv_file("mapped_keypoints.txt")


def transform_waypoints(pt_list, vision, surface_height):
    """
    Transform 2D point into 3D.

    Args:
        pt_list: Way points list in Kinect.
        vision: Vision object for points transform.
        surface_height: Height of the table in robot base.
    Returns:
        Transformed way points.
    """
    block = []
    for i, line in enumerate(pt_list):
        if len(line) < 3:  # check empty line (hack).
            break
        b = line.split()
        b = [float(e) for e in b]  # File line in float.
        transformed_point = vision.find3Dfrom2D(b[0:2])
        transformed_point[-1] = b[-1]
        block.append(transformed_point)

    first_point = [float(e) for e in pt_list[0].split()]
    last_point = [float(e) for e in pt_list[-1].split()]
    pixel_dist = math.sqrt((first_point[0] - last_point[0])**2 +
                           (first_point[1] - last_point[1])**2)
    dist = math.sqrt((block[0][0] - block[-1][0])**2 +
                     (block[0][1] - block[-1][1])**2)
    ratio = dist/pixel_dist

    for entry in block:
        entry[2] *= ratio
        entry[2] += surface_height

    return block


def parse_waypoints(pt_list):
    """
    Retrive way point from the list.

    Args:
        pt_list: Way points list.
    Return:
        Parsed way points list for left arm, right arm.
    """
    part = []
    block = []
    for line in pt_list:
        if len(line) < 3:
            block.append(part)
            part = []
        else:
            part.append(line.strip('\r\n'))
    return block


def fold_cloth(pts, fc, vision, height):
    """Folding a sweater.
       1) Left arm folds.
       2) Right arm folds.
       3) Both arms fold.

    Args:
        pts: Way points to move the arm.
        fc: Fold controller.
        vision: Vision object for points transform.
        height: Picking height.
    Return:

    """
    way_points_left_fold = transform_waypoints(pts[0], vision, height)
    way_points_right_fold = transform_waypoints(pts[1], vision, height)
    way_points_left_roll = transform_waypoints(pts[2], vision, height)
    way_points_right_roll = transform_waypoints(pts[3], vision, height)
    for w in way_points_right_roll:
        w[0] += 0.01
        w[1] += 0.05

    # Pad with specially chosen end effector position.
    for p in way_points_left_roll:
        p.extend(orientation_list['left_pick'])
    for p in way_points_right_roll:
        p.extend(orientation_list['right_pick'])

    # Perform movements.
    fc.pick_place_waypoint('left', way_points_left_fold)
    fc.open_arms('both')
    fc.pick_place_waypoint('right', way_points_right_fold)
    fc.open_arms('both')
    fc.pick_place_waypoint('both', (way_points_left_roll,
                                    way_points_right_roll))
    fc.open_arms('both')


def fold_pants(pts, fc, vision, height):
    """Folding a pair of pants.
       1) Both arms fold.
       2) Left arm folds.

    Args:
        pts: Way points to move the arm.
        fc: Fold controller.
        vision: Vision object for points transform.
        height: Picking height.
    Return:
    """
    way_points_left_roll = transform_waypoints(pts[0], vision, height)
    way_points_right_roll = transform_waypoints(pts[1], vision, height)
    way_points_left_fold = transform_waypoints(pts[2], vision, height)
    for w in way_points_right_roll:
        w[0] += 0.01
        w[1] += 0.05

    # Perform movements
    fc.pick_place_waypoint('both', (way_points_left_roll, way_points_right_roll))
    fc.open_arms('both')
    fc.pick_place_waypoint('left', way_points_left_fold)
    fc.open_arms('both')


def fold_towel(pts, fc, vision, height):
    """The same as folding a pair of pants.

    Args:
        pts: Way points to move the arm.
        fc: Fold controller.
        vision: Vision object for points transform.
        height: Picking height.
    Return:
    """
    fold_pants(pts, fc, vision, height)


def main():
    height = -0.175
    rospy.init_node('baxter_fold')
    tros = tf.TransformListener()
    vision = baxter_vision.DepthInterface(tros)

    # RPC module, IP address of the Kinect Windows machine
    com = Client('128.59.22.121', sys.argv[1])
    fc = fold_control.FoldControl(height-0.04, tros)
    fc.open_arms('both')

    # Fetch key points from remote machine.
    rospy.sleep(4)
    fetch_keypoint(vision, com)

    # Parse and transform fetched way point locations
    pts = [line for line in open('mapped_keypoints.txt')]
    pts = parse_waypoints(pts)

    fold_cloth(pts, fc, vision, height)
    # fold_pants(pts, fc, vision, height)
    # fold_towel(pts, fc, vision, height)

    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())


