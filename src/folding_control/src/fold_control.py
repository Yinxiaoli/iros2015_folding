#!/usr/bin/env python

import copy
import rospy
import roslib
roslib.load_manifest('baxter_fold')

import baxter_haptics
import baxter_vision
import moveit_interface
import signal
from std_msgs.msg import String
from baxter_fold.srv import *
import numpy as np
import time


class FoldControl:
    """A class for folding action implementation.
    """
    gripper_length = 0.07
    time_offset = 1.5
    x_offset = 0.03
    y_offset = 0.05

    left_log_on = False
    right_log_on = False

    def reach_for_grasp(self, side):
        """
        Measure distance and reach down for grasping.

        Args:
            side: Left arm or right arm.
        Return:
        """
        distance = self.haptics.ir_distance(side)
        self.moveit.move_limb_height(side, -(distance-self.gripper_length), True)

    def generate_waypoints(self, way_pt, mode, param=0):
        """
        Generate a triangle shape for the way points.

        Args:
            way_pt: The input way point empty list.
            mode: If using triangle shape.
            param: Some control parameters.
        Return:
            way_pt: Generated way points list.
        """
        if mode == 'triangle':
            wp = [(way_pt[0][0]+way_pt[1][0])/2, (way_pt[0][1]+way_pt[1][1])/2, param]
            way_pt[0] = wp

        return way_pt

    def pick(self, side, location):
        """
        Do the picking action. Note that the end effector of the robot is not
        in the gripper but in the forelimb. Need more transform here.

        Args:
            side: Left or right arm, or both.
            location: Picking location.
        Return:
        """
        # Reach end effector downward from current pose
        if side == 'both':
            # Pad with specially chose end effector position.

            location[0][2] = self.grasp_height + 0.1
            location[1][2] = self.grasp_height + 0.1
            path = [[location[0]], [location[1]]]  # Both sides.
            self.moveit.move_cartesian_path(side, path)

            rospy.sleep(5)  # Waiting for TF to complete.

            location[0][2] = self.grasp_height-0.01
            location[1][2] = self.grasp_height-0.02
            path = [[location[0]], [location[1]]]  # Both sides.

        else:
            location[2] = self.grasp_height
            # Correct pick up offset on single arm folding step.
            if side == 'right':
                location[1] += 0.025  # Error correction.
                location[0] += 0.01  # Error correction.

            wp = copy.deepcopy(location)
            wp[2] += 0.1
            path = [wp, location]

        self.moveit.move_cartesian_path(side, path)
        self.haptics.tactile_grip(side)

    def place(self, side, target_location):
        """
        Follow the way points, move the arm, and place the picking point.

        Args:
            side: Both arm or single arm.
            target_location: Where to place the picking point.
        Return:
        """
        # self.moveit.move_limb(side, location, False)
        if side == 'both':
            target_location[0][2] = 0.1
            target_location[1][2] = 0.1
            current_location_left = self.moveit.current_gripper_pose('left')[0]
            current_location_right = self.moveit.current_gripper_pose('right')[0]
            path_left = self.generate_waypoints([current_location_left,
                                                 target_location[0]],
                                                'triangle', 0.2)
            path_right = self.generate_waypoints([current_location_right,
                                                  target_location[1]],
                                                 'triangle', 0.2)
            path = [path_left, path_right]
        else: #either side
            target_location[2] = 0.1
            current_location = self.moveit.current_position(side)[0]
            path = self.generate_waypoints([current_location, target_location],
                                           'triangle', 0.2)

        self.moveit.move_cartesian_path(side, path)
        self.haptics.release(side)

    def pick_place(self, side, way_pts):
        """
        A function that calls pick and place.

        Args:
            side: Both arm or single arm.
            way_pts: Generated way point list.
        Return:
        """
        self.haptics.release(side)
        if side == 'both':
            self.pick(side, [way_pts[0], way_pts[2]])
            self.place(side, [way_pts[1], way_pts[3]])
        else:
            self.pick(side, way_pts[0])
            self.place(side, way_pts[1])

    def pick_place_waypoint(self, side, place_way_points):
        """
        Pick and place using pre-simulated way points.

        Args:
            side: Left, right, or both arm.
            place_way_points: pre-simulated way point list.
        Return:
        """
        # Using pre-computed trajectory
        self.haptics.release(side)
        if side == 'both':
            pick_pose = [place_way_points[0][0], place_way_points[1][0]]

        else:
            pick_pose = place_way_points[0]

        self.pick(side, pick_pose)
        self.moveit.move_cartesian_path(side, place_way_points)
        self.haptics.release(side)

    def open_arms(self, side):
        """
        Function call to open arm for the next folding.

        Args:
           side: Left or right, or both arm.
        Return:
        """
        self.haptics.release(side)
        self.moveit.move_predef_joint_position(side, 'open')

    def __init__(self, height, t=None):
        """
        Initialization of picking the garment.

        Args:
            height: Picking height.
            t:
        Return:
        """
        self.haptics = baxter_haptics.HapticsInterface()
        self.moveit = moveit_interface.MoveitInterface(t)
        self.grasp_height = height
        self.buffer = []
        # rospy.Subscriber("/baxter_vision/left/score", String,
        # self.left_adjust_callback)
        # rospy.Subscriber("/baxter_vision/right/score", String,
        # self.right_adjust_callback)
        # import IPython
        # IPython.embed()
        # self.init_service()


"""Quick test module.
"""


def pick_point(q, vision):
    pt = raw_input(q + '\n')
    pt_2d = [float(p) for p in pt.split(' ')]
    return vision.find3Dfrom2D(pt_2d)


def transform_points(pt, vision):
    new_pt = []
    for p in pt:
        p = [float(p[0]), float(p[1])]
        new_pt.append(vision.find3Dfrom2D(p))
    return new_pt


def test():
    rospy.init_node('fold_control')
    vision = baxter_vision.DepthInterface()
    fc = FoldControl(-0.185)
    while True:
        fc.open_arms('both')
        raw_input('pick_place (Enter to continue)')
        pt = [line.strip('\n').split() for line in open('ui_dir/pt.txt')]
        pt = transform_points(pt, vision)
        print pt
        fc.open_arms('both')
        # fc.pick_place('left', pt[0:2])
        # fc.pick_place('right', pt[2:4])
        fc.pick_place('both', pt)

    rospy.spin()


