#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import signal


print "============ Starting tutorial setup"
    
rospy.init_node('move_group_python_interface_tutorial',
		anonymous=True)
print sys.argv
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("left_arm")


#import IPython
#IPython.embed()
group.clear_pose_targets()
"""
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.6923
pose_target.orientation.y = 0.1
pose_target.orientation.z = 0.0128
pose_target.orientation.w = 0.7214
pose_target.position.x = 0.35968
pose_target.position.y = 0.07712
pose_target.position.z = 0.12349
"""
waypoints = []

# start with the current pose
waypoints.append(group.get_current_pose().pose)
print group.get_current_pose()


# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.10
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))
print waypoints

(plan3, fraction) = group.compute_cartesian_path(
                            waypoints,   # waypoints to follow
                           0.01,        # eef_step
                           0.0)         # jump_threshold

import IPython
IPython.embed()
print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)
#rospy.spin()

#group.set_pose_target(group.get_current_pose().pose)
#group.set_random_target()
group.go()
