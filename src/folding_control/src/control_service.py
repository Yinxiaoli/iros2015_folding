#!/usr/bin/env python

import sys
import copy
import rospy
import math
from moveit_interface import MoveitInterface
from baxter_fold.srv import *

class ControlService:
    """A class defines a set of control services.
    """
    limb = MoveitInterface()

    def move_limb_srv_pos(self, req):
        self.limb.move_limb(req.limb_name, req.pose, req.do_displacement)
        return True

    def move_limb_srv_all(self, req):
        self.limb.move_limb(req.limb_name, req.pose, req.do_displacement)
        return True

    def query_pose(self, req):
        return self.limb.query_pose(req.limb_name)

    def query_joint(self, req):
        return self.limb.query_joint(req.limb_name)

    def move_limb_height(self, req):
        self.limb.move_limb_height(req.limb_name, req.target_height, req.do_displacement)
        return True

    def move_limb_joint_single(self, req):
        self.limb.move_limb_joint_single(req.limb_name, req.joint_index, req.joint_position)
        return True

    def move_predef_position(self, req):
        self.limb.move_predef_position(req.limb_name, req.pos_name)
        return True

    def move_predef_joint_position(self, req):
        self.limb.move_predef_joint_position(req.limb_name, req.pos_name)
        return True


def main(is_q):
    rospy.init_node("fold_control_service_" + is_q)
    if is_q == '1':
        print 'query service!'
        c_q = ControlService()
        rospy.Service('query_pose', QueryPose, c_q.query_pose)
        rospy.Service('query_joint', QueryJoint, c_q.query_joint)
    else:
        c = ControlService()
        rospy.Service('move_limb_all', MoveLimbAll, c.move_limb_srv_all)
        rospy.Service('move_limb_pos', MoveLimbAll, c.move_limb_srv_pos)
        rospy.Service('move_limb_height', MoveLimbHeight, c.move_limb_height)
        rospy.Service('move_limb_joint_single', MoveLimbJointSingle, c.move_limb_joint_single)
        rospy.Service('move_predef_position',MovePredefPosition, c.move_predef_position)
        rospy.Service('move_predef_joint_position', MovePredefPosition, c.move_predef_joint_position)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
