#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import signal
import math
import tf
from copy import deepcopy
import transformation

def signal_handler(signal, frame):
    print("Disabling robot... ")
    moveit_commander.roscpp_shutdown()
    print("done.")
    rospy.signal_shutdown("finished")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

pose_list = \
{
'left_open': [0.6451299414163874, 0.48308154941436016, 0.688241579565718, 0.11442618882303603, 0.9926119706997465, 0.028043380270852243, 0.02901192441022871],
'right_open': [0.6462863106297506, -0.48390075862008375, 0.6859732922530206, -0.15210562530968288, 0.9881075743671162, 0.0028034494110560793, 0.02234817439712085]
}

joint_list = \
{
'left_open': [-0.04525243319091797, -1.185383652484131, -0.6304661031005859, 0.9851991598937989, 0.2657621711975098, 1.7587089713012696, 0.44638840876464847],
'right_open': [-0.024543692578125, -1.1293933537902834, 0.7673738882629395, 0.9560535249572755, -0.2922233397583008, 1.8089468420471193, -0.4793689956665039]
}


def distance():
    """
    Calculate the distance between two points.
    return:
        Distance.
    """
    return lambda a, b: math.sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*+(a.y-b.y)+(a.z-b.z)*(a.z-b.z))


class MoveitInterface:
    """
    A wrapper class that calls functions from the MoveIt.
    """
    gripper_translation_dist = 0.22
    print_flag = False

    def __init__(self, t=None):
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        # self.left_arm.set_end_effector_link('left_gripper')
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        # self.left_arm.set_end_effector_link('right_gripper')
        self.both_arms = moveit_commander.MoveGroupCommander("both_arms")
        self.left_arm.set_planner_id('SBLkConfigDefault')
        self.right_arm.set_planner_id('SBLkConfigDefault')
        self.both_arms.set_planner_id('SBLkConfigDefault')
        if t == None:
            self.tros = tf.TransformListener()
        else:
            self.tros = t

    def move_predef_position(self, limb_name, pos_name, speed=0.3,
                             timeout = 15.0):
        """
        Move to some pre-defined pose.
        Args:
            limb_name: Joint name.
            pos_name: refer to the dict() defined at the beginning
        """
        self.move_limb(limb_name, pose_list[limb_name+'_'+pos_name], False,
                       speed)
    
    def move_predef_joint_position(self, limb_name, pose_name, speed = 0.3,
                                   timeout = 15.0):
        """
        Move to some pre-defined joints.
        Args:
            limb_name: Both arm or left or right.
            pos_name: refer to the dict() defined at the beginning
        """
        if limb_name == 'both':
            self.move_limb_joint(limb_name, joint_list['left_'+pose_name]+
                                 joint_list['right_'+pose_name], speed)
        else:
            self.move_limb_joint(limb_name, joint_list[limb_name+'_'+pose_name],
                                 speed)
    
    def move_limb_height(self, limb_name, target_height_displacement,
                         do_displacement, speed=0.1):
        """
        Move the arm vertically.
        Args:
            limb_name: Left or right arm.
            target_height_displacement: Move offset.
            do_displacement: True if move from the current position.
            speed: Move speed.
        Return:
        """
        # Both arm not implemented.
        print target_height_displacement
        if limb_name == 'left':
            target_pos = self.left_arm.get_current_pose().pose
        elif limb_name == 'right':
            target_pos = self.right_arm.get_current_pose().pose

        if do_displacement:
            target_height_displacement += target_pos.position.z
        print [target_pos.position.x, target_pos.position.y,
               target_height_displacement]
        self.move_limb(limb_name, [target_pos.position.x, target_pos.position.y,
                                   target_height_displacement])
        
    def move_limb_joint_single(self, limb_name, joint_index, joint_position):
        """
        Move a joint of the arm.

        Args:
            limb_name: Left or right arm.
            joint_index: Joint index. There are 7 joints on each arm.
            joint_position: joint angle.
        Return:
        """
        # Both arm not implemented.
        joint_val = self.query_joint(limb_name)
        joint_val[joint_index] = joint_position
        self.move_limb_joint(limb_name, joint_val)

    def move_limb_joint(self, limb_name, joint_positions, speed=0.1):
        """
        Move the arm with 7 joints in a whole.

        Args:
            limb_name: Left or right arm.
            joint_index: Joint index of 7. There are 7 joints on each arm.
            joint_position: joint angle.
        Return:
        """
        if limb_name == 'both':
            print joint_positions
            self.both_arms.set_joint_value_target(joint_positions)
            self.both_arms.go()
            return

        joint_pos = {limb_name+'_s0': joint_positions[0],
                     limb_name+'_s1': joint_positions[1],
                     limb_name+'_e0': joint_positions[2],
                     limb_name+'_e1': joint_positions[3],
                     limb_name+'_w0': joint_positions[4],
                     limb_name+'_w1': joint_positions[5],
                     limb_name+'_w2': joint_positions[6]}
        if limb_name == 'left':
            self.left_arm.set_joint_value_target(joint_pos)
            self.left_arm.go()
        if limb_name == 'right':
            self.right_arm.set_joint_value_target(joint_pos)
            self.right_arm.go()

    def move_limb_position(self, limb_name, position, speed=0.1):
        """
        Move a single arm by position.

        Args:
            limb_name: Left or right arm.
            position: 3D position in terms of the robot base.
            speed: Moving speed.
        Return:
        """
        if limb_name == 'left':
            self.left_arm.set_position_target(position[0], position[1],
                                              position[2])
            self.left_arm.go()
        if limb_name == 'right':
            self.right_arm.set_position_target(position[0], position[1],
                                               position[2])
            self.right_arm.go()

    def move_limb(self, limb_name, position, do_displacement = False,
                  sync_option = 'wait', speed=0.3, timeout = 15.0):
        """
        Move a limb to the target position & orientation
        ---position (float list):
        - The first 3 elements are cartesian coordinate of the target position.
        - THe 4 following elements are the quaternion of the end effector
        (optional)

        Args:
            limb_name: Left or right arm.
            position: 3D position in terms of the robot base.
            do_displacement: True if move from the current position.
            sync_option:
            speed: Moving speed.
            timeout: If not reachable, time to release.
        Return:
        """
        if limb_name == 'left':
            self.left_arm.clear_pose_targets()
            pose = self.generate_pose(limb_name, position, do_displacement)
            self.left_arm.set_pose_target(pose)
            self.left_arm.go()
        if limb_name == 'right':
            self.right_arm.clear_pose_targets()
            pose = self.generate_pose(limb_name, position, do_displacement)
            self.right_arm.set_pose_target(pose)
            self.right_arm.go()
        if limb_name == 'both':
            self.left_arm.clear_pose_targets()
            self.right_arm.clear_pose_targets()
            if len(position) == 6:
                # Position only, same orientation.
                pose_left = self.generate_pose(
                    'left', position[0:3], do_displacement)
                pose_right = self.generate_pose(
                    'right', position[3:], do_displacement)
            elif len(position) == 14: # Pose.
                pose_left = self.generate_pose(
                    'left', position[0:7], do_displacement)
                pose_right = self.generate_pose(
                    'right', position[7:], do_displacement)
            elif len(position) == 2:
                # For the case that the parameter is a tuple of two lists.
                pose_left = self.generate_pose(
                    'left', position[0], do_displacement)
                pose_right = self.generate_pose(
                    'right', position[1], do_displacement)
            self.right_arm.set_pose_target(pose_right)
            self.left_arm.set_pose_target(pose_left)
            pa = self.right_arm.plan()
            pb = self.left_arm.plan()
            pc = self.merge_plans(pa, pb, sync_option)
            self.both_arms.execute(pc)
        return 1

    def generate_pose(self, side, position, do_displacement=False):
        """
        Generate arm pose for a target moving position.

        Args:
            side: Left or right arm.
            position: Target moving position.
            do_displacement: True if move from the current position.
        Return: The pose of a list of 3 or 7 elements for the target position.
        """
        if side == 'left':
            limb = self.left_arm
        else:
            limb = self.right_arm
        target_pose = limb.get_current_pose()
        if do_displacement:
            target_pose.pose.position.x += position[0]
            target_pose.pose.position.y += position[1]
            target_pose.pose.position.z += position[2]
        else:
            target_pose.pose.position.x = position[0]
            target_pose.pose.position.y = position[1]
            target_pose.pose.position.z = position[2]

        if len(position) == 7:
            print "size of 7! Including orientation."
            target_pose.pose.orientation.w = position[3]
            target_pose.pose.orientation.x = position[4]
            target_pose.pose.orientation.y = position[5]
            target_pose.pose.orientation.z = position[6]

        target_pose = self.transform_gripper_to_wrist(side, target_pose)

        return target_pose.pose

    def move_cartesian_path(self, limb_name, way_points,
                            sync_option = 'wait'):
        """
        Move the arm in cartesian path defined by a set of way points.

        Args:
            limb_name: Left or right arm.
            way_points: A set of 3D points to move the arm.
            sync_option: 'wait' if move the arm one by one.
        Return:
        """
        if limb_name == 'both':
            assert len(way_points) == 2
            pa = self.generate_cartesian_path_plan('left', way_points[0])
            pb = self.generate_cartesian_path_plan('right', way_points[1])
            if pa and pb: #if both are valid
                plan = self.merge_plans(pa, pb, sync_option)
                self.both_arms.execute(plan)
            else:
                print "Invalid Cartesian Path"
        elif limb_name == 'left':
            plan = self.generate_cartesian_path_plan(limb_name, way_points)
            if plan:
                self.left_arm.execute(plan)
            else:
                print "Invalid Cartesian Path"
                exit()
        elif limb_name == 'right':
            plan = self.generate_cartesian_path_plan(limb_name, way_points)
            if plan:
                self.right_arm.execute(plan)
            else:
                print "Invalid Cartesian Path"
                exit()

    def generate_cartesian_path_plan(self, side, way_points,
                                     eef_step = 0.01, jump_threshold = 0):
        """
        Generate a plan based on a sequence of cartesian way-points
        Args:
            group: move group
            way_points: a list of cartesian way points, can be either x,y,z
                        (position) or x,y,z,w,x,y,z (pose)
            eef_step: end effector step constraint
            jump_threshold:
        Return:a MoveIt plan.
        """
        if side == 'left':
            group = self.left_arm
        else:
            group = self.right_arm
        # Always start with the current pose.
        way_point_poses = []
        for w in way_points:
            way_point_poses.append(self.generate_pose(side, w))

        (plan, fraction) = group.compute_cartesian_path(
                            way_point_poses,   # Way points to follow.
                            eef_step,        # eef_step.
                            jump_threshold)   # jump_threshold.
        if fraction == -1:  # Error.
            return False
        return plan

    def merge_plans(self, pa, pb, sync_option='wait'):
        """
        Merge two MoveIt plans.

        Args:
            pa: Plan a.
            pb: Plan b.
            sync_option: wait
        Return:
            Merged plan.
        """
        # Merge two (left & right) plans.
        self.merge_trajectories(pa.joint_trajectory, pb.joint_trajectory,
                                sync_option)
        if len(pa.joint_trajectory.points) == 0:
            return pb
        else:
            return pa

    def merge_points(self, target, source):
        """
        Merge trajectory data points.

        Args:
            target:
            source:
        Return:
        """
        target.joint_names.extend(source.joint_names)
        for i in range(len(target.points)):
            target.points[i].positions = target.points[i].positions + \
                                         source.points[i].positions
            target.points[i].accelerations = target.points[i].accelerations + \
                                             source.points[i].accelerations
            target.points[i].velocities = target.points[i].velocities + \
                                          source.points[i].velocities
            target.points[i].effort = target.points[i].effort + \
                                      source.points[i].effort
        source.points[:] = []
        return target

    def merge_trajectories(self, traj_l, traj_r, sync_option='wait'):
        """
        Merge two trajectories by various synchronization options.

        Args:
            traj_l: Left arm trajectory.
            traj_r: Right arm trajectory.
            sync_option: See below.
        Return:
        """
        #
        if len(traj_l.points) < len(traj_r.points):
            long = traj_r
            short = traj_l
        else:
            long = traj_l
            short = traj_r

        if sync_option == 'trim' or len(short.points) == len(long.points):
            # merge to shorter trajectory by trimming the longer one.
            self.merge_points(short, long) # merge long to short.

        elif sync_option == 'wait':
            # merge to longer trajectory by waiting for the shorter one.
            size_diff = len(long.points) - len(short.points)
            state = deepcopy(short.points[-1])

            # zero velocities & accelerations state.
            state.accelerations = (0,)*len(state.accelerations)
            state.velocities = (0,)*len(state.velocities)
            for i in range(size_diff):
                short.points.append(deepcopy(state))
            assert(len(short.points) == len(long.points))
            self.merge_points(long, short) #merge short to long

        elif sync_option == 'fastforward':
            # Merge to shorter trajectory by fast forwarding the longer one.
            pass
        elif sync_option == 'slowmotion':
            pass

    def adjust_trajectory_speed(self, traj, target_time):
        pass

    def query_pose(self, limb_name):
        """
        Query the current pose (7 parameters) of arms.

        Args:
            limb_name: Left or right arm.
        Return:
            Pose of the queried arm.
        """
        if limb_name == 'left':
            pose = self.left_arm.get_current_pose().pose
        if limb_name == 'right':
            pose = self.right_arm.get_current_pose().pose
        return pose

    def query_joint(self, limb_name):
        """
        Query the current joint of arms.

        Args:
            limb_name: Left or right arm.
        Return:
            Joint of the queried arm.
        """
        if limb_name == 'left':
            pose = self.left_arm.get_current_joint_values()
        if limb_name == 'right':
            pose = self.right_arm.get_current_joint_values()
        return pose

    def current_position(self, limb_name):
        """
        Query the current pose (position) of arms.

        Args:
            limb_name: Left or right arm.
        Return:
            Pose of the queried arm.
        """
        if limb_name == 'left':
            pose = self.left_arm.get_current_pose().pose
        if limb_name == 'right':
            pose = self.right_arm.get_current_pose().pose
        return [pose.position.x, pose.position.y, pose.position.z]

    def current_orientation(self, limb_name):
        """
        Query the current pose (orientation) of arms.

        Args:
            limb_name: Left or right arm.
        Return:
            Pose of the queried arm.
        """
        if limb_name == 'left':
            pose = self.left_arm.get_current_pose().pose
        if limb_name == 'right':
            pose = self.right_arm.get_current_pose().pose
        return [pose.orientation.w, pose.orientation.x, pose.orientation.y,
                pose.orientation.z]

    def current_gripper_pose(self, limb_name):
        """
        Query the current pose (orientation) of gripper.
        End effector with transform to the gripper.

        Args:
            limb_name: Left or right arm.
        Return:
            Pose of the queried gripper (7 parameters).
        """
        if limb_name == 'left':
            pose = self.left_arm.get_current_pose()
        if limb_name == 'right':
            pose = self.right_arm.get_current_pose()
        pose = self.transform_wrist_to_gripper(limb_name, pose)
        return ([pose.pose.position.x, pose.pose.position.y,
                 pose.pose.position.z],
                [pose.pose.orientation.w, pose.pose.orientation.x,
                 pose.pose.orientation.y, pose.pose.orientation.z])

    def transform_gripper_to_wrist(self, side, gripper_target_pose):
        """
        Transform a pose in side_gripper_center frame to side_wrist frame.

        Args:
            side: Left or right arm.
            gripper_target_pose: End effector position.
        Return:
            New pose of the end effector.
        """
        self.tros.waitForTransform('/base', side + '_gripper_center',
                                   rospy.Time(), rospy.Duration(4))
        gripper_target_pose.header.stamp = \
            self.tros.getLatestCommonTime('/base', side + '_gripper_center')
        p = self.tros.transformPose(side + '_gripper_center',
                                    gripper_target_pose)
        p.header.frame_id = side + '_wrist'
        self.tros.waitForTransform('/base', side + '_wrist',
                                   rospy.Time(), rospy.Duration(4))
        p.header.stamp = self.tros.getLatestCommonTime('/base', side + '_wrist')
        p_new = self.tros.transformPose('base', p)

        return p_new

    def transform_wrist_to_gripper(self, side, wrist_pose):
        """
        Transform between end effector and gripper. The default end effector
        is the wrist part.

        Args:
            side: Left or right arm.
            wrist_pose: Current end effector position.
        Return:
            3D position of the gripper.
        """
        self.tros.waitForTransform(
            '/base', side + '_wrist',rospy.Time(), rospy.Duration(4))
        wrist_pose.header.stamp = self.tros.getLatestCommonTime(
            '/base', side + '_wrist')
        p = self.tros.transformPose(side + '_wrist', wrist_pose)
        p.header.frame_id = side + '_gripper_center'
        self.tros.waitForTransform(
            '/base', side + '_gripper_center',rospy.Time(), rospy.Duration(4))
        p.header.stamp = self.tros.getLatestCommonTime(
            '/base', side + '_gripper_center')
        p_new = self.tros.transformPose('base', p)
        return p_new

def main():
    #MOVE THE ROBOT WITH INTERACTIVE UI
    rospy.init_node("moveit_interface")
        
    limb = MoveitInterface()
    print_flag = False
    limb_name = sys.argv[1]
    while True:
        mode = raw_input(
            'Choose Mode: Position(p)/Move(m)/Query(q)/Preset Movment(pm)/Validate(v)')
        if mode == 'q':
            print limb.query_pose(limb_name)
            continue
        if mode == 'v':
            q_pos = limb.query_pose(limb_name).position
            q_ori = limb.query_pose(limb_name).orientation
            target_pos = [q_pos.x, q_pos.y, q_pos.z,  q_ori.w,
                          q_ori.x, q_ori.y, q_ori.z]
            result = limb.move_limb(limb_name, target_pos, False, 0.3)
            if result <0: 
                print "Invalid position"
                continue
            else:
                print target_pos
                continue
        if mode != 'm' and mode !='p' and mode!='pm' and mode!= 'j' and mode!='jm': continue
        incre = []
        speed = 0
        if mode == 'pm':
            select = raw_input('1. right gripper to far left. 2.right gripper to far right. 3.others')
            if int(select) == 1:
                speed = 0.1
                incre = [0,0,0,-3,1,0,0]
                mode = 'm'
            if int(select) == 2:
                speed = 0.1
                incre = [0,0,0,3,1,0,0]
                mode = 'm'
            if int(select) == 3:
                speed = 0.3
                limb.move_predef_position(
                    limb_name, raw_input('movement name'), speed)
        if mode == 'jm':
            limb.move_predef_joint_position(
                limb_name, raw_input('movement name'), 0.3)
        else:
            pos_input = raw_input('Please enter: x y z (x y z w) ')
            speed = raw_input('Speed?: \n')
            pos_input = pos_input.split(' ')
            incre = [float(n) for n in pos_input]
        if len(incre) == 3 or len(incre) == 7:
            result = 1
            if mode == 'm': result = limb.move_limb(
                limb_name, incre, True, float(speed))
            elif mode == 'p': result = limb.move_limb(
                limb_name, incre, False, float(speed))
            # elif mode == 'j': result = limb.move_joint_position(limb_name, incre, float(speed))
            else:
                continue
                
            if result < 0:
                print "Invalid pose, try again" 
                continue


if __name__ == '__main__':
    sys.exit(main())
