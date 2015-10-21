#!/usr/bin/env python

import rospy

import roslib
roslib.load_manifest('baxter_fold')

from takktile_ros.msg import Raw, Touch, Contact, Info
from baxter_core_msgs.msg import SEAJointState
from sensor_msgs.msg import Range
from std_msgs.msg import UInt32
from baxter_core_msgs.msg import AnalogIOState
import baxter_interface
import rospy, sys
from collections import deque


class HapticsLowLevel:
    """
    A class that implements the low level sensor call backs.
    """
    tactile_reading = 0
    ir_reading = 0

    def ir_callback(self, data):
        self.ir_reading = float(data.data)/1000

    def torque_callback(self, torque):
        self.torque_log.append(torque.gravity_model_effort[1])

    def tactile_callback(self, data):
		self.tactile_reading = sum(data.pressure)

    def limb_contact(self, thresh):
        count = 0
        for t in self.torque_log:
            if t > 0:
                count += 1
        print count
        return float(count)/float(len(self.torque_log)) > thresh
    
    def gripper_contact(self):
        return self.tactile_reading < -1000

    def ir_distance(self):
        return self.ir_reading

    def tactile_grip(self, block):
        self.gripper.close(block)
        if self.gripper_contact():
            return True
        else:
            #self.gripper.open()
            return False

    def release(self):
        self.gripper.open()

    def __init__(self, side):
        self.torque_log = deque(maxlen=100)
        sensor_ID = 1 if side == 'left' else 0
        torque_sub = rospy.Subscriber("/robot/limb/" + side + "/gravity_compensation_torques", SEAJointState, self.torque_callback)
        rospy.Subscriber('/takktile/' + str(sensor_ID) + '/calibrated/', Touch, self.tactile_callback) #subscribe to takktile sensor topic
        rospy.Subscriber('/robot/analog_io/'+side+'_hand_range/value_uint32', UInt32, self.ir_callback) #subscribe to ir range sensor topic
        self.gripper = baxter_interface.Gripper(side) #Gripper interface from Baxter API
        self.gripper.calibrate()


class HapticsInterface:
    """
    A class that implements interfaces of haptics sensors of the robot.
    """
    def limb_contact(self, side, thresh):
        if side == 'left':
            return self.left.limb_contact(thresh)
        else:
            return self.right.limb_contact(thresh)

    def gripper_contact(self, side):
        if side == 'left':
            return self.left.gripper_contact()
        elif side == 'right':
            return self.right.gripper_contact()

    def ir_distance(self, side):
        if side == 'left':
            return self.left.ir_distance()
        else:
            return self.right.ir_distance()

    def tactile_grip(self, side, block=True):
        if side == 'left':
            return self.left.tactile_grip(block)
        elif side == 'right':
            return self.right.tactile_grip(block)
        else:
            self.left.tactile_grip(False)
            return self.right.tactile_grip(False)

    def release(self, side):
        if side == 'left':
            return self.left.release() 
        elif side == 'right':
            return self.right.release() 
        else:
            self.left.release()
            return self.right.release()

    def __init__(self):
        self.left = HapticsLowLevel('left')
        self.right = HapticsLowLevel('right')


def test():
    rospy.init_node('baxter_haptics')
    h = HapticsInterface()
    import IPython
    IPython.embed()
    rospy.spin()
