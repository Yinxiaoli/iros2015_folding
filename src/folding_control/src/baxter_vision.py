#!/usr/bin/env python

import sys
import copy
import rospy

import roslib
roslib.load_manifest('baxter_fold')

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2, cv
import numpy
import numpy as np
import transformation

import scipy.io as sio


class VisionInterface:
    """
    A class that implements several useful vision functions.
    """
    bridge = CvBridge()

    def compute_score(self, mask, side):
        """
        Compute the image score based on the mask image.

        Args:
            mask: Pre-defined mask image to find best gripper position.
            side: Left or right arm.
        Return:
        """
        if side == 'left':
          count_left = np.sum(np.bitwise_and(mask, self.left_mask_l))/float(np.sum(self.left_mask_l))
          count_right = np.sum(np.bitwise_and(mask, self.left_mask_r))/float(np.sum(self.left_mask_r))
          self.left_score = abs(count_left - count_right)
          self.left_score_pub.publish(str(self.left_score))
        else:
          count_left = np.sum(np.bitwise_and(mask, self.right_mask_l))/float(np.sum(self.right_mask_l))
          count_right = np.sum(np.bitwise_and(mask, self.right_mask_r))/float(np.sum(self.right_mask_r))
          self.right_score = abs(count_left - count_right)
          self.right_score_pub.publish(str(self.right_score))

    def get_score(self, side):
        """
        Return the computed image score between the acquired image and
        template image.

        Args:
            side: Left or right arm.
        Return:
            The computed image score.
        """
        if self.side == 'left':
          return self.left_score
        else:
          return self.right_score

    def image_handler_left(self, cam_image):
        """
        Call back function for the left hand camera.
        Args:
            cam_image: Image from the left hand camera.
        Return:
        """
        try:
          frame = self.bridge.imgmsg_to_cv(cam_image, "bgr8") #convert ROS image to OpenCV image
        except CvBridgeError, e:
          print e

        frame = numpy.array(frame)#Convert frame to numpy array
        (res, mask) = self.kMeans(frame)
        self.compute_score(mask, 'left')

        display = cv.fromarray(res)
        try:
          self.left_image_pub.publish(self.bridge.cv_to_imgmsg(display, "bgr8"))
        except CvBridgeError, e:
          print e


    def image_handler_right(self, cam_image):
        """
        Call back function for the right hand camera.
        Args:
            cam_image: Image from the right hand camera.
        Return:
        """
        try:
          frame = self.bridge.imgmsg_to_cv(cam_image, "bgr8") #convert ROS image to OpenCV image
        except CvBridgeError, e:
          print e

        frame = numpy.array(frame)#Convert frame to numpy array
        (res, mask) = self.kMeans(frame)
        self.compute_score(mask, 'right')

        display = cv.fromarray(res)
        try:
          self.right_image_pub.publish(self.bridge.cv_to_imgmsg(display, "bgr8"))
        except CvBridgeError, e:
          print e

    def kMeans(self, img):
        """
        A wrapper of the KMeans algorithm from cv2.
        Args:
            img: The input image.
        Return:
        """
        Z = img.reshape((-1,3))
        # convert to np.float32
        Z = numpy.float32(Z)
        # define criteria, number of clusters(K) and apply kmeans()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 2
        ret,label,center=cv2.kmeans(Z,K,criteria,10,cv2.KMEANS_RANDOM_CENTERS)

        # Now convert back into uint8, and make original image
        center = numpy.uint8(center)
        res = center[label.flatten()]
        res2 = res.reshape((img.shape))
        return (res2, label.reshape(img[:,:,0].shape))

    def __init__(self):
        self.left_mask_l = numpy.uint8(numpy.array([line.strip('\n').split(',') for line in open("/home/USER/ros/ws_baxter/src/baxter_fold/src/parameters/left_mask_l.txt")]))
        self.left_mask_r = numpy.uint8(numpy.array([line.strip('\n').split(',') for line in open("/home/USER/ros/ws_baxter/src/baxter_fold/src/parameters/left_mask_r.txt")]))

        self.right_mask_l = numpy.uint8(numpy.array([line.strip('\n').split(',') for line in open("/home/USER/ros/ws_baxter/src/baxter_fold/src/parameters/right_mask_l.txt")]))
        self.right_mask_r = numpy.uint8(numpy.array([line.strip('\n').split(',') for line in open("/home/USER/ros/ws_baxter/src/baxter_fold/src/parameters/right_mask_r.txt")]))

        rospy.Subscriber("/cameras/left_hand_camera/image", Image, self.image_handler_left)
        rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.image_handler_right)
        self.left_image_pub = rospy.Publisher("/baxter_vision/left/kmean", Image)
        self.right_image_pub = rospy.Publisher("/baxter_vision/right/kmean", Image)

        self.left_score_pub = rospy.Publisher("/baxter_vision/left/score", String)
        self.right_score_pub = rospy.Publisher("/baxter_vision/right/score", String)


class DepthInterface:
    """
    A class that implements the 2D to 3D frame conversion.
    """
    # transformation error from static transform
    transformation_error = [-0.03, -0.1, 0]
    bridge = CvBridge()

    def depth_callback(self, depth_img):
        """
        Call back function for acquiring depth image from the Prime sense.

        Args:
            depth_img: depth image stream from the sensor.
        Return:
        """
        try:
            frame = self.bridge.imgmsg_to_cv(depth_img, desired_encoding="passthrough")
        except CvBridgeError, e:
            print e

        # Convert frame to numpy array
        self.depth_image = np.nan_to_num(np.array(frame))
        # print self.depth_image
        # sio.savemat('depth_img.mat', self.depth_image)

    def image_callback(self, cam_image):
        """
        Call back function for acquiring RGB image from the Prime sense.

        Args:
            cam_image: Image from the Prime sense.
        Return:
        """
        try:
            # convert ROS image to OpenCV image
            frame = self.bridge.imgmsg_to_cv(cam_image, "bgr8")
        except CvBridgeError, e:
            print e

        self.img = numpy.array(frame)  # Convert frame to numpy array

    def save_img(self, fname):
        """
        Save image.

        Args:
            fname: Image file name.
        Return:
        """
        cv2.imwrite(fname, self.img)

    def find3Dfrom2D(self, _2d_point):
        """
        Conversion between 2D frame and 3D frame.

        Args:
            _2d_point: Image points from the 2D image.
        Return:
            The corresponding 3D point in the robot base frame.
        """
        (h, w) = self.depth_image.shape
        _2d_point[1] = h - _2d_point[1]

        c_xy = [self.intrinsic[0,2], self.intrinsic[1,2]]
        f_xy = [self.intrinsic[0,0], self.intrinsic[1,1]]
        d = self.depth_image[_2d_point[1], _2d_point[0]]
        x = (_2d_point[0]-c_xy[0])*d/f_xy[0]
        y = (_2d_point[1]-c_xy[1])*d/f_xy[1]
        pt_3d =  transformation.primesense_to_baxter([d, -x, y], self.tros)
        c_pt_3d = [pt_3d[i] + self.transformation_error[i] for i,k in enumerate(pt_3d)]
        return c_pt_3d

    def __init__(self, t=None):
        rospy.Subscriber("/camera/depth/image", Image, self.depth_callback)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.tros = t
        self.intrinsic = numpy.array([line.strip('\n').split(' ') for line in open("parameters/primesense_intrinsic.txt")])
        self.intrinsic = self.intrinsic.astype(float)


def main():
    # MOVE THE ROBOT WITH INTERACTIVE UI
    rospy.init_node('baxter_vision')
    # v = VisionInterface()
    d = DepthInterface()
    rospy.spin()

    
if __name__ == '__main__':
    sys.exit(main())
