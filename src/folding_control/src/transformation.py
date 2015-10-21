import roslib
import rospy
roslib.load_manifest('baxter_dan')
import tf
from geometry_msgs.msg import (Point, PointStamped)
from std_msgs.msg import Header


def primesense_to_baxter(point, tros=None):
    """
    Transformation from the Kinect 2D points to robot 3D position.

    Args:
        point: 2D points from the Kinect 2D image.
        tros: Transformer listener from ROS.
    Return:
        The corresponding 3D points in robot world.
    """
    p = Point(point[0],point[1],point[2])
    hdr = Header(stamp=rospy.Time(), frame_id='/camera_link')
    #hdr = Header(stamp=rospy.Time(), frame_id='/world')
    pt = PointStamped(header=hdr, point=p)
    if tros == None:
        tros = tf.TransformListener()
    
    tros.setUsingDedicatedThread(True)
    tros.waitForTransform('/world','/camera_link',rospy.Time(), rospy.Duration(2))
    t =  tros.lookupTransform('world','camera_link',rospy.Time())
    #tros.waitForTransform('/camera_link','/world',rospy.Time(), rospy.Duration(2))
    #t =  tros.lookupTransform('camera_link','world',rospy.Time())
    result = tros.transformPoint('/world',pt) 
    return [result.point.x, result.point.y, result.point.z]



