#!/usr/bin/env python
"""
.. module:: getAbsolutePose
:platform: Linux
:synopsis: Python module for get absolute position of object (cup)
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Service :
    /sofar/rel_to_absolute_pose --> pass relative position of object to absolute position to other node

:Get Absolute Pose node description:  
    the necessary step to find the absolute position of the object w.r.t TIAGo 
    uses tf2 that lets the user keep track of multiple coordinate frames over time.   
"""

# Header
import rospy
import tf2_ros
from tf.transformations import *
from tf2_geometry_msgs import *
import ros_numpy
from geometry_msgs.msg import Pose, Quaternion, PointStamped
from SOFAR_Assignment.srv import RelToAbsolute, RelToAbsoluteResponse

# Defin global variabile 
global tfBuffer
global listener


def compute_absolute_pose(rel_pose):
    """
    compute_absolute_pose function calculates relative pose transform between xtion_rgb_frame & base_footprint, 
    then calculates absolute frame by object in order to reach object correctly
    Args:
        rel_pose (msg): take relative positon of object respect xtion_rgb_frame

    Returns:
        msg(float): return absolute position of object respect base_footprint 
    """

    # Defin global variabile 
    global tfBuffer
    global listener
    
    # generate status 
    rospy.loginfo('New request received')

    # relative pose transformation between xtion_rgb_frame & base_footprint
    trans_base_rel = tfBuffer.lookup_transform('base_footprint',
                                                  rel_pose.relative_pose.header.frame_id, rospy.Time())

    point_from_rel = PointStamped(point=rel_pose.relative_pose.pose.position)

    # pose of model in absolute entity frame
    abs_pose = Pose()

    # transformation to get absolute position 
    abs_pose.position = do_transform_point(
        point_from_rel, trans_base_rel).point

    # ***an orientation is the destination that you reach at the end of a rotation***; 
    # ***the rotation is the route to that destination***
    # calculates rotation between xtion_rgb_frame & base_footprint
    q0 = [trans_base_rel.transform.rotation.x, trans_base_rel.transform.rotation.y,
          trans_base_rel.transform.rotation.z, trans_base_rel.transform.rotation.w]
    # calculates orientation to get abs pose 
    q1 = [rel_pose.relative_pose.pose.orientation.x, rel_pose.relative_pose.pose.orientation.y,
          rel_pose.relative_pose.pose.orientation.z, rel_pose.relative_pose.pose.orientation.w]
    # simply multiply the previous quaternion of the pose by the quaternion representing
    q = quaternion_multiply(q1, q0)

    # Quaternion coordinate (x,y,z,w)[rad]
    abs_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    # substitute relative pose with absolute pose in return_pose
    return_pose = RelToAbsoluteResponse()
    return_pose.absolute_pose.pose=abs_pose
    # use as refrence frame base_footprint
    return_pose.absolute_pose.header.frame_id = 'base_footprint'
    print(return_pose)
    return return_pose


if __name__ == '__main__':

    # ros node initialization --> Get Absolut Pose
    rospy.init_node('GetAbsolutePose')

    # service pass relative to abs position info on --> geometry_msgs/PoseStamped to communicate with other node
    absolute_service = rospy.Service("/sofar/rel_to_absolute_pose",RelToAbsolute,compute_absolute_pose)
    
    # ***provides an easy way to request and receive coordinate frame transform information.***
    # keep track of multiple coordinate frames over time.
    tfBuffer = tf2_ros.Buffer()
    # constructor for transform listener
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("Service ready.")
    # start infinite loop until it receivces a shutdown  signal (Ctrl+C)
    rospy.spin()