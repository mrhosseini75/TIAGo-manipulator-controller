#!/usr/bin/env python3
"""
.. module:: findObject
:platform: Linux
:synopsis: Python module for finding object
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Subscribes to:
    /xtion/rgb/image_raw
Publishes to:
    /sofar/target_pose/relative
    /sofar/target_pose/relative/stamped

:Find Object node description:
    Allows TIAGo to find relative position of the object (cup) in Gazebo's 3d enviorment.
"""

# Header 
from sensor_msgs.msg import Image  
import rospy
import ros_numpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import mediapipe as mp
from scipy.spatial.transform import Rotation as R

# Define mobile real-time 3D object detection solution 
mp_objectron = mp.solutions.objectron

# Define global variabile 
global pub_target_rel_pose
global pub_target_rel_pose_stamped


def recognize(image):
    """
        recognize function for find the object in the image taken from media pipe and replace 
        the coordinates of the object inside the Pose matrix to find position and orientation

    Args:
        image (msg): use available media pipe information uploaded
    Returns:
        msg(float): return relative position of object respect xtion_rgb_frame 
    """
    # Define global variabile 
    global pub_target_rel_pose
    global pub_target_rel_pose_stamped


    # Define static image model carried from media pipe objectron
    with mp_objectron.Objectron(
            static_image_mode=False,
            max_num_objects=1,
            # confidence interval
            min_detection_confidence=0.5,
            # define object 
            model_name='Cup') as objectron:
        
        # Get results as image
        results = objectron.process(ros_numpy.numpify(image))

        if results.detected_objects:
            # generate status 
            rospy.loginfo("Object found")

            # a representation of pose in free space, composed of position & orientation. 
            messageTargetPose = Pose()
            # if object detected substitute coordinate use result to do translation 
            # translation coordinate (z,x,y)[meter]
            p = results.detected_objects[0].translation
            print(-p[2], p[0], p[1])

            # if object detected substitute coordinate use result to do rotation as quaternions
            # *** Rotation in 3 dimensions can be represented using unit norm quaternions***.
            # quaternion coordinate (x,y,z,w) [rad]
            q = R.from_matrix(results.detected_objects[0].rotation).as_quat()

            # find orientation via quaternions
            messageTargetPose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            # find position via transformation 
            messageTargetPose.position = Point(-p[2], p[0], p[1])
            
            # publish messageTargetPose on target relative position 
            pub_target_rel_pose.publish(messageTargetPose)
            
            # substitute in pose_stamped, a Pose with reference coordinate frame & timestamp
            pose_stamped = PoseStamped(pose = messageTargetPose)
            # pose_stamped use as frame_id --> frame of rgd camera 
            pose_stamped.header.frame_id='xtion_rgb_frame'

            # publish relative postion of object on pose_stamped 
            pub_target_rel_pose_stamped.publish(pose_stamped)


if __name__ == '__main__':

    # ros node initialization --> Find Object 
    rospy.init_node('FindObject')

    # subscribe to rgb camera to take image information
    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, recognize)

    # publishe to traget_pose/relative positione of object detected 
    pub_target_rel_pose = rospy.Publisher(
        '/sofar/target_pose/relative', Pose, queue_size=1)

    # publishe to traget_pose/relative/stamped to print positione of object detected 
    pub_target_rel_pose_stamped = rospy.Publisher(
        '/sofar/target_pose/relative/stamped', PoseStamped, queue_size=1)   

    # start infinite loop until it receivces a shutdown  signal (Ctrl+C)
    rospy.spin()
