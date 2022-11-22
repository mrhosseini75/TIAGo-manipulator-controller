#!/usr/bin/env python
"""
.. module:: pickClient
:platform: Linux
:synopsis: Python module to pick up & put down object 
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Publishes to:
    '/gripper_controller/command'
Service :
    "/sofar/approach_object" 
    "/sofar/pick_object"

:Pick Object node description:
    1. TIAGo's manipulator goes to the object
    2. Closes the grippers
    3. TIAGo lifts the object from the table and puts it back on the table
    4. Opens gripper
   
"""

# Header
from operator import sub
import rospy
from tf.transformations import *
from tf2_geometry_msgs import *
from geometry_msgs.msg import Pose, Quaternion, Point
from scipy.spatial.transform import Rotation as R
import moveit_commander
from moveit_msgs.msg import Grasp, CollisionObject
from moveit_commander.conversions import pose_to_list
import sys
from std_srvs.srv import Empty, EmptyRequest
from SOFAR_Assignment.srv import ApproachObject, ApproachObjectResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Define global variabile 
global sub_target_abs_pose
global robot
global scene
global move_group
global grasp_pose


def goToObject(object_pose):
    """
        Puts the manipulator in the right position to take the object
    Args:
        object_pose (msg): get object position information in quaternion coordinate

    Returns:
        msg: gets into correct configuration
    """    
    # Define global variabile 
    global move_group
    global grasp_pose

    # define quaternion coordinate (x,y,z,w)[rad] of object found on the table
    object_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
    
    # save grasp pose
    grasp_pose = copy.deepcopy(object_pose.pose)
    # define distance between EE & object 
    grasp_pose.position.y-=0.22

    # pre-grasp pose with limits
    object_pose.pose.position.y = -0.2
    object_pose.pose.position.z += 0.1

    # generate robot status
    rospy.loginfo('attempting to reach:')
    rospy.loginfo([object_pose.pose.position.x,
                   object_pose.pose.position.y, object_pose.pose.position.z])
    # call joint group in order to reach object 
    move_group.set_pose_target(object_pose.pose)

    # continue to move the manipulator group to reach the desired position
    move_group.go(wait=True)

    # stop manipulator group after reaching position
    move_group.stop()

    move_group.clear_pose_targets()

    response = ApproachObjectResponse()
    response.result = True
    return response


def pick(msg):
    """
        series of movements to lift and put object back on the table; 
        using move_group of TIAGo's manipolator
    Args:
        msg (float) 
    """
    # Define global variabile     
    global move_group
    global grasp_pose


    #move to grasp_pose
    rospy.loginfo('move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo('closing gripper...')
    close_gripper()
    rospy.sleep(1)

    # define post_grasp_pose
    post_grasp_pose = copy.deepcopy(grasp_pose)
    post_grasp_pose.position.z += 0.3

    # move to post_grasp_pose
    rospy.loginfo('move to post_grasp_pose...')
    move_group.set_pose_target(post_grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #move to grasp_pose
    rospy.loginfo('move to grasp_pose...')
    move_group.set_pose_target(grasp_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo('opening gripper...')
    open_gripper()
    rospy.sleep(1)

    rospy.loginfo('Done.')


def close_gripper():
    """
        function for close gripper of TIAGo's EE when reach object 
    """

    # publish gripper status on joint trajectory when TIAGo close gripper 
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    # loop continues until the grippers close well
    for i in  range(10):
        trajectory = JointTrajectory()
        # call joint group for take object 
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()
        # define the distance to the right & left of the two gripper w.r.t the object
        trajectory_points.positions = [0.0, 0.0]
        # time action  
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        # interval to start next movement
        rospy.sleep(0.1)  

def open_gripper():
    """
        function for open gripper of end effector when TIAGo made mission  
    """
    # publish gripper status on joint trajectory when TIAGo open gripper
    pub_gripper_controller = rospy.Publisher(
        '/gripper_controller/command', JointTrajectory, queue_size=1)

    # loop continues until the grippers open &  object is released
    for i in  range(10):
        trajectory = JointTrajectory()
        trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        trajectory_points = JointTrajectoryPoint()
        # define the distance to the right & left of the two gripper for opening
        trajectory_points.positions = [0.044, 0.044]
        # time action  
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)

        pub_gripper_controller.publish(trajectory)
        # interval to start next movement
        rospy.sleep(0.1)

        


# def pickObject(object_pose):
#     global move_group

#     # Setting grasp pose
#     grasp = Grasp()
#     grasp.grasp_pose.header.frame_id = "base_footprint"
#     grasp.grasp_pose.pose.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
#     grasp.grasp_pose.pose.position = object_pose.position
#     # grasp.grasp_pose.pose.position.y = max(-0.2, object_pose.pose.position.y-0.3)

#     # Setting pre-grasp approach
#     grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
#     grasp.pre_grasp_approach.direction.vector.y = 1.0
#     grasp.pre_grasp_approach.min_distance = 0
#     grasp.pre_grasp_approach.desired_distance = 0

#     # Setting post-grasp retreat
#     grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
#     grasp.post_grasp_retreat.direction.vector.z = 1.0
#     grasp.post_grasp_retreat.min_distance = 0.0
#     grasp.post_grasp_retreat.desired_distance = 0.0

#     # Setting posture of eef before grasp
#     openGripper(grasp.pre_grasp_posture)

#     # Setting posture of eef during grasp
#     closedGripper(grasp.grasp_posture)

#     print(grasp)

#     move_group.pick("object", grasp)


# def openGripper(posture):

#     # Add both finger joints of the robot.
#     posture.joint_names.append("gripper_left_finger_joint")
#     posture.joint_names.append("gripper_right_finger_joint")

#     # Set them as open, wide enough for the object to fit.
#     posture.points.append(JointTrajectoryPoint())
#     posture.points[0].positions.append(0.04)
#     posture.points[0].positions.append(0.04)
#     posture.points[0].time_from_start = rospy.Duration(0.5)


# def closedGripper(posture):

#     # Add both finger joints of the robot.
#     posture.joint_names.append("gripper_left_finger_joint")
#     posture.joint_names.append("gripper_right_finger_joint")

#     # Set them as closed. */
#     posture.points.append(JointTrajectoryPoint())
#     posture.points[0].positions.append(0.00)
#     posture.points[0].positions.append(0.00)
#     posture.points[0].time_from_start = rospy.Duration(0.5)


if __name__ == '__main__':

    # ros node initialization --> Pick Object
    rospy.init_node('PickObject')

    # ***moveit --> Simple interfaces are available for motion planning,***
    # ***computation of Cartesian paths, and pick and place ***
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    # call group of joint to adjust the height of TIAGo
    move_group = moveit_commander.MoveGroupCommander('arm_torso')

    # service pass info to geometry_msgs/Pose pose
    approach_object_service = rospy.Service(
        "/sofar/approach_object", ApproachObject, goToObject)
    # service pass info to geometry_msgs/Pose pose
    grasp_object_service = rospy.Service(
        "/sofar/pick_object", Empty, pick)

    rospy.loginfo("Service ready.")
    # call function to open gripper after TIAGo made a mission
    open_gripper()

    # object_pose = Pose()
    # object_pose.position.x = 0.6
    # object_pose.position.y = 0.1
    # object_pose.position.z = 0.9 #mettiamo z+0.1

    # goToObject(object_pose)
    # pick()
    #pickObject(object_pose)

    # start infinite loop until it receivces a shutdown  signal (Ctrl+C)
    rospy.spin()
