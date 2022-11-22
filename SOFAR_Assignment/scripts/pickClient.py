  #!/usr/bin/env python3
"""
.. module:: pickClient
:platform: Linux
:synopsis: Python module for prepare TIAGo to pregrasp configuration to take object
:moduleauthor: M.Macchia S.Pedrazzi M.Haji Hosseini

Subscribes to:
    '/sofar/target_pose/relative'
Publishes to:
    '/head_controller/command'
    '/mobile_base_controller/cmd_vel'
Service :
    '/sofar/rel_to_absolute_pose' --> pass position info to other node
    '/sofar/approach_object'      --> pass trajectory to reach object to other node 
    '/sofar/pick_object'          --> pass info to pick up object to other node 

:Pick Clinet node description:
   1. TIAGo is placed at home configuaration
   2. Moves its head until object is found
   3. Takes information relating to position 
   4. In case of finding displacement with between object and TIAGo, TIAGo moves to find the best position to take object 
   5. Puts in pregrasp configuration to take the object
"""
# Header 
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, Point
import math
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from SOFAR_Assignment.srv import RelToAbsolute, RelToAbsoluteResponse
from SOFAR_Assignment.srv import ApproachObject, ApproachObjectResponse
from std_srvs.srv import Empty, EmptyRequest

# Define global variabile
global action_client
global pub_head_controller
global sub_target_rel_pose
global head_2_movement
global object_found
global pmgoal
global count
global displacement
global object_rel_pose
global object_abs_pose


def go_to_home_position():
    """
        go_to_home_position function use Paly Motion planning to reach initial positon of TIAGo
    """   
    # Define global variabile 
    global action_client
    global pmgoal
    # generate status
    rospy.loginfo("Go into the home position")

    # define Play motion configuration
    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'home'
    pmgoal.skip_planning = False
    # send play motion goal --> home configuration & wait till reach position 
    action_client.send_goal_and_wait(pmgoal)

    rospy.loginfo("Done.")


def move_head():
    """
        move_head function use a while loop to move head joints until object is found by TIAGo
    """
    # Define global variabile
    global pub_head_controller
    global head_2_movement
    global object_found

    # generate status
    rospy.loginfo("Moving head")

    # loop function until object was found
    while not object_found:
        # trajectory_msgs --> JointTrajectory
        trajectory = JointTrajectory()
        # Define joint in use in order to move head 
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        # ***The action can be used to give a trajectory to the head expressed in several waypoints***. 
        trajectory_points = JointTrajectoryPoint()
        # change coordinate just along z axis
        trajectory_points.positions = [0.0, head_2_movement]
        # Define time action 
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)
        
        pub_head_controller.publish(trajectory)
        # interval to start next movement
        rospy.sleep(0.8)
        # Define head movment in a lowering cycle with -0.1 step to a max -1, until object is detected
        head_2_movement = max(-1, head_2_movement-0.1)

    rospy.loginfo("Done.")


def get_object_relative_pose(msg):
    """
        function to take relative position of object (cup)
    Args:
        msg(float): get object_rel_pose
    """   
    # Define global variabile 
    global object_found
    global sub_target_rel_pose
    global count
    global object_rel_pose

    object_rel_pose = msg
    # call function to find absolute pose from relative 
    get_absolute_object_pose()

    # count is a counter used to make sure that all the object fits the camera.
    # In order to ensure this, seen the object recognition frequency,
    # the object has to be found for at least 20 times
    count += 1
    if count == 20:
        object_found = True


def prepare_robot():
    """
        Use Play motion planning to put TIAGo in pregrasp configuration in order to reach object (cup) 
    """
    # Define global variabile
    global action_client
    global pmgoal

    # generate status
    rospy.loginfo("Go into the initial position")

    # define Play motion configuration
    pmgoal = PlayMotionGoal()
    pmgoal.motion_name = 'pregrasp'
    pmgoal.skip_planning = False
    # send play motion goal --> pregrasp configuration & wait till reach position
    action_client.send_goal_and_wait(pmgoal)

    rospy.loginfo("Done.")


def adjust_position():
    """
        adjust_position function control absolute position of object w.r.t TIAGo 
        in case of find displacemente,TIAGo moves to put his hand in a better position for better grip 
    """
    # Define global variabile
    global displacement

    # generate displacement status 
    rospy.loginfo("Found displacement of %f, moving the robot",displacement)
    
    # Publish velocity to change position of TIAGo
    pub_vel = rospy.Publisher(
        '/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    velocity = Twist()

    # Define angular velocity to turn with w.r.t z axis
    # prepare to move to the sides of the table
    for i in range(21):
        velocity.angular.z = -math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    # Define linear velocity to move along x axis to reach necessary displacement
    for i in range(math.ceil(displacement*10/0.25)):
        velocity.linear.x = 0.25
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.linear.x = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    # Define angular velocity to turn with w.r.t z axis
    # reverse movment to get to the table
    for i in range(20):
        velocity.angular.z = math.pi/4
        pub_vel.publish(velocity)
        rospy.sleep(0.1)

    velocity.angular.z = 0
    pub_vel.publish(velocity)
    rospy.sleep(1)

    rospy.loginfo("Done.")


def get_absolute_object_pose():
    """
        function to take absolute position of object (cup) frome relative pose 
    """ 
    # Define global variabile
    global object_abs_pose
    global object_rel_pose

    try:
        # service pass info on geometry_msgs/PoseStamped relative_pose 
        rel_to_absolute_pose = rospy.ServiceProxy('/sofar/rel_to_absolute_pose', RelToAbsolute)
        msg = PoseStamped( pose = object_rel_pose)
        # w.r.t xtion_rgb_frame
        msg.header.frame_id = 'xtion_rgb_frame'

        object_abs_pose = rel_to_absolute_pose(msg).absolute_pose.pose
     # in case of lack of communication with server 
    except rospy.ServiceException as e:
        rospy.logerr("Could not connect to /sofar/rel_to_absolute_pose service")
        exit()

if __name__ == '__main__':

    # ros node initialization --> Pick Client 
    rospy.init_node('PickClient')

    # use action client class to to communicate to the action server the paly motion 
    action_client = SimpleActionClient('/play_motion', PlayMotionAction)

    # time out in case of lack communication between client & server 
    if not action_client.wait_for_server(rospy.Duration(20)):
        rospy.logerr("Could not connect to /play_motion")
        exit()

    pmgoal = PlayMotionGoal()

    # call function to put initial pose 
    go_to_home_position()

    # publsih to head controller the join trajectory 
    pub_head_controller = rospy.Publisher(
        '/head_controller/command', JointTrajectory, queue_size=1)

    # subscribe to target relative pose the position 
    sub_target_rel_pose = rospy.Subscriber(
        '/sofar/target_pose/relative', Pose, get_object_relative_pose)

    # Define empty value 
    head_2_movement = 0
    object_found = False
    count = 0

    # call function to move head orientation    
    move_head()

    # service pass info to geometry_msgs/PoseStamped relative_pose
    rospy.wait_for_service('/sofar/rel_to_absolute_pose')

    # definition of displacement 
    displacement = 0.1-object_abs_pose.position.y

    if displacement > 0:
        #call function in case of necessary displacement
        adjust_position()

    # call function to prepare pregras configuration 
    prepare_robot()
    # wait to compile the configuration
    rospy.sleep(2)

    # wating for info from service --> geometry_msgs/Pose pose
    rospy.wait_for_service('/sofar/approach_object')
    try:
        sub_target_rel_pose.unregister()

        approach_object = rospy.ServiceProxy('/sofar/approach_object', ApproachObject)
        approach_object(object_abs_pose)

    # in case of lack of communication with server 
    except rospy.ServiceException as e:
        rospy.logerr("Could not connect to /sofar/approach_object service")
        exit()

    # wating for info from service
    rospy.wait_for_service('/sofar/pick_object')
    try:
        sub_target_rel_pose.unregister()

        pick_object = rospy.ServiceProxy('/sofar/pick_object', Empty)
        pick_object()
    # in case of lack of communication with server 
    except rospy.ServiceException as e:
        rospy.logerr("Could not connect to /sofar/pick_object service")
        exit()    