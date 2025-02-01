#!/usr/bin/env python

import rospy
import threading
import actionlib
from motion_test_pkg.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult, MoveRobotGoal
from geometry_msgs.msg import Point


#!/usr/bin/env python3

import rospy
import random
import moveit_commander
import moveit_msgs
import tf2_ros

import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

import sys
import rospy
import copy
import PyKDL 
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Pose
import tf
from geometry_msgs.msg import PoseStamped


from motion_test_pkg.srv import BoxSpawner, BoxSpawnerRequest, RandomPose
from motion_test_pkg.srv import BoxAttach, BoxAttachRequest
from moveit_commander import RobotCommander, MoveGroupCommander
import tf2_ros
import geometry_msgs.msg
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
from motion_test_pkg.msg import PickAndPlaceAction, PickAndPlaceFeedback, PickAndPlaceResult
from motion_test_pkg.action import PickAndPlaceAction


def frame_to_pose(frame):

	pose_result = Pose()
	pose_result.position.x = frame.p[0] 
	pose_result.position.y = frame.p[1] 
	pose_result.position.z = frame.p[2] 
	ang = frame.M.GetQuaternion() 
	pose_result.orientation.x = ang[0] 
	pose_result.orientation.y = ang[1] 
	pose_result.orientation.z = ang[2] 
	pose_result.orientation.w = ang[3]
	return pose_result


def get_random_pose():
    """Call the random pose generator service and get random coordinates."""
    rospy.wait_for_service('/random_pose_generator')
    try:
        random_pose_service = rospy.ServiceProxy('/random_pose_generator', RandomPose)
        response = random_pose_service()
        return response.x, response.y, response.z
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None, None


class MoveRobotServer:

    def __init__(self):

        self.server = actionlib.SimpleActionServer('/process_action', MoveRobotAction, self.execute, False)
        self.target_position = Point(5.0, 5.0, 0.0)  # Define target coordinates here
        self.server.start()

    def execute(self, goal):

        feedback = MoveRobotFeedback()
        result = MoveRobotResult()

        if goal.command != "start":
            rospy.logwarn("Server: Received invalid command")
            result.success = False
            self.server.set_aborted(result)
            return

        rospy.loginfo(f"Server: Moving to position: {self.target_position}")

        ###########################################################

        rospy.init_node("motion_control_node")

        rospy.loginfo("Starting motion control node")

        # Wait for required services
        rospy.wait_for_service('/scene_spawner/spawn_box')
        rospy.wait_for_service('/scene_spawner/attach_release_box')

        # Service proxies
        spawn_box_service = rospy.ServiceProxy('/scene_spawner/spawn_box', BoxSpawner)
        attach_box_service = rospy.ServiceProxy('/scene_spawner/attach_release_box', BoxAttach)
  
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        #robot = moveit_commander.RobotCommander()
        #scene = moveit_commander.PlanningSceneInterface()

        arm1 = moveit_commander.MoveGroupCommander("arm_one")
        gripper1 = moveit_commander.MoveGroupCommander("one_gripper")

        arm2 = moveit_commander.MoveGroupCommander("arm_two")
        gripper2 = moveit_commander.MoveGroupCommander("two_gripper")

        arm1.clear_pose_targets()
        gripper1.clear_pose_targets()

        arm2.clear_pose_targets()
        gripper2.clear_pose_targets()
          
        ###################################

        transfer_x = 0
        transfer_y = 0
        transfer_z = 0.7

        top_x, top_y, top_z  = (0, 0, 0.7)
    
        print("top_x, top_y, top_z", top_x, top_y, top_z )

        object_frame_tr = PyKDL.Frame()    
        object_frame_tr.p = PyKDL.Vector(top_x, top_y, top_z)
        gripper_length = 0.24
    
        goal_frame_tr = copy.deepcopy(object_frame_tr)
        goal_frame_tr.p[2] += gripper_length
        goal_frame_tr.M.DoRotX(3.14)  # Gripper pointing down
    
        goal_pose_tr = frame_to_pose(goal_frame_tr)
        pose_stamped_target_wrist_tr = geometry_msgs.msg.PoseStamped()
        pose_stamped_target_wrist_tr.header.frame_id = "world"  # Adjust if needed
        pose_stamped_target_wrist_tr.pose = goal_pose_tr
        pose_stamped_target_offset_wrist_tr = copy.deepcopy(pose_stamped_target_wrist_tr)

        arm1.set_pose_target(pose_stamped_target_offset_wrist_tr)
        success, plan_arm1_1, _, _ = arm1.plan()

        print("sucess", success)

        arm2.set_pose_target(pose_stamped_target_offset_wrist_tr)
        success, plan_arm2_1, _, _ = arm1.plan()
   
        print("sucess", success)

        success = False

        object_frame_top = PyKDL.Frame()   

        world_frame = "world"
        base_link_frame = "one_base_link"
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(world_frame, base_link_frame, rospy.Time(0), rospy.Duration(3.0))
        (trans_one, rot) = tf_listener.lookupTransform(world_frame, base_link_frame, rospy.Time(0))

        base_link_frame = "two_base_link"
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(world_frame, base_link_frame, rospy.Time(0), rospy.Duration(3.0))
        (trans_two, rot) = tf_listener.lookupTransform(world_frame, base_link_frame, rospy.Time(0))

        while not success:

            top_x, top_y, top_z = get_random_pose()  

        #################################
        
            distance_one = (trans_one[0]-top_x) **2 + (trans_one[1]-top_y)**2   # (x, y, z)
            distance_two = (trans_two[0]-top_x) **2 + (trans_two[1]-top_y)**2   # (x, y, z)

        ##################################

            print("top_x, top_y, top_z", top_x, top_y, top_z )

            #object_frame_top = PyKDL.Frame()    
            object_frame_top.p = PyKDL.Vector(top_x, top_y, top_z)
            gripper_length = 0.24
    
            goal_frame_top = copy.deepcopy(object_frame_top)
            goal_frame_top.p[2] += gripper_length 
            goal_frame_top.M.DoRotX(3.14)  # Gripper pointing down
    
            goal_pose_top = frame_to_pose(goal_frame_top)
            pose_stamped_target_wrist_top = geometry_msgs.msg.PoseStamped()
            pose_stamped_target_wrist_top.header.frame_id = "world"  # Adjust if needed
            pose_stamped_target_wrist_top.pose = goal_pose_top
            pose_stamped_target_offset_wrist_top = copy.deepcopy(pose_stamped_target_wrist_top)

            # group.set_pose_target(target_pose)
            arm1.set_pose_target(pose_stamped_target_offset_wrist_top)
            success, plan_arm1, _, _ = arm1.plan()
        
            print(success)

        print(distance_one, distance_two)        
    
        if top_x is not None:    

            top_box_request = BoxSpawnerRequest(name="top_box", x=top_x, y=top_y, base=False)
            spawn_box_service(top_box_request)
            rospy.loginfo("Spawned top box")
    
        #pose_stamped_target_offset_wrist.pose.position.z += 0.1

        success = False

        object_frame_base = PyKDL.Frame()   
        
        while not success:

            base_x, base_y, base_z = get_random_pose()        
         
            object_frame_base.p = PyKDL.Vector(base_x, base_y, base_z)
            gripper_length = 0.24
    
            goal_frame_base = copy.deepcopy(object_frame_base)
            goal_frame_base.p[2] += gripper_length + 0.055
            goal_frame_base.M.DoRotX(3.14)  # Gripper pointing down
    
            goal_pose_base = frame_to_pose(goal_frame_base)
            pose_stamped_target_wrist_base = geometry_msgs.msg.PoseStamped()
            pose_stamped_target_wrist_base.header.frame_id = "world"  # Adjust if needed
            pose_stamped_target_wrist_base.pose = goal_pose_base
            pose_stamped_target_offset_wrist_base = copy.deepcopy(pose_stamped_target_wrist_base)

            # group.set_pose_target(target_pose)
            arm2.set_pose_target(pose_stamped_target_offset_wrist_base)
            success, plan_arm2, _, _ = arm2.plan()

            print(success)

        if base_x is not None:
            base_box_request = BoxSpawnerRequest(name="base_box", x=base_x, y=base_y, base=True)
            spawn_box_service(base_box_request)
            rospy.loginfo("Spawned base box")
    
    #########################################
    
        # Open gripper
        rospy.loginfo("Opening gripper")
        gripper1.set_named_target('gripper1 open')
        gripper1.go(wait=True)
    
        # Move to target pose
        rospy.loginfo("Moving to target pose")
        #arm1.set_pose_target(pose_stamped_target_offset_wrist_base)
        arm1.execute(plan_arm1, wait=True)

        #   arm1.go(wait=True)
    
        # Attach the top box
        rospy.loginfo("Attaching top box")
        box_attach_request_1 = BoxAttachRequest(robot_name="robot1", box_name="top_box", attach=True)
        attach_box_service(box_attach_request_1)

        # Close gripper
        rospy.loginfo("Closing gripper")
        gripper1.set_named_target('gripper1 pinch')
        gripper1.go(wait=True)

    #####################33
    ###################################

        transfer_x = 0
        transfer_y = 0
        transfer_z = 0.7

        top_x, top_y, top_z  = (0, 0, 0.7)
    
        print("top_x, top_y, top_z", top_x, top_y, top_z )

        object_frame_tr = PyKDL.Frame()    
        object_frame_tr.p = PyKDL.Vector(top_x, top_y, top_z)
        gripper_length = 0.24
    
        goal_frame_tr = copy.deepcopy(object_frame_tr)
        goal_frame_tr.p[2] += gripper_length
        goal_frame_tr.M.DoRotX(3.14)  # Gripper pointing down
    
        goal_pose_tr = frame_to_pose(goal_frame_tr)
        pose_stamped_target_wrist_tr = geometry_msgs.msg.PoseStamped()
        pose_stamped_target_wrist_tr.header.frame_id = "world"  # Adjust if needed
        pose_stamped_target_wrist_tr.pose = goal_pose_tr
        pose_stamped_target_offset_wrist_tr = copy.deepcopy(pose_stamped_target_wrist_tr)

        # group.set_pose_target(target_pose)
        arm1.set_pose_target(pose_stamped_target_offset_wrist_tr)
        success, plan_arm1_1, _, _ = arm1.plan()
        arm1.execute(plan_arm1_1, wait=True)

        ######################
    
        gripper1.set_named_target('gripper1 open')
        gripper1.go(wait=True)
       
        box_attach_request_1_2 = BoxAttachRequest(robot_name="robot1", box_name="top_box", attach=False)
        attach_box_service(box_attach_request_1_2)

        # Move to home position
        rospy.loginfo("arm1 moving to home position")
        arm1.set_named_target('arm1 home')
    
        success, plan_arm1, _, _ = arm1.plan()
        arm1.execute(plan_arm1, wait=True)

        gripper2.set_named_target('girpper2 open')
        gripper2.go(wait=True)

        arm2.set_pose_target(pose_stamped_target_offset_wrist_tr)
        success, plan_arm2_1, _, _ = arm2.plan()
        arm2.execute(plan_arm2_1, wait=True)
   
        box_attach_request_2_1 = BoxAttachRequest(robot_name="robot2", box_name="top_box", attach=True)
        attach_box_service(box_attach_request_2_1)

        gripper2.set_named_target('girpper2 pinch')
        gripper2.go(wait=True)

        #pose_stamped_target_offset_wrist_base = copy.deepcopy(pose_stamped_target_wrist_base)
        # group.set_pose_target(target_pose)

        arm2.clear_pose_targets()

        arm2.set_pose_target(pose_stamped_target_offset_wrist_base)
        success, plan_arm2, _, _ = arm2.plan()
        arm2.execute(plan_arm2, wait=True)

        gripper2.set_named_target('girpper2 open')
        gripper2.go(wait=True)

        box_attach_request_2_2 = BoxAttachRequest(robot_name="robot2", box_name="top_box", attach=False)
        attach_box_service(box_attach_request_2_2)

         # Move to home position
        rospy.loginfo("arm2 moving to home position")
        arm2.set_named_target('arm2 home')
        success, plan_arm2_3, _, _ = arm2.plan()
        arm2.execute(plan_arm2_3, wait=True)

        rospy.loginfo("Motion control complete")

        ###########################################################            
            
        feedback.status = f"Distance remaining: {distance_remaining:.2f} meters"
        self.server.publish_feedback(feedback)
        rospy.sleep(0.1)

        rospy.loginfo('Server: Target reached!')
        result.success = True
        self.server.set_succeeded(result)

    
    def calculate_distance(self, target_position):
        return ((target_position.x)**2 + (target_position.y)**2 + (target_position.z)**2)**0.5




class MoveRobotClient:

    def __init__(self):
        self.client_thread = threading.Thread(target=self.run_client)
        self.client_thread.start()

    def run_client(self):
        rospy.sleep(1)  # Wait for server to start
        client = actionlib.SimpleActionClient('/process_action', MoveRobotAction)
        rospy.loginfo('Client: Waiting for server...')
        client.wait_for_server()

        goal = MoveRobotGoal()
        goal.command = "start"  # Command sent to the server

        rospy.loginfo('Client: Sending goal...')
        client.send_goal(goal, feedback_cb=self.feedback_callback)

        client.wait_for_result()
        result = client.get_result()
        rospy.loginfo(f'Client: Result - Success = {result.success}')

    def feedback_callback(self, feedback):
        rospy.loginfo(f'Client: Feedback - {feedback.status}')



if __name__ == '__main__':
    rospy.init_node('motion_control')
    server = MoveRobotServer()
    client = MoveRobotClient()
    rospy.spin()
