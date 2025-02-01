#!/usr/bin/env python3

import rospy
import threading
import actionlib
#from motion_test_pkg.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult, MoveRobotGoal
from geometry_msgs.msg import Point



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

#from motion_test_pkg.msg import PickAndPlaceAction, PickAndPlaceFeedback, PickAndPlaceResult
from motion_test_pkg.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult




class MoveRobotServer:
    """
    MoveRobotServer is a ROS action server that handles the motion planning and execution
    for a robotic arm. It receives goals for moving the robot to specified poses and
    provides feedback and results on the execution status.

    Attributes:
        action_name (str): The name of the action server.
        server (SimpleActionServer): The action server instance.
        arm (MoveGroupCommander): The MoveIt commander for the robotic arm.
        gripper (MoveGroupCommander): The MoveIt commander for the gripper.
    """

    def __init__(self):
        """
        Initializes the MoveRobotServer with the given action name.

        Args:
            name (str): The name of the action server.
        """

        self.server = actionlib.SimpleActionServer('/process_action', MoveRobotAction, self.execute, False)
        self.target_position = Point(5.0, 5.0, 0.0)  # Define target coordinates here
        self.server.start()

    def execute(self, goal):
        """
        Callback function that is called when a new goal is received by the action server.
        This function handles the motion planning and execution to move the robot to the
        specified goal pose.

        Args:
            goal (MoveRobotGoal): The goal containing the target pose for the robot.
        """

        feedback = MoveRobotFeedback()
        result = MoveRobotResult()

        if goal.command != "start":
            rospy.logwarn("Server: Received invalid command")
            result.success = False
            self.server.set_aborted(result)
            return
        
        ###########################################################
        #rospy.loginfo("Starting motion control node")
        feedback.status = f"Starting motion control node"
        self.server.publish_feedback(feedback)

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

        arm1_0 = moveit_commander.MoveGroupCommander("arm_one")
        gripper1_0 = moveit_commander.MoveGroupCommander("one_gripper")

        arm2_0 = moveit_commander.MoveGroupCommander("arm_two")
        gripper2_0 = moveit_commander.MoveGroupCommander("two_gripper")

        arm1_0.clear_pose_targets()
        gripper1_0.clear_pose_targets()

        arm2_0.clear_pose_targets()
        gripper2_0.clear_pose_targets()


        world_frame = "world"
        base_link_frame = "one_base_link"
        trans_one = self.get_robot_coordinates(world_frame, base_link_frame)
        base_link_frame = "two_base_link"
        trans_two = self.get_robot_coordinates(world_frame, base_link_frame)
          
        ###################################
        ### generating boxes     
        ###################################
   
        success = False

        object_frame_top = PyKDL.Frame()   
       
        while not success:

            top_x, top_y, top_z = self.get_random_pose()  
                   
            distance_top_one = self.calculate_distance(trans_one, (top_x, top_y, top_z))
            distance_top_two = self.calculate_distance(trans_two, (top_x, top_y, top_z))

            if distance_top_one < distance_top_two :
                arm1 = copy.copy(arm1_0)
                gripper1 = copy.copy(gripper1_0)
                gripper1_open = 'gripper1 open'
                gripper1_close = 'gripper1 pinch'
                robot1_box = "robot1"
                arm1_home = 'arm1 home'
                arm1_name ="arm1"
                flag_top = 1     

            else:
                arm1 = copy.copy(arm2_0)
                gripper1 = copy.copy(gripper2_0)
                gripper1_open = 'girpper2 open'
                gripper1_close = 'girpper2 pinch'
                robot1_box = "robot2"
                arm1_home = 'arm2 home'
                arm1_name ="arm2"
                flag_top = 2     

        ##################################
            
            object_frame_top.p = PyKDL.Vector(top_x, top_y, top_z)
            gripper_length = 0.24
    
            goal_frame_top = copy.deepcopy(object_frame_top)
            goal_frame_top.p[2] += gripper_length 
            goal_frame_top.M.DoRotX(3.14)  # Gripper pointing down
    
            goal_pose_top = self.frame_to_pose(goal_frame_top)
            pose_stamped_target_wrist_top = geometry_msgs.msg.PoseStamped()
            pose_stamped_target_wrist_top.header.frame_id = "world"  # Adjust if needed
            pose_stamped_target_wrist_top.pose = goal_pose_top
            pose_stamped_target_offset_wrist_top = copy.deepcopy(pose_stamped_target_wrist_top)

            arm1.set_pose_target(pose_stamped_target_offset_wrist_top)
            success, plan_arm1, _, _ = arm1.plan()                
    
        if top_x is not None:    

            top_box_request = BoxSpawnerRequest(name="top_box", x=top_x, y=top_y, base=False)
            spawn_box_service(top_box_request)
            #rospy.loginfo("Spawned top box")   

        success = False

        object_frame_base = PyKDL.Frame()   
        
        while not success:

            base_x, base_y, base_z = self.get_random_pose()  

            if abs(base_x - top_x) < 0.15 and abs(base_y - top_y) < 0.15:
                continue                            
        
            distance_base_one = (trans_one[0]-base_x) **2 + (trans_one[1]-base_y)**2   # (x, y, z)
            distance_base_two = (trans_two[0]-base_x) **2 + (trans_two[1]-base_y)**2   # (x, y, z)

            if distance_base_one < distance_base_two :
                arm2 = copy.copy(arm1_0)
                gripper2 = copy.copy(gripper1_0)
                gripper2_open = 'gripper1 open'
                gripper2_close = 'gripper1 pinch'
                robot2_box = "robot1"
                arm2_home = 'arm1 home'
                arm2_name ="arm1"
                flag_base = 1     

            else:
                arm2 = copy.copy(arm2_0)
                gripper2 = copy.copy(gripper2_0)
                gripper2_open = 'girpper2 open'
                gripper2_close = 'girpper2 pinch'
                robot2_box = "robot2"
                arm2_home = 'arm2 home'
                arm2_name ="arm2"
                flag_base = 2
            
            object_frame_base.p = PyKDL.Vector(base_x, base_y, base_z)
            gripper_length = 0.24
    
            goal_frame_base = copy.copy(object_frame_base)
            goal_frame_base.p[2] += gripper_length + 0.055
            goal_frame_base.M.DoRotX(3.14)  # Gripper pointing down
    
            goal_pose_base = self.frame_to_pose(goal_frame_base)
            pose_stamped_target_wrist_base = geometry_msgs.msg.PoseStamped()
            pose_stamped_target_wrist_base.header.frame_id = "world"  # Adjust if needed
            pose_stamped_target_wrist_base.pose = goal_pose_base
            pose_stamped_target_offset_wrist_base = copy.deepcopy(pose_stamped_target_wrist_base)

            arm2.set_pose_target(pose_stamped_target_offset_wrist_base)
            success, plan_arm2, _, _ = arm2.plan()

        if base_x is not None:

            base_box_request = BoxSpawnerRequest(name="base_box", x=base_x, y=base_y, base=True)
            spawn_box_service(base_box_request)
            rospy.loginfo("Spawned base box")
                  
        feedback.status = f"Created box objects, top box will be picked by {arm1_name} and put on base by {arm2_name}"
        self.server.publish_feedback(feedback)
             
    ###################################
    
        # Open gripper
        #rospy.loginfo("Opening gripper")
        gripper1.set_named_target(gripper1_open)
        gripper1.go(wait=True)
    
        # Move to target pose
        #rospy.loginfo("Moving to target pose")
        arm1.execute(plan_arm1, wait=True)
    
        # Attach the top box
        #rospy.loginfo("Attaching top box")
        box_attach_request_1 = BoxAttachRequest(robot_name = robot1_box, box_name = "top_box", attach=True)
        attach_box_service(box_attach_request_1)

        # Close gripper
        #rospy.loginfo("Closing gripper")
        gripper1.set_named_target(gripper1_close)
        gripper1.go(wait=True)

        feedback.status = f"{arm1_name} picked top box"
        self.server.publish_feedback(feedback)

    ###################################

        if flag_top != flag_base:
            
            transfer_x, transfer_y, transfer_z  = (0, 0, 0.7)   

            object_frame_tr = PyKDL.Frame()    
            object_frame_tr.p = PyKDL.Vector(transfer_x, transfer_y, transfer_z)
            gripper_length = 0.24
    
            goal_frame_tr = copy.deepcopy(object_frame_tr)
            goal_frame_tr.p[2] += gripper_length
            goal_frame_tr.M.DoRotX(3.14)  # Gripper pointing down
    
            goal_pose_tr = self.frame_to_pose(goal_frame_tr)
            pose_stamped_target_wrist_tr = geometry_msgs.msg.PoseStamped()
            pose_stamped_target_wrist_tr.header.frame_id = "world"  # Adjust if needed
            pose_stamped_target_wrist_tr.pose = goal_pose_tr
            pose_stamped_target_offset_wrist_tr = copy.deepcopy(pose_stamped_target_wrist_tr)

            arm1.set_pose_target(pose_stamped_target_offset_wrist_tr)
            success, plan_arm1_1, _, _ = arm1.plan()
            arm1.execute(plan_arm1_1, wait=True)

            ###################################
    
            gripper1.set_named_target(gripper1_open)
            gripper1.go(wait=True)
       
            box_attach_request_1_2 = BoxAttachRequest(robot_name=robot1_box, box_name="top_box", attach=False)
            attach_box_service(box_attach_request_1_2)

            feedback.status = f"Top box in transfer position"
            self.server.publish_feedback(feedback)

            arm1.set_named_target(arm1_home)
    
            success, plan_arm1, _, _ = arm1.plan()
            arm1.execute(plan_arm1, wait=True)

            feedback.status = f"{arm1_name} in home position"
            self.server.publish_feedback(feedback)

            gripper2.set_named_target(gripper2_open)
            gripper2.go(wait=True)

            arm2.set_pose_target(pose_stamped_target_offset_wrist_tr)
            success, plan_arm2_1, _, _ = arm2.plan()
            arm2.execute(plan_arm2_1, wait=True)
   
            box_attach_request_2_1 = BoxAttachRequest(robot_name=robot2_box, box_name="top_box", attach=True)
            attach_box_service(box_attach_request_2_1)

            gripper2.set_named_target(gripper2_close)
            gripper2.go(wait=True)       

            feedback.status = f"{arm2_name} picked top box"
            self.server.publish_feedback(feedback)

        arm2.clear_pose_targets()

        arm2.set_pose_target(pose_stamped_target_offset_wrist_base)
        success, plan_arm2, _, _ = arm2.plan()
        arm2.execute(plan_arm2, wait=True)

        gripper2.set_named_target(gripper2_open)
        gripper2.go(wait=True)

        box_attach_request_2_2 = BoxAttachRequest(robot_name=robot2_box, box_name="top_box", attach=False)
        attach_box_service(box_attach_request_2_2)

        feedback.status = f"Top box transferred to base box by {arm2_name}"
        self.server.publish_feedback(feedback)

         # Move to home position
        #rospy.loginfo("arm2 moving to home position")
        arm2.set_named_target(arm2_home)
        success, plan_arm2_3, _, _ = arm2.plan()
        arm2.execute(plan_arm2_3, wait=True)

        feedback.status = f"{arm2_name} in home position"
        self.server.publish_feedback(feedback)

        #rospy.loginfo("Motion control complete")

        ###################################           
       
        #rospy.loginfo('Server: Target reached!')
        
        result.success = True
        result.message = "The motion was successfully completed."
        self.server.set_succeeded(result)

    @staticmethod
    def get_robot_coordinates(world_frame = "world", base_link_frame = "one_base_link"): 

        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform(world_frame, base_link_frame, rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = tf_listener.lookupTransform(world_frame, base_link_frame, rospy.Time(0))

        return trans
    
    @staticmethod     
    def calculate_distance(trans_one, top):

        distance_top_one = (trans_one[0]-top[0]) **2 + (trans_one[1]-top[1])**2   # (x, y, z)
        
        return distance_top_one   
    
    @staticmethod
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

    @staticmethod
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


if __name__ == '__main__':
    rospy.init_node('motion_control')
    server = MoveRobotServer()
    rospy.spin()
