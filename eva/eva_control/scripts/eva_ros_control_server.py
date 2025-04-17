#! /usr/bin/env python3
"""
ROS Action Server node also acts as Service client node: for controlling the Eva robotic arm using ROS.

This script defines a `DriverAction` class that acts as a ROS action server to handle motion planning and execution for the Eva robot. 
It receives target poses via an action interface, plans motions using MoveIt, and executes them either in simulation or on the real robot using a custom service (`sendCommand`). 

Key features:
- Uses MoveIt! for path planning and joint trajectory generation.
- Supports execution in both simulation and live robot modes.
- Can optionally send trigger signals at the end of the trajectory.
- Includes configurable parameters from a YAML file.


Remember: Refer to lightbot/action/driver_action.action before running the script
"""

import rospy
import rospkg
import actionlib
from lightbot.msg import*
from eva_control.srv import *
from tf.transformations import *
import moveit_commander
import yaml

rospack = rospkg.RosPack()

class DriverAction:
    _feedback = lightbot.msg.driver_actionFeedback()
    _result = lightbot.msg.driver_actionResult()   

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.pkg_path = rospack.get_path('lightbot')
        self._action_name = "eva_driver"
        self._as = actionlib.SimpleActionServer(self._action_name, lightbot.msg.driver_actionAction, execute_cb=self.execute_cb, auto_start = False)

        # Initialize MoveIt for motion planning
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander() # instance of robot commander
        self.scene = moveit_commander.PlanningSceneInterface() # planning scene
        self.group = moveit_commander.MoveGroupCommander("eva") # Eva move group
        
        # self.group.set_planning_time(10)  
        # self.group.set_planner_id("RRTConnect")
        # self.group.set_goal_tolerance(0.001)  
        # self.group.set_max_velocity_scaling_factor(0.05)  
        # self.group.set_max_acceleration_scaling_factor(0.05)  
        
        print("Current Pose: ", self.group.get_current_pose().pose)
        print("Current Joint Values: ", self.group.get_current_joint_values())
        print(f"Base Link: ", self.group.get_planning_frame())  # this link is also "Planning Frame"
        print(f"End Effector Link: ", self.group.get_end_effector_link())

        # load the YAML file
        with open(self.pkg_path + "/config/system_config.yaml", 'r') as stream:
            try:
                system_config_params=yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.robot_live = system_config_params['robot_live'] # checks if controlling real Eva or just in simulation
        self.ros_solution = system_config_params['ros_solution'] # if ros_solution is True then move Eva smoothly

        self.group.set_named_target("home") # Sets the robot's target joint configuration to a predefined "home" position
        self._as.start()    # Starting the action server
    
    # Callback function is triggered when a new action goal is received
    def execute_cb(self, goal):       

        print("Received a move request")
        r = rospy.Rate(1)
        
        print("Goal Pose:", goal.target_pose.pose)

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted() 
        
        # function to execute motion using Eva in hardware or simulation
        self.ros_execute(goal.target_pose.pose, goal.trigger_signal)
        
        # if motion succeeded:
        if self.succeeded: 
            self._result.actual_acquired_pose = goal.target_pose
            self._result.succeeded.data = self.succeeded
            rospy.loginfo("Succeeded: " + str(self.succeeded))
            self._as.set_succeeded(self._result) # Sends the result back to the action client, marking the action as successfully completed.
        else: 
            rospy.loginfo("Failed to move the robot!!!")
            self._result.actual_acquired_pose = goal.target_pose
            self._result.system_err.data = self.system_err
            self._result.succeeded.data = self.succeeded
            rospy.loginfo("Succeeded: " + str(self.succeeded))
            print("system err: " + str(self.system_err))
            self._as.set_succeeded(self._result)

    # Execute the motion using moveit, once you have the goal pose:
    def ros_execute(self, pose_target, trigger_signal):
        success = True
        self.system_err = True # temporarily set to false. should be true in case of choosing eva solution
        print("Planning motion for the point : " + str(pose_target) + " in the world co-ordinate")
        
        # This tells MoveIt where you want the end effector to go, and MoveIt will compute the joint angles to achieve it.
        self.group.set_pose_target(pose_target, end_effector_link="ee_link")
        
        """Try to reach the position without considering orientation:"""
        # xyz = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
        # self.group.set_position_target(xyz, end_effector_link="ee_link")

        success, plan, _, _ = self.group.plan() # Generating the motion plan using moveit
        print("Generated Plan:", plan)
        print("Number of points:", len(plan.joint_trajectory.points))
        
        # checking if motion planning is successful:
        if success and len(plan.joint_trajectory.points) > 0:
            print("Valid plan found. Executing...")
            self.group.execute(plan, wait=True)
        else:
            print("Planning failed. No valid motion plan found.")
        
        # if motion planning failed:
        if not plan.joint_trajectory.points:
            print("Motion planning failed")
            success = False
            self.system_err = False
        
        # if real Eva arm is not connected, then execute motion in RVIZ                                
        if not self.robot_live:
            print("Robot is in offline Mode!!!")
            print("robot live: " + str(self.robot_live))
            self.group.go(wait=True)
            
        # if connected, then control the real Eva arm
        else:
            print("Trajectory execution using EVA Hardware: ")                    
            """
            If:
            --> Call the server with only the first and last points of the planned trajectory.
            --> This is used for smooth motion.
            --> This will take the start point and the end point and move smoothly b/w them.
            Else:
            --> Take each point of the planned trajectory, and send the points to the Eva arm.
            --> The arm will move in chunks.
            """
            if self.ros_solution:
                print("*******************eva_ros_control_server*******************************")
                nb_pts = 0
                nb_way_points = len(plan.joint_trajectory.points) 
                trigger = False           
                for pt in plan.joint_trajectory.points:                
                    if nb_pts == 0 or nb_pts == (nb_way_points-1): # Call the server, only if it's the first or last point
                        print(pt.positions)
                        if nb_pts == nb_way_points-1: # if it's the last point, capture image
                            trigger = trigger_signal.data 

                        rospy.wait_for_service('sendCommand') # wait for service
                        success = False
                        response = None
                        try: # call the service to move the robot
                            s = rospy.ServiceProxy('sendCommand', sendCommand)
                            response = s(pt.positions, trigger, self.ros_solution)
                        except rospy.ServiceException as e:
                            print("Service call failed: %s"%e)
                        
                        # if server responded, the movement was successfull
                        if response is not None:
                            success = response.successful
                            if response.error == "self_collision":
                                self.system_err = False
                    nb_pts = nb_pts+1
            else:
                print("########################eva_ros_control_server##########################")
                nb_pts = 0
                nb_way_points = len(plan.joint_trajectory.points) 
                trigger = False           
                for pt in plan.joint_trajectory.points: # Call the server, for each point of the planned trajectory       
                    print(pt.positions)
                    if nb_pts == nb_way_points-1:
                        trigger = trigger_signal.data

                    rospy.wait_for_service('sendCommand')
                    success = False
                    response = None
                    try:
                        s = rospy.ServiceProxy('sendCommand', sendCommand)
                        response = s(pt.positions, trigger, self.ros_solution)                        
                    except rospy.ServiceException as e:
                        print("Service call failed: %s"%e)
                    
                    if response is not None:
                        success = response.successful
                        if response.error == "self_collision":
                            self.system_err = False
                    nb_pts = nb_pts+1
        
        # robot movement is executed successfully: so stop and Clear stored goal poses in the move group.
        self.group.stop()
        self.group.clear_pose_targets()            
        rospy.sleep(1)
        self.succeeded = success        

if __name__ == '__main__':
    rospy.init_node('eva_driver_server')
    server = DriverAction()
    rospy.spin()