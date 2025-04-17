#!/usr/bin/env python3

# This file is used for simulating Eva Robotic Arm and the XY Plotter in Rviz and Gazebo 

"""
Take robot to a specific position using moveit GUI, and then once the robot reaches 
the position in rviz and gazebo, then run the following command to get the 
Translation and rotation of gripper (position in: xyz, and rotation in rpy):
 
command:
$: rosrun tf tf_echo /base_link /ee_link
$: rosrun tf tf_echo /world /ee_link   
use second command --- to know the world-coordinates

Note: 
-   Box has a height of 0.8m from the ground

Up Position (w.r.t /world):
- Translation: [-0.477, 0.680, 1.728]
- Rotation: [1.567, -0.071, 0.143]

Bend Position (w.r.t /world):
- Translation: [-0.585, 0.724, 1.254]
- Rotation: [-0.248, -0.119, 1.370]

Random Position 1 (w.r.t /world):
- Translation: [-0.492, 0.314, 1.661]
- Rotation: [1.553, -0.044, 0.079]

Random Position 2 (w.r.t /world):
- Translation: [-0.762, 0.421, 1.309]
- Rotation: [1.555, -0.115, 0.029]

"""

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf
from geometry_msgs.msg import PoseStamped

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)
    
    robot = moveit_commander.RobotCommander() # interface to interact with the robot's links and joints etc
    scene = moveit_commander.PlanningSceneInterface() # interface to interact with the environment in which the arm is operating
    
    # group = moveit_commander.MoveGroupCommander("eva")
    group = moveit_commander.MoveGroupCommander("eva", wait_for_servers=20.0) # interface to control the robotic arm

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
    
    rospy.loginfo("Reference frame: %s", group.get_planning_frame())
    rospy.loginfo("End effector: %s", group.get_end_effector_link())
    
    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)
    
    rospy.sleep(2)

    # Target position w.r.t world
    target_pose = geometry_msgs.msg.Pose()
    quaternion = tf.quaternion_from_euler(-0.248, -0.119, 1.370) 
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    target_pose.position.x = -0.585
    target_pose.position.y = 0.724
    target_pose.position.z = 1.254

    group.set_pose_target(target_pose)

    plan = group.plan()
    rospy.loginfo("Visualizing plan %s", "SUCCESS" if plan else "FAILED")
    
    # Move the eva group
    group.go(wait=True)
    rospy.sleep(1.0)
    group.stop()
    group.clear_pose_targets()
        
    rospy.sleep(2.0)
    
    moveit_commander.roscpp_shutdown()
    
if __name__ == '__main__':
    main()
