#! /usr/bin/env python3
"""
ROS Action Clinet node: used to send predefined joint positions to the Eva robotic arm via the eva_driver action server.

Remember: Refer to lightbot/action/driver_action.action before running the script
"""

import rospy
import actionlib
import lightbot.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Function for action client:
def lightbot_sample_client():
    print("Connecting to action server...")
    client = actionlib.SimpleActionClient('eva_driver', lightbot.msg.driver_actionAction)

    print("Waiting for action server...")
    client.wait_for_server()
    print("Connected!")

    goal = lightbot.msg.driver_actionGoal() 
    target_pose = JointState()
    target_pose.header = Header()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.name = ["joint01", "joint12", "joint23", "joint34", "joint45", "joint56"]
    # target_pose.position = [-0.26, 0.95, -2.49, 0.19, 1.48, -1.89]    # Pose 1
    # target_pose.position = [0, 0, 0, 0, 1.57, 0]  # Pose 2
    target_pose.position = [-0.26, 0.95, -2.49, 0.19, 0.88, -1.89]  # Pose 3
    
    # Send goal pose to action server, and get the result (if executed or not)
    print("Sending goal:", target_pose.position)
    goal.target_pose = target_pose
    client.send_goal(goal)

    print("Waiting for result...")
    client.wait_for_result()
    
    result = client.get_result()
    print("Received result:", result)
    return result

if __name__ == '__main__':
    try:
        rospy.init_node('eva_control_without_rail_sample_client')
        result = lightbot_sample_client()
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
