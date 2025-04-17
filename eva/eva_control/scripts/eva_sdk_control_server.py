#! /usr/bin/env python3
""" 
ROS Action Server node: used to control the Eva robotic arm using the evasdk API (without MoveIt).
Publishes joint states, receives pose goals via action messages, to reach specific goal poses.

Remember: Refer to lightbot/action/driver_action.action before running the script

This node acts as an HTTP client, using the EvaHTTPClient class to send HTTP requests to the robot and retrieve information or send commands.

"""
import rospy
import actionlib
from lightbot.msg import*
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import evasdk
import time
import math
import threading    # For running publisher and action server in parallel

host_ip = "192.168.187.1"   # Eva IP address
token = "2e49551cc7cb55556ee0468e30296755e9f2cf01"  # Token
print('ip: [{}], token: [{}]\n'.format(host_ip, token))
http_client = evasdk.EvaHTTPClient(host_ip, token) # Create a connection to Eva for reading data (such as: data_servo_positions) via REST API
session_token = http_client.auth_create_session()
users = http_client.users_get()
print('Eva at {} users: {}\n'.format(host_ip, users))
eva = evasdk.Eva(host_ip, token)    # Creating an Eva SDK instance for controlling the robot

class DriverAction:
    _feedback = lightbot.msg.driver_actionFeedback()
    _result = lightbot.msg.driver_actionResult()

    def __init__(self):
        self._action_name = "eva_driver"
        # Creating the action server
        self._as = actionlib.SimpleActionServer(self._action_name, lightbot.msg.driver_actionAction, execute_cb=self.execute_cb, auto_start = False)
        self.joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        
        threads = list()
        
        # Start the action server in a separate thread
        server_thread = threading.Thread(target=self._as.start)
        threads.append(server_thread)
        server_thread.start()
        
        # Start joint state publishing thread
        publisher_thread = threading.Thread(target=self.joint_state_publisher)
        threads.append(publisher_thread)
        publisher_thread.start()

    # Function for getting the current joint angles from the Eva robotic arm
    def joint_state_publisher(self):
        while not rospy.is_shutdown():
            current_joint_angles = http_client.data_servo_positions()  # Read the current joint angles from robot 
            current_joints_state = JointState()
            current_joints_state.header = Header()
            current_joints_state.header.stamp = rospy.Time.now()
            current_joints_state.name = ["joint01", "joint12", "joint23", "joint34", "joint45", "joint56"]
            joint5_angle = ((- current_joint_angles[4] + math.radians(179)) % math.radians(360) + math.radians(360)) % math.radians(360) - math.radians(179)
            current_joints_state.position = [current_joint_angles[0], current_joint_angles[1], current_joint_angles[2], current_joint_angles[3], joint5_angle, current_joint_angles[5]]
            current_joints_state.velocity = []
            current_joints_state.effort = []
            r = rospy.Rate(10)
            self.joint_states_pub.publish(current_joints_state)
            r.sleep()
    
    # Callback function is triggered when a new action goal is received
    def execute_cb(self, goal):       

        print("Received a move request")
        r = rospy.Rate(1)
        success = True
        
        # Handling the cancel requests:
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        with eva.lock():
            eva.control_wait_for_ready()
            print("Received a target pose: " + str(goal.target_pose.position))
            joint5_angle = ((-goal.target_pose.position[4] + math.radians(179)) % math.radians(360) + math.radians(360)) % math.radians(360) - math.radians(179);
            try: 
                eva.control_go_to([goal.target_pose.position[0], goal.target_pose.position[1], goal.target_pose.position[2], goal.target_pose.position[3], joint5_angle, goal.target_pose.position[5]], mode="automatic", max_speed=0.5)
            except evasdk.EvaError as e:
                print(e)
                success = False
            
            print("Camera trigger signal: " + str(goal.trigger_signal.data))
            if goal.trigger_signal.data:
                eva.gpio_set('d0', True)
                eva.gpio_set('d1', True)
                eva.gpio_set('d2', True)
                eva.gpio_set('d3', True)
                print("Set the eva gpio's to High")       
                time.sleep(1)
                eva.gpio_set('d0', False)
                eva.gpio_set('d1', False)
                eva.gpio_set('d2', False)
                eva.gpio_set('d3', False) 
                print("Set the eva gpio's to Low")
                print("Fulfilled the request successfully")

        # Send back the result to action client
        if success:
            self._result.actual_acquired_pose = goal.target_pose
            rospy.loginfo('Succeeded')
            self._as.set_succeeded(self._result)
        else: 
            rospy.loginfo('Failed to move the robot')
            self._as.set_aborted()
        
if __name__ == '__main__':
    rospy.init_node('eva_driver_server')
    server = DriverAction()
    rospy.spin()
