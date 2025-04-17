#! /usr/bin/env python3
"""
Service Server node: This script sets up a ROS-based service interface to control an Eva robotic arm using the Automata Eva SDK.

Note: This node continuously publishes the real-time joint angles of the physical 
      Eva robot to the joint_states topic, enabling accurate live visualization in RViz.

1. Eva SDK Communication:
   - Authenticates with the Eva robot via IP and token
   - Creates an SDK session to send commands, read joint angles, and manipulate GPIO outputs.

2. Command Execution:
   - Accepts motion requests via the `sendCommand` service, which include:
     - Target joint angles
     - Whether to send a trigger signal to the robot's digital outputs
     - A flag (`ros_solution`) that decides whether to execute the motion via toolpath planning or direct motion control.
   - If `ros_solution` is False:
     - Constructs a toolpath (metadata, avoidance zones, waypoints, timeline) and executes it.
   - If `ros_solution` is True:
     - Directly sends the robot to the desired pose using "teach" mode with a safe speed.
     - Handles triggering of GPIO pins (useful for activating external systems like cameras or grippers).

3. Safety & Feedback:
   - Applies joint wrapping for joint 5 to keep it within safe bounds (-179° to +179°).

4. Multithreading:
   - ROS service server and joint state publisher run in parallel threads for responsiveness.
"""

from eva_control.srv import sendCommand,sendCommandResponse
import rospy
import rospkg
import evasdk
import time
import math
import threading
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

rospack = rospkg.RosPack()

class send_command():
    def __init__(self):
        self.pkg_path = rospack.get_path('lightbot')

        with open(self.pkg_path + "/config/system_config.yaml", 'r') as stream:
            try:
                system_config_params=yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        self.robot_live = system_config_params['robot_live'] # check if robot is live
        
        if self.robot_live:
            self.host_ip = "192.168.187.1"  # Eva IP address
            self.token = "2e49551cc7cb55556ee0468e30296755e9f2cf01" # Token
            print('ip: [{}], token: [{}]\n'.format(self.host_ip, self.token))
            self.http_client = evasdk.EvaHTTPClient(self.host_ip, self.token) # Authenticate and create an Eva client session
            self.session_token = self.http_client.auth_create_session()
            self.users = self.http_client.users_get()
            print('Eva at {} users: {}\n'.format(self.host_ip, self.users))
            self.eva = evasdk.Eva(self.host_ip, self.token) # Creating an Eva SDK instance for controlling the robot
            
        self.joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        threads = list()

        # Start the service server in a separate thread
        server_thread = threading.Thread(target=self.sendCommand_server)
        threads.append(server_thread)
        server_thread.start()
        
        # Start joint state publishing thread
        publisher_thread = threading.Thread(target=self.joint_state_publisher)
        threads.append(publisher_thread)
        publisher_thread.start()
        
        rospy.spin()
        
    # Function for getting the current joint angles from the Eva robotic arm
    def joint_state_publisher(self):
        print("Publishing the joint states")
        while not rospy.is_shutdown() and self.robot_live:
            current_joint_angles = self.http_client.data_servo_positions() # Read the current joint angles from robot  
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
    
    """
    Initializes a ROS service named 'sendCommand' that listens for movement requests.
    When a request is received, it calls the 'send_goal_to_robot' function to handle it.
    """    
    def sendCommand_server(self):       
        rospy.Service('sendCommand', sendCommand, self.send_goal_to_robot)
        print("Ready to send commands to eva")

    # function to control the real robotic arm:
    def send_goal_to_robot(self, req):
        target_pose = req.target_pose
        print(target_pose)
        trigger_signal = req.trigger_signal
        success = True 
        self.ros_solution = req.ros_solution
        
        print("Send command request received")
        print("Trigger signal: " + str(trigger_signal))
        
        """
        It hasn't been updated for current setup, so don't use it:
        This toolpath dictionary defines a complete motion plan for the Eva robot arm. 
        It includes metadata, motion waypoints, and a timeline of actions.
        """
        if not self.ros_solution:
            print("*******************eva_service*******************************")
            joint5_angle = ((-target_pose[4] + math.radians(179)) % math.radians(360) + math.radians(360)) % math.radians(360) - math.radians(179)
            toolpath = {
                    "metadata": {
                        "version": 2, 
                        "default_max_speed": 0.25, 
                        "next_label_id": 5, 
                        "analog_modes": {"i0": "voltage", "i1": "voltage", "o0": "voltage", "o1": "voltage"}, 
                        "payload": 0, 
                        "avoidance_zones": [
                            {"name": "camera", "position": {"x": 0.200946171, "y": -0.0, "z": -0.032585143}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "dimensions": {"x": 0.150, "y": 0.02, "z": 0.325}, "avoid": "inside"}, 
                            {"name": "Surface", "position": {"x": 0.49665058, "y": -0.23507829, "z": -0.011363999}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "dimensions": {"x": 0.005, "y": 0.55, "z": 0.55}, "avoid": "inside"},
                            {"name": "table", "position": {"x": -0.52629024, "y": -0.2758804, "z": -0.014161825}, "orientation": {"x": 0, "y": 0, "z": -0.008722955, "w": 0.999962}, "dimensions": {"x": 1, "y": 0.5, "z": 0.005}, "avoid": "inside"}
                        ]
                    }, 
                    "waypoints": [
                        {"joints": [target_pose[0], target_pose[1], target_pose[2], target_pose[3], joint5_angle, target_pose[5]], "label_id": 1}
                    ], 
                    "timeline": [
                        {"type": "home", "waypoint_id": 0}, 
                        {"type": "wait", "condition": {"type": "time", "duration": 1}}, 
                        {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": trigger_signal},
                        {"type": "wait", "condition": {"type": "time", "duration": 2}}, 
                        {"type": "output-set", "io": {"location": "base", "type": "digital", "index": 0}, "value": False}
                    ]
                }
            with self.eva.lock():
                self.eva.control_wait_for_ready()
                print(toolpath)
                self.eva.toolpaths_use(toolpath)  
                try:
                    self.eva.control_home()                    
                except evasdk.EvaError as e: 
                    print(e)
                    success = False
                self.eva.control_run(loop=1)
        # if ros_solution is True, execute the motion using real Eva arm
        else:
            print("########################eva_service##########################")
            print("Please check the speed - don't increase it too much - use Teach mode")
            error = ""
            # lock the eva arm
            with self.eva.lock():
                self.eva.control_wait_for_ready()
                print("Received a target pose: " + str(target_pose))
                joint5_angle = ((-target_pose[4] + math.radians(179)) % math.radians(360) + math.radians(360)) % math.radians(360) - math.radians(179)
                try: 
                    self.eva.control_go_to([target_pose[0], target_pose[1], target_pose[2], target_pose[3], joint5_angle, target_pose[5]], mode="teach", max_speed=0.075)                    
                except evasdk.EvaError as e:
                    print(e)
                    if "Self Collision: Self collision will occur in the following tool-path." in str(e):
                        error = "self_collision"                
                    success = False
                
                print("Camera trigger signal: " + str(trigger_signal))
                if trigger_signal:
                    self.eva.gpio_set('d0', True)
                    self.eva.gpio_set('d1', True)
                    self.eva.gpio_set('d2', True)
                    self.eva.gpio_set('d3', True)
                    print("Set the eva gpio's to High")       
                    time.sleep(1)
                    self.eva.gpio_set('d0', False)
                    self.eva.gpio_set('d1', False)
                    self.eva.gpio_set('d2', False)
                    self.eva.gpio_set('d3', False) 
                    print("Set the eva gpio's to Low")
                    print("Fulfilled the request successfully")
                    print(success)        
        return success, error
                
if __name__ == "__main__":
    rospy.init_node('sendCommand_server')
    rospy.loginfo("Firing up the eva robot arm")
    sendCommand_object = send_command()  