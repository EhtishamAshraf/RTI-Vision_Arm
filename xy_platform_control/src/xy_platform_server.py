#! /usr/bin/env python3

"""
    X = rosservice call /xy_platform_service "target_position: 10.0 target_axis: true"
    Y = rosservice call /xy_platform_service "target_position: 10.0 target_axis: false"
    
    This is applied to the current setup of the Box in which Robot and the Plotter is present:
    +ve X = right
    -ve X = left
    +ve Y = inward
    -ve Y = outward
    
    Positions are absolute w.r.t (0,0)
    
    The code can be tested without having hardware connected by setting the following argument to True:
    USE_MOCK_SERIAL = True
"""

import serial
import time
import rospy
from xy_platform_control.srv import xy_platform_service, xy_platform_serviceResponse
from std_msgs.msg import Float32
import rospkg
import yaml
from unittest.mock import MagicMock

USE_MOCK_SERIAL = False  # Set to False when using the actual Arduino

class xy_platform_control():
    def __init__(self, robot_name="xy_platform"):
        rospack = rospkg.RosPack()
        
        if USE_MOCK_SERIAL:
            # Mocking the serial interface
            self.arduino = MagicMock()
            self.arduino.readline.return_value = b"Mock Arduino initialized.\n"
        else:
            # Using the actual serial interface
            self.arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        
        # Homing the platform
        self.home_platform()

        self.rate = rospy.Rate(10)
        self.pkg_path = rospack.get_path('xy_platform_control')

        # Loading system configuration from YAML file
        with open(self.pkg_path + "/config/xyplatform_config.yaml", 'r') as stream:
            try:
                system_config_params = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                rospy.logerr(f"Error loading YAML file: {exc}")
        
        # Accessing calibration values
        self.xy_platform_x_calibration = system_config_params['xy_platform_x_calibration']
        self.xy_platform_y_calibration = system_config_params['xy_platform_y_calibration']
        
        # ROS publisher topics for X and Y positions
        self.x_position_topic = "cartesian_platform_x_position"
        self.y_position_topic = "cartesian_platform_y_position"
        
        self.x_publisher = rospy.Publisher(self.x_position_topic, Float32, latch=True, queue_size=10)
        self.y_publisher = rospy.Publisher(self.y_position_topic, Float32, latch=True, queue_size=10)
        
        self.x_position = 0.0
        self.y_position = 0.0
                
        # ROS service for platform control
        rospy.Service('xy_platform_service', xy_platform_service, self.execute_xy_platform_service)
        
        rospy.spin()
    
    def home_platform(self):
        rospy.loginfo("Homing the platform...")
        command = "ii"  # Command to home the platform
        self.send_command(command)
        self.x_position = 0.0
        self.y_position = 0.0
        rospy.loginfo("Platform homed. X: 0.0, Y: 0.0")
    
    def send_command(self, command):
        if USE_MOCK_SERIAL:
            # Mock the Arduino's response
            self.arduino.readline.return_value = b"Mock Arduino response.\n"
        else:
            self.arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        
        time.sleep(1)
        rawString = self.arduino.readline()
        print(rawString)
        
        self.arduino.write(command.encode())  # Sending command
        self.arduino.write(b'\n')             # Ensuring newline

        time.sleep(1)
        rawString = self.arduino.readline()
        print(rawString)
        rospy.loginfo(f"Arduino response: {rawString.decode()}")
        
        if not USE_MOCK_SERIAL:
            self.arduino.close()
        return
    
    # function to move platform horizontally - Explanation on Github
    def move_surface_x(self, x):
        x_displacement = x - self.x_position
        rospy.loginfo(f"Moving surface to x: {x}, y: 0")
        if x_displacement > 0.0:
            steps = x_displacement 
            command = "mx" + str(float(steps))
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
        elif x_displacement < 0.0:
            steps = x_displacement
            command = "mx" + str(float(steps))
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
        elif x == 0:
            command = "ix"
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
            
        self.x_position = x            
        x_msg = Float32()
        x_msg.data = self.x_position
        self.x_publisher.publish(x_msg)

    # function to move platform horizontally
    def move_surface_y(self, y):
        y_displacement = y - self.y_position
        rospy.loginfo(f"Moving surface to x: 0, y: {y}")
        if y_displacement > 0.0:
            steps = y_displacement 
            command = "my" + str(float(steps))
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
        elif y_displacement < 0.0:
            steps = y_displacement
            command = "my" + str(float(steps))
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
        elif y == 0:
            command = "iy"
            rospy.loginfo(f"Sending command: {command}")
            self.send_command(command)
            
        self.y_position = y
        y_msg = Float32()
        y_msg.data = self.y_position
        self.y_publisher.publish(y_msg)
    
    # moving platform based on the client's node request 
    def execute_xy_platform_service(self, req):
        rospy.loginfo(f"Received a move request: {req.target_position}, {req.target_axis}")
        if req.target_axis:
            if req.target_position > 27.0:
                rospy.logerr("Exceeding the stroke length")
                return xy_platform_serviceResponse(successfully_displaced=False)
            self.move_surface_x(req.target_position)
        else:
            if req.target_position > 27.0:
                rospy.logerr("Exceeding the stroke length")
                return xy_platform_serviceResponse(successfully_displaced=False)
            self.move_surface_y(req.target_position)

        return xy_platform_serviceResponse(successfully_displaced=True)

if __name__ == "__main__":
    rospy.init_node('xy_platform_service')
    rospy.loginfo("Initializing cartesian platform")
    xy_platform_controlObject = xy_platform_control()


