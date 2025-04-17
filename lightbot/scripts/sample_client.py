#! /usr/bin/env python3
"""
Service client node:
1. Load light position data from a file (`light_pos.lp`).
2. Set various camera parameters (e.g., exposure, gain).
3. Acquire images using the camera.

Command to check the vimba camera parameters:
    rosrun dynamic_reconfigure dynparam get /camera
"""

from lightbot.srv import *
import rospy

f = open("/home/esirem/Desktop/M2_Thesis/4-RTI_Bot/eva_rti_ws/src/lightbot/data/light_pos.lp", "r")
data = f.read()
print(data)

# function to call the load_lps service with the light positions data:
def set_lps():
    rospy.init_node("sample_client")
    print("Calling load lps service")
    rospy.wait_for_service('load_lps')
    try:
        s = rospy.ServiceProxy('load_lps', load_lps)
        response = s(data)
        return response.successfully_loaded
    except rospy.ServiceException as e:
        print("load lps Service call failed: %s"%e)

# function to call the execute_acquisition service, to capture the images
def acquire():
    print("Calling acquire service")
    try:
        s = rospy.ServiceProxy('execute_acquisition', execute_acquisition)
        response = s()
        return True
    except rospy.ServiceException as e:
        print("Acquire robot Service call failed: %s"%e)

# function to set camaera parameters using camera_params service
def set_camera_param(param_name, param_val):
    try:
        s = rospy.ServiceProxy('camera_params', camera_params)
        success = s(param_name, str(param_val))
        if success:
            print("%s set to %s"%(param_name, param_val))
        else:
            print("Failed to set %s"%param_name)
        return success
    except rospy.ServiceException as e:
        print("Camera param Service call failed: %s"%e)
        return False

# Calling the functions:
set_lps() # get all the light positions

# Camera settings ----- comment camera related properties, when working without camera in simulation
print("Calling camera service")
set_camera_param("exposure_auto", "Off")
rospy.sleep(0.5)
set_camera_param("gain_auto", "Off")
rospy.sleep(0.5)
set_camera_param("exposure", "3503.0")
set_camera_param("gain", "12.0")

acquire() # acquire the images