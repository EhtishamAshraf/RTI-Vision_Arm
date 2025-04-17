#! /usr/bin/env python3

"""
This file is used to call the xy_platform_service to control the hardware.
Target axis = True for X or horizontal 
Target axis = False for Y or vertical
"""

import rospy
from xy_platform_control.srv import xy_platform_service

count = 0 # This variable is used to make sure that the service is only called when the previous action has been completed

# call the service server to move the platform
def call_move_service(target_position, target_axis):
    # Wait for the service to be available
    rospy.wait_for_service('/xy_platform_service')

    try:
        # Create a proxy to the service
        move_service = rospy.ServiceProxy('/xy_platform_service', xy_platform_service)

        # Call the service with the target position and axis
        response = move_service(target_position, target_axis)
        
        rospy.sleep(1)

        if response.successfully_displaced:
            global count 
            rospy.loginfo(f"Successfully moved to position {target_position} along axis {target_axis}")
            
            count = count + 1
            print("count: ", count)
        else:
            rospy.logwarn(f"Failed to move to position {target_position} along axis {target_axis}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('xy_platform_controller')  # Initialize the ROS node

    call_move_service(10.0, True)  # Move to position 10.0 along the X axis
    rospy.sleep(2)
    
    if count > 0:
        print("Now count = 1")
        call_move_service(9.0, False)  # Move to position 9.0 along the Y axis
        rospy.sleep(2)
    
