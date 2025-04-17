#! /usr/bin/env python3
"""
This script controls the operation of the Lightbot, a robot designed to acquire light position data using a camera system. 

Key Functionality:
  - Sets up services to load light poses (lps), execute data acquisition, and adjust camera parameters dynamically at runtime.
  - Uses a camera callback to capture images, resize, and publish them for display.
  - Loads light poses, computes their orientations, sorts them by closest distance, and publishes them as markers in the robot's environment.
  - Handles the process of acquiring data by moving the robot to each light pose, taking images, and saving them.
  
Acquisition Process: 
  - A new thread is created to perform data acquisition in the background.
  - The robot moves to each light pose and attempts to acquire data. If a pose is unreachable, it’s logged as a "missed pose".
  
Dynamic Camera Parameter Adjustment:
  - The script allows the robot's camera settings (such as exposure and gain) to be adjusted dynamically via the `dynamic_reconfigure` library.

Remember to consult: src/assets/python_codes for visual representation of polar and spherical coordinates, and direction vectors.
"""

import rospy
import rospkg
import utilities
from lightbot.srv import load_lps, load_lpsResponse, execute_acquisition, execute_acquisitionResponse, camera_params, camera_paramsResponse
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
import datetime
import time
import os
import yaml
import threading
import cv2
from cv_bridge import CvBridge
import dynamic_reconfigure.client

timeout = 2  # seconds
start_time = time.time()

rospack = rospkg.RosPack()

class RobotControl():
    def __init__(self, robot_name="lightbot"):
        
        self.rate = rospy.Rate(10)
        self.pkg_path = rospack.get_path('lightbot')
        self.bridge = CvBridge()
        
        # Initialize publishers:
        self.lp_topic = "loaded_lps"                       # publishing the sorted light poses
        self.camera_image_topic_name = "/camera/image_raw" # subscribing to this topic to get the images
        self.display_image_topic_name = "/display_image"   # publishing the resized images on this topic
        self.acq_image_topic_name = "/acquired_image"      # publishing the resized image ros msg on this topic
        
        self.lp_publisher = rospy.Publisher(self.lp_topic, PoseArray, latch = True, queue_size=10)
        self.image_subscriber = rospy.Subscriber(self.camera_image_topic_name, Image, self.camera_call_back, queue_size=1)
        self.display_image_publisher = rospy.Publisher(self.display_image_topic_name, CompressedImage, latch = True, queue_size = 1)
        self.acq_image_publisher = rospy.Publisher(self.acq_image_topic_name, CompressedImage, latch = True, queue_size = 1)

        self.waypoint_pub = rospy.Publisher('/waypoints_marker', Marker, queue_size=10)
        self.direction_pub = rospy.Publisher('/direction_vectors', MarkerArray, queue_size=10)
        self.box_pub = rospy.Publisher('/box_marker', Marker, queue_size=10)
        
        rospy.sleep(1) 

        # Get the name of the robot driver action server from ROS parameter server
        self.robot_driver_server_name = rospy.get_param("/lightbot/robot_driver_server_name")

        # Initialize different variables to None at the start:
        self.loaded_lps = None
        self.image_capture = None
        self.display_image = None
        self.acq_image = None
    
        # Registering ROS services to handle specific requests:
        rospy.Service('load_lps', load_lps, self.load_the_lps)
        rospy.Service('execute_acquisition', execute_acquisition, self.execute_acquisition)
        rospy.Service('camera_params', camera_params, self.set_camera_params)
        
        rospy.loginfo("Initializing the system!!!")
        
        """Calling get_circular_waypoints_and_directions function to create a circle of light poses around the object"""
        # self.waypoint_marker, self.direction_markers, self.box_marker = utilities.get_circular_waypoints_and_directions()
        
        """Calling get_spherical_waypoints_and_directions function to create a circle of light poses around the object"""
        self.waypoint_marker, self.direction_markers, self.box_marker = utilities.get_spherical_waypoints_and_directions()
        
        # Publish the waypoints and direction vectors
        rospy.loginfo("Publishing Waypoints, and Direction Vectors...")
        self.waypoint_pub.publish(self.waypoint_marker)
        self.direction_pub.publish(self.direction_markers)
        self.box_pub.publish(self.box_marker)

        self.counter = 0    
        self.missed_poses = []  # List to store missed poses
            
        rospy.spin()
    
    # function to set the camera parameters based on the request
    def set_camera_params(self, req):
        param_name = req.param_name
        param_val = req.param_val
        try:
            """
            Initializing dynamic_reconfigure client for the camera node
            This allows dynamic configuration changes at runtime.
            """
            client = dynamic_reconfigure.client.Client(
                "/camera",  # The camera's dynamic reconfigure server
                timeout=30
            )
            # Handle special cases first
            if param_name == "exposure":
                client.update_configuration({"exposure_auto": "Off"}) # Disable auto exposure first
                rospy.sleep(0.2)
                param_val = float(param_val)
            elif param_name == "gain":
                client.update_configuration({"gain_auto": "Off"}) # Disable auto gain first
                rospy.sleep(0.2)
                param_val = float(param_val)

            # Applying the parameter change
            client.update_configuration({param_name: param_val})
            rospy.sleep(0.1) 
            
            # Verify that the parameter was actually updated by checking the current configuration
            new_params = client.get_configuration()
            if str(new_params[param_name]) != str(param_val):
                return False 

            return True  # Success

        except Exception as e:
            rospy.logerr(f"Error setting {param_name}: {str(e)}")
            return False

    # camera callback function to capture and resize an image, and publishing it:
    def camera_call_back(self, image_data):
        self.image_capture = self.bridge.imgmsg_to_cv2(image_data, "mono8") # getting the camera image
        with open(self.pkg_path + "/config/system_config.yaml", 'r') as stream:
            try:
                system_config_params=yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        display_image_width_scale_down_percentage = system_config_params['display_image_width_scale_down_percentage']
        display_image_height_scale_down_percentage = system_config_params['display_image_height_scale_down_percentage']
        width = int(self.image_capture.shape[1]*display_image_width_scale_down_percentage/100)
        height = int(self.image_capture.shape[0]*display_image_height_scale_down_percentage/100)
        dim = (width, height)
        
        self.display_image = self.image_capture.copy()
        self.display_image = cv2.resize(self.display_image, dim, interpolation=cv2.INTER_AREA)
        display_img_msg = self.bridge.cv2_to_compressed_imgmsg(self.display_image, "jpg")
        self.display_image_publisher.publish(display_img_msg)

    """
    --> Calling the decipher_lps function to get the XYZ position of the light poses
    --> Calling the compute_pose_towards_object function to figure out the correct orientation towards the object
    --> Calling the sort_poses_by_proximity function to sort the poses based on the shortest distance
    """
    def load_the_lps(self, req):
        print("---------------------------------------------------------------")
        print("Loading the light poses!!")
        img_file_names, x_array, y_array, z_array = utilities.decipher_lps(req.lps)
        self.loaded_lps = None
        self.loaded_lps = PoseArray()
        self.loaded_lps.header.frame_id = "/world"
        self.loaded_lps.header.stamp = rospy.Time.now()
        self.nb_lps = len(img_file_names) # get total number of light poses
        self.actual_image_file_names = []
        self.sorted_indices = []

        # open the system_config file:
        with open(self.pkg_path + "/config/system_config.yaml", 'r') as stream:
            try:
                system_config_params=yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        for i in range(0, len(img_file_names)):
            print(f"The initial x, y, z are: {x_array[i]}, {y_array[i]}, {z_array[i]}")
            x = x_array[i] 
            y = y_array[i] 
            z = z_array[i]            
            pt = utilities.compute_pose_towards_object([x,y,z]) 
            
            self.actual_image_file_names.append(img_file_names[i])
            self.loaded_lps.poses.append(pt)   
            print ("the loaded light poses after orientation are: ", self.loaded_lps)
            
        self.loaded_lps, self.sorted_indices = utilities.sort_poses_by_proximity(self.loaded_lps)
        print("after sorting the poses we get: ", self.loaded_lps)
        print("after sorting the indices we get: ", self.sorted_indices)
        rospy.loginfo("Number of loaded light poses: " + str(len(self.loaded_lps.poses)))
        self.lp_publisher.publish(self.loaded_lps) # publish the light poses
        print("---------------------------------------------------------------")
        
        self.rate.sleep()
        return True

    # function to acquire the data: 
    def execute_acquisition(self,req):
        # Check if light position data has been loaded; if not, return False
        if self.loaded_lps is None:
            return False
        # Create a new thread to handle the acquisition process in the background
        acquire_thread = threading.Thread(target=self.acquire)
        acquire_thread.start()
        return True

    # acquiring data with the Eva arm
    def acquire(self):
        acquisition_data_folder_name = str(datetime.datetime.now()).split('.')[0]
        acquisition_data_folder_name.replace(' ',"_")
        acquisition_data_folder_name.replace(':',"-")
        self.log_file = open(self.pkg_path + "/data/log.txt", "a")
        os.mkdir(self.pkg_path + "/data/"+acquisition_data_folder_name)
        self.output_lp_file = open(self.pkg_path + "/data/"+acquisition_data_folder_name+"/actual_lps.lp", "a")

        # open system config file:
        with open(self.pkg_path + "/config/system_config.yaml", 'r') as stream:
            try:
                system_config_params=yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.robot_live = system_config_params['robot_live'] # check if robot is live or not

        for count, lp in enumerate(self.loaded_lps.poses):
            self.counter += 1
            print("Current Counter is: ", self.counter)
            print("Total number is: ", len(self.loaded_lps.poses))
            if rospy.is_shutdown():
                break
            
            pose_stamped = geometry_msgs.msg.PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.header.stamp = rospy.Time(0)
            pose_stamped.pose.position = lp.position
            pose_stamped.pose.orientation = lp.orientation

            print("Counter is lesser than or equal to the total number of poses?    :", self.counter)
            
            pose_target = pose_stamped.pose
            
            print("Plan request for the point: " + str(pose_target) + " in the world co-ordinate")
            _, success_flag = utilities.send_goal_to_robot(
                self.robot_driver_server_name, 
                pose_target, 
                True
            )          
            print("Success flag: " + str(success_flag))
            
            # Check if the pose was missed
            if not success_flag.data:
                self.missed_poses.append(pose_stamped)
                rospy.loginfo(f"Missed {len(self.missed_poses)} poses. Latest pose: \n"
                            f"Position -> x: {pose_stamped.pose.position.x}, y: {pose_stamped.pose.position.y}, z: {pose_stamped.pose.position.z}\n"
                            f"Orientation -> x: {pose_stamped.pose.orientation.x}, y: {pose_stamped.pose.orientation.y}, "
                            f"z: {pose_stamped.pose.orientation.z}, w: {pose_stamped.pose.orientation.w}")
                
                # Call the function to update the color of the missed waypoint
                direction_markers = utilities.update_missed_waypoint_color(pose_stamped, self.direction_markers)

                # If markers are updated, publish the updated direction markers
                if direction_markers:
                    self.direction_pub.publish(direction_markers)
                    rospy.loginfo("Published updated direction markers.")
                rospy.loginfo("Number of light poses not reachable: " + str(len(self.missed_poses)))
                continue
            
            # if no image is captured in 2 seconds, break:
            while self.image_capture is None:
                print("Waiting for new frame...")
                if time.time() - start_time > timeout:
                    print("No new frame found, moving to the next step.")
                    break
            
            # if image captured, save it, and put text on the image
            if self.image_capture is not None:
                print("***********Camera is working!!***********")
                cv2.imwrite(self.pkg_path + "/data/"+acquisition_data_folder_name+"/"+self.actual_image_file_names[self.sorted_indices[count]], self.image_capture) # save the images in the folder
                
                # resizing the actual image to a specific size and then publishing it on a topic:
                acq_display_image_width_scale_down_percentage = system_config_params['display_image_width_scale_down_percentage'] # % to scale down the image size
                acq_display_image_height_scale_down_percentage = system_config_params['display_image_height_scale_down_percentage'] # % to scale down the image size
                width = int(self.image_capture.shape[1]*acq_display_image_width_scale_down_percentage/100)
                height = int(self.image_capture.shape[0]*acq_display_image_height_scale_down_percentage/100)
                dim = (width, height)
                
                self.acq_image = self.image_capture.copy() # make a copy of the original image
                self.acq_image = cv2.resize(self.acq_image, dim, interpolation=cv2.INTER_AREA) # resize the image size
                font = cv2.FONT_HERSHEY_SIMPLEX
                origin = (150, 150)  
                fontScale = 1
                color = 255  
                thickness = 2
                self.acq_image = cv2.putText(self.acq_image, "Capture # " + str(count), origin, font, fontScale, color, thickness, cv2.LINE_AA) # put text on the image
                acq_display_img_msg = self.bridge.cv2_to_compressed_imgmsg(self.acq_image, "jpg")
                self.acq_image_publisher.publish(acq_display_img_msg)
                
                # saving the image name with X, Y, and Z position in the actual_lps.lp file
                self.output_lp_file.write(self.actual_image_file_names[self.sorted_indices[count]] + " " + str(lp.position.x) + " " + str(lp.position.y) + " " + str(lp.position.z) + "\n")
                cv2.imwrite('captured_image.jpg', self.acq_image)                
                rospy.sleep(1) 
            
            else:
                print(f"Skipping frame {count} due to missing image.")  
   

        rospy.loginfo("Total Number of light poses: " + str(len(self.loaded_lps.poses)))
        rospy.loginfo("Number of light poses not reachable: " + str(len(self.missed_poses)))
        
       # Handle home position separately:
        print("Last pose!!")
        home_pose = geometry_msgs.msg.Pose()
        home_pose.position = geometry_msgs.msg.Point(-0.04999736484652709, 0.6999998686619133, 1.72150889802346) 
        home_pose.orientation = geometry_msgs.msg.Quaternion(-3.5355464936718715e-06, -0.7071054825023977, -0.7071080798575269, 1.6591465345405167e-06)
        print("Sending robot to Home position")
        _, success_flag = utilities.send_goal_to_robot(
            self.robot_driver_server_name, 
            home_pose, 
            True
        )
        
        self.nb_lps_not_reachable = len(self.missed_poses)
        self.nb_lps_reachable = len(self.loaded_lps.poses) - self.nb_lps_not_reachable
        ct = datetime.datetime.now()
        self.log_file.write("New Acquisition @ "+ str(ct) + "\n")
        self.log_file.write("Nb of LPs reached: " + str(self.nb_lps_reachable) + ", Nb of LPs not reachable: " + str(self.nb_lps_not_reachable) + "\n")
        self.log_file.close()
        self.output_lp_file.close()
        
        return 

if __name__ == "__main__":
    rospy.init_node('lightbot')
    rospy.loginfo("Firing up the light bot")
    robotcontrol_object = RobotControl()