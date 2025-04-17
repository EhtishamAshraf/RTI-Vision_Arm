"""
This file contains all the functions required to control the lightbot.
Remember: Adjust the "tilt_angle_rad" in get_circular_waypoints_and_directions and compute_pose_towards_object as required:
          Currently it is -10 degrees for spherical and -20 degrees for circular points (if light doesn't point perfectly towards the object, adjust it accordingly) 


How tilt is calculated in Code:**
We take cross product of Direction vector with Z-axis (as it is a global UP), and we get a vector that is perpendicular to both vectors. 
So we rotate around that newly found perpendicular vector to tilt the original vector a little downward.
For example, if the direction vector is pointing in X-axis, the global UP is Z-axis and the newly found axis is Y-axis,
when we rotate around Y-axis, the initial direction vector will point in the up or down direction.

Think of it like this:
    Your face is pointed East (X-axis)

    The Z-axis is UP

    The cross product gives you a horizontal rotation axis that goes left-right across your head (Y-axis).

    If you rotate around that axis, your face starts pointing a little downward (or upward, depending on the sign of the angle).
"""
import rospy
import actionlib
import lightbot.msg
import math
import numpy as np
import copy 
from geometry_msgs.msg import Quaternion, Pose, Point, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import *
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray

"""
Function to create a circular ring of light positions around the object:
--> Returns circular waypoints and direction vectors.
--> Saves Circular waypoints in the .lp file
--> Creates waypoint markers, box marker, and direction vectors in Rviz
--> Circular ring is made based on number of points, circle's radius, object's position
--> Makes a semi circle, as the current setup doesn't have big reachable workspace, starts the semicircle from -70 degrees
--> Finds the correct tilted direction vector for pointing the light correctly towards the object
"""
def get_circular_waypoints_and_directions():    
    # Marker for waypoints:
    waypoint_marker = Marker()
    waypoint_marker.header.frame_id = "world"  
    waypoint_marker.header.stamp = rospy.Time.now()
    waypoint_marker.ns = "waypoints_namespace"
    waypoint_marker.id = 0
    waypoint_marker.type = Marker.POINTS
    waypoint_marker.action = Marker.ADD
    waypoint_marker.pose.orientation.w = 1.0
    waypoint_marker.scale.x = 0.05  # Size of points
    waypoint_marker.scale.y = 0.05
    waypoint_marker.color.r = 1.0  # Red color
    waypoint_marker.color.a = 1.0  # Full opacity

    # Circle parameters
    num_points = 12     # Number of waypoints per semicircle
    radius = 0.28       # Radius (all the points will be created at 0.28 radius from the object)
    center_x = -0.803   # Object's X position
    center_y = 0.716    # Y position
    fixed_z = 1.037     # Z position
    object_position = np.array([center_x, center_y, fixed_z])  # Object position

    direction_markers = MarkerArray() # MarkerArray for direction vectors
    
    # File writing setup:
    filename = "/home/esirem/Desktop/M2_Thesis/4-RTI_Bot/eva_rti_ws/src/lightbot/data/light_pos.lp"
    with open(filename, "w") as file:
        file.write(f"{num_points}\n")  # First line: Number of waypoints
                                   
        for i in range(num_points):
            angle = (math.pi * i / num_points) + math.radians(-70)  # Semicircle starting from -70 degrees (can reach from 5 till 58 and -70 to -35)
            
            print("The angle in degrees is: ", math.degrees(angle))
            # adjust some points (which fall in the unreachable region) so they can be reached
            if math.degrees(angle) > -35 and math.degrees(angle) < -17:
                angle = math.radians(-35)
            elif math.degrees(angle) > 50 and math.degrees(angle) < 70:
                angle = math.radians(55)
            print("the current angle is: ", math.degrees(angle))
            
            # Position of the points on the circle:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)  
            z = fixed_z
          
            theta = math.degrees(angle) % 360  # Computing Theta (rotation around Z-axis) in degrees: Normalize to [0,360]
          
            file_name = f"LDR_{i}_Theta_{theta:.2f}.png"
            file.write(f"{file_name} {x:.6f} {y:.6f} {z:.6f}\n")

            # Add waypoint to marker
            point = Point(x, y, z)
            waypoint_marker.points.append(point)

            # Computing the direction vector from waypoint to object
            waypoint_position = np.array([x, y, z])
            direction = object_position - waypoint_position
            direction = direction / np.linalg.norm(direction) # direction vector before tilt
            
            # Applying a slight downward tilt to the direction vector
            tilt_angle_rad = np.radians(-20)

            # Finding the perpendicular axis for rotation
            axis = np.cross(direction, [0, 0, 1])  
            if np.linalg.norm(axis) < 1e-6:  
                axis = np.array([1, 0, 0])  # Default axis if direction is vertical
            axis = axis / np.linalg.norm(axis)  # Normalize

            # Apply tilt rotation
            tilt_rotation = R.from_rotvec(tilt_angle_rad * axis)
            direction = tilt_rotation.apply(direction)
            direction = direction / np.linalg.norm(direction) # direction vector after tilt
        
            # Create a Marker for the direction vector (Arrow)
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "world"
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "direction_vectors"
            arrow_marker.id = num_points + i  # Unique ID for each arrow
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.02  # Shaft diameter
            arrow_marker.scale.y = 0.04  # Head diameter
            arrow_marker.scale.z = 0.05  # Head length
            arrow_marker.color.r = 0.0   
            arrow_marker.color.g = 1.0   # Green color
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0

            # Start and End points for the arrow
            arrow_marker.points.append(Point(x, y, z)) # starts from the point on the circle
            arrow_marker.points.append(Point(x + direction[0] * 0.2, 
                                            y + direction[1] * 0.2,
                                            z + direction[2] * 0.2))

            direction_markers.markers.append(arrow_marker)
            
        rospy.loginfo(f"Waypoints saved to {filename}")
    
    # Box Marker (Square Box)
    box_marker = Marker()
    box_marker.header.frame_id = "world"
    box_marker.header.stamp = rospy.Time.now()
    box_marker.ns = "box_marker"
    box_marker.id = 99             # Unique ID
    box_marker.type = Marker.CUBE  # Box type
    box_marker.action = Marker.ADD
    box_marker.pose.position.x = center_x  
    box_marker.pose.position.y = center_y
    box_marker.pose.position.z = fixed_z - 0.2
    box_marker.pose.orientation.w = 1.0
    box_marker.scale.x = 0.15  # Width
    box_marker.scale.y = 0.15  # Depth
    box_marker.scale.z = 0.15  # Height
    box_marker.color.r = 0.0  
    box_marker.color.g = 0.0
    box_marker.color.b = 1.0   # Blue box
    box_marker.color.a = 1.0   # Fully visible

    return waypoint_marker, direction_markers, box_marker

"""
Function to create a semisphere of light positions around the object:
--> Returns spherical waypoints and direction vectors.
--> Saves spherical waypoints in the .lp file
--> Creates waypoint markers, box marker, and direction vectors in Rviz
--> spherical points are made based on number of points, radius, object's position
--> Makes a semi sphere, as the current setup doesn't have big reachable workspace, (phi from 0 to 65 degrees, thetha from -60 to 35 degrees)
--> Finds the correct tilted direction vector for pointing the light correctly towards the object
"""
def get_spherical_waypoints_and_directions():
    # Marker for waypoints:
    waypoint_marker = Marker()
    waypoint_marker.header.frame_id = "world"
    waypoint_marker.header.stamp = rospy.Time.now()
    waypoint_marker.ns = "waypoints_namespace"
    waypoint_marker.id = 0
    waypoint_marker.type = Marker.POINTS
    waypoint_marker.action = Marker.ADD
    waypoint_marker.pose.orientation.w = 1.0
    waypoint_marker.scale.x = 0.03  
    waypoint_marker.scale.y = 0.03
    waypoint_marker.color.r = 1
    waypoint_marker.color.g = 1  
    waypoint_marker.color.b = 1  
    waypoint_marker.color.a = 1.0   

    # Semi-sphere parameters
    num_theta = 5    # Number of horizontal divisions
    num_phi = 6      # Number of vertical divisions
    radius = 0.28     # Radius
    center_x = -0.803  
    center_y = 0.716  
    fixed_z = 1.037  
    object_position = np.array([center_x, center_y, fixed_z])  # Object's position

    direction_markers = MarkerArray() # MarkerArray for direction vectors
    
    # File writing Setup:
    filename = "/home/esirem/Desktop/M2_Thesis/4-RTI_Bot/eva_rti_ws/src/lightbot/data/light_pos.lp"
    with open(filename, "w") as file:
        file.write(f"{num_theta * num_phi}\n")  # Number of waypoints written on the first line

        waypoint_id = 0
        for phi_idx in range(num_phi):                         # Vertical elevation angle
            phi = math.radians(65 - (phi_idx * 65 / num_phi))  # Varies from 65° (top) to 0°

            for theta_idx in range(num_theta):  # Horizontal spread
                theta = math.radians(-60 + (theta_idx * 95 / (num_theta - 1)))  # Spread from -60° to 35°
                """
                the formulas for Cartesian coordinates in terms of spherical coordinates 
                are x=ρsinϕcosθ y=ρsinϕsinθ z=ρcosϕ in literature review.
                https://mathinsight.org/spherical_coordinates#:~:text=In%20summary%2C%20the%20formulas%20for,%CE%B8z%3D%CF%81cos%CF%95.
                """        
                x = center_x + radius * math.cos(phi) * math.cos(theta)
                y = center_y + radius * math.cos(phi) * math.sin(theta)
                z = fixed_z + radius * math.sin(phi)                   # Elevation

                # Computing the angles for file_name:
                theta_deg = math.degrees(theta) % 360  
                phi_deg = math.degrees(phi)  

                file_name = f"LDR_{waypoint_id}_Theta_{theta_deg:.2f}_Phi_{phi_deg:.2f}.png"
                file.write(f"{file_name} {x:.6f} {y:.6f} {z:.6f}\n")

                # Add waypoint marker
                point = Point(x, y, z)
                waypoint_marker.points.append(point)

                # Computing the direction vector for correct positioning of the light source
                waypoint_position = np.array([x, y, z])
                direction = object_position - waypoint_position # direction of the vector without tilt
                direction = direction / np.linalg.norm(direction)  

                # Apply slight downward tilt:
                tilt_angle_rad = np.radians(-10)
                axis = np.cross(direction, [0, 0, 1])  
                if np.linalg.norm(axis) < 1e-6:
                    axis = np.array([1, 0, 0])  
                axis = axis / np.linalg.norm(axis)  
                tilt_rotation = R.from_rotvec(tilt_angle_rad * axis)
                direction = tilt_rotation.apply(direction)
                direction = direction / np.linalg.norm(direction) # tilted direction vector

                # Create arrow marker for direction
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "world"
                arrow_marker.header.stamp = rospy.Time.now()
                arrow_marker.ns = "direction_vectors"
                arrow_marker.id = waypoint_id  
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.scale.x = 0.02  
                arrow_marker.scale.y = 0.04  
                arrow_marker.scale.z = 0.05  
                arrow_marker.color.r = 0.0  
                arrow_marker.color.g = 1.0  
                arrow_marker.color.b = 0.0
                arrow_marker.color.a = 1.0
                arrow_marker.points.append(Point(x, y, z))
                arrow_marker.points.append(Point(x + direction[0] * 0.2,
                                                y + direction[1] * 0.2,
                                                z + direction[2] * 0.2))
                direction_markers.markers.append(arrow_marker)

                waypoint_id += 1

        rospy.loginfo(f"Waypoints saved to {filename}")

    # Box Marker (Target Object)
    box_marker = Marker()
    box_marker.header.frame_id = "world"
    box_marker.header.stamp = rospy.Time.now()
    box_marker.ns = "box_marker"
    box_marker.id = 99
    box_marker.type = Marker.CUBE  
    box_marker.action = Marker.ADD
    box_marker.pose.position.x = center_x  
    box_marker.pose.position.y = center_y
    box_marker.pose.position.z = fixed_z - 0.2
    box_marker.pose.orientation.w = 1.0
    box_marker.scale.x = 0.15  
    box_marker.scale.y = 0.15  
    box_marker.scale.z = 0.15  
    box_marker.color.r = 0.0  
    box_marker.color.g = 0.0
    box_marker.color.b = 1.0
    box_marker.color.a = 1.0  

    return waypoint_marker, direction_markers, box_marker

"""
Function to read the data from the .lp file: 
--> It extracts the file_name along with position of the light
"""
def decipher_lps(lp_txt):
  img_file_names = []
  x_array = []
  y_array = []
  z_array = [] 
  lines = lp_txt.splitlines()
  for line in lines[1:]: # starting from line 2 as the first line just contains the total number of light positions
      img_file_names.append(line.split(" ")[0])        
      x_array.append(float(line.split(" ")[1]))
      y_array.append(float(line.split(" ")[2]))
      z_array.append(float(line.split(" ")[3]))
  return img_file_names, x_array, y_array, z_array

"""
Computes the end-effector's orientation such that it reaches (x_goal, y_goal, z_goal)
while pointing towards (x_obj, y_obj, z_obj) with the specified tilt.

--> Finding the vector pointing from the goal position to the object's position
--> The orientation is computed by aligning the default Y-axis (using TF in RVIZ, we can see that Y-axis is pointing forward, so "viewing direction" is aligned with the positive Y-axis) towards the vector pointing to the object.
--> A slight downward tilt of -10 degrees (in case of semi-sphere) is applied for better lighting effect.
"""
def compute_pose_towards_object(position):    
    x_goal, y_goal, z_goal = position                 # 3D point on the circle/sphere
    x_obj, y_obj, z_obj = [-0.803, 0.716, 1.037]      # object position

    direction = np.array([x_obj - x_goal, y_obj - y_goal, z_obj - z_goal]) # direction vector
    direction = direction / np.linalg.norm(direction) # normalized
    print("normalized direction vector is: ", direction)
     
    # Computing rotation from default Y-axis to the direction
    default_y_axis = np.array([0, 1, 0])
    
    # Creating a rotation matrix that aligns default_axis to the direction vector
    rotation, _ = R.align_vectors([direction], [default_y_axis])
    quaternion__ = rotation.as_quat() # Converting to quaternion
    
    # Ensuring the quaternion consistency to avoid flips
    if quaternion__[3] < 0:  # Enforcing a consistent sign for the scalar part
        quaternion__ *= -1
    print("Quaternion before rotation is: ", quaternion__)
    
    
    # Applying a slight downward tilt to the direction vector so that we get the correct orientation taking into consideration the tilt angle
    
    # tilt_angle_rad = np.radians(-20)  # for circular points
    tilt_angle_rad = np.radians(-10)    # for spherical points
    
    # Finding the perpendicular axis for rotation
    axis = np.cross(direction, [0, 0, 1])  # Cross product with Z-axis
    if np.linalg.norm(axis) < 1e-6:  
        axis = np.array([1, 0, 0])  # Default axis if direction is purely vertical
    axis = axis / np.linalg.norm(axis) 

    # Applying the tilt
    tilt_rotation = R.from_rotvec(tilt_angle_rad * axis)
    tilted_direction = tilt_rotation.apply(direction)
    tilted_direction = tilted_direction / np.linalg.norm(tilted_direction)

    default_y_axis = np.array([0, 1, 0])  # Assuming default EE direction is along Y-axis
    rotation_, _ = R.align_vectors([tilted_direction], [default_y_axis])
    
    # Convert rotation to quaternion
    quaternion = rotation_.as_quat()
    quaternion = quaternion / np.linalg.norm(quaternion)

    # Ensuring the quaternion consistency
    if quaternion[3] < 0:
        quaternion *= -1
    print("Quaternion after rotation is: ", quaternion)
  
    target_pose = Pose()
    target_pose.position = Point(x_goal, y_goal, z_goal)
    target_pose.orientation = Quaternion(*quaternion)

    return target_pose

"""
Function to calculate the Euclidean distance between two points in 3D
"""
def distance(p1, p2):
    return math.sqrt((p1.position.x - p2.position.x)**2 +
                     (p1.position.y - p2.position.y)**2 +
                     (p1.position.z - p2.position.z)**2)

"""
Function to sort the poses based on the Euclidean distance:
--> Starts from the first pose and then reach the next pose based on the minimum distance
"""
def sort_poses_by_proximity(pose_array):
    output = copy.deepcopy(pose_array)
    poses = output.poses
    sorted_poses = []
    sorted_indices = []

    remaining = poses.copy()
    current_pose = remaining.pop(0)  # Starting from the first pose
    sorted_poses.append(current_pose)
    sorted_indices.append(0)

    while remaining:
        closest_pose = min(remaining, key=lambda p: distance(current_pose, p))
        index = poses.index(closest_pose)
        sorted_poses.append(closest_pose)
        sorted_indices.append(index)
        remaining.remove(closest_pose)
        current_pose = closest_pose

    output.poses = sorted_poses
    return output, sorted_indices

"""
Function to send goal pose to the robot:
--> Creats a SimpleActionClient to communicate with the robot driver server
--> Sends the goal pose and waits for the execution result
"""
def send_goal_to_robot(robot_driver_server_name, pt, trigger):
  
  client = actionlib.SimpleActionClient(robot_driver_server_name, lightbot.msg.driver_actionAction)
  client.wait_for_server()
  print("Utilities send goal to robot")
  
  goal = lightbot.msg.driver_actionGoal()  # Creating a goal message
  # Preparing the pose with proper frame and timestamp:
  pose_stamped = PoseStamped()
  pose_stamped.header.frame_id = "world"
  pose_stamped.header.stamp = rospy.Time(0)
  pose_stamped.pose = pt
  goal.target_pose = pose_stamped
  msg = Bool()
  msg.data = trigger
  goal.trigger_signal = msg
  client.send_goal(goal)  # Sending the goal to the action server
  
  finished_before_timeout = client.wait_for_result(rospy.Duration(100.0)) # Waits up to 100 seconds for the robot to finish its task
  success = False
  system_err = False
  if client.get_result() is not None:
    success = client.get_result().succeeded.data
    if client.get_result().system_err.data:
      system_err = True
      
  """
  Retry sending the goal if:
  - The previous goal did not complete within the 100 seconds, OR
  - No result was returned from the robot action server, OR
  - The robot reported a system error (e.g., hardware fault)
  """
  while finished_before_timeout is not True or client.get_result() is None or system_err:  
    system_err = False
    print("Robot arm is down. Please fix and restart the arm server.")   
    client = actionlib.SimpleActionClient(robot_driver_server_name, lightbot.msg.driver_actionAction)
    client.wait_for_server()
    client.send_goal(goal)
    finished_before_timeout = client.wait_for_result(rospy.Duration(100.0))  
    if client.get_result() is not None:
      success = client.get_result().succeeded.data
      if client.get_result().system_err.data:
        system_err = True
  
  return client.get_result().actual_acquired_pose, client.get_result().succeeded

"""
The function to update the color of the direction vector that was not reachable or not reached by the Robotic arm during acquisition:

- Pose_stamped contains the actual missed pose's position
- marker contains the position of the direction vectors
--> We compare the two positions to make sure that we are changing the color of the missed pose only:

What math.isclose(a, b, abs_tol=0.01) Does:
    Returns True if |a - b| <= 0.01
"""
def update_missed_waypoint_color(pose_stamped, direction_markers):
    found = False
    for marker in direction_markers.markers:
        if (math.isclose(marker.points[0].x, pose_stamped.pose.position.x, abs_tol=0.01) and
            math.isclose(marker.points[0].y, pose_stamped.pose.position.y, abs_tol=0.01) and
            math.isclose(marker.points[0].z, pose_stamped.pose.position.z, abs_tol=0.01)):

            # Change color to red for any missed waypoints
            marker.color.r = 1.0  
            marker.color.g = 0.0  
            marker.color.b = 0.0  
            marker.color.a = 1.0  
            found = True
            break  

    if not found:
        return None  # If the waypoint isn't found then return nothing
    return direction_markers  # If found, return the updated markers

""" 
--------------------------------------------------------------------------------
Function for rotating the robotic arm about a specific axis (currently y-axis):
--> Useful when a certain position can't be reached with a specific orientation.
--> This function is not used in the final version, as correct direction of light source is very important for current setup
"""
def rotate_about_y_axis(pose, angle):
  p = copy.deepcopy(pose)
  q = [0, 0, 0, 1]
  q[0] = pose.orientation.x
  q[1] = pose.orientation.y
  q[2] = pose.orientation.z
  q[3] = pose.orientation.w
  q_y = quaternion_about_axis(math.radians(angle), [0,1,0])
  q = quaternion_multiply(q,q_y)
  p.orientation.x = q[0]
  p.orientation.y = q[1]
  p.orientation.z = q[2]
  p.orientation.w = q[3]

  return p
""" 
--------------------------------------------------------------------------------
"""
  