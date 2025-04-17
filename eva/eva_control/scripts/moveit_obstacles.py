#! /usr/bin/env python3
""" 
This Python script adds obstacles to the MoveIt planning scene to ensure safe motion planning:
1. Adds the sides of the box as obstacles.
2. Places a cylinder under the camera to prevent the robot from moving underneath it.
3. Adds the XY platform as an obstacle to avoid collisions during motion.
"""
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def add_obstacles():
    rospy.init_node("add_obstacles_to_scene", anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)  

    frame_id = "world"  

    # Box:
    
    # top
    tabletop_pose = PoseStamped()
    tabletop_pose.header.frame_id = frame_id
    tabletop_pose.pose.position.x = -0.75
    tabletop_pose.pose.position.y = 0.75
    tabletop_pose.pose.position.z = 1.95  
    tabletop_pose.pose.orientation.w = 1.0
    scene.add_box("tabletop", tabletop_pose, (1.5, 1.46, 0.1))
    # Left wall
    left_wall_pose = PoseStamped()
    left_wall_pose.header.frame_id = frame_id
    left_wall_pose.pose.position.x = -1.5
    left_wall_pose.pose.position.y = 0.75
    left_wall_pose.pose.position.z = 1.325  
    left_wall_pose.pose.orientation.w = 1.0
    scene.add_box("left_wall", left_wall_pose, (0.05, 1.46, 1.35))
    # Right wall
    right_wall_pose = PoseStamped()
    right_wall_pose.header.frame_id = frame_id
    right_wall_pose.pose.position.x = 0.0
    right_wall_pose.pose.position.y = 0.75
    right_wall_pose.pose.position.z = 1.325
    right_wall_pose.pose.orientation.w = 1.0
    scene.add_box("right_wall", right_wall_pose, (0.05, 1.46, 1.35))
    # Back wall
    back_wall_pose = PoseStamped()
    back_wall_pose.header.frame_id = frame_id
    back_wall_pose.pose.position.x = -0.75
    back_wall_pose.pose.position.y = 1.5
    back_wall_pose.pose.position.z = 1.325
    back_wall_pose.pose.orientation.w = 1.0
    scene.add_box("back_wall", back_wall_pose, (1.5, 0.05, 1.35))
    # Bottom surface
    bottom_pose = PoseStamped()
    bottom_pose.header.frame_id = frame_id
    bottom_pose.pose.position.x = -0.75
    bottom_pose.pose.position.y = 0.75
    bottom_pose.pose.position.z = 0.75  # Bottom height
    bottom_pose.pose.orientation.w = 1.0
    scene.add_box("bottom_surface", bottom_pose, (1.5, 1.46, 0.05))
    rospy.loginfo("table added to MoveIt planning scene.")
    
    # xy platform:
    table_x = -0.75  
    table_y = 0.75   
    table_z = 0.90   
    table_length = 0.67  
    table_width = 0.62   
    boundary_thickness = 0.05 
    z_position = table_z + (boundary_thickness / 2)

    # Front Boundary
    front_pose = PoseStamped()
    front_pose.header.frame_id = "world"
    front_pose.pose.position.x = table_x
    front_pose.pose.position.y = table_y + (table_length / 2)
    front_pose.pose.position.z = z_position
    front_pose.pose.orientation.w = 1.0
    scene.add_box("front_boundary", front_pose, (table_length, boundary_thickness, boundary_thickness))
    # Back Boundary
    back_pose = PoseStamped()
    back_pose.header.frame_id = "world"
    back_pose.pose.position.x = table_x
    back_pose.pose.position.y = table_y - (table_length / 2)
    back_pose.pose.position.z = z_position
    back_pose.pose.orientation.w = 1.0
    scene.add_box("back_boundary", back_pose, (table_length, boundary_thickness, boundary_thickness))
    # Left Boundary
    left_pose = PoseStamped()
    left_pose.header.frame_id = "world"
    left_pose.pose.position.x = table_x - (table_width / 2)
    left_pose.pose.position.y = table_y
    left_pose.pose.position.z = z_position
    left_pose.pose.orientation.w = 1.0
    scene.add_box("left_boundary", left_pose, (boundary_thickness, table_width, boundary_thickness))
    # Right Boundary
    right_pose = PoseStamped()
    right_pose.header.frame_id = "world"
    right_pose.pose.position.x = table_x + (table_width / 2)
    right_pose.pose.position.y = table_y
    right_pose.pose.position.z = z_position
    right_pose.pose.orientation.w = 1.0
    scene.add_box("right_boundary", right_pose, (boundary_thickness, table_width, boundary_thickness))
    rospy.loginfo("xy plotter added to MoveIt planning scene.")

    # cylinder:
    cylinder_radius = 0.05 
    cylinder_height = 0.65 

    cylinder_pose = PoseStamped()
    cylinder_pose.header.frame_id = "world"  
    cylinder_pose.pose.position.x = -0.715  
    cylinder_pose.pose.position.y = 0.745  
    cylinder_pose.pose.position.z = 1.28  
    cylinder_pose.pose.orientation.w = 1.0  
    scene.add_cylinder("cylinder_obstacle", cylinder_pose, cylinder_height, cylinder_radius)
    rospy.loginfo("Cylinder obstacle added to MoveIt planning scene.")

    rospy.sleep(2)  


if __name__ == "__main__":
    add_obstacles()

