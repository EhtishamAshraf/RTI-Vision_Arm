search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=eva_arm.srdf
robot_name_in_srdf=eva_arm
moveit_config_pkg=eva_without_rail_moveit_config
robot_name=eva_arm
planning_group_name=eva
ikfast_plugin_pkg=eva_ikfast_plugin
base_link_name=base_link
eef_link_name=link6
ikfast_output_path=/home/ram/ws/src/imvia/eva_ikfast_plugin/src/eva_arm_eva_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
